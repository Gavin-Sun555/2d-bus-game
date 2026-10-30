package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// AutopilotState represents the current behavioral state of the autonomous bus driver
type AutopilotState int

const (
	AutopilotStateCruising AutopilotState = iota
	AutopilotStateApproaching
	AutopilotStateDocked
	AutopilotStateDeparting
)

func (s AutopilotState) String() string {
	switch s {
	case AutopilotStateCruising:
		return "巡航行驶"
	case AutopilotStateApproaching:
		return "进站减速"
	case AutopilotStateDocked:
		return "停靠上下客"
	case AutopilotStateDeparting:
		return "出站汇入"
	default:
		return "巡航行驶"
	}
}

// Autopilot tuning constants (px, px/s, px/s²)
const (
	apCruiseSpeed        = 150.0                    // Standard cruising speed (~40 km/h)
	apBerserkCruiseSpeed = 445.0                    // Berserk mode straightaway top cruise speed (80 km/h)
	apRoundaboutSpeed    = sim.RoundaboutSpeedLimit // Speed cap while circulating a roundabout
	apCornerDecel        = 120.0                    // Planned deceleration used when anticipating corners
	apStopDecel          = 85.0                     // Planned deceleration used when braking into a stop bay
	apApproachDist       = 220.0                    // Remaining path length at which the bus is "approaching"
	apTrackWindow        = 260.0                    // Arc-length window searched ahead of the current segment
	apRelocalizeDist     = 140.0                    // Lateral error beyond which the tracker re-localizes globally
	apBayStraight        = 60.0                     // Straight, curb-aligned run-in before the bay center
)

// VehicleObstacle represents any vehicle in the world (civilian car or another bus)
type VehicleObstacle struct {
	Bus              *sim.Bus `json:"-"`
	YieldingTo       *sim.Bus
	JunctionTurn     *sim.CarJunctionTurn
	JunctionCorridor *sim.JunctionCorridor
	Pos              sim.Vec2
	Heading          float64
	Speed            float64
	Length           float64
	Width            float64
}

// Autopilot overtaking states (Berserk Mode on multi-lane roads)
const (
	OvertakeNone         = 0
	OvertakeChangingLane = 1
	OvertakePassing      = 2
	OvertakeReturning    = 3
)

// AutopilotDriver encapsulates autonomous driving intelligence for both the player's bus
// (when autopilot is enabled) and background AI route buses.
type AutopilotDriver struct {
	stationQueries                 stationDecisionCache
	stationQueriesActive           bool
	temporaryGeometry              *temporaryGeometry
	disableSweepCandidates         bool
	roundaboutEntryGeometry        sim.RoundaboutEntryGeometry
	disableRoundaboutEntryGeometry bool
	roundaboutEdgeRetry            float64
	roundaboutEdgeRecovery         *RoundaboutEdgeRecovery
	dockDepartureRun               float64
	smoothDockDeparture            bool
	articulatedPath                bool
	queueStopID                    int
	parallelEntryCenter            *sim.Vec2
	uTurn                          *uTurnManeuver
	approachMerge                  *sim.BusLaneMergeRequest
	approachMergeJoined            bool
	roadSafetyHold                 bool
	roadRecoverySpeed              float64
	midLaneRecovery                *sim.BusLaneMergeRequest
	midLaneSweepClear              bool
	arrivalEscapeRetry             float64
	sidecarEscapeRetry             float64
	sidecarPresteer                float64
	sidecarPresteerValid           bool
	sidecarEscapeActive            bool
	departurePresteerValid         bool
	departureEscapeActive          bool
	State                          AutopilotState
	DisallowOncomingOvertake       bool    // Fleet vehicles may pass only in same-direction lanes.
	BerserkMode                    bool    // true if Berserk mode is active (player bus fast straightaways)
	CruiseSpeedLimit               float64 // Optional fixed fleet-vehicle speed limit; zero uses player settings.
	BerserkMaxSpeed                float64 // Configurable Berserk mode cruise speed in px/s (default: 445.0 / 80 km/h)
	DwellTimer                     float64
	DwellTotal                     float64
	DwellFinished                  bool
	DockedStopID                   int
	LastTargetID                   int
	WaypointIdx                    int // Index of the drive-path segment currently being tracked (monotonic)

	// Overtaking state for Berserk mode on multi-lane roads
	OvertakeState        int     // OvertakeNone, OvertakeChangingLane, OvertakePassing, OvertakeReturning
	LaneOffset           float64 // Dynamic lateral offset from drivePath (-lw = left lane, +lw = right lane)
	TargetLaneOffset     float64 // Target lateral offset
	lastTargetLaneOffset float64 // Previous target lateral offset to detect target changes
	LaneOffsetStart      float64 // Starting lateral offset for smooth S-curve interpolation
	LaneOffsetProgress   float64 // Normalized progress [0.0, 1.0] along the transition S-curve
	LaneOffsetDuration   float64 // Duration in seconds for current transition S-curve
	OvertakeTimer        float64 // Duration in current overtake phase

	// Cached drive path for the active leg (prevStop -> targetStop)
	drivePath    []sim.Vec2
	cumLen       []float64
	pathWorld    *sim.World
	pathFromID   int
	pathToID     int
	pathLane     float64
	pathSynced   bool // false => next projection performs a global (non-monotonic) search
	pathRevision int

	stationDock          *stationDockPlan
	departureFromDock    bool
	pathSkipTarget       bool
	bypassStopID         int
	skippedDeparture     *skippedStopDeparture
	directRoundaboutExit *sim.Vec2

	busLaneJunction     *sim.Vec2
	rightTurnJunction   *sim.Vec2
	rightTurnHeading    float64
	rightTurnWaiting    bool
	busJunctionEntered  bool
	ordinaryBusLeft     bool
	pathOrdinaryBusLeft bool
	busTurnHoldDistance float64
	pathReturnHold      *sim.BusLaneMergeRequest // transient road-aligned hold while a route taper is blocked
	CanSkipTargetStop   bool                     // Whether the active target stop can be skipped (no alighting pax and station empty/occupied)

	// Last-frame tracking diagnostics (used by tests and debugging overlays)
	LastPathS       float64  // Arc-length position of the bus along the drive path
	LastRemaining   float64  // Remaining drive-path length to the target bay
	LastLateralErr  float64  // Perpendicular distance from the bus to the drive path
	LastPursuitPt   sim.Vec2 // Pure-pursuit lookahead target point
	LastTargetSpeed float64  // Commanded target speed
}

// NewAutopilotDriver creates a fresh autonomous bus controller
func NewAutopilotDriver() *AutopilotDriver {
	return &AutopilotDriver{
		queueStopID:        -1,
		State:              AutopilotStateCruising,
		BerserkMaxSpeed:    apBerserkCruiseSpeed,
		DwellTimer:         0,
		DwellTotal:         0, // Retained for save compatibility; passengers determine dwell.
		DwellFinished:      false,
		DockedStopID:       -1,
		LastTargetID:       -1,
		WaypointIdx:        0,
		LaneOffsetProgress: 1.0,
		LaneOffsetDuration: 1.0,
		pathFromID:         -1,
		pathToID:           -1,
		bypassStopID:       -1,
	}
}

// GetBerserkCruiseSpeed returns the configured top cruise speed for Berserk mode in px/s
func (ad *AutopilotDriver) GetBerserkCruiseSpeed() float64 {
	if ad != nil && ad.BerserkMaxSpeed > 50.0 {
		return ad.BerserkMaxSpeed
	}
	return apBerserkCruiseSpeed
}

// Reset clears autonomous driving internal state
func (ad *AutopilotDriver) Reset() {
	ad.temporaryGeometry = nil
	ad.roundaboutEdgeRecovery = nil
	ad.roundaboutEdgeRetry = 0
	ad.pathReturnHold = nil
	ad.queueStopID = -1
	ad.parallelEntryCenter = nil
	ad.uTurn = nil
	ad.State = AutopilotStateCruising
	ad.busLaneJunction = nil
	ad.rightTurnJunction = nil
	ad.rightTurnWaiting = false
	ad.busJunctionEntered = false
	ad.ordinaryBusLeft = false
	ad.pathOrdinaryBusLeft = false
	ad.busTurnHoldDistance = 0
	ad.arrivalEscapeRetry = 0
	ad.sidecarEscapeRetry = 0
	ad.sidecarPresteer, ad.sidecarPresteerValid = 0, false
	ad.sidecarEscapeActive = false
	ad.departurePresteerValid = false
	ad.departureEscapeActive = false
	ad.roadSafetyHold = false
	ad.roadRecoverySpeed = 0
	ad.midLaneRecovery = nil
	ad.midLaneSweepClear = false
	ad.approachMerge = nil
	ad.approachMergeJoined = false
	ad.stationDock = nil
	ad.departureFromDock = false
	ad.dockDepartureRun = 0
	ad.smoothDockDeparture = false
	ad.pathSkipTarget = false
	ad.bypassStopID = -1
	ad.skippedDeparture = nil
	ad.directRoundaboutExit = nil
	ad.DwellTimer = 0
	ad.DwellFinished = false
	ad.DockedStopID = -1
	ad.LastTargetID = -1
	ad.WaypointIdx = 0
	ad.OvertakeState = OvertakeNone
	ad.LaneOffset = 0.0
	ad.TargetLaneOffset = 0.0
	ad.lastTargetLaneOffset = 0.0
	ad.LaneOffsetStart = 0.0
	ad.LaneOffsetProgress = 1.0
	ad.LaneOffsetDuration = 1.0
	ad.OvertakeTimer = 0.0
	ad.drivePath = nil
	ad.cumLen = nil
	ad.pathWorld = nil
	ad.pathFromID = -1
	ad.pathToID = -1
	ad.pathSynced = false
}

// DrivePath returns the currently cached drive path (read-only; for HUD/debug/tests)
func (ad *AutopilotDriver) DrivePath() []sim.Vec2 {
	return ad.drivePath
}

// PrepareInitialLeg initializes a newly dispatched bus midway through a leg.
// The first control tick localizes globally; no terminal departure or boarding
// is fabricated for the untraveled portion of the route.
func (ad *AutopilotDriver) PrepareInitialLeg(bus *sim.Bus, w *sim.World, r *sim.Route, lw float64) bool {
	if bus == nil || w == nil || r == nil || len(r.StopIDs) < 2 || r.CurrentIndex < 0 || r.CurrentIndex >= len(r.StopIDs) {
		return false
	}
	from := r.StopIDs[(r.CurrentIndex-1+len(r.StopIDs))%len(r.StopIDs)]
	to := r.StopIDs[r.CurrentIndex]
	if from < 0 || to < 0 || from >= len(w.Stops) || to >= len(w.Stops) {
		return false
	}
	ad.Reset()
	ad.articulatedPath = bus.IsArticulated()
	ad.PrepareLeg(w, w.Stops[from], w.Stops[to], lw)
	return len(ad.drivePath) > 1
}

// ComputeControls calculates vehicle inputs (throttle, brake, steer, handbrake, toggleDoors)
// along with navigation guidance for any bus following a Route in World.
func (ad *AutopilotDriver) ComputeControls(
	bus *sim.Bus,
	route *sim.Route,
	world *sim.World,
	tls *sim.TrafficLightSystem,
	obstacles []VehicleObstacle,
	laneWidth float64,
	dt float64,
) (throttle, brake, steer float64, handbrake, toggleDoors bool, isDocked bool) {
	response := 1.0
	if ad != nil && ad.BerserkMode {
		response = 1.2
	}
	bus.SetNextAccelerationScale(response)
	if ad != nil {
		ad.roundaboutEdgeRetry = math.Max(0, ad.roundaboutEdgeRetry-dt)
		ad.arrivalEscapeRetry = math.Max(0, ad.arrivalEscapeRetry-dt)
		ad.sidecarEscapeRetry = math.Max(0, ad.sidecarEscapeRetry-dt)
	}
	bus.StationMergeRequest = nil
	bus.LaneMergeRequest = nil
	bus.RoundaboutEntry = nil
	if ad == nil || route == nil || len(route.StopIDs) == 0 || world == nil || len(world.Stops) == 0 {
		return 0.0, 1.0, 0.0, true, false, false
	}
	ad.stationQueries = stationDecisionCache{}
	ad.stationQueriesActive = true
	defer func() { ad.stationQueriesActive = false; ad.stationQueries = stationDecisionCache{} }()
	world.BeginBoundaryQueries(laneWidth)
	defer world.EndBoundaryQueries()
	bus.PreparePosePredictions()

	targetStopID := route.GetCurrentTargetStopID()
	if targetStopID < 0 || targetStopID >= len(world.Stops) {
		return 0.0, 1.0, 0.0, true, false, false
	}
	targetStop := world.Stops[targetStopID]
	ad.LastTargetID = targetStopID

	// Find previous stop in sequence to obtain the driving lane path of the active leg
	prevIdx := (route.CurrentIndex - 1 + len(route.StopIDs)) % len(route.StopIDs)
	prevStopID := route.StopIDs[prevIdx]
	var prevStop *sim.BusStop
	if prevStopID >= 0 && prevStopID < len(world.Stops) {
		prevStop = world.Stops[prevStopID]
	} else {
		prevStop = targetStop
	}

	distToStop := bus.Pos.Distance(targetStop.Pos)
	speedAbs := math.Abs(bus.Speed)

	if ad.roundaboutEdgeRecovery != nil {
		f := autopilotFrame{bus: bus, world: world, obstacles: obstacles, laneWidth: laneWidth}
		th, br, st := ad.roundaboutEdgeControls(f)
		return th, br, st, false, false, false
	}
	// Ensure automatic transmission stays in Drive
	if bus.Gear != sim.GearDrive {
		bus.ShiftTo(sim.GearDrive)
	}

	// =========================================================================
	// 1. DOCKED STATE: Passenger Boarding, Dwell Timer & Door Cycling
	// =========================================================================
	if ad.State == AutopilotStateDocked {
		return ad.dockedControls(bus, route, speedAbs, dt)
	}
	if inputs, active := ad.stationQueueControls(bus, targetStop, obstacles); active {
		return inputs.throttle, inputs.brake, inputs.steer, false, inputs.toggleDoors, inputs.isDocked
	}

	// =========================================================================
	// 2. PATH LOCALIZATION
	// =========================================================================
	if ad.articulatedPath != bus.IsArticulated() {
		ad.articulatedPath = bus.IsArticulated()
		ad.drivePath = nil
	}
	ad.PrepareLeg(world, prevStop, targetStop, laneWidth)
	ad.reconnectServedBayDeparture(bus, prevStop, laneWidth)
	if !ad.pathSynced {
		ad.trackPath(bus.Pos)
	}

	remDist := ad.LastRemaining
	if remDist <= 0.0 && len(ad.cumLen) > 0 {
		remDist = ad.cumLen[len(ad.cumLen)-1] - ad.LastPathS
	}
	if remDist <= 0.0 {
		remDist = bus.Pos.Distance(targetStop.Pos)
	}

	if ad.prepareSkippedStopLane(bus, targetStop, world, laneWidth, remDist) {
		ad.trackPath(bus.Pos)
		remDist = ad.cumLen[len(ad.cumLen)-1] - ad.LastPathS
	}

	if ad.departureFromDock {
		ad.connectDockDeparture(bus, prevStop)
		ad.departureFromDock = false
	}
	frameForDock := autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, targetStop: targetStop, laneWidth: laneWidth, s: ad.LastPathS, speedAbs: speedAbs, remaining: remDist, distToStop: distToStop}
	ad.clearPassedBusLaneJunction(frameForDock)
	ad.requestStationMerge(frameForDock)
	if inputs, active := ad.stationDockControls(frameForDock); active {
		if (ad.sidecarEscapeActive || (inputs.throttle == 0 && inputs.brake > 0)) && !inputs.isDocked && !inputs.toggleDoors {
			desired, _, _, _ := ad.pursuitSteering(frameForDock)
			if escapeSteer, clear := ad.stationArrivalEscape(frameForDock, desired); clear {
				bus.SetNextAccelerationScale(1)
				th, br := ad.longitudinalControls(bus.Speed, stationArrivalCrawlSpeed(frameForDock))
				ad.LastTargetSpeed = stationArrivalCrawlSpeed(frameForDock)
				return th, br, escapeSteer, false, false, false
			}
			if escapeSteer, clear := ad.sidecarEscape(frameForDock, desired); clear {
				bus.SetNextAccelerationScale(1)
				th, br := ad.longitudinalControls(bus.Speed, 18)
				ad.LastTargetSpeed = 18
				return th, br, escapeSteer, false, false, false
			} else if ad.sidecarPresteerValid {
				return 0, 1, escapeSteer, false, false, false
			}
		}
		return inputs.throttle, inputs.brake, inputs.steer, false, inputs.toggleDoors, inputs.isDocked
	}

	ad.updateUTurn(bus, world, laneWidth)

	// Update overtaking controller (Berserk Mode on multi-lane roads)
	ad.updateOvertaking(bus, world, tls, obstacles, laneWidth, ad.LastPathS, speedAbs, remDist, dt, targetStop)

	posForTracking := ad.basePathPosition(bus.Pos, world, bus.Heading, laneWidth)

	s, lateralErr := ad.trackPath(posForTracking)
	total := 0.0
	if n := len(ad.cumLen); n > 0 {
		total = ad.cumLen[n-1]
	}
	remaining := math.Max(0, total-s)

	// =========================================================================
	// 3. DOCKING TRIGGER (Only when arriving, never while departing)
	// =========================================================================
	alignedNearBay := distToStop < 55.0 && math.Abs(sim.NormalizeAngle(bus.Heading-targetStop.Heading)) < 0.45
	// Fallback: the bus has come to rest at the very end of its path right at the stop
	restingAtStop := remaining < 15.0 && distToStop < 70.0
	maxDockSpeed := 10.0
	if ad.BerserkMode {
		maxDockSpeed = 26.0 // Snap into docked state smoothly as bus enters the bay
	}
	inBaseLane := math.Abs(ad.LaneOffset) < 10.0
	isBerserkOuterDock := ad.BerserkMode && targetStop.ContainsBusInOuterLane(bus, laneWidth) &&
		(speedAbs < 15.0 || (ad.OvertakeState != OvertakePassing && speedAbs < maxDockSpeed))
	platformDockReady := targetStop.ContainsBus(bus)
	fallbackDockReady := inBaseLane && (alignedNearBay || restingAtStop)
	if targetStop.HasTwoBerths() {
		platformDockReady = false
		fallbackDockReady = false
		if speedAbs < maxDockSpeed && distToStop < targetStop.Length+bus.OverallLength()+64 {
			for _, berth := range ad.availableStationBerths(bus, targetStop, obstacles) {
				if atSelectedStationBerth(bus, berth, targetStop.Heading) {
					platformDockReady = true
					break
				}
			}
		}
	}
	outerDockReady := isBerserkOuterDock
	bayDockReady := (platformDockReady || fallbackDockReady) && ad.stationBoardingCapacity(bus, targetStop, obstacles)
	if !ad.CanSkipTargetStop && ad.State != AutopilotStateDeparting && speedAbs < maxDockSpeed &&
		(outerDockReady || bayDockReady) && stationBoardingPoseClear(frameForDock) {
		if outerDockReady {
			rank := 1
			if targetStop.ContainsBus(bus) {
				rank = 0
			}
			if ad.stationDock == nil {
				ad.stationDock = &stationDockPlan{stopID: targetStop.ID, rank: rank, target: bus.Pos}
			} else if rank > 0 {
				ad.stationDock.rank = rank
			}
		}
		ad.beginDocking(targetStop)
		if speedAbs < 15.0 && !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			toggleDoors = true
		}
		return 0.0, 1.0, 0.0, false, toggleDoors, true
	}

	// If doors are open outside docked state (e.g. player parked then toggled autopilot),
	// command doors to close immediately so vehicle can start moving!
	if bus.DoorsOpen || bus.DoorProgress > 0.05 {
		closeCmd := false
		if bus.DoorsOpen && speedAbs < 15.0 {
			closeCmd = true
		}
		return 0.0, 1.0, 0.0, false, closeCmd, false
	}

	ad.updateDrivingState(bus, prevStop, remaining, speedAbs)

	// =========================================================================
	// 4. PURE PURSUIT STEERING (lookahead measured from the bus's own path position)
	// =========================================================================
	frame := autopilotFrame{
		bus: bus, world: world, tls: tls, obstacles: obstacles,
		targetStop: targetStop, laneWidth: laneWidth,
		s: s, speedAbs: speedAbs, remaining: remaining,
		distToStop: distToStop, lateralErr: lateralErr,
	}
	var targetPt sim.Vec2
	steer, frame.alpha, targetPt, frame.nearRoundabout = ad.pursuitSteering(frame)
	steer = ad.midLaneSafeSteering(frame, steer)
	ad.publishJunctionCorridor(frame)
	// A mandatory queue merge retains its calibrated low-speed response so
	// the bus can straighten within the available slot.
	if ad.approachMerge != nil || ad.midLaneRecovery != nil {
		bus.SetNextAccelerationScale(1)
	}
	targetSpd := math.Min(ad.targetSpeed(frame), ad.controlSweptSpeed(frame, steer))
	targetSpd = math.Min(targetSpd, ad.articulatedSweptSpeed(frame, steer))
	if ad.busTurnHoldDistance > 0 {
		targetSpd = math.Min(targetSpd, math.Sqrt(2*135*math.Max(0, ad.busTurnHoldDistance-10)))
	}

	if targetSpd <= .5 {
		if escapeSteer, clear := ad.stationArrivalEscape(frame, steer); clear {
			steer, targetSpd = escapeSteer, stationArrivalCrawlSpeed(frame)
			bus.SetNextAccelerationScale(1)
		}
	}
	if targetSpd <= .5 || ad.departureEscapeActive {
		if escapeSteer, clear := ad.stationDepartureEscape(frame, steer); clear {
			steer, targetSpd = escapeSteer, 18
			bus.SetNextAccelerationScale(1)
		} else if ad.departurePresteerValid {
			return 0, 1, escapeSteer, false, false, false
		} else if ad.departureEscapeActive {
			targetSpd = 0
		}
	}
	if targetSpd <= .5 || ad.sidecarEscapeActive {
		if escapeSteer, clear := ad.sidecarEscape(frame, steer); clear {
			steer, targetSpd = escapeSteer, 18
			bus.SetNextAccelerationScale(1)
		} else if ad.sidecarPresteerValid {
			steer, targetSpd = escapeSteer, 0
		}
	}
	if targetSpd <= .5 && ad.tryRoundaboutEdgeRecovery(frame, steer) {
		th, br, st := ad.roundaboutEdgeControls(frame)
		return th, br, st, false, false, false
	}
	throttle, brake = ad.longitudinalControls(bus.Speed, targetSpd)

	ad.LastPathS = s
	ad.LastRemaining = remaining
	ad.LastLateralErr = lateralErr
	ad.LastPursuitPt = targetPt
	ad.LastTargetSpeed = targetSpd

	return throttle, brake, steer, false, false, false
}

// autopilotFrame contains observations shared by steering and speed planning for
// one control tick. It is passed by value; persistent state stays on the driver.
type autopilotFrame struct {
	forwardEscape  *forwardEscapeScan
	departureCache *departureSweepCache
	bus            *sim.Bus
	world          *sim.World
	tls            *sim.TrafficLightSystem
	obstacles      []VehicleObstacle
	targetStop     *sim.BusStop

	laneWidth      float64
	s              float64
	speedAbs       float64
	remaining      float64
	distToStop     float64
	lateralErr     float64
	alpha          float64
	nearRoundabout bool
}

// PathTargetStopID identifies the destination of the cached driving leg.
func (ad *AutopilotDriver) PathTargetStopID() int { return ad.pathToID }

// InvalidatePath forces the next control tick to rebuild the current driving leg.
func (ad *AutopilotDriver) InvalidatePath() { ad.drivePath = nil }
