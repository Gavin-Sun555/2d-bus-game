package game

import (
	"math"
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
	apCruiseSpeed     = 150.0 // Standard cruising speed (~40 km/h)
	apRoundaboutSpeed = 90.0  // Speed cap while circulating a roundabout
	apCornerDecel     = 120.0 // Planned deceleration used when anticipating corners
	apStopDecel       = 85.0  // Planned deceleration used when braking into a stop bay
	apApproachDist    = 220.0 // Remaining path length at which the bus is "approaching"
	apTrackWindow     = 260.0 // Arc-length window searched ahead of the current segment
	apRelocalizeDist  = 140.0 // Lateral error beyond which the tracker re-localizes globally
	apBayStraight     = 60.0  // Straight, curb-aligned run-in before the bay center
)

// VehicleObstacle represents any vehicle in the world (civilian car or another bus)
type VehicleObstacle struct {
	Pos     Vec2
	Heading float64
	Speed   float64
	Length  float64
	Width   float64
}

// AutopilotDriver encapsulates autonomous driving intelligence for both the player's bus
// (when autopilot is enabled) and background AI route buses.
type AutopilotDriver struct {
	State         AutopilotState
	DwellTimer    float64
	DwellTotal    float64
	DwellFinished bool
	DockedStopID  int
	LastTargetID  int
	WaypointIdx   int // Index of the drive-path segment currently being tracked (monotonic)

	// Cached drive path for the active leg (prevStop -> targetStop)
	drivePath  []Vec2
	cumLen     []float64
	pathWorld  *World
	pathFromID int
	pathToID   int
	pathLane   float64
	pathSynced bool // false => next projection performs a global (non-monotonic) search

	// Last-frame tracking diagnostics (used by tests and debugging overlays)
	LastPathS       float64 // Arc-length position of the bus along the drive path
	LastRemaining   float64 // Remaining drive-path length to the target bay
	LastLateralErr  float64 // Perpendicular distance from the bus to the drive path
	LastPursuitPt   Vec2    // Pure-pursuit lookahead target point
	LastTargetSpeed float64 // Commanded target speed
}

// NewAutopilotDriver creates a fresh autonomous bus controller
func NewAutopilotDriver() *AutopilotDriver {
	return &AutopilotDriver{
		State:         AutopilotStateCruising,
		DwellTimer:    0,
		DwellTotal:    2.8,
		DwellFinished: false,
		DockedStopID:  -1,
		LastTargetID:  -1,
		WaypointIdx:   0,
		pathFromID:    -1,
		pathToID:      -1,
	}
}

// Reset clears autonomous driving internal state
func (ad *AutopilotDriver) Reset() {
	ad.State = AutopilotStateCruising
	ad.DwellTimer = 0
	ad.DwellFinished = false
	ad.DockedStopID = -1
	ad.LastTargetID = -1
	ad.WaypointIdx = 0
	ad.drivePath = nil
	ad.cumLen = nil
	ad.pathWorld = nil
	ad.pathFromID = -1
	ad.pathToID = -1
	ad.pathSynced = false
}

// DrivePath returns the currently cached drive path (read-only; for HUD/debug/tests)
func (ad *AutopilotDriver) DrivePath() []Vec2 {
	return ad.drivePath
}

// BuildAutopilotDrivePath converts the orthogonal road route between two stops into a
// drivable polyline: instead of the sideways 90° jogs between bay and lane used for drawing,
// the bus merges diagonally out of the departure bay and pulls into the target bay along a
// tapered entry that ends in a straight, curb-aligned run-in.
func BuildAutopilotDrivePath(w *World, from, to *BusStop, lw float64) []Vec2 {
	if to == nil {
		return nil
	}
	h2 := Vec2{X: math.Cos(to.Heading), Y: math.Sin(to.Heading)}
	bayEntry := to.Pos.Sub(h2.Mul(apBayStraight))
	if w == nil || from == nil || from.ID == to.ID {
		return []Vec2{to.Pos.Sub(h2.Mul(200)), bayEntry, to.Pos}
	}

	raw := computeRoadPathBetweenStops(w, from, to, lw)
	if len(raw) < 4 {
		// Unreachable on the grid: best-effort direct approach into the bay
		return dedupePath([]Vec2{from.Pos, bayEntry, to.Pos})
	}

	h1 := Vec2{X: math.Cos(from.Heading), Y: math.Sin(from.Heading)}
	n := len(raw)
	out := make([]Vec2, 0, n+4)

	// --- Departure: bay -> straight pull-out -> diagonal merge into the travel lane ---
	start := 1
	out = append(out, raw[0])
	lane1 := raw[1]
	if math.Abs(lane1.Sub(raw[0]).Dot(h1)) < 10.0 {
		seg := raw[2].Sub(lane1)
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h1) > 0.9 {
			mergeDist := math.Min(160.0, segLen*0.4)
			out = append(out, raw[0].Add(h1.Mul(25.0)), lane1.Add(h1.Mul(mergeDist)))
			start = 2
		}
	}

	// --- Arrival: travel lane -> tapered entry -> straight run-in -> bay center ---
	end := n - 1 // exclusive index of the raw points copied verbatim
	lane2 := raw[n-2]
	tail := []Vec2{to.Pos}
	if math.Abs(lane2.Sub(to.Pos).Dot(h2)) < 10.0 && n-3 >= 0 {
		seg := lane2.Sub(raw[n-3])
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h2) > 0.9 {
			taperDist := math.Min(170.0, segLen*0.4)
			tail = []Vec2{lane2.Sub(h2.Mul(taperDist + apBayStraight)), bayEntry, to.Pos}
			end = n - 2
		}
	}

	for i := start; i < end; i++ {
		out = append(out, raw[i])
	}
	out = append(out, tail...)
	return dedupePath(out)
}

func dedupePath(pts []Vec2) []Vec2 {
	res := make([]Vec2, 0, len(pts))
	for _, p := range pts {
		if len(res) == 0 || res[len(res)-1].Distance(p) > 3.0 {
			res = append(res, p)
		}
	}
	return res
}

// ensurePath (re)builds the cached drive path whenever the leg, world or lane width changes
func (ad *AutopilotDriver) ensurePath(w *World, from, to *BusStop, lw float64) {
	if ad.drivePath != nil && ad.pathWorld == w && ad.pathFromID == from.ID &&
		ad.pathToID == to.ID && ad.pathLane == lw {
		return
	}
	ad.drivePath = BuildAutopilotDrivePath(w, from, to, lw)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	ad.pathWorld = w
	ad.pathFromID = from.ID
	ad.pathToID = to.ID
	ad.pathLane = lw
	ad.WaypointIdx = 0
	ad.pathSynced = false
}

// projectToSegment returns the clamped parameter t, and distance of p to segment a-b
func projectToSegment(p, a, b Vec2) (float64, float64) {
	ab := b.Sub(a)
	l2 := ab.Dot(ab)
	if l2 < 1e-9 {
		return 0, p.Distance(a)
	}
	t := Clamp(p.Sub(a).Dot(ab)/l2, 0, 1)
	return t, p.Distance(a.Add(ab.Mul(t)))
}

// trackPath localizes the bus on the drive path. Progress is monotonic within a bounded
// arc-length window (so the bus can never "jump back" and turn around); a global search is
// only used on first sync or when the bus has strayed far from the path.
// Returns the arc-length position s and the lateral distance to the path.
func (ad *AutopilotDriver) trackPath(pos Vec2) (float64, float64) {
	path := ad.drivePath
	n := len(path)
	if n < 2 {
		return 0, 0
	}
	best, bestT, bestD := -1, 0.0, math.Inf(1)
	consider := func(i int) {
		t, d := projectToSegment(pos, path[i], path[i+1])
		if d < bestD-0.5 {
			best, bestT, bestD = i, t, d
		}
	}

	if ad.pathSynced {
		for i := ad.WaypointIdx; i < n-1; i++ {
			if i > ad.WaypointIdx && ad.cumLen[i] > ad.LastPathS+apTrackWindow {
				break
			}
			consider(i)
		}
	}
	if best < 0 || bestD > apRelocalizeDist {
		best, bestT, bestD = -1, 0.0, math.Inf(1)
		for i := 0; i < n-1; i++ {
			consider(i)
		}
	}

	ad.WaypointIdx = best
	ad.pathSynced = true
	s := ad.cumLen[best] + bestT*(ad.cumLen[best+1]-ad.cumLen[best])
	ad.LastPathS = s
	return s, bestD
}

// pointAt returns the point at arc length s on the drive path, extrapolating straight past the end
func (ad *AutopilotDriver) pointAt(s float64) Vec2 {
	path := ad.drivePath
	n := len(path)
	if n == 0 {
		return Vec2{}
	}
	if n == 1 {
		return path[0]
	}
	total := ad.cumLen[n-1]
	if s >= total {
		dir := path[n-1].Sub(path[n-2]).Normalize()
		return path[n-1].Add(dir.Mul(s - total))
	}
	if s <= 0 {
		return path[0]
	}
	i := ad.WaypointIdx
	if i < 0 || i >= n-1 || ad.cumLen[i] > s {
		i = 0
	}
	for i < n-2 && ad.cumLen[i+1] < s {
		i++
	}
	segLen := ad.cumLen[i+1] - ad.cumLen[i]
	if segLen < 1e-9 {
		return path[i]
	}
	t := (s - ad.cumLen[i]) / segLen
	return path[i].Add(path[i+1].Sub(path[i]).Mul(t))
}

// minDistanceToPath calculates the minimum distance from pt to the active drivePath segments near currentWP
func minDistanceToPath(pt Vec2, path []Vec2, currentWP int) float64 {
	if len(path) < 2 {
		return 999.0
	}
	start := currentWP - 1
	if start < 0 {
		start = 0
	}
	end := currentWP + 10
	if end >= len(path) {
		end = len(path) - 1
	}
	minD := 999.0
	for i := start; i < end; i++ {
		d, _ := distToSegment(pt, path[i], path[i+1])
		if d < minD {
			minD = d
		}
	}
	return minD
}

// cornerSpeedLimit anticipates upcoming turns along the path and returns the speed the bus
// may currently drive so that it can comfortably slow down for each corner in time.
func (ad *AutopilotDriver) cornerSpeedLimit(s, speed float64) float64 {
	path := ad.drivePath
	n := len(path)
	limit := math.Inf(1)
	horizon := speed*speed/(2*apCornerDecel) + 80.0
	for k := ad.WaypointIdx + 1; k < n-1; k++ {
		d := ad.cumLen[k] - s
		if d < -20 {
			continue
		}
		if d > horizon {
			break
		}
		a := path[k].Sub(path[k-1])
		b := path[k+1].Sub(path[k])
		turn := math.Abs(NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X)))
		vc := math.Inf(1)
		switch {
		case turn > 1.2:
			vc = 55.0
		case turn > 0.7:
			vc = 75.0
		case turn > 0.3:
			vc = 100.0
		}
		if math.IsInf(vc, 1) {
			continue
		}
		allowed := math.Sqrt(vc*vc + 2*apCornerDecel*math.Max(0, d-15.0))
		limit = math.Min(limit, allowed)
	}
	return limit
}

// ComputeControls calculates vehicle inputs (throttle, brake, steer, handbrake, toggleDoors)
// along with navigation guidance for any bus following a Route in World.
func (ad *AutopilotDriver) ComputeControls(
	bus *Bus,
	route *Route,
	world *World,
	tls *TrafficLightSystem,
	obstacles []VehicleObstacle,
	laneWidth float64,
	dt float64,
) (throttle, brake, steer float64, handbrake, toggleDoors bool, isDocked bool) {
	if route == nil || len(route.StopIDs) == 0 || world == nil || len(world.Stops) == 0 {
		return 0.0, 1.0, 0.0, true, false, false
	}

	targetStopID := route.GetCurrentTargetStopID()
	if targetStopID < 0 || targetStopID >= len(world.Stops) {
		return 0.0, 1.0, 0.0, true, false, false
	}
	targetStop := world.Stops[targetStopID]
	ad.LastTargetID = targetStopID

	// Find previous stop in sequence to obtain the driving lane path of the active leg
	prevIdx := (route.CurrentIndex - 1 + len(route.StopIDs)) % len(route.StopIDs)
	prevStopID := route.StopIDs[prevIdx]
	var prevStop *BusStop
	if prevStopID >= 0 && prevStopID < len(world.Stops) {
		prevStop = world.Stops[prevStopID]
	} else {
		prevStop = targetStop
	}

	distToStop := bus.Pos.Distance(targetStop.Pos)
	speedAbs := math.Abs(bus.Speed)

	// Ensure automatic transmission stays in Drive
	if bus.Gear != GearDrive {
		bus.ShiftTo(GearDrive)
	}

	// =========================================================================
	// 1. DOCKED STATE: Passenger Boarding, Dwell Timer & Door Cycling
	// =========================================================================
	if ad.State == AutopilotStateDocked {
		throttle = 0.0
		brake = 1.0
		steer = 0.0
		isDocked = true

		// If dwell is not yet finished:
		if !ad.DwellFinished {
			// Phase 1: Open doors if stopped and doors not yet opened
			if !bus.DoorsOpen && speedAbs < 5.0 && bus.DoorProgress <= 0.05 {
				toggleDoors = true
				return throttle, brake, steer, handbrake, toggleDoors, isDocked
			}

			// Phase 2: Dwell timer while doors are open
			if bus.DoorsOpen && ad.DwellTimer > 0 {
				ad.DwellTimer -= dt
				return throttle, brake, steer, handbrake, false, isDocked
			}

			// Phase 3: Dwell expired, close doors
			if bus.DoorsOpen && ad.DwellTimer <= 0 && bus.DoorProgress >= 0.80 {
				toggleDoors = true
				ad.DwellFinished = true
				return throttle, brake, steer, handbrake, toggleDoors, isDocked
			}
		}

		// Phase 4: Wait for doors to finish closing completely, then advance and depart
		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			if route.GetCurrentTargetStopID() == ad.DockedStopID {
				route.AdvanceToNextStop()
			}
			ad.LastTargetID = route.GetCurrentTargetStopID()
			ad.State = AutopilotStateDeparting
			ad.DockedStopID = -1
			ad.WaypointIdx = 0
			ad.pathSynced = false
			ad.DwellFinished = false
			return 1.0, 0.0, 0.0, false, false, false
		}
		return throttle, brake, steer, handbrake, false, isDocked
	}

	// =========================================================================
	// 2. PATH LOCALIZATION
	// =========================================================================
	ad.ensurePath(world, prevStop, targetStop, laneWidth)
	s, lateralErr := ad.trackPath(bus.Pos)
	total := 0.0
	if n := len(ad.cumLen); n > 0 {
		total = ad.cumLen[n-1]
	}
	remaining := math.Max(0, total-s)

	// =========================================================================
	// 3. DOCKING TRIGGER (Only when arriving, never while departing)
	// =========================================================================
	alignedNearBay := distToStop < 45.0 && math.Abs(NormalizeAngle(bus.Heading-targetStop.Heading)) < 0.45
	// Fallback: the bus has come to rest at the very end of its path right at the stop
	restingAtStop := remaining < 10.0 && distToStop < 70.0
	if ad.State != AutopilotStateDeparting && speedAbs < 8.0 &&
		(targetStop.ContainsBus(bus) || alignedNearBay || restingAtStop) {
		ad.State = AutopilotStateDocked
		ad.DockedStopID = targetStop.ID
		ad.DwellTimer = ad.DwellTotal
		ad.DwellFinished = false
		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			toggleDoors = true
		}
		return 0.0, 1.0, 0.0, false, toggleDoors, true
	}

	// If doors are open outside docked state (e.g. player parked then toggled autopilot),
	// command doors to close immediately so vehicle can start moving!
	if bus.DoorsOpen || bus.DoorProgress > 0.05 {
		closeCmd := false
		if bus.DoorsOpen && speedAbs < 5.0 {
			closeCmd = true
		}
		return 0.0, 1.0, 0.0, false, closeCmd, false
	}

	// Transition between Depart, Cruise, and Approach
	if remaining < apApproachDist {
		ad.State = AutopilotStateApproaching
	} else if ad.State == AutopilotStateDeparting {
		if bus.Pos.Distance(prevStop.Pos) > 120.0 {
			ad.State = AutopilotStateCruising
		}
	} else {
		ad.State = AutopilotStateCruising
	}

	// =========================================================================
	// 4. PURE PURSUIT STEERING (lookahead measured from the bus's own path position)
	// =========================================================================
	nearRoundabout := false
	if world != nil {
		for _, rb := range world.Roundabouts {
			if bus.Pos.Distance(rb.Center) <= rb.Radius+80.0 {
				nearRoundabout = true
				break
			}
		}
	}

	lookahead := Clamp(speedAbs*0.55+45.0, 55.0, 140.0)
	if nearRoundabout {
		lookahead = Clamp(speedAbs*0.35+30.0, 35.0, 52.0)
	}
	targetPt := ad.pointAt(s + lookahead)
	toTarget := targetPt.Sub(bus.Pos)
	alpha := NormalizeAngle(math.Atan2(toTarget.Y, toTarget.X) - bus.Heading)
	// A target behind the bus only happens when badly off-path: turn at full lock, never reverse
	alpha = Clamp(alpha, -math.Pi/2, math.Pi/2)
	ld := math.Max(toTarget.Length(), 1.0)
	delta := math.Atan2(2.0*bus.WheelBase*math.Sin(alpha), ld)
	steer = Clamp(delta/bus.MaxSteerAngle, -1.0, 1.0)

	// =========================================================================
	// 5. SPEED & BRAKING CONTROLLER
	// =========================================================================
	targetSpd := apCruiseSpeed

	// A. Anticipate corners / roundabout arcs ahead, and slow while circulating
	targetSpd = math.Min(targetSpd, ad.cornerSpeedLimit(s, speedAbs))
	if nearRoundabout {
		targetSpd = math.Min(targetSpd, apRoundaboutSpeed)
	}

	// B. Large heading error or lateral deviation: slow down to recover the lane
	if math.Abs(alpha) > 0.6 || lateralErr > 50.0 {
		targetSpd = math.Min(targetSpd, 55.0)
	}

	// C. Station approach: brake along a constant-deceleration profile to stop at the bay center
	stopSpd := math.Sqrt(2 * apStopDecel * math.Max(0, remaining-2.0))
	if remaining > 6.0 {
		stopSpd = math.Max(stopSpd, 14.0)
	} else {
		stopSpd = 0
	}
	targetSpd = math.Min(targetSpd, stopSpd)

	// D. Red / Yellow Light Stopping
	if tls != nil {
		isUpcomingLeft := false
		isUpcomingRight := false
		for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
			d := ad.cumLen[k] - s
			if d < -10.0 {
				continue
			}
			if d > 160.0 {
				break
			}
			a := ad.drivePath[k].Sub(ad.drivePath[k-1])
			b := ad.drivePath[k+1].Sub(ad.drivePath[k])
			turnAngle := NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
			if turnAngle > 0.35 && math.Abs(turnAngle) < 2.8 {
				isUpcomingRight = true
			} else if turnAngle < -0.35 || math.Abs(turnAngle) >= 2.8 {
				isUpcomingLeft = true
			}
		}

		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		bumperPos := bus.Pos.Add(Vec2{X: cosH * (bus.Length * 0.5), Y: sinH * (bus.Length * 0.5)})
		shouldStop, distToLine := tls.CheckStopForVehicle(bumperPos, bus.Heading, 130.0, laneWidth, isUpcomingLeft, isUpcomingRight)
		if shouldStop {
			if distToLine <= 14.0 {
				targetSpd = 0.0
			} else {
				targetSpd = math.Min(targetSpd, math.Max(0.0, (distToLine-12.0)*1.8))
			}
		}
	}

	// E. Forward Vehicle Obstacle Avoidance
	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)
	for _, obs := range obstacles {
		// 1. If bus is near a roundabout, vehicles that are stopped well outside
		// the roundabout curb and away from the bus's path should not trigger false braking
		if world != nil {
			isSafelyWaitingOutside := false
			for _, rb := range world.Roundabouts {
				if bus.Pos.Distance(rb.Center) < rb.Radius+10.0 {
					// Obstacle is truly outside the circular roadway, essentially stopped, and safely distanced from driving path
					if obs.Pos.Distance(rb.Center) > rb.Radius+15.0 && obs.Speed < 5.0 {
						if len(ad.drivePath) >= 2 {
							dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
							if dPath > (bus.Width/2 + obs.Width/2 + 6.0) {
								isSafelyWaitingOutside = true
								break
							}
						} else if obs.Pos.Distance(bus.Pos) > (bus.Length/2 + obs.Length/2 + 25.0) {
							isSafelyWaitingOutside = true
							break
						}
					}
				}
			}
			if isSafelyWaitingOutside {
				continue
			}
		}

		delta := obs.Pos.Sub(bus.Pos)
		fwdDist := delta.X*cosH + delta.Y*sinH
		latDist := math.Abs(-delta.X*sinH + delta.Y*cosH)

		// Immediate physical proximity / collision containment:
		// If obstacle directly overlaps with or is right against the bus front envelope:
		directDist := obs.Pos.Distance(bus.Pos)
		collisionDist := bus.Length/2 + obs.Length/2 + 8.0
		if directDist < collisionDist && fwdDist > -10.0 && latDist < (bus.Width/2+obs.Width/2+8.0) {
			targetSpd = 0.0
			continue
		}

		if fwdDist > 2.0 && fwdDist < 140.0 && latDist < (bus.Width/2+obs.Width/2+12.0) {
			// For vehicles where we have a valid tracked path: verify obstacle is within our path corridor
			if len(ad.drivePath) >= 2 && lateralErr <= 40.0 {
				dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
				if dPath > (bus.Width/2 + obs.Width/2 + 10.0) {
					continue
				}
			}

			// For oncoming vehicles (facing opposite direction), ignore if safely separated laterally
			cosHeadDiff := math.Cos(obs.Heading - bus.Heading)
			if cosHeadDiff < -0.2 && latDist > (bus.Width/2+obs.Width/2+2.0) {
				continue
			}

			safeStopDist := bus.Length/2 + obs.Length/2 + 22.0
			if fwdDist < safeStopDist {
				targetSpd = 0.0
			} else {
				leadSpd := math.Max(0.0, obs.Speed)
				targetSpd = math.Min(targetSpd, leadSpd+(fwdDist-safeStopDist)*2.2)
			}
		}
	}

	// F. Acceleration & Foot Braking
	switch {
	case targetSpd <= 0.5:
		throttle = 0.0
		if speedAbs < 10.0 {
			brake = 1.0
		} else {
			brake = Clamp(speedAbs/60.0, 0.5, 1.0)
		}
	case bus.Speed < targetSpd-3.0:
		throttle = Clamp((targetSpd-bus.Speed)/35.0, 0.3, 1.0)
		brake = 0.0
	case bus.Speed > targetSpd+4.0:
		throttle = 0.0
		brake = Clamp((bus.Speed-targetSpd)/40.0, 0.08, 1.0)
	case bus.Speed < targetSpd:
		throttle = 0.15
		brake = 0.0
	default:
		throttle = 0.0 // coast (rolling resistance) inside the hold band
		brake = 0.0
	}

	ad.LastPathS = s
	ad.LastRemaining = remaining
	ad.LastLateralErr = lateralErr
	ad.LastPursuitPt = targetPt
	ad.LastTargetSpeed = targetSpd

	return throttle, brake, steer, false, false, false
}
