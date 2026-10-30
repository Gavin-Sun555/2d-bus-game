package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

const currentDrivePathRevision = 4

// DriverSnapshot stores controller progress without serializing world pointers.
type DriverSnapshot struct {
	DepartureEscapeActive  bool                     `json:",omitempty"`
	SidecarEscapeActive    bool                     `json:",omitempty"`
	RoundaboutEdgeRecovery *RoundaboutEdgeRecovery  `json:",omitempty"`
	PathReturnHold         *sim.BusLaneMergeRequest `json:",omitempty"`
	DockDepartureRun       float64
	SmoothDockDeparture    bool
	ArticulatedPath        bool
	Public                 AutopilotDriver
	QueueStopID            *int
	Turn                   *TurnSnapshot
	Dock                   *DockSnapshot
	SkippedStop            int
	SkippedPosition        sim.Vec2
	SkippedServed          bool
	ParallelEntryCenter    *sim.Vec2
	ApproachMerge          *sim.BusLaneMergeRequest
	ApproachMergeJoined    bool
	RoadSafetyHold         bool
	RoadRecoverySpeed      float64
	MidLaneRecovery        *sim.BusLaneMergeRequest
	MidLaneSweepClear      bool
	LastTargetLaneOffset   float64
	DrivePath              []sim.Vec2
	PathRevision           int
	PathFromID             int
	PathToID               int
	PathLane               float64
	PathSynced             bool
	DepartureFromDock      bool
	PathSkipTarget         bool
	BypassStopID           int
	DirectRoundaboutExit   *sim.Vec2
	BusLaneJunction        *sim.Vec2
	RightTurnJunction      *sim.Vec2
	RightTurnHeading       float64
	RightTurnWaiting       bool
	BusJunctionEntered     bool
	OrdinaryBusLeft        bool
	PathOrdinaryBusLeft    bool
	BusTurnHoldDistance    float64
}
type TurnSnapshot struct {
	Center, Forward sim.Vec2
	Entered         bool
}
type DockSnapshot struct {
	AlignmentRecovery bool `json:",omitempty"`
	Waiting           bool
	StopID, Rank      int
	Target            sim.Vec2
	Points            []sim.Vec2
}

func (ad *AutopilotDriver) Snapshot() DriverSnapshot {
	s := DriverSnapshot{Public: *ad, SkippedStop: -1, ArticulatedPath: ad.articulatedPath, DockDepartureRun: ad.dockDepartureRun, SmoothDockDeparture: ad.smoothDockDeparture}
	s.SidecarEscapeActive = ad.sidecarEscapeActive
	s.DepartureEscapeActive = ad.departureEscapeActive
	queue := ad.queueStopID
	s.QueueStopID = &queue
	s.ParallelEntryCenter = ad.parallelEntryCenter
	if ad.roundaboutEdgeRecovery != nil {
		copy := *ad.roundaboutEdgeRecovery
		s.RoundaboutEdgeRecovery = &copy
	}
	s.PathReturnHold = ad.pathReturnHold
	s.ApproachMerge = ad.approachMerge
	s.ApproachMergeJoined = ad.approachMergeJoined
	s.RoadSafetyHold = ad.roadSafetyHold
	s.RoadRecoverySpeed = ad.roadRecoverySpeed
	s.MidLaneRecovery = ad.midLaneRecovery
	s.MidLaneSweepClear = ad.midLaneSweepClear
	s.LastTargetLaneOffset = ad.lastTargetLaneOffset
	s.DrivePath = ad.drivePath
	s.PathRevision = ad.pathRevision
	s.PathFromID = ad.pathFromID
	s.PathToID = ad.pathToID
	s.PathLane = ad.pathLane
	s.PathSynced = ad.pathSynced
	s.DepartureFromDock = ad.departureFromDock
	s.PathSkipTarget = ad.pathSkipTarget
	s.BypassStopID = ad.bypassStopID
	s.DirectRoundaboutExit = ad.directRoundaboutExit
	s.BusLaneJunction = ad.busLaneJunction
	s.RightTurnJunction = ad.rightTurnJunction
	s.RightTurnHeading = ad.rightTurnHeading
	s.RightTurnWaiting = ad.rightTurnWaiting
	s.BusJunctionEntered = ad.busJunctionEntered
	s.OrdinaryBusLeft = ad.ordinaryBusLeft
	s.PathOrdinaryBusLeft = ad.pathOrdinaryBusLeft
	s.BusTurnHoldDistance = ad.busTurnHoldDistance

	if ad.uTurn != nil {
		s.Turn = &TurnSnapshot{ad.uTurn.center, ad.uTurn.forward, ad.uTurn.entered}
	}
	if ad.stationDock != nil {
		d := ad.stationDock
		s.Dock = &DockSnapshot{StopID: d.stopID, Rank: d.rank, Target: d.target, Points: d.points, Waiting: d.waiting, AlignmentRecovery: d.alignmentRecovery}
	}
	if ad.skippedDeparture != nil {
		s.SkippedStop = ad.skippedDeparture.stop.ID
		s.SkippedPosition = ad.skippedDeparture.position
		s.SkippedServed = ad.skippedDeparture.served
	}
	return s
}

func (s DriverSnapshot) Restore(w *sim.World) (*AutopilotDriver, error) {
	validStop := func(id int) bool { return id >= 0 && id < len(w.Stops) }
	if s.Public.State < AutopilotStateCruising || s.Public.State > AutopilotStateDeparting || s.Public.DockedStopID < -1 || (s.Public.DockedStopID >= 0 && !validStop(s.Public.DockedStopID)) ||
		s.Public.WaypointIdx < 0 || (len(s.DrivePath) > 0 && s.Public.WaypointIdx >= len(s.DrivePath)) ||
		(s.Dock != nil && !validStop(s.Dock.StopID)) || s.SkippedStop < -1 || (s.SkippedStop >= 0 && !validStop(s.SkippedStop)) ||
		(len(s.DrivePath) > 0 && (!validStop(s.PathFromID) || !validStop(s.PathToID))) {
		return nil, fmt.Errorf("invalid saved autopilot progress")
	}
	ad := s.Public
	ad.sidecarEscapeActive = s.SidecarEscapeActive
	ad.departureEscapeActive = s.DepartureEscapeActive
	ad.articulatedPath = s.ArticulatedPath
	ad.dockDepartureRun = s.DockDepartureRun
	ad.smoothDockDeparture = s.SmoothDockDeparture
	ad.queueStopID = -1
	if s.QueueStopID != nil {
		if *s.QueueStopID < -1 || (*s.QueueStopID >= 0 && !validStop(*s.QueueStopID)) {
			return nil, fmt.Errorf("invalid saved station queue")
		}
		ad.queueStopID = *s.QueueStopID
	}
	ad.parallelEntryCenter = s.ParallelEntryCenter
	ad.approachMerge = s.ApproachMerge
	ad.approachMergeJoined = s.ApproachMergeJoined
	ad.roadSafetyHold = s.RoadSafetyHold
	ad.roadRecoverySpeed = s.RoadRecoverySpeed
	ad.midLaneRecovery = s.MidLaneRecovery
	ad.midLaneSweepClear = s.MidLaneSweepClear
	ad.lastTargetLaneOffset = s.LastTargetLaneOffset
	if r := s.RoundaboutEdgeRecovery; r != nil {
		if math.IsNaN(r.Start.X) || math.IsNaN(r.Start.Y) || math.IsInf(r.Start.X, 0) || math.IsInf(r.Start.Y, 0) || r.Distance < 6 || r.Distance > 10 || math.IsNaN(r.Distance) || math.Abs(r.ReverseSteer) != 1 {
			return nil, fmt.Errorf("invalid saved roundabout adjustment")
		}
		copy := *r
		ad.roundaboutEdgeRecovery = &copy
	}
	ad.pathReturnHold = s.PathReturnHold
	ad.drivePath = s.DrivePath
	ad.pathRevision = s.PathRevision
	ad.pathFromID = s.PathFromID
	ad.pathToID = s.PathToID
	ad.pathLane = s.PathLane
	ad.pathSynced = s.PathSynced
	ad.departureFromDock = s.DepartureFromDock
	ad.pathSkipTarget = s.PathSkipTarget
	ad.bypassStopID = s.BypassStopID
	ad.directRoundaboutExit = s.DirectRoundaboutExit
	ad.busLaneJunction = s.BusLaneJunction
	ad.rightTurnJunction = s.RightTurnJunction
	ad.rightTurnHeading = s.RightTurnHeading
	ad.rightTurnWaiting = s.RightTurnWaiting
	ad.busJunctionEntered = s.BusJunctionEntered
	ad.ordinaryBusLeft = s.OrdinaryBusLeft
	ad.pathOrdinaryBusLeft = s.PathOrdinaryBusLeft
	ad.busTurnHoldDistance = s.BusTurnHoldDistance

	ad.pathWorld = w
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	if s.Turn != nil {
		ad.uTurn = &uTurnManeuver{s.Turn.Center, s.Turn.Forward, s.Turn.Entered}
	}
	if s.Dock != nil {
		d := s.Dock
		ad.stationDock = &stationDockPlan{stopID: d.StopID, rank: d.Rank, target: d.Target, points: d.Points, waiting: d.Waiting, alignmentRecovery: d.AlignmentRecovery}
	}
	if s.SkippedStop >= 0 {
		ad.skippedDeparture = &skippedStopDeparture{stop: w.Stops[s.SkippedStop], position: s.SkippedPosition, served: s.SkippedServed}
	}
	ad.refreshUnenteredSavedPath(w)
	return &ad, nil
}

// Upgrade a legacy cached route only while its changed portion is still ahead.
// Preserve an in-progress turn/dock rather than moving its pursuit target across
// traffic on load; the next leg then obtains the new geometry normally.
func (ad *AutopilotDriver) refreshUnenteredSavedPath(w *sim.World) {
	if len(ad.drivePath) < 2 || !ad.pathSynced || ad.stationDock != nil || ad.skippedDeparture != nil || ad.directRoundaboutExit != nil || ad.approachMerge != nil || ad.midLaneRecovery != nil {
		return
	}
	if ad.articulatedPath && !ad.pathSkipTarget {
		to := w.Stops[ad.pathToID]
		if path, prefix, changed := joinFoldedArrival(w, ad.drivePath, to.FrontBerthPosition(), to.Heading, ad.pathLane); changed && ad.LastPathS+64 < prefix {
			// Preserve customized departure/turn prefixes from the saved leg.
			// Only replace the folded arrival while it is still safely ahead.
			ad.drivePath, ad.pathRevision, ad.pathSynced = path, currentDrivePathRevision, false
			ad.WaypointIdx = 0
			ad.cumLen = make([]float64, len(path))
			for i := 1; i < len(path); i++ {
				ad.cumLen[i] = ad.cumLen[i-1] + path[i].Distance(path[i-1])
			}
			return
		}
	}
	if ad.pathRevision == currentDrivePathRevision {
		return
	}
	path := buildAutopilotDrivePath(w, w.Stops[ad.pathFromID], w.Stops[ad.pathToID], ad.pathLane, false, ad.pathSkipTarget, sim.RoadPathOptions{OrdinaryBusLaneLeft: ad.ordinaryBusLeft, LongVehicle: ad.articulatedPath})
	common, i := 0., 0
	for i < len(path) && i < len(ad.drivePath) && path[i].Distance(ad.drivePath[i]) < .01 {
		if i > 0 {
			common += ad.drivePath[i].Distance(ad.drivePath[i-1])
		}
		i++
	}
	if i == len(path) && i == len(ad.drivePath) {
		ad.pathRevision = currentDrivePathRevision
		return
	}
	// Different sampling of the same straight approach still shares a prefix.
	if i > 0 && i < len(path) && i < len(ad.drivePath) {
		a, b := path[i].Sub(path[i-1]), ad.drivePath[i].Sub(ad.drivePath[i-1])
		if a.Normalize().Dot(b.Normalize()) > .999999 {
			common += math.Min(a.Length(), b.Length())
		}
	}
	// Earlier long-bus paths could extend the bay straight beyond the
	// following merge point. A bus already at that fold cannot reach a target
	// behind its nose. Repair only this unentered departure prefix; normal
	// in-progress turns retain their saved path.
	repairDeparture := len(ad.drivePath) >= 3 && ad.busJunctionEntered == false && ad.uTurn == nil && ad.LastPathS < ad.cumLen[2]+96 && departureFold(ad.drivePath, w.Stops[ad.pathFromID].Heading)
	if ad.LastPathS+64 >= common && !repairDeparture {
		return
	}
	ad.drivePath = path
	ad.pathRevision = currentDrivePathRevision
	ad.pathSynced = false
	ad.WaypointIdx = 0
	ad.cumLen = make([]float64, len(path))
	for i := 1; i < len(path); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + path[i].Distance(path[i-1])
	}
}

// A departure must advance along the stop heading before its first merge.
func departureFold(path []sim.Vec2, heading float64) bool {
	if len(path) < 3 {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	return path[1].Sub(path[0]).Dot(forward) > 0 && path[2].Sub(path[1]).Dot(forward) < -1
}
