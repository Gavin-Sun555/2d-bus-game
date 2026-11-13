package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Upgrade queued legacy curb approaches without resetting the vehicle or its
// journey. A body already crossing a divider or occupying the box keeps its
// current maneuver until it can finish safely.
func (ad *AutopilotDriver) upgradeWideCurbPath(bus *sim.Bus, w *sim.World, lw float64) {
	if ad.pathRevision >= currentDrivePathRevision || len(ad.drivePath) < 2 || ad.stationDock != nil || ad.uTurn != nil || ad.busJunctionEntered {
		return
	}
	// Rebuilding the whole leg re-localizes onto a fresh route whose nearest
	// point may be a turn. Only a queued bus can adopt it safely; a moving
	// bus keeps its legacy path until the next leg is prepared.
	if math.Abs(bus.Speed) > 5 {
		return
	}
	h := approachRoadHeading(bus.Heading)
	center, ew, _, _ := sim.CurrentRoadInfo(w, bus.Pos, h, lw)
	road, ok := w.RoadAt(center, ew, bus.Pos)
	if !ok || !road.AllowsCurbBusLeft() {
		return
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	r := sim.Vec2{X: -math.Sin(h), Y: math.Cos(h)}
	request := &sim.BusLaneMergeRequest{Heading: h, Target: origin.Add(r.Mul((float64(road.LaneCount()/2) - .5) * lw))}
	if !wholeBusInMergeLane(bus, request, lw) || sidecarBodyInJunction(autopilotFrame{bus: bus, world: w, laneWidth: lw}) {
		return
	}
	if ad.approachMerge != nil && !ad.approachMerge.LeftTurn {
		return
	}
	// The rebuilt leg may take a different route (for example one avoiding
	// a U-turn) that never passes the bus. Adopt it only if the bus already
	// lies on it in its direction of travel; otherwise keep the legacy path
	// for this leg instead of stranding the bus on an unrelated route.
	if ad.pathFromID < 0 || ad.pathFromID >= len(w.Stops) || ad.pathToID < 0 || ad.pathToID >= len(w.Stops) ||
		!pathPassesPose(buildAutopilotDrivePath(w, w.Stops[ad.pathFromID], w.Stops[ad.pathToID], lw, false, ad.CanSkipTargetStop,
			sim.RoadPathOptions{LongVehicle: ad.articulatedPath, CurbBusLaneRightAt: ad.curbRightPathChoice()}), bus.Pos, bus.Heading, 1.5*lw) {
		ad.pathRevision = currentDrivePathRevision
		return
	}
	ad.drivePath = nil
	ad.ordinaryBusLeft = false
	ad.approachMerge, ad.midLaneRecovery = nil, nil
	ad.approachMergeJoined = false
	ad.busLaneJunction = nil
	ad.busLaneTurnChosen = false
	ad.busTurnHoldDistance = 0
	ad.holdLaneOffset(0)
}

// A bus already using the curb on a median-BRT boulevard may also turn left
// there. Replace only the upcoming turn, preserving the protected median path
// elsewhere on the leg and the existing trip/passenger state.
func (ad *AutopilotDriver) keepMedianRoadCurbLeft(f autopilotFrame) {
	if f.world == nil || f.tls == nil || ad.busLaneTurnChosen || ad.busJunctionEntered || len(ad.drivePath) < 2 {
		return
	}
	h := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(h)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	coord, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, h, f.laneWidth)
	road, ok := f.world.RoadAt(coord, ew, f.bus.Pos)
	if !ok || !road.HasMedianBusLane() {
		return
	}
	origin := sim.Vec2{Y: coord}
	if !ew {
		origin = sim.Vec2{X: coord}
	}
	curb := 3.5 * f.laneWidth
	if !wholeBusInMergeLane(f.bus, &sim.BusLaneMergeRequest{Target: origin.Add(r.Mul(curb)), Heading: h}, f.laneWidth) {
		return
	}
	if sidecarBodyInJunction(f) {
		return
	}
	a := ad.approachingIntersection(f, true, true)
	if a.approachingInter == nil || !a.isUpcomingLeft {
		return
	}
	ad.joinCurbLeftTurn(f, a.approachingInter.Center, u, curb, false)
}

// Replace the upcoming turn and keep the remaining leg and its service state.
func (ad *AutopilotDriver) joinCurbLeftTurn(f autopilotFrame, center, u sim.Vec2, curb float64, recenter bool) bool {
	g := f.world.JunctionAt(center, f.laneWidth)
	out := sim.Vec2{X: u.Y, Y: -u.X}
	rOut := sim.Vec2{X: -out.Y, Y: out.X}
	half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
	exit := -1
	for i := max(1, ad.WaypointIdx); i < len(ad.drivePath); i++ {
		if ad.drivePath[i].Sub(center).Dot(out) >= half+80 && ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize().Dot(out) > .98 {
			exit = i
			break
		}
	}
	if exit < 0 {
		return false
	}
	outOffset := ad.drivePath[exit].Sub(center).Dot(rOut)
	var curve []sim.Vec2
	if recenter {
		coord, ew := center.Y, math.Abs(out.X) > .5
		if !ew {
			coord = center.X
		}
		road, _ := f.world.RoadAt(coord, ew, center.Add(out.Mul(half+80)))
		lanes := road.LaneCount()
		if road.HasMedianBusLane() {
			lanes = 0
		}
		incoming := 4
		inCoord, inEW, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, math.Atan2(u.Y, u.X), f.laneWidth)
		if inRoad, ok := f.world.RoadAt(inCoord, inEW, f.bus.Pos); ok {
			incoming = inRoad.LaneCount()
		}
		curve = sim.CurbBusLeftTurnPath(center, u, out, curb, outOffset, g, incoming, lanes)
	} else {
		curve = sim.WideLeftTurnPath(center, u, out, curb, outOffset, g)
	}
	runway := curve[0].Sub(f.bus.Pos).Dot(u)
	if runway < 0 {
		return false
	}
	points := []sim.Vec2{f.bus.Pos}
	right := sim.Vec2{X: -u.Y, Y: u.X}
	shift := curve[0].Sub(f.bus.Pos).Dot(right)
	if recenter && math.Abs(shift) > 1 {
		// Leave a straight section at the stop line after recentering.
		taper := math.Min(runway-f.bus.Length/2, math.Max(100, 9*math.Abs(shift)))
		if taper < math.Max(70, 5*math.Abs(shift)) {
			return false
		}
		for i := 1; i <= 8; i++ {
			t := float64(i) / 8
			points = append(points, f.bus.Pos.Add(u.Mul(taper*t)).Add(right.Mul(shift*t*t*(3-2*t))))
		}
	}
	points = append(points, curve...)
	points = append(points, ad.drivePath[exit+1:]...)
	ad.drivePath = points
	ad.cumLen = make([]float64, len(points))
	for i := 1; i < len(points); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + points[i].Distance(points[i-1])
	}
	ad.WaypointIdx = 0
	ad.pathSynced = false
	ad.holdLaneOffset(0)
	ad.approachMerge = nil
	ad.approachMergeJoined = false
	ad.busLaneJunction = &center
	ad.busLaneTurnChosen = true
	ad.busTurnHoldDistance = 0
	ad.dockDepartureRun = 0
	ad.OvertakeState = OvertakeNone
	f.bus.LaneMergeRequest = nil
	if c := f.bus.JunctionCorridor; c != nil && !c.Entered && c.Center.Distance(center) < 1 {
		f.bus.JunctionCorridor = nil
	}
	return true
}

// pathPassesPose reports whether some segment of path runs within tolerance
// of pos while pointing along heading.
func pathPassesPose(path []sim.Vec2, pos sim.Vec2, heading, tolerance float64) bool {
	u := sim.UnitFromAngle(heading)
	for i := 1; i < len(path); i++ {
		a, b := path[i-1], path[i]
		if b.Sub(a).Length() < 1e-6 || b.Sub(a).Normalize().Dot(u) < .9 {
			continue
		}
		if _, distance := projectToSegment(pos, a, b); distance <= tolerance {
			return true
		}
	}
	return false
}
