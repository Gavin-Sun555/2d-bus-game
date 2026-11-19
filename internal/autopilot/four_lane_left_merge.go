package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Ordinary 4/6/8-lane roads (no bus lane) turn left from the inner lane.
// Normal-mode (fleet) buses publish that merge: silently pursuing the inner
// path from the curb never asks the civilian queue to leave a rear slot.
// On every ordinary road, a bus that served a stop just before the junction
// commits to the curb turn instead when traffic leaves no room to cross.
func (ad *AutopilotDriver) requestFourLaneLeftMerge(f autopilotFrame) {
	if ad.busLaneJunction != nil || ad.approachMerge != nil || f.world == nil || f.tls == nil || len(ad.drivePath) < 2 || ad.midLaneRecovery != nil || (ad.uTurn != nil && ad.uTurn.entered) {
		return
	}
	heading := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .42 {
		return
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if lanes := road.LaneCount(); !ok || (lanes != 4 && lanes != 6 && lanes != 8) || road.HasBusLane() {
		return
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if f.bus.Pos.Sub(origin).Dot(r) < f.laneWidth*.5+4 {
		return
	}
	approach := ad.intersectionApproachWithRunway(f, true, false, true)
	if approach.approachingInter == nil || !approach.isUpcomingLeft || approach.distToStopLine < f.bus.Length/2 || f.world.IsAtRoundabout(approach.approachingInter.Center) {
		return
	}
	junction := approach.approachingInter.Center
	// After a stop just before the junction, slow traffic in the lanes to be
	// crossed may leave no room to reach the inner lane. Commit to the curb
	// left turn (protected arrow only) while still aligned in the curb lane,
	// instead of wedging across the queue.
	curbLine := origin.Add(r.Mul((float64(road.LaneCount()/2) - .5) * f.laneWidth))
	if !leftMergeRunwayClear(f, origin, u, r) && protectedLeftArrow(f.tls, junction, heading) && ad.departedStopBeforeJunction(f, junction, heading) &&
		wholeBusInMergeLane(f.bus, &sim.BusLaneMergeRequest{Target: curbLine, Heading: heading}, f.laneWidth) &&
		ad.joinCurbLeftTurn(f, junction, u, f.bus.Pos.Sub(junction).Dot(r), true) {
		return
	}
	// Berserk buses manage wider-road approaches with their own required
	// merge. A normal-mode (fleet) bus must request it like on four lanes:
	// silently following the taper lets an articulated bus put its front in
	// the inner lane beside a car, which its trailer can then never pass.
	if road.LaneCount() != 4 && ad.BerserkMode {
		return
	}
	ad.approachMerge = &sim.BusLaneMergeRequest{Target: origin.Add(r.Mul(f.laneWidth * .5)), Heading: heading, LeftTurn: true, Junction: junction, Remaining: .3}
	ad.approachMergeJoined = false
}

// leftMergeRunwayClear reports whether slow traffic ahead in the lanes between
// the bus and the inner lane leaves room for the whole move at a realistic
// bus merge (5:1 forward to sideways, plus a body length to straighten).
func leftMergeRunwayClear(f autopilotFrame, origin, forward, right sim.Vec2) bool {
	lateral := f.bus.Pos.Sub(origin).Dot(right)
	current := int(math.Floor(lateral / f.laneWidth))
	if current <= 1 {
		return true
	}
	needed := 5*math.Max(0, lateral+f.bus.Width/2-f.laneWidth) + f.bus.OverallLength()
	for _, ob := range f.obstacles {
		if math.Abs(ob.Speed) > 20 || math.Cos(ob.Heading-math.Atan2(forward.Y, forward.X)) < .9 {
			continue
		}
		along := ob.Pos.Sub(f.bus.Pos).Dot(forward)
		if along <= 0 || along-f.bus.Length/2-ob.Length/2 > needed {
			continue
		}
		if lane := int(math.Floor(ob.Pos.Sub(origin).Dot(right) / f.laneWidth)); lane >= 1 && lane < current {
			return false
		}
	}
	return true
}

// On an ordinary road (no bus lane) a left-turning bus uses the inner
// left-turn lane. After serving a stop just before the junction, heavy traffic
// can make that merge impossible; only then may a slow bus turn left from the
// curb lane, and only at a junction with a protected left arrow (the arrow
// phase holds the through traffic it would cross). Normal signal, junction
// admission and swept-body checks still apply.
func (ad *AutopilotDriver) tryBlockedCurbLeft(f autopilotFrame, request *sim.BusLaneMergeRequest, dt float64) bool {
	if !request.LeftTurn || ad.busLaneJunction != nil || f.world == nil || f.tls == nil || f.speedAbs > 25 ||
		(ad.uTurn != nil && ad.uTurn.center.Distance(request.Junction) < 1) ||
		math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) > .2 {
		ad.blockedLeftMerge = nil
		return false
	}
	u := sim.UnitFromAngle(request.Heading)
	coord, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(coord, ew, f.bus.Pos)
	lanes := road.LaneCount()
	if !ok || (lanes != 4 && lanes != 6 && lanes != 8) || road.HasBusLane() || f.world.IsAtRoundabout(request.Junction) ||
		!protectedLeftArrow(f.tls, request.Junction, request.Heading) || !ad.departedStopBeforeJunction(f, request.Junction, request.Heading) {
		return false
	}
	// Wider roads queue further back from the line before the merge stalls.
	gap := approachMergeStopDistance(f, request)
	if gap < 0 || gap > math.Max(2.5*f.bus.OverallLength(), 600) {
		ad.blockedLeftMerge = nil
		return false
	}
	if ad.blockedLeftMerge != request {
		ad.blockedLeftMerge = request
		ad.blockedLeftMergeTime = 0
	}
	ad.blockedLeftMergeTime += dt
	if ad.blockedLeftMergeTime < .75 {
		return false
	}
	center := request.Junction
	g := f.world.JunctionAt(center, f.laneWidth)
	stop := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY + sim.JunctionStopInset
	turn, known := ad.intersectionExitTurn(center, u.X, u.Y, stop, 1.6*f.laneWidth)
	_, uTurn := ad.uTurnMovement(center, u, stop)
	if !known || turn != routeTurnLeft || uTurn {
		return false
	}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	curb := (float64(lanes)/2 - .5) * f.laneWidth
	curbLine := center.Add(r.Mul(curb))
	if wholeBusInMergeLane(f.bus, &sim.BusLaneMergeRequest{Target: curbLine, Heading: request.Heading}, f.laneWidth) {
		// At the line there may be no room for a taper. The whole body
		// already fits this lane, so follow its settled entry position.
		curb = f.bus.Pos.Sub(center).Dot(r)
	} else {
		// A bus stopped midway into a merge must have a safe gap back to
		// the curb before committing to a new approach.
		if f.bus.Pos.Sub(center).Dot(r) < f.laneWidth || !requiredMergeLaneClear(f, request.Heading, 0, curbLine.Sub(f.bus.Pos).Dot(r)) {
			return false
		}
	}
	return ad.joinCurbLeftTurn(f, center, u, curb, true)
}

// protectedLeftArrow reports whether the junction gives this approach its own
// left arrow. Minor junctions share one light for through and left movements.
func protectedLeftArrow(tls *sim.TrafficLightSystem, center sim.Vec2, heading float64) bool {
	for _, inter := range tls.Intersections {
		if inter.Center.Distance(center) < 1 {
			return inter.IsMajor && inter.ApproachHasDedicatedLanesForHeading(heading)
		}
	}
	return false
}

// departedStopBeforeJunction reports whether this leg started at a stop on the
// same approach road, before the junction, with no other junction in between.
func (ad *AutopilotDriver) departedStopBeforeJunction(f autopilotFrame, junction sim.Vec2, heading float64) bool {
	from := ad.fromStop(f.world)
	if from == nil || math.Cos(from.Heading-heading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if math.Abs(junction.Sub(from.RoadCenter).Dot(r)) > 1 {
		return false
	}
	start, end := from.RoadCenter.Dot(u), junction.Dot(u)
	if end <= start {
		return false
	}
	for _, p := range f.world.Intersections {
		if along := p.Dot(u); math.Abs(p.Sub(junction).Dot(r)) < 1 && along > start && along < end-1 {
			return false
		}
	}
	return true
}
