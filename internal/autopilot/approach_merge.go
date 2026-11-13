package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Route-required merges precede queue balancing and further passing. Keep the
// intent while a gap is blocked, but steer only once the entire adjacent gap fits.
func (ad *AutopilotDriver) updateRequiredApproachMerge(f autopilotFrame, dt float64) bool {
	if f.tls == nil {
		ad.approachMerge = nil
		return false
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			ad.approachMerge = nil
			return false
		}
	}
	if ad.approachMerge != nil {
		return ad.advanceApproachMerge(f, dt)
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	approach := ad.intersectionApproachWithRunway(f, multi, six, true)
	if approach.approachingInter == nil || approach.distToStopLine <= -f.bus.Length/2 {
		ad.approachMerge = nil
		return false
	}
	if f.world != nil && f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		ad.approachMerge = nil
		return false
	}

	lanes := f.world.GetRoadLanes(center, ew, f.bus.Pos)
	is2Lane := !multi && !six || lanes <= 2
	rightTurn := approach.isUpcomingRight
	if road, ok := f.world.RoadAt(center, ew, f.bus.Pos); ok && road.HasMedianBusLane() && rightTurn && !ad.usesCurbRightAt(approach.approachingInter.Center) {
		return false
	}

	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lateral := f.bus.Pos.Sub(origin).Dot(right)

	twoLaneApproachDist := math.Max(350.0, f.speedAbs*1.5)
	twoLaneMerge := is2Lane && approach.distToStopLine < twoLaneApproachDist && (ad.LaneOffset < -2.0 || lateral < 0.1*f.laneWidth || ad.OvertakeState != OvertakeNone)
	nearMouth := rightTurn && approach.distToStopLine > -f.bus.Length/2 && approach.isApproachingIntersection
	straightMerge := multi && approach.isApproachingIntersection && !approach.isUpcomingLeft && !rightTurn && approach.approachingInter.ApproachHasDedicatedLanesForHeading(f.bus.Heading) && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0)
	if road, ok := f.world.RoadAt(center, ew, f.bus.Pos); ok && road.HasMedianBusLane() {
		straightMerge = false
	}
	if straightMerge {
		straightMerge = ad.hasStraightMergeExit(f, approach.approachingInter)
	}

	if !twoLaneMerge && !nearMouth && !straightMerge {
		ad.approachMerge = nil
		return false
	}

	targetLateral := 1.5 * f.laneWidth
	if is2Lane {
		targetLateral = 0.5 * f.laneWidth
	} else if rightTurn {
		targetLateral = (float64(lanes/2) - .5) * f.laneWidth
	}

	if ad.approachMerge == nil {
		if is2Lane {
			if math.Abs(lateral-targetLateral) < f.laneWidth*0.4 && ad.LaneOffset >= -2.0 && ad.OvertakeState == OvertakeNone {
				return false
			}
		} else if rightTurn {
			if math.Abs(lateral-targetLateral) < f.laneWidth*0.4 {
				return false
			}
		} else {
			if lateral >= f.laneWidth {
				return false
			}
		}
	}

	target := origin.Add(right.Mul(targetLateral))
	// Use an early gap when it is open. A curb queue well before the turn
	// must not become an obstacle in our still-clear current lane; continue
	// alongside it and retry within the normal mandatory merge zone.
	if rightTurn && approach.distToStopLine > requiredApproachDistance(f, lanes) {
		base := ad.pointAt(f.s)
		current, desired := f.bus.Pos.Sub(base).Dot(right), target.Sub(base).Dot(right)
		if !requiredMergeLaneClear(f, heading, current, desired) {
			return false
		}
	}
	if ad.approachMerge == nil {
		ad.approachMerge = &sim.BusLaneMergeRequest{}
		ad.approachMergeJoined = false
	}
	*ad.approachMerge = sim.BusLaneMergeRequest{
		Target:    target,
		Heading:   heading,
		Remaining: .3,
		RightTurn: rightTurn,
		LeftTurn:  approach.isUpcomingLeft,
		Is2Lane:   is2Lane,
		Junction:  approach.approachingInter.Center,
	}
	return ad.advanceApproachMerge(f, dt)
}

// A lane-return lock must not infer straight travel from a point still on a
// wide turning arc. Verify the route actually crosses the far exit parallel
// to the incoming road before retaining that axis through the whole junction.
func (ad *AutopilotDriver) hasStraightMergeExit(f autopilotFrame, inter *sim.TrafficIntersection) bool {
	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	exit := inter.GetStopLineDistForHeading(heading, f.laneWidth) + f.bus.Length/2
	for i := ad.WaypointIdx; i < len(ad.drivePath)-1; i++ {
		a, b := ad.drivePath[i].Sub(inter.Center), ad.drivePath[i+1].Sub(inter.Center)
		fa, fb := a.Dot(forward), b.Dot(forward)
		if fa > exit || fb <= exit || fb <= fa {
			continue
		}
		t := (exit - fa) / (fb - fa)
		lateral := a.Add(b.Sub(a).Mul(t)).Dot(right)
		return b.Sub(a).Normalize().Dot(forward) > .98 && math.Abs(lateral) < inter.ApproachRoadHalfWidth(heading, f.laneWidth)
	}
	return false
}

func (ad *AutopilotDriver) advanceApproachMerge(f autopilotFrame, dt float64) bool {
	request := ad.approachMerge
	ad.chooseStraightMergeLane(f, request)
	heading := request.Heading
	right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	target := request.Target
	// Once the front has joined the left lane inside the box, the turn
	// controller owns the remaining motion. A trailing vehicle may still
	// overlap the merge's reserved space, but must not stop the turn inside.
	if request.LeftTurn && ad.busLaneJunction == nil && f.world != nil {
		u := sim.UnitFromAngle(heading)
		g := f.world.JunctionAt(request.Junction, f.laneWidth)
		half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
		nose := f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(f.bus.Length / 2))
		if nose.Sub(request.Junction).Dot(u) > -half && math.Abs(nose.Sub(target).Dot(right)) < f.laneWidth/2 {
			ad.approachMerge = nil
			ad.approachMergeJoined = false
			f.bus.LaneMergeRequest = nil
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
			return false
		}
	}
	if ad.approachMergeJoined {
		g := f.world.JunctionAt(request.Junction, f.laneWidth)
		half := math.Abs(math.Cos(heading))*g.HalfX + math.Abs(math.Sin(heading))*g.HalfY
		forward := sim.UnitFromAngle(heading)
		if f.bus.Pos.Sub(request.Junction).Dot(forward) > half+f.bus.Length/2+80 {
			ad.approachMerge = nil
			ad.approachMergeJoined = false
			return false
		}
	}
	// Joining the physical curb lane is not enough when an early merge has
	// advanced ahead of the route taper. Keep its fixed road corridor until
	// the base path reaches it; clearing to offset zero would steer back left.
	earlyMedianMerge := false
	if request.RightTurn && f.world != nil && f.world.CenterBRT && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		coord, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
		if road, ok := f.world.RoadAt(coord, ew, f.bus.Pos); ok {
			earlyMedianMerge = road.HasMedianBusLane()
		}
	}
	handoffReady := !earlyMedianMerge || math.Abs(target.Sub(ad.pointAt(f.s)).Dot(right)) < 4
	if request.LeftTurn {
		// The route has reached the turn lane once it is level with it or
		// already beyond it toward the turn. A route turning left out of the
		// target lane itself moves away from the corridor as the turn begins;
		// keeping the corridor then drives the bus straight through the junction.
		handoffReady = target.Sub(ad.pointAt(f.s)).Dot(right) > -4
	}
	if request.RightTurn && f.world != nil {
		// Keep the curb-lane commitment until the turn tracker can take over.
		// An early handoff would be discarded outside the junction envelope,
		// allowing the passing policy to leave the required lane again.
		handoffReady = handoffReady && withinRightTurnTrackingArea(f, request.Junction)
	}
	// The path lock and the physical lane change finish at different times.
	// Keep the curb corridor while the base route still follows the median,
	// but release maneuver speed once the whole vehicle has straightened.
	mergeComplete := approachMergeComplete(f.bus, request, f.laneWidth)
	if (request.RightTurn || request.LeftTurn) && !handoffReady {
		ad.approachMergeJoined = mergeComplete && wholeBusInMergeLane(f.bus, request, f.laneWidth) && (!ad.BerserkMode || mergePoseSettled(f, request))
	}

	if mergeComplete && handoffReady {
		if request.RightTurn {
			center := request.Junction
			ad.rightTurnJunction = &center
			ad.rightTurnHeading = heading
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
			ad.approachMerge = nil
			ad.approachMergeJoined = false
			f.bus.LaneMergeRequest = nil
			return false
		}
		if request.LeftTurn || request.Is2Lane {
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
			ad.approachMerge = nil
			ad.approachMergeJoined = false
			return false
		}
		if straightMergeHasPassingRunway(f, request) {
			ad.approachMerge = nil
			ad.approachMergeJoined = false
			f.bus.LaneMergeRequest = nil
			return false
		}
		ad.approachMergeJoined = true
	}
	request.Joined = ad.approachMergeJoined
	request.Remaining = .3
	f.bus.LaneMergeRequest = request
	if request.RightTurn {
		ad.rightTurnJunction = nil
		ad.rightTurnWaiting = false
	}
	// Do not enter the junction in its left-only lane while the gap is blocked.

	base := ad.pointAt(f.s)
	actual := f.bus.Pos.Sub(base).Dot(right)
	desired := target.Sub(base).Dot(right)
	if ad.approachMergeJoined || requiredMergeLaneClear(f, heading, actual, desired) {
		ad.blockedLeftMerge = nil
		if math.Abs(ad.LaneOffset-actual) > f.laneWidth*2 {
			ad.LaneOffset = desired
			ad.LaneOffsetStart = desired
			ad.lastTargetLaneOffset = desired
		}
		ad.TargetLaneOffset = desired
		ad.OvertakeState = OvertakeReturning
		ad.progressLaneOffset(dt, f.speedAbs)
		if math.Abs(ad.LaneOffset-desired) < 1.0 {
			ad.LaneOffset = desired
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	} else {
		if ad.tryBlockedCurbLeft(f, request, dt) {
			return true
		}
		ad.busTurnHoldDistance = math.Max(.01, approachMergeStopDistance(f, request)-25)
		ad.holdLaneOffset(actual)
		ad.retreatBlockedStraightMerge(f, request, actual, desired, dt)
	}
	return true
}

// Either through lane is legal on an ordinary six-lane approach. Select once
// per request and keep that target while waiting for a safe gap.
func (ad *AutopilotDriver) chooseStraightMergeLane(f autopilotFrame, request *sim.BusLaneMergeRequest) {
	if f.world == nil || ad.straightMergeChoice == request || request.RightTurn || request.LeftTurn || request.Is2Lane || ad.approachMergeJoined || wholeBusInMergeLane(f.bus, request, f.laneWidth) {
		return
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok || road.LaneCount() != 6 || road.HasBusLane() {
		return
	}
	forward := sim.UnitFromAngle(request.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	geometry := f.world.JunctionAt(request.Junction, f.laneWidth)
	half := math.Abs(forward.X)*geometry.HalfX + math.Abs(forward.Y)*geometry.HalfY
	exit := request.Junction.Add(forward.Mul(half + f.bus.OverallLength() + 80))
	if f.world.GetRoadLanes(center, ew, exit) < 6 {
		return
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lane := int(math.Round(request.Target.Sub(origin).Dot(right)/f.laneWidth - .5))
	lane = max(1, min(2, lane))
	best := ad.busLaneTrafficScore(f, forward, right, lane, true)
	for _, candidate := range []int{2, 1} {
		score := ad.busLaneTrafficScore(f, forward, right, candidate, true)
		if score+1 < best && straightMergeCrossingClear(f, origin, forward, right, candidate) {
			best, lane = score, candidate
		}
	}
	request.Target = origin.Add(right.Mul((float64(lane) + .5) * f.laneWidth))
	ad.straightMergeChoice = request
}

// A farther through lane is only useful if the bus can cross the lanes in
// between first. Slow traffic ahead in a lane being crossed, within the
// runway that lateral shift needs, would leave the bus straddling in front of
// that queue.
func straightMergeCrossingClear(f autopilotFrame, origin, forward, right sim.Vec2, to int) bool {
	lateral := f.bus.Pos.Sub(origin).Dot(right)
	current := int(math.Floor(lateral / f.laneWidth))
	// Lanes strictly between the current lane and the candidate, and the
	// lateral distance until the whole body has left them.
	lo, hi := current+1, to
	clearShift := float64(to)*f.laneWidth + f.bus.Width/2 - lateral
	if to < current {
		lo, hi = to+1, current
		clearShift = lateral + f.bus.Width/2 - float64(to+1)*f.laneWidth
	}
	if lo >= hi {
		return true
	}
	// Forward room for that shift at a typical ~17° merge yaw (3.25:1).
	needed := 3.25 * math.Max(0, clearShift)
	for _, ob := range f.obstacles {
		if math.Abs(ob.Speed) > 20 || math.Cos(ob.Heading-math.Atan2(forward.Y, forward.X)) < .9 {
			continue
		}
		along := ob.Pos.Sub(f.bus.Pos).Dot(forward)
		if along <= 0 || along-f.bus.Length/2-ob.Length/2 > needed {
			continue
		}
		if obLane := int(math.Floor(ob.Pos.Sub(origin).Dot(right) / f.laneWidth)); obLane >= lo && obLane < hi {
			return false
		}
	}
	return true
}

// If a required return is blocked alongside, settle back into the lane being
// left. Retain the through-lane request and retry after the checked retreat.
func (ad *AutopilotDriver) retreatBlockedStraightMerge(f autopilotFrame, request *sim.BusLaneMergeRequest, actual, desired, dt float64) {
	if f.world == nil || request.RightTurn || request.LeftTurn || request.Is2Lane || f.speedAbs > 25 || math.Abs(desired-actual) < 4 {
		return
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok || road.LaneCount() != 6 || road.HasBusLane() {
		return
	}
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lateral := f.bus.Pos.Sub(origin).Dot(right)
	lane := math.Floor(lateral/f.laneWidth - .5)
	if desired < actual {
		lane = math.Ceil(lateral/f.laneWidth - .5)
	}
	if lane < 0 || lane >= 3 {
		return
	}
	target := origin.Add(right.Mul((lane + .5) * f.laneWidth))
	offset := target.Sub(ad.pointAt(f.s)).Dot(right)
	if math.Abs(offset-actual) < 4 || !requiredMergeLaneClear(f, request.Heading, actual, offset) {
		return
	}
	ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: target, Heading: request.Heading, Junction: request.Junction, Remaining: .3}
	ad.advanceMidLaneRecovery(f, dt)
}

// Once the bus has cleared the left-only lane, an early straight merge no
// longer needs to reserve the middle lane all the way through the junction.
func straightMergeHasPassingRunway(f autopilotFrame, request *sim.BusLaneMergeRequest) bool {
	if f.world == nil {
		return false
	}
	forward := sim.UnitFromAngle(request.Heading)
	toJunction := request.Junction.Sub(f.bus.Pos).Dot(forward)
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, request.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok || road.LaneCount() != 6 || road.HasBusLane() {
		return false
	}
	junction := f.world.JunctionAt(request.Junction, f.laneWidth)
	half := math.Abs(forward.X)*junction.HalfX + math.Abs(forward.Y)*junction.HalfY
	runway := toJunction - half - sim.JunctionStopInset - f.bus.Length/2
	return runway > math.Max(650, f.speedAbs*2.4+160)
}

func approachMergeComplete(bus *sim.Bus, request *sim.BusLaneMergeRequest, laneWidth float64) bool {
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	error := math.Abs(bus.Pos.Sub(request.Target).Dot(right))
	yaw := math.Abs(sim.NormalizeAngle(bus.Heading - request.Heading))
	if error >= 4 || yaw >= .1 {
		return false
	}
	if bus.IsArticulated() {
		for _, part := range bus.BodyParts() {
			yaw := part.Heading - request.Heading
			half := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
			if math.Abs(part.Center.Sub(request.Target).Dot(right))+half > laneWidth/2-.25 {
				return false
			}
		}
	}
	if request.RightTurn {
		return true
	}
	halfWidth := bus.Length/2*math.Abs(math.Sin(yaw)) + bus.Width/2*math.Abs(math.Cos(yaw))
	return error+halfWidth <= laneWidth/2-.25 && math.Abs(bus.SteeringAngle) < .08
}

// A confirmed yield permits a modest rear gap only after reserving the following
// vehicle's full braking distance. Alongside and oncoming vehicles retain normal margins.
func yieldedVehicleClearsRear(bus *sim.Bus, other VehicleObstacle, cosH, sinH float64) bool {
	granted := other.YieldingTo == bus
	// Several queued buses can request the same lane. A stopped follower
	// already yielding to the rear request also protects a bus ahead of it;
	// insisting on a different pointer makes those leaders wait on each other.
	if !granted && other.YieldingTo != nil && math.Abs(other.Speed) <= 3 {
		own, held := bus.LaneMergeRequest, other.YieldingTo.LaneMergeRequest
		forward := sim.Vec2{X: cosH, Y: sinH}
		right := sim.Vec2{X: -sinH, Y: cosH}
		granted = own != nil && held != nil && own.Remaining > 0 && held.Remaining > 0 && !own.Joined && !held.Joined && math.Cos(own.Heading-held.Heading) > .999 && math.Abs(own.Target.Sub(held.Target).Dot(right)) < 2 && bus.Pos.Sub(other.YieldingTo.Pos).Dot(forward) >= 0
	}
	if !granted || (math.Cos(other.Heading-math.Atan2(sinH, cosH)) < .98 && (math.Abs(other.Speed) > 20 || math.Cos(other.Heading-math.Atan2(sinH, cosH)) < .95)) {
		return false
	}
	yaw := bus.Heading - math.Atan2(sinH, cosH)
	body := bus.BodyEnvelope()
	extent := body.HalfLen*math.Abs(math.Cos(yaw)) + body.HalfWid*math.Abs(math.Sin(yaw))
	braking := other.Speed*other.Speed/(2*220) + math.Abs(other.Speed)*.2
	otherYaw := other.Heading - math.Atan2(sinH, cosH)
	otherExtent := other.Length/2*math.Abs(math.Cos(otherYaw)) + other.Width/2*math.Abs(math.Sin(otherYaw))
	return other.Pos.Sub(body.Center).Dot(sim.Vec2{X: cosH, Y: sinH}) < -(extent + otherExtent + 20 + braking)
}

func wholeBusInMergeLane(bus *sim.Bus, request *sim.BusLaneMergeRequest, lw float64) bool {
	if math.Abs(bus.SteeringAngle) > .08 {
		return false
	}
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - request.Heading
		half := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
		if math.Abs(part.Center.Sub(request.Target).Dot(right))+half > lw/2-.25 {
			return false
		}
	}
	return true
}

func approachMergeStopDistance(f autopilotFrame, request *sim.BusLaneMergeRequest) float64 {
	heading := request.Heading
	distToStop := 0.0
	if f.world != nil {
		forward := sim.UnitFromAngle(heading)
		bumperPos := f.bus.Pos.Add(forward.Mul(f.bus.Length / 2))
		stopLineDist := 0.0
		if f.tls != nil {
			for _, it := range f.tls.Intersections {
				if it.Center.Distance(request.Junction) < 1 {
					stopLineDist = it.GetStopLineDistForHeading(heading, f.laneWidth) + f.world.RailIntersectionPullback(it, bumperPos, heading, f.laneWidth, f.bus.OverallLength())
					break
				}
			}
		}
		if stopLineDist == 0 {
			g := f.world.JunctionAt(request.Junction, f.laneWidth)
			stopLineDist = g.HalfX + sim.JunctionStopInset
			if math.Abs(math.Cos(heading)) <= .5 {
				stopLineDist = g.HalfY + sim.JunctionStopInset
			}
		}
		toJunction := request.Junction.Sub(f.bus.Pos).Dot(forward)
		distToStop = toJunction - stopLineDist - f.bus.Length/2
	}
	return distToStop
}
