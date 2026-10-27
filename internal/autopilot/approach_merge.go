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
	heading := request.Heading
	right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	target := request.Target
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
	if approachMergeComplete(f.bus, request, f.laneWidth) {
		if request.RightTurn {
			center := request.Junction
			ad.rightTurnJunction = &center
			ad.rightTurnHeading = heading
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
			ad.approachMerge = nil
			return false
		}
		if request.LeftTurn || request.Is2Lane {
			ad.holdLaneOffset(0)
			ad.OvertakeState = OvertakeNone
			ad.approachMerge = nil
			ad.approachMergeJoined = false
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
		ad.TargetLaneOffset = desired
		ad.OvertakeState = OvertakeReturning
		ad.progressLaneOffset(dt, f.speedAbs)
		if math.Abs(ad.LaneOffset-desired) < 1.0 {
			ad.LaneOffset = desired
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	} else {
		distToStop := 0.0
		if f.world != nil {
			forward := sim.UnitFromAngle(heading)
			stopLineDist := 0.0
			if f.tls != nil {
				for _, it := range f.tls.Intersections {
					if it.Center.Distance(request.Junction) < 1 {
						stopLineDist = it.GetStopLineDistForHeading(heading, f.laneWidth)
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
		if distToStop > 0 {
			ad.busTurnHoldDistance = math.Max(.01, distToStop-25)
		} else {
			ad.busTurnHoldDistance = .01
		}
		ad.holdLaneOffset(actual)
	}
	return true
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
