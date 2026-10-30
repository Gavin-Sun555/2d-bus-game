package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Bus-lane roads have four lanes per direction. Their base route follows the
// reserved lane; offsets are measured from that route, never from a six-lane template.
func (ad *AutopilotDriver) updateBusLane(f autopilotFrame, dt float64) bool {
	if f.world == nil || len(ad.drivePath) < 2 {
		return false
	}
	if ad.busLaneJunction != nil {
		center := *ad.busLaneJunction
		g := f.world.JunctionAt(center, f.laneWidth)
		d := f.bus.Pos.Sub(center)
		if math.Abs(d.X) <= g.HalfX+f.bus.Length/2 && math.Abs(d.Y) <= g.HalfY+f.bus.Length/2 {
			// Lane changes finish on the approach. Do not carry a passing offset around
			// a turn and out onto a narrower road, where it can point into the pavement.
			ad.busJunctionEntered = true
			ad.TargetLaneOffset = 0
			ad.progressLaneOffset(dt, f.speedAbs)
			return true
		}
		if ad.busJunctionEntered {
			ad.busLaneJunction = nil
			ad.busJunctionEntered = false
		}
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
	if !ok || !road.HasBusLane() {
		return false
	}
	heading := approachRoadHeading(f.bus.Heading)
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	approach := ad.approachingIntersection(f, true, true)
	if approach.approachingInter != nil && f.world.IsAtRoundabout(approach.approachingInter.Center) {
		return false
	}
	turning := approach.isUpcomingLeft || approach.isUpcomingRight
	if approach.isApproachingIntersection && turning {
		inter := approach.approachingInter
		if ad.busLaneJunction == nil && approach.isUpcomingLeft && approach.distToStopLine > 350 {
			curbQueue := ad.busLaneTrafficScore(f, forward, right, 3, true)
			innerQueue := ad.busLaneTrafficScore(f, forward, right, 0, true)
			if innerQueue+1 < curbQueue && isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, ad.LaneOffset, -3*f.laneWidth, f.laneWidth) {
				ad.ordinaryBusLeft = true
				var from, to *sim.BusStop
				for _, stop := range f.world.Stops {
					if stop.ID == ad.pathFromID {
						from = stop
					}
					if stop.ID == ad.pathToID {
						to = stop
					}
				}
				if from != nil && to != nil {
					ad.PrepareLeg(f.world, from, to, f.laneWidth)
					if !ad.joinMedianLeftApproach(f, heading, inter.Center) {
						ad.ordinaryBusLeft = false
						ad.PrepareLeg(f.world, from, to, f.laneWidth)
					}
					f.s, _ = ad.trackPath(f.bus.Pos)
				} else {
					ad.ordinaryBusLeft = false
				}
			}
		}
		c := inter.Center
		ad.busLaneJunction = &c

		desired := 0.0
		actual := f.bus.Pos.Sub(ad.pointAt(f.s)).Dot(right)
		// A mandatory path taper needs the same explicit rear-slot request as
		// other required merges. Otherwise a waiting follower brakes for the
		// leader but never grants the slot the leader is waiting to receive.
		if math.Abs(actual) > 4 {
			request := f.bus.LaneMergeRequest
			if request == nil {
				request = &sim.BusLaneMergeRequest{}
			}
			*request = sim.BusLaneMergeRequest{Target: ad.pointAt(f.s), Heading: heading, Remaining: .3, LeftTurn: approach.isUpcomingLeft, RightTurn: approach.isUpcomingRight, Junction: inter.Center}
			f.bus.LaneMergeRequest = request
		}
		// Recentring inside the lane is not a lane change. A following
		// vehicle must not reserve our own lane and deadlock its leader.
		withinLane := math.Abs(actual-desired)+f.bus.Length/2*math.Abs(math.Sin(f.bus.Heading-heading))+f.bus.Width/2*math.Abs(math.Cos(f.bus.Heading-heading)) <= f.laneWidth/2-.25
		if withinLane || requiredMergeLaneClear(f, heading, actual, desired) {
			ad.TargetLaneOffset = desired
		} else if math.Abs(actual) > 1 || math.Abs(ad.LaneOffset) > 1 {
			if ad.ordinaryBusLeft && ad.recoverBlockedMedianChoice(f, heading, center, ew, dt) {
				return true
			}
			if !ad.ordinaryBusLeft && actual > -1.1*f.laneWidth && actual <= -.55*f.laneWidth && f.speedAbs < 5 && math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) < .22 {
				// A whole-lane return needs a real merge reservation. Otherwise
				// a following bus can close the rear slot while waiting for us.
				ad.approachMerge = &sim.BusLaneMergeRequest{Target: ad.pointAt(f.s), Heading: heading, Remaining: .3, LeftTurn: true, Junction: inter.Center}
				ad.approachMergeJoined = false
				return ad.advanceApproachMerge(f, dt)
			}
			if !ad.ordinaryBusLeft && math.Abs(actual) < .55*f.laneWidth && f.speedAbs < 5 && math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) < .22 {
				// A close follower need not provide a full passing gap for a
				// fractional recenter. Crawl only when actuator/body prediction
				// clears the white car beside the rear and the following bus.
				ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: ad.pointAt(f.s), Heading: heading, Remaining: .3}
				ad.advanceMidLaneRecovery(f, dt)
				return true
			}
			ad.holdLaneOffset(actual)
			ad.busTurnHoldDistance = math.Max(.01, approach.distToStopLine-f.bus.Length/2-25)
		}
		ad.progressLaneOffset(dt, f.speedAbs)
		return true
	}
	if !ad.CanSkipTargetStop && f.remaining < math.Max(420, f.speedAbs*2.2+150) {
		if ad.BerserkMode {
			ad.updateStationApproachLane(f.bus, f.targetStop, f.obstacles, f.laneWidth, f.speedAbs, f.remaining, dt, true, true)
		} else {
			ad.TargetLaneOffset = 0
			ad.progressLaneOffset(dt, f.speedAbs)
		}
		return true
	}
	// Protected bus priority by default; use a general through lane only when
	// its queue or moving traffic is materially better, and return once clear.
	desiredLane := 3
	best := ad.busLaneTrafficScore(f, forward, right, 3, approach.isApproachingIntersection)
	candidates := []int{2, 1}
	if !approach.isApproachingIntersection {
		candidates = append(candidates, 0)
	}
	if ad.BerserkMode {
		for _, lane := range candidates {
			score := ad.busLaneTrafficScore(f, forward, right, lane, approach.isApproachingIntersection)
			if score+1 < best {
				best = score
				desiredLane = lane
			}
		}
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	base := ad.pointAt(f.s).Sub(origin).Dot(right)
	desired := (float64(desiredLane)+.5)*f.laneWidth - base
	// A path taper is part of a turn/stop, not another cruising lane to score.
	if math.Abs(base-3.5*f.laneWidth) > f.laneWidth*.2 {
		desired = 0
	}
	// A frozen fractional offset is not a completed lane return.
	if math.Abs(desired-ad.TargetLaneOffset) > 1 && isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, ad.LaneOffset, desired, f.laneWidth) {
		ad.TargetLaneOffset = desired
		if desired < ad.LaneOffset {
			ad.OvertakeState = OvertakeChangingLane
		} else {
			ad.OvertakeState = OvertakeReturning
		}
	}
	ad.progressLaneOffset(dt, f.speedAbs)
	if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1 {
		if math.Abs(ad.TargetLaneOffset) < 1 {
			ad.OvertakeState = OvertakeNone
		} else {
			ad.OvertakeState = OvertakePassing
		}
	}
	return true
}

func (ad *AutopilotDriver) busLaneTrafficScore(f autopilotFrame, forward, right sim.Vec2, lane int, queueOnly bool) float64 {
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	roadCenter := sim.Vec2{Y: center}
	if !ew {
		roadCenter = sim.Vec2{X: center}
	}
	score := 0.0
	cruiseSpeed := f.speedAbs
	if ad != nil && ad.BerserkMode {
		cruiseSpeed = ad.GetBerserkCruiseSpeed()
	} else if cruiseSpeed < apBerserkCruiseSpeed {
		cruiseSpeed = apBerserkCruiseSpeed
	}
	horizon := math.Max(650, cruiseSpeed*2.5+150)
	if queueOnly {
		horizon = 1800
		if f.tls != nil {
			for _, inter := range f.tls.Intersections {
				d := inter.Center.Sub(f.bus.Pos)
				if d.Dot(forward) > 0 && math.Abs(d.Dot(right)) < 4.8*f.laneWidth {
					horizon = math.Min(horizon, d.Dot(forward)-inter.GetStopLineDistForHeading(math.Atan2(forward.Y, forward.X), f.laneWidth)+100)
				}
			}
		}
	}
	for _, ob := range f.obstacles {
		if math.Cos(ob.Heading-math.Atan2(forward.Y, forward.X)) < .8 {
			continue
		}
		// A bus fully inside its bay does not occupy the reserved travel lane.
		extent := ob.Length/2*math.Abs(math.Sin(ob.Heading-math.Atan2(forward.Y, forward.X))) + ob.Width/2*math.Abs(math.Cos(ob.Heading-math.Atan2(forward.Y, forward.X)))
		if math.Abs(ob.Speed) < 5 && ob.Pos.Sub(roadCenter).Dot(right)-extent > 4*f.laneWidth+1 {
			continue
		}
		d := ob.Pos.Sub(f.bus.Pos).Dot(forward)
		lateral := ob.Pos.Sub(roadCenter).Dot(right)
		if d < 0 || d > horizon || math.Abs(lateral-(float64(lane)+.5)*f.laneWidth) > (f.bus.Width+ob.Width)/2+8 {
			continue
		}
		if ob.Speed < 20 {
			score += 4.0 * (1.0 + math.Max(0.0, (horizon-d)/horizon))
		} else if !queueOnly {
			if ob.Speed < cruiseSpeed {
				slowdown := (cruiseSpeed - ob.Speed) / cruiseSpeed
				w := 1.0 + 2.5*slowdown
				if ob.Speed < cruiseSpeed*0.85 {
					w = math.Max(w, 2.5)
				}
				distWeight := math.Max(0.0, (horizon-d)/horizon)
				score += w * (1.0 + distWeight)
			}
		}
	}
	return score
}
