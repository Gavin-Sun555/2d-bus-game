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
	forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	approach := ad.approachingIntersection(f, true, true)
	if approach.approachingInter != nil && f.world.IsAtRoundabout(approach.approachingInter.Center) {
		return false
	}
	turning := approach.isUpcomingLeft || approach.isUpcomingRight
	if approach.isApproachingIntersection && turning {
		inter := approach.approachingInter
		if ad.busLaneJunction == nil && approach.isUpcomingLeft && approach.distToStopLine > 350 {
			curbQueue := busLaneTrafficScore(f, forward, right, 3, true)
			innerQueue := busLaneTrafficScore(f, forward, right, 0, true)
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
					ad.trackPath(f.bus.Pos)
				} else {
					ad.ordinaryBusLeft = false
				}
			}
		}
		c := inter.Center
		ad.busLaneJunction = &c

		desired := 0.0
		if isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, ad.LaneOffset, desired, f.laneWidth) {
			ad.TargetLaneOffset = desired
		} else if math.Abs(ad.LaneOffset) > 1 {
			ad.TargetLaneOffset = ad.LaneOffset
			ad.LaneOffsetProgress = 1
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
	best := busLaneTrafficScore(f, forward, right, 3, approach.isApproachingIntersection)
	candidates := []int{2, 1}
	if !approach.isApproachingIntersection {
		candidates = append(candidates, 0)
	}
	if ad.BerserkMode {
		for _, lane := range candidates {
			score := busLaneTrafficScore(f, forward, right, lane, approach.isApproachingIntersection)
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
	currentLane := int(math.Round((base+ad.LaneOffset)/f.laneWidth - .5))
	if desiredLane != currentLane && isLaneClearToChange(f.bus, f.obstacles, forward.X, forward.Y, ad.LaneOffset, desired, f.laneWidth) {
		ad.TargetLaneOffset = desired
		ad.OvertakeState = OvertakeChangingLane
	}
	ad.progressLaneOffset(dt, f.speedAbs)
	if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1 {
		ad.OvertakeState = OvertakeNone
	}
	return true
}

func busLaneTrafficScore(f autopilotFrame, forward, right sim.Vec2, lane int, queueOnly bool) float64 {
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	roadCenter := sim.Vec2{Y: center}
	if !ew {
		roadCenter = sim.Vec2{X: center}
	}
	score := 0.0
	horizon := math.Max(650, f.speedAbs*2.5+150)
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
			score += 4
		} else if !queueOnly {
			score += 3 * math.Max(0, 1-ob.Speed/math.Max(150, f.speedAbs))
		}
	}
	return score
}
