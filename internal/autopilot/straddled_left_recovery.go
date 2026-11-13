package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A failed median merge can be frozen across two through lanes at the line.
// Ordinary retreat needs forward runway which no longer exists there. The
// fleet's local recovery may settle into a clear legal left-turn lane after a
// sustained stop; actual queues already centered in a lane retain their place.
func (ad *AutopilotDriver) recoverStraddledLeftApproach(bus *sim.Bus, w *sim.World, obstacles []VehicleObstacle, lw, dt float64, signals ...*sim.TrafficLightSystem) (handled, recovered bool) {
	if ad.busJunctionEntered || ad.State != AutopilotStateCruising ||
		bus.DoorsOpen || bus.DoorProgress > .05 || bus.PassengerServiceRemaining() > 0 || math.Abs(bus.Speed) > 1 || len(ad.drivePath) < 2 {
		return false, false
	}
	heading := approachRoadHeading(bus.Heading)
	// Six-lane roads encode the median taper directly in the base path, without
	// the eight-lane ordinaryBusLeft flag. Recognize that same intended turn.
	plannedSix := !ad.ordinaryBusLeft || ad.busLaneJunction == nil
	var center sim.Vec2
	if plannedSix {
		if yaw := math.Abs(sim.NormalizeAngle(bus.Heading - heading)); yaw < .45 || yaw > .75 {
			return false, false
		}
		f := autopilotFrame{bus: bus, world: w, laneWidth: lw, s: ad.LastPathS, speedAbs: math.Abs(bus.Speed)}
		if len(signals) > 0 {
			f.tls = signals[0]
		}
		approach := ad.approachingIntersection(f, true, true)
		if approach.approachingInter == nil || !approach.isUpcomingLeft {
			return false, false
		}
		center = approach.approachingInter.Center
	} else {
		center = *ad.busLaneJunction
		if math.Abs(sim.NormalizeAngle(bus.Heading-heading)) > .1 {
			return false, false
		}
	}
	forward := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	geometry := w.JunctionAt(center, lw)
	half := math.Abs(forward.X)*geometry.HalfX + math.Abs(forward.Y)*geometry.HalfY
	beforeBox := center.Sub(bus.Pos).Dot(forward) - half
	if beforeBox < bus.Length/2+2 || beforeBox > bus.OverallLength()+200 {
		return false, false
	}
	coord, ew := center.Y, math.Abs(forward.X) > .5
	if !ew {
		coord = center.X
	}
	road, ok := w.RoadAt(coord, ew, bus.Pos)
	if !ok || road.HasMedianBusLane() || (plannedSix && road.LaneCount() != 6) || (!plannedSix && (road.LaneCount() != 8 || !road.HasBusLane())) {
		return false, false
	}
	side := bus.Pos.Sub(center).Dot(right)
	if side < lw/2 || side > (float64(road.LaneCount()/2)-.5)*lw {
		return false, false
	}
	if plannedSix {
		f := autopilotFrame{bus: bus, world: w, obstacles: obstacles, laneWidth: lw}
		sidecar := false
		for _, ob := range obstacles {
			if offset, _ := stoppedSidecarOffset(f, ob, forward, right); offset != 0 {
				sidecar = true
				break
			}
		}
		if !sidecar {
			return false, false
		}
	}
	nearest := math.Round(side/lw - .5)
	laneCenter := center.Add(right.Mul((nearest + .5) * lw))
	if wholeBusInMergeLane(bus, &sim.BusLaneMergeRequest{Target: laneCenter, Heading: heading}, lw) {
		return false, false
	}
	// A slightly yawed trailer at a red light is not a failed lane merge.
	// Count only time the signal allows this movement. Once controls can move
	// the bus, discard stale recovery time before its first acceleration tick.
	if ad.LastTargetSpeed > .5 {
		ad.strandedMedianWait = 0
		return false, false
	}
	if len(signals) > 0 && signals[0] != nil {
		f := autopilotFrame{bus: bus, world: w, tls: signals[0], laneWidth: lw, speedAbs: math.Abs(bus.Speed), s: ad.LastPathS}
		if plannedSix {
			for _, inter := range signals[0].Intersections {
				if inter.Center.Distance(center) < 1 && inter.GetSignalForHeading(heading, true) != sim.SignalGreen {
					return true, false
				}
			}
		}
		if ad.trafficLightSpeed(f, 30) <= .5 {
			return true, false
		}
	}
	// Continue the existing timer only for a truly stranded, non-moving body.
	if ad.strandedMedianWait == 0 || ad.strandedMedianStart.Distance(bus.Pos) > 3 {
		ad.strandedMedianStart = bus.Pos
		ad.strandedMedianWait = 0
	}
	ad.strandedMedianWait += math.Max(0, dt)
	timeout := 10.
	if plannedSix {
		timeout = 5
	}
	if ad.strandedMedianWait < timeout {
		return true, false
	}
	ad.strandedMedianWait = timeout - 1
	var from, to *sim.BusStop
	for _, stop := range w.Stops {
		if stop.ID == ad.pathFromID {
			from = stop
		}
		if stop.ID == ad.pathToID {
			to = stop
		}
	}
	if from == nil || to == nil {
		return true, false
	}
	curbLane := road.LaneCount()/2 - 1
	lanes := []int{0, curbLane}
	if side > 2*lw {
		lanes = []int{curbLane, 0}
	}
	for _, lane := range lanes {
		if math.Abs((float64(lane)+.5)*lw-side) > 2*lw {
			continue
		}
		pose := *bus
		pose.Pos = bus.Pos.Add(right.Mul((float64(lane)+.5)*lw - side))
		pose.Heading = heading
		pose.ResetArticulation()
		pose.Speed = 0
		pose.SteeringAngle = 0
		if pose.CheckBoundaries(w, lw).Zone != sim.ZoneRoad {
			continue
		}
		clear := true
		for _, rail := range w.Railways() {
			if math.Abs(pose.Pos.Y-rail.Y) < rail.HalfWidth+pose.OverallLength()+sim.RailGateMargin {
				clear = false
				break
			}
		}
		for _, ob := range obstacles {
			if ob.Bus == bus {
				continue
			}
			if ob.Speed < 0 && ob.Pos.Distance(pose.Pos) < pose.OverallLength()+ob.Length+math.Abs(ob.Speed) {
				clear = false
				break
			}
			for elapsed := 0.; elapsed <= 1; elapsed += .25 {
				if sweepBodiesCollision(pose.BodyParts(), ob, math.Max(0, ob.Speed)*elapsed, 2, 2) {
					clear = false
					break
				}
			}
			if !clear {
				break
			}
		}
		if !clear {
			continue
		}
		if ad.BeforeRecovery != nil {
			ad.BeforeRecovery()
		}
		bus.Pos, bus.Heading = pose.Pos, pose.Heading
		bus.ResetArticulation()
		bus.Speed, bus.SteeringAngle = 0, 0
		bus.JunctionCorridor, bus.LaneMergeRequest, bus.StationMergeRequest, bus.YieldingToBus = nil, nil, nil, nil
		ad.headonRecovery, ad.approachMerge, ad.midLaneRecovery = nil, nil, nil
		ad.headonDeadlockTimer, ad.busTurnHoldDistance, ad.strandedMedianWait = 0, 0, 0
		ad.ordinaryBusLeft = !plannedSix && lane == 0
		ad.PrepareLeg(w, from, to, lw)
		ad.pathSynced = false
		s, _ := ad.trackPath(bus.Pos)
		ad.holdLaneOffset(bus.Pos.Sub(ad.pointAt(s)).Dot(right))
		ad.busLaneTurnChosen = true
		return true, true
	}
	return true, false
}
