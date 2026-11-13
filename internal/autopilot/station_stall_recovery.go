package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// RecoverStrandedStationDeparture is a fleet-only last resort for a geometrical
// bay-exit lock. Unlike collision deadlocks, this can happen with separated
// bodies: the controller keeps stopping before a bent trailer hits the shelter.
// Retain the bus, passengers and route; straighten locally only at a clear pose.
func (ad *AutopilotDriver) RecoverStrandedStationDeparture(bus *sim.Bus, w *sim.World, tls *sim.TrafficLightSystem, obstacles []VehicleObstacle, lw, dt float64, held bool) bool {
	if ad == nil {
		return false
	}
	if bus == nil || w == nil || !bus.IsArticulated() || ad.State != AutopilotStateDeparting || !held ||
		math.Abs(bus.Speed) > .5 || bus.DoorsOpen || bus.DoorProgress > .05 || bus.PassengerServiceRemaining() > 0 ||
		ad.pathFromID < 0 || ad.pathFromID >= len(w.Stops) || len(ad.drivePath) < 2 {
		ad.stationStallWait = 0
		return false
	}
	stop := w.Stops[ad.pathFromID]
	if stop.CenterBRT || bus.Pos.Distance(stop.Pos) > stop.Length+bus.OverallLength() || w.InsideJunction(bus.Pos, lw, bus.OverallLength()/2) {
		ad.stationStallWait = 0
		return false
	}
	f := autopilotFrame{bus: bus, world: w, tls: tls, obstacles: obstacles, laneWidth: lw, s: ad.LastPathS, speedAbs: math.Abs(bus.Speed), remaining: ad.LastRemaining}
	if ad.pathToID >= 0 && ad.pathToID < len(w.Stops) {
		f.targetStop = w.Stops[ad.pathToID]
	}
	if ad.trafficLightSpeed(f, 150) <= .5 {
		ad.stationStallWait = 0
		return false
	}
	if ad.stationStallWait == 0 || ad.stationStallStart.Distance(bus.Pos) > 3 {
		ad.stationStallStart = bus.Pos
		ad.stationStallWait = 0
	}
	ad.stationStallWait += math.Max(0, dt)
	if ad.stationStallWait < 12 {
		return false
	}
	ad.stationStallWait = 11 // Retry a blocked landing at most once per second.
	// Traffic waits do not qualify: even with traffic removed, the commanded
	// departure must be blocked by static geometry or trailer articulation.
	probe := *ad
	steer, _, _, _ := probe.pursuitSteering(f)
	empty := f
	empty.obstacles = nil
	if probe.articulatedSweptSpeed(empty, steer) > .5 {
		return false
	}
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	ew := math.Abs(forward.X) > .5
	coord := stop.RoadCenter.X
	if ew {
		coord = stop.RoadCenter.Y
	}
	road, ok := w.RoadAt(coord, ew, bus.Pos)
	if !ok {
		return false
	}
	side := bus.Pos.Sub(stop.RoadCenter).Dot(right)
	// Try the actual berth position first; if the straight trailer does not fit
	// there, use the neighboring travel lane with at most twelve pixels of forward adjustment.
	lateralTargets := []float64{side, stop.Pos.Sub(stop.RoadCenter).Dot(right)}
	for lane := road.LaneCount()/2 - 1; lane >= 0; lane-- {
		lateralTargets = append(lateralTargets, (float64(lane)+.5)*lw)
	}
	for _, lateral := range lateralTargets {
		if math.Abs(lateral-side) > 1.25*lw {
			continue
		}
		for advance := 0.; advance <= 12; advance += 4 {
			pose := *bus
			pose.Pos = bus.Pos.Add(right.Mul(lateral - side)).Add(forward.Mul(advance))
			pose.Heading = stop.Heading
			pose.ResetArticulation()
			pose.Speed = 0
			pose.SteeringAngle = 0
			if lateral < bus.Width/2+2 || (!stop.ContainsBus(&pose) && lateral > road.GetWidth(lw)/2-bus.Width/2-2) || pose.CheckBoundaries(w, lw).Zone != sim.ZoneRoad {
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
					if sweepBodiesCollision(pose.BodyParts(), ob, math.Max(0, ob.Speed)*elapsed, 1, 1) {
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
			// Also require usable room ahead, so a clear landing does not merely move
			// the deadlock. No route or passenger state is changed by this prediction.
			for distance := 4.; distance <= 40 && clear; distance += 4 {
				next := pose.AtPose(pose.Pos.Add(forward.Mul(distance)), stop.Heading)
				if next.CheckBoundaries(w, lw).Zone != sim.ZoneRoad {
					clear = false
					break
				}
				for _, ob := range obstacles {
					if ob.Bus != bus && sweepBodiesCollision(next.BodyParts(), ob, math.Max(0, ob.Speed)*distance/18, 1, 1) {
						clear = false
						break
					}
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
			ad.beginRoadDeparture(bus, stop, lw)
			ad.skippedDeparture.served = true
			return true
		}
	}
	return false
}
