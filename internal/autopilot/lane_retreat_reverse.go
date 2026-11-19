package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) straightLaneRetreatActive() bool {
	request := ad.approachMerge
	return ad.midLaneRecovery != nil && request != nil && !request.RightTurn && !request.LeftTurn && !request.Is2Lane
}

// A tight parallel gap needs a physical reverse sweep. Its conservative
// following corridor alone cannot decide whether the adjacent car blocks it.
func (ad *AutopilotDriver) laneRetreatReverseClear(f autopilotFrame, distance, steer float64) bool {
	_, ok := ad.simulateReversePose(f, distance, steer)
	return ok
}

func (ad *AutopilotDriver) simulateReversePose(f autopilotFrame, distance, steer float64) (*sim.Bus, bool) {
	var nearby []VehicleObstacle
	const speed = 17.
	duration := (distance + 5) / speed
	for _, ob := range f.obstacles {
		reach := distance + f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*duration + 20
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > reach*reach {
			continue
		}
		nearby = append(nearby, ob)
	}
	predicted := *f.bus
	predicted.JunctionCorridor, predicted.RoundaboutEntry = nil, nil
	predicted.LaneMergeRequest, predicted.StationMergeRequest = nil, nil
	predicted.Gear = sim.GearReverse
	predicted.FrictionRate = 0
	if steer == 0 && f.speedAbs <= .5 {
		// Parallel retreats center the wheels while stopped before backing.
		predicted.SteeringAngle = 0
	}
	initialParts := f.bus.BodyParts()
	for elapsed := 0.; elapsed <= duration; elapsed += .025 {
		if elapsed > 0 {
			predicted.Speed = -speed
			predicted.Update(.025, 0, 0, steer, false, false)
			if predicted.Speed >= 0 {
				return nil, false
			}
		}
		if predicted.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
			return nil, false
		}
		parts := predicted.BodyParts()
		for _, ob := range nearby {
			if sweepBodiesCollision(parts, ob, ob.Speed*elapsed, .1, .1) {
				// Existing separation can be smaller than the prediction skin.
				// Keep checking real bodies throughout the retreat, but don't
				// freeze a disjoint pair solely on padding already touching at
				// the start. New contacts retain the usual extra clearance.
				paddingOnly := sweepBodiesCollision(initialParts, ob, 0, .1, .1) &&
					!sweepBodiesCollision(initialParts, ob, 0, 0, 0) &&
					!sweepBodiesCollision(parts, ob, ob.Speed*elapsed, 0, 0)
				if !paddingOnly {
					return nil, false
				}
			}
			if ob.Speed < 0 && elapsed > 0 {
				backed := ob
				if backed.Bus != nil {
					backed.Width += 8
					backed.Length += 8
					backed.Bus = nil
				}
				backed.Pos = ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(ob.Speed * elapsed))
				backed.Speed = 0
				if sweepBodiesCollision(parts, backed, 0, .1, .1) {
					return nil, false
				}
			}
		}
	}
	return &predicted, true
}
