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
	var nearby []VehicleObstacle
	const speed = 17.
	duration := (distance + 5) / speed
	for _, ob := range f.obstacles {
		reach := distance + f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*duration + 20
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > reach*reach {
			continue
		}
		if ob.Speed < 0 {
			return false
		}
		nearby = append(nearby, ob)
	}
	predicted := *f.bus
	predicted.JunctionCorridor, predicted.RoundaboutEntry = nil, nil
	predicted.LaneMergeRequest, predicted.StationMergeRequest = nil, nil
	predicted.Gear = sim.GearReverse
	predicted.FrictionRate = 0
	for elapsed := 0.; elapsed <= duration; elapsed += .025 {
		if elapsed > 0 {
			predicted.Speed = -speed
			predicted.Update(.025, 0, 0, steer, false, false)
			if predicted.Speed >= 0 {
				return false
			}
		}
		if predicted.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
			return false
		}
		parts := predicted.BodyParts()
		for _, ob := range nearby {
			if sweepBodiesCollision(parts, ob, ob.Speed*elapsed, .1, .1) {
				return false
			}
		}
	}
	return true
}
