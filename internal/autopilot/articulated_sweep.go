package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Simulate both sections and the actual hitch constraint, including in junctions.
func (ad *AutopilotDriver) articulatedSweptSpeed(f autopilotFrame, steer float64, checkStraight ...bool) float64 {
	if !f.bus.IsArticulated() || f.world == nil {
		return math.Inf(1)
	}
	rearCars := ad.departureRearCars(f)
	copy := *f.bus
	speed := math.Max(16, f.speedAbs)
	obstacles := f.obstacles
	if ad == nil || !ad.disableSweepCandidates {
		var storage [128]int
		if candidates, filtered := articulatedSweepCandidates(f, speed, steer, storage[:0]); filtered {
			var local [128]VehicleObstacle
			for i, index := range candidates {
				local[i] = f.obstacles[index]
			}
			obstacles = local[:len(candidates)]
		}
	}
	for tick := 1; tick <= 16; tick++ {
		elapsed := float64(tick) * .05
		wheel := copy.SteeringAngle + sim.Clamp(steer*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
		angle := copy.Heading + speed/copy.WheelBase*math.Tan(wheel)*.05
		pos := copy.Pos.Add(sim.UnitFromAngle(angle).Mul(speed * .05))
		copy = copy.AtPose(pos, angle)
		copy.SteeringAngle = wheel
		var ownBodies [2]sim.OBB
		ownReady := false
		collision := false
		if tick%4 == 0 && ((len(checkStraight) > 0 && checkStraight[0]) || math.Abs(wheel) > .02 || math.Abs(sim.NormalizeAngle(copy.Heading-copy.TailHeading())) > .03) {
			if copy.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
				collision = true
			}
		}
		for _, ob := range obstacles {
			if rearCars.matches(ob) {
				continue
			}
			delta := ob.Pos.Sub(copy.Pos)
			reach := copy.OverallLength() + ob.Length + 100
			if delta.Dot(delta) > reach*reach {
				continue
			}
			hit := false
			for _, body := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen += 1
				body.HalfWid += 1
				if !ownReady {
					ownBodies, ownReady = copy.BodyParts(), true
				}
				for _, own := range ownBodies {
					if own.HalfLen <= 0 {
						continue
					}
					if touching, _, depth := sim.CheckOBBCollision(own, body); touching && depth > 0 {
						hit = true
						break
					}
				}
				if hit {
					break
				}
			}
			if hit {
				collision = true
				break
			}
		}
		if collision {
			return math.Sqrt(2 * 180 * math.Max(0, speed*elapsed-20))
		}
	}
	return math.Inf(1)
}
