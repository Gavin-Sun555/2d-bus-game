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
	filteredByCandidates := false
	if ad == nil || !ad.disableSweepCandidates {
		var storage [128]int
		if candidates, filtered := articulatedSweepCandidates(f, speed, steer, storage[:0]); filtered {
			var local [128]VehicleObstacle
			for i, index := range candidates {
				local[i] = f.obstacles[index]
			}
			obstacles = local[:len(candidates)]
			filteredByCandidates = true
		}
	}
	var near [128]VehicleObstacle
	if !filteredByCandidates && len(obstacles) > 0 && len(obstacles) <= len(near) {
		maxTravel := speed * .8
		n := 0
		for _, ob := range obstacles {
			if rearCars.matches(ob) {
				continue
			}
			reach := f.bus.OverallLength() + ob.Length + maxTravel + 102
			delta := ob.Pos.Sub(f.bus.Pos)
			if delta.Dot(delta) > reach*reach {
				continue
			}
			near[n] = ob
			n++
		}
		obstacles = near[:n]
	}
	if len(obstacles) == 0 && (len(checkStraight) == 0 || !checkStraight[0]) && steer == 0 && f.bus.SteeringAngle == 0 && math.Abs(sim.NormalizeAngle(f.bus.Heading-f.bus.TailHeading())) <= .03 {
		return math.Inf(1)
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
			if copy.HitsHardBarrier(f.world, f.laneWidth) {
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
			if !ownReady {
				ownBodies, ownReady = copy.BodyParts(), true
			}
			distance := math.Max(0, ob.Speed) * elapsed
			var parts [2]sim.OBB
			if ob.Bus != nil && distance > 0 {
				position, heading := predictedObstacleBusPose(ob, distance)
				if predictedBusSeparated(ownBodies, ob.Bus, position, heading, 1, 1) {
					continue
				}
				parts = ob.Bus.BodyPartsAtPose(position, heading)
			} else {
				parts = predictedObstacleBodies(ob, distance)
			}
			hit := false
			for _, body := range parts {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen += 1
				body.HalfWid += 1
				for _, own := range ownBodies {
					if own.HalfLen <= 0 {
						continue
					}
					if touching, _, depth := sim.CheckOBBCollision(own, body); touching && depth > 0 {
						if isRearFollowerContact(f, own, ob, body) {
							continue
						}
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

func isRearFollowerContact(f autopilotFrame, own sim.OBB, ob VehicleObstacle, paddedBody sim.OBB) bool {
	if f.bus == nil || f.bus.Speed < -0.5 {
		return false
	}
	forward := sim.UnitFromAngle(f.bus.Heading)
	delta := ob.Pos.Sub(f.bus.Pos)
	alignedWithTail := f.bus.IsArticulated() && math.Cos(ob.Heading-f.bus.TailHeading()) >= 0.5
	if delta.Dot(forward) >= 0 || (math.Cos(ob.Heading-f.bus.Heading) < 0.5 && !alignedWithTail) {
		return false
	}
	// The follower's envelope spans empty space beside a bent trailer.
	// Check actual bodies before deciding that the vehicles already touch.
	followerBodies := predictedObstacleBodies(ob, 0)
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		for _, follower := range followerBodies {
			if follower.HalfLen <= 0 {
				continue
			}
			if touching, _, depth := sim.CheckOBBCollision(part, follower); touching && depth > 0 {
				return false
			}
		}
	}
	unpadded := sim.OBB{Center: paddedBody.Center, Heading: paddedBody.Heading, HalfLen: paddedBody.HalfLen - 1, HalfWid: paddedBody.HalfWid - 1}
	if touching, _, depth := sim.CheckOBBCollision(own, unpadded); touching && depth > 0 {
		return false
	}
	return true
}

func isClosingRearFollowerHit(f autopilotFrame, ob VehicleObstacle, maxDist float64) bool {
	if f.bus == nil || f.bus.Speed < -0.5 || ob.Speed <= 0 || maxDist <= 0 {
		return false
	}
	forward := sim.UnitFromAngle(f.bus.Heading)
	delta := ob.Pos.Sub(f.bus.Pos)
	if delta.Dot(forward) >= 0 || math.Cos(ob.Heading-f.bus.Heading) < 0.7 {
		return false
	}
	reach := f.bus.OverallLength() + ob.Length + maxDist + 40
	if delta.Dot(delta) > reach*reach {
		return false
	}
	baseParts := f.bus.BodyParts()
	if sweepBodiesCollision(baseParts, ob, 0, 0, 0) {
		return false
	}
	for frac := .1; frac <= 1.01; frac += .1 {
		if sweepBodiesCollision(baseParts, ob, maxDist*math.Min(1, frac), 1, 1) {
			return true
		}
	}
	return false
}

func frameWithoutClosingRearFollowers(f autopilotFrame) autopilotFrame {
	if len(f.obstacles) == 0 {
		return f
	}
	filtered := make([]VehicleObstacle, 0, len(f.obstacles))
	changed := false
	for _, ob := range f.obstacles {
		if isClosingRearFollowerHit(f, ob, math.Max(0, ob.Speed)*.8) {
			changed = true
			continue
		}
		filtered = append(filtered, ob)
	}
	if !changed {
		return f
	}
	f.obstacles = filtered
	return f
}

// articulatedBarrierUnwindSteer softens a full-lock pursuit steer when static
// roadside barriers clip the articulated sweep while gentler or road-aligned
// steering clears.
func (ad *AutopilotDriver) articulatedBarrierUnwindSteer(f autopilotFrame, desired float64) (float64, bool) {
	if f.nearRoundabout || f.world == nil || f.bus == nil || ad.followingSpeed(f) <= .5 || ad.obstacleSpeed(f, 30) <= .5 {
		return 0, false
	}
	clean := frameWithoutClosingRearFollowers(f)
	if ad.controlSweptSpeed(clean, desired) > .5 && ad.articulatedSweptSpeed(clean, desired) > .5 {
		return 0, false
	}
	savedHold, savedSpeed := ad.roadSafetyHold, ad.roadRecoverySpeed
	defer func() {
		ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
	}()
	unwindFrame := clean
	unwindFrame.allowBarrierUnwindYaw = true
	accept := func(cand float64) bool {
		checked := ad.roadLaneSafeSteering(unwindFrame, cand)
		hold := ad.roadSafetyHold
		ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
		if hold || math.Abs(checked-cand) > .01 {
			return false
		}
		return ad.controlSweptSpeed(clean, cand) >= 16 && ad.articulatedSweptSpeed(clean, cand, true) >= 16
	}
	sharpJunctionTurn := f.world != nil && f.bus != nil &&
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) &&
		math.Abs(desired) >= .30 && len(ad.drivePath) >= 2 &&
		ad.pointAt(f.s+100).Sub(ad.pointAt(f.s)).Normalize().Dot(sim.UnitFromAngle(f.bus.Heading)) < .85
	if math.Abs(desired) > .05 {
		scales := []float64{.65, .35, .15, 0}
		if sharpJunctionTurn {
			scales = []float64{.75, .65, .5, .35}
		}
		for _, scale := range scales {
			cand := desired * scale
			if accept(cand) {
				return cand, true
			}
		}
	}
	var signs []float64
	yawErr := ad.barrierRecoveryYaw(f)
	if math.Abs(yawErr) > .1 && math.Abs(yawErr) < math.Pi*.75 {
		sign := math.Copysign(1, yawErr)
		signs = append(signs, sign)
		if !sharpJunctionTurn {
			// A curb-side shelter can leave the road-yaw side blocked while the
			// opposite lock cleanly unwinds away from the obstacle.
			signs = append(signs, -sign)
		}
	} else {
		// Even when nearly aligned with a bay taper (|yawErr| <= 0.1), a
		// curb-side barrier or trailer sweep can still require stronger steer.
		sign := 1.0
		if math.Abs(desired) > .05 {
			sign = math.Copysign(1, desired)
		} else if yawErr != 0 {
			sign = math.Copysign(1, yawErr)
		}
		if sharpJunctionTurn {
			signs = append(signs, sign)
		} else {
			signs = append(signs, sign, -sign)
		}
	}
	for _, s := range signs {
		for _, mag := range []float64{.35, .65, .85, 1} {
			cand := s * mag
			if accept(cand) {
				return cand, true
			}
		}
	}
	return 0, false
}

func (ad *AutopilotDriver) localPathHeading(f autopilotFrame) (float64, bool) {
	if ad == nil || len(ad.drivePath) < 2 || len(ad.cumLen) != len(ad.drivePath) {
		return 0, false
	}
	total := ad.cumLen[len(ad.cumLen)-1]
	if f.s >= total-5 {
		return 0, false
	}
	ahead := ad.pointAt(math.Min(f.s+80, total)).Sub(ad.pointAt(f.s))
	if ahead.Length() < 1e-3 {
		return 0, false
	}
	return math.Atan2(ahead.Y, ahead.X), true
}

func (ad *AutopilotDriver) barrierRecoveryYaw(f autopilotFrame) float64 {
	if heading, ok := ad.localPathHeading(f); ok {
		return sim.NormalizeAngle(heading - f.bus.Heading)
	}
	if f.world == nil {
		return 0
	}
	_, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	heading := 0.0
	if ew {
		if math.Cos(f.bus.Heading) < 0 {
			heading = math.Pi
		}
	} else if math.Sin(f.bus.Heading) >= 0 {
		heading = math.Pi / 2
	} else {
		heading = -math.Pi / 2
	}
	return sim.NormalizeAngle(heading - f.bus.Heading)
}
