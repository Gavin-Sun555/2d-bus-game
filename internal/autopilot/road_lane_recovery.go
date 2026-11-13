package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func roadMedianClearance(bus *sim.Bus, pos sim.Vec2, angle, heading, center float64, ew bool) float64 {
	normal := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	yaw := angle - heading
	return pos.Sub(origin).Dot(normal) - bus.Length/2*math.Abs(math.Sin(yaw)) - bus.Width/2*math.Abs(math.Cos(yaw))
}

// A stopped bus needs a checked crawl to regain its lane. Small intrusions
// must improve body clearance throughout. A stranded median merge may also
// return from the opposing side while its center moves toward its own lane.
func (ad *AutopilotDriver) roadLaneRecovery(f autopilotFrame, steer, heading, center float64, ew bool) (float64, bool) {
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	clearance := roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew)
	minimumClearance := -math.Min(8, f.laneWidth*.12)
	strandedApproach := clearance < minimumClearance && ad.ordinaryBusLeft && ad.busLaneJunction != nil && !ad.busJunctionEntered
	if f.speedAbs > 25 || math.Abs(yaw) > .4 || (clearance < minimumClearance && !strandedApproach) {
		return steer, false
	}
	if strandedApproach && f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneRoad {
		return steer, false
	}
	const recoverySpeed = 20.
	ad.roadRecoverySpeed = recoverySpeed
	sweepSpeed := math.Max(recoverySpeed, f.speedAbs)
	candidates := []float64{sim.Clamp(steer, -.65, .65), 0, sim.Clamp(-yaw*3, -.65, .65), -.65, .65}
	if strandedApproach {
		candidates[0] = sim.Clamp((.3-yaw)*3, -.65, .65)
	}
	for _, input := range candidates {
		pos, angle, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
		previous := clearance
		safe := true
		const dt = .02
		for elapsed := dt; elapsed <= .8; elapsed += dt {
			rate := f.bus.SteerTurnSpeed
			if input == 0 {
				rate = f.bus.SteerReturnSpd
			}
			steering += sim.Clamp(input*f.bus.MaxSteerAngle-steering, -rate*dt, rate*dt)
			angle += sweepSpeed / f.bus.WheelBase * math.Tan(steering) * dt
			pos = pos.Add(sim.UnitFromAngle(angle).Mul(sweepSpeed * dt))
			next := roadMedianClearance(f.bus, pos, angle, heading, center, ew)
			minimum := math.Min(previous, 2) - .002
			if strandedApproach {
				// A saved bus may point slightly farther across the median.
				// Check center progress while allowing the tail to swing through
				// the obstacle-tested return, rather than freezing it on that side.
				next = pos.Sub(f.bus.Pos).Dot(sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)})
				minimum = -.25
			}
			if next < minimum {
				safe = false
				break
			}
			previous = next
			body := sim.OBB{Center: pos, Heading: angle, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
			for _, part := range f.bus.BodyPartsAtPose(pos, angle) {
				if part.HalfLen <= 0 {
					continue
				}
				part.HalfLen++
				part.HalfWid++
				for _, corner := range sim.BodyCorners(part) {
					if f.world.CheckBoundaries(corner, 0, f.laneWidth).Zone == sim.ZoneHardBarrier {
						safe = false
					}
				}
			}
			if !safe {
				break
			}
			for _, ob := range f.obstacles {
				if ob.Pos.Distance(f.bus.Pos) > 200+math.Abs(ob.Speed)*.8 {
					continue
				}
				other := sim.OBB{Center: ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(ob.Speed * elapsed)), Heading: ob.Heading, HalfLen: ob.Length/2 + 1, HalfWid: ob.Width/2 + 1}
				if ob.JunctionTurn != nil {
					other.Center, other.Heading = ob.JunctionTurn.Predict(math.Max(0, ob.Speed) * elapsed)
				}
				if hit, _, _ := sim.CheckBusAtBody(f.bus, body, other); hit {
					safe = false
					break
				}
			}
			if !safe {
				break
			}
		}
		improved := previous > clearance+.1
		if strandedApproach {
			improved = previous > .1
		}
		if safe && improved {
			return input, true
		}
	}
	// Turn the wheels while stationary before retrying the checked crawl.
	return sim.Clamp(-yaw*3, -.65, .65), false
}
