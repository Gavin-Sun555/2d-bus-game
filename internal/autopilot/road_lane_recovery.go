package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
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

// A stopped, slightly oblique bus cannot repair its body clearance without
// moving. Permit only a slow sweep whose clearance never gets worse, with all
// vehicle corners and nearby traffic checked throughout the recovery.
func (ad *AutopilotDriver) roadLaneRecovery(f autopilotFrame, steer, heading, center float64, ew bool) (float64, bool) {
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	clearance := roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew)
	if f.speedAbs > 20 || math.Abs(yaw) > .4 || clearance < -math.Min(8, f.laneWidth*.12) {
		return steer, false
	}
	const recoverySpeed = 20.
	ad.roadRecoverySpeed = recoverySpeed
	candidates := []float64{sim.Clamp(steer, -.65, .65), 0, sim.Clamp(-yaw*3, -.65, .65), -.65, .65}
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
			angle += recoverySpeed / f.bus.WheelBase * math.Tan(steering) * dt
			pos = pos.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(recoverySpeed * dt))
			next := roadMedianClearance(f.bus, pos, angle, heading, center, ew)
			if next < math.Min(previous, 2)-.002 {
				safe = false
				break
			}
			previous = next
			body := sim.OBB{Center: pos, Heading: angle, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
			forward := sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			for _, a := range []float64{-1, 1} {
				for _, b := range []float64{-1, 1} {
					corner := pos.Add(forward.Mul(a * body.HalfLen)).Add(right.Mul(b * body.HalfWid))
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
				other := sim.OBB{Center: ob.Pos.Add(sim.Vec2{X: math.Cos(ob.Heading), Y: math.Sin(ob.Heading)}.Mul(ob.Speed * elapsed)), Heading: ob.Heading, HalfLen: ob.Length/2 + 1, HalfWid: ob.Width/2 + 1}
				if ob.JunctionTurn != nil {
					other.Center, other.Heading = ob.JunctionTurn.Predict(math.Max(0, ob.Speed) * elapsed)
				}
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					safe = false
					break
				}
			}
			if !safe {
				break
			}
		}
		if safe && previous > clearance+.1 {
			return input, true
		}
	}
	// Turn the wheels while stationary before retrying a sweep. This is only
	// for a small existing intrusion at low speed, never a high-speed override.
	return sim.Clamp(-yaw*3, -.65, .65), false
}
