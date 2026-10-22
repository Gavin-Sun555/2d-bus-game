package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// controlSweptSpeed checks the physical pose produced by the steering actuator,
// including the nose and tail during a lane change or turn exit. Lane-center
// exemptions in ordinary following are not enough when the bus is yawed.
func (ad *AutopilotDriver) controlSweptSpeed(f autopilotFrame, steer float64) float64 {
	if f.world == nil || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return math.Inf(1)
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return math.Inf(1)
		}
	}
	speed := math.Max(20, f.speedAbs)
	horizon := math.Min(1.2, .20+speed/180)
	limit := math.Inf(1)
	for _, ob := range f.obstacles {
		reach := speed*horizon + math.Abs(ob.Speed)*horizon + (f.bus.Length+ob.Length)/2 + 20
		d := ob.Pos.Sub(f.bus.Pos)
		if d.Dot(d) > reach*reach {
			continue
		}
		pos, heading, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
		for elapsed := .04; elapsed <= horizon; elapsed += .04 {
			rate := f.bus.SteerTurnSpeed
			if steer == 0 {
				rate = f.bus.SteerReturnSpd
			}
			steering += sim.Clamp(steer*f.bus.MaxSteerAngle-steering, -rate*.04, rate*.04)
			heading += speed / f.bus.WheelBase * math.Tan(steering) * .04
			pos = pos.Add(sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}.Mul(speed * .04))
			other := sim.OBB{Center: ob.Pos.Add(sim.Vec2{X: math.Cos(ob.Heading), Y: math.Sin(ob.Heading)}.Mul(ob.Speed * elapsed)), Heading: ob.Heading, HalfLen: ob.Length/2 + 1, HalfWid: ob.Width/2 + 1}
			if predicted, ok := sim.PredictJunctionBody(ob.JunctionCorridor, math.Max(0, ob.Speed)*elapsed); ok {
				other.Center, other.Heading = predicted.Center, predicted.Heading
			}
			if ob.JunctionTurn != nil {
				other.Center, other.Heading = ob.JunctionTurn.Predict(math.Max(0, ob.Speed) * elapsed)
			}
			body := sim.OBB{Center: pos, Heading: heading, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
			if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
				limit = math.Min(limit, math.Sqrt(2*180*math.Max(0, speed*elapsed-18)))
				break
			}
		}
	}
	return limit
}
