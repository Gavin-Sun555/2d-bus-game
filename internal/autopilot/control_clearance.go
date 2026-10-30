package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A stopped rear bus beside a leading bus can touch only the extra sweep
// padding. Stopping the leader there also prevents the rear bus from merging.
// Permit only a low-speed, same-direction leader to use its actual clearance;
// never waive a physical intersection or the checks against any other vehicle.
func controlPaddingRelease(f autopilotFrame, ob VehicleObstacle, steer, speed, horizon float64) bool {
	if ob.Bus == nil || math.Abs(ob.Speed) > 1 || f.speedAbs > 45 || math.Abs(steer) > .02 || math.Cos(f.bus.Heading-ob.Bus.Heading) < .98 {
		return false
	}
	forward := sim.UnitFromAngle(f.bus.Heading)
	delta := ob.Bus.Pos.Sub(f.bus.Pos)
	behindRear := delta.Dot(forward) < -(f.bus.OverallLength()+ob.Bus.OverallLength())/2 && math.Cos(f.bus.TailHeading()-ob.Bus.Heading) > .98
	currentBody := sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length / 2, HalfWid: f.bus.Width / 2}
	if delta.Dot(forward) >= -8 || (!behindRear && math.Abs(delta.X*forward.Y-delta.Y*forward.X) < (f.bus.Width+ob.Bus.Width)*.4) || sweepBodiesCollision(sweepBodies(f.bus, currentBody), ob, 0, 0, 0) {
		return false
	}
	// Reproduce the complete actuator sweep with a smaller quarter-pixel skin,
	// including the articulated tail. This is not an adjacent-lane exemption.
	pos, heading, wheel := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
	for elapsed := 0.; elapsed <= horizon; elapsed += .04 {
		if elapsed == 0 && behindRear {
			continue
		}
		if elapsed > 0 {
			rate := f.bus.SteerTurnSpeed
			if steer == 0 {
				rate = f.bus.SteerReturnSpd
			}
			wheel += sim.Clamp(steer*f.bus.MaxSteerAngle-wheel, -rate*.04, rate*.04)
			heading += speed / f.bus.WheelBase * math.Tan(wheel) * .04
			pos = pos.Add(sim.UnitFromAngle(heading).Mul(speed * .04))
		}
		front := sim.OBB{Center: pos, Heading: heading, HalfLen: f.bus.Length/2 + .25, HalfWid: f.bus.Width/2 + .25}
		if sweepBodiesCollision(sweepBodies(f.bus, front), ob, math.Max(0, ob.Speed)*elapsed, .25, .25) {
			return false
		}
	}
	return true
}
