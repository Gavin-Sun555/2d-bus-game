package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A turning bus sweeps across more than its straight-road width. Check this
// before adjacent-lane exemptions can discard a car on the outgoing road.
func (ad *AutopilotDriver) junctionSweptSpeed(f autopilotFrame) float64 {
	if f.world == nil || len(f.obstacles) == 0 || len(ad.drivePath) < 2 || !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return math.Inf(1)
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		u := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		p = p.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(ad.LaneOffset))
		return p, math.Atan2(u.Y, u.X)
	}
	base, angle := pointAt(0)
	errorPos := f.bus.Pos.Sub(base)
	errorHeading := sim.NormalizeAngle(f.bus.Heading - angle)
	limit := math.Inf(1)
	lookahead := math.Max(140, f.speedAbs*f.speedAbs/(2*180)+40)
	for _, ob := range f.obstacles {
		radius := lookahead + math.Abs(ob.Speed)*1.5 + (f.bus.Length+ob.Length)/2
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > radius*radius {
			continue
		}
		for distance := 0.; distance <= lookahead; distance += 6 {
			pos, heading := pointAt(distance)
			body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: f.bus.Length/2 + 6, HalfWid: f.bus.Width/2 + 4}
			arrival := distance / math.Max(25, f.speedAbs)
			other := sim.OBB{Center: ob.Pos.Add(sim.Vec2{X: math.Cos(ob.Heading), Y: math.Sin(ob.Heading)}.Mul(ob.Speed * arrival)), Heading: ob.Heading, HalfLen: ob.Length/2 + 4, HalfWid: ob.Width/2 + 2}
			if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
				limit = math.Min(limit, math.Sqrt(2*180*math.Max(0, distance-30)))
				break
			}
		}
	}
	return limit
}
