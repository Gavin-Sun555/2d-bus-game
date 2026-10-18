package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A taper into a turn lane can be clear even while the road-axis lateral
// distance still puts the bus in the through lane. Check the actual forward
// envelope before letting those two coordinate systems classify a queue.
func (ad *AutopilotDriver) clearsThroughQueueDuringMerge(f autopilotFrame, obs VehicleObstacle) bool {
	if f.world == nil || f.nearRoundabout || len(ad.drivePath) < 2 || math.Abs(obs.Speed) >= 5 {
		return false
	}
	if math.Cos(obs.Heading-f.bus.Heading) < 0.8 || ad.TargetLaneOffset > ad.LaneOffset+1 {
		return false
	}
	_, _, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	if !multi && !six {
		return false
	}
	forward := sim.Vec2{X: math.Cos(obs.Heading), Y: math.Sin(obs.Heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	delta := obs.Pos.Sub(f.bus.Pos)
	ahead := delta.Dot(forward)
	if ahead <= 0 || ahead > 220 {
		return false
	}
	offset := f.bus.Pos.Sub(ad.basePathPosition(f.bus.Pos, f.world, f.bus.Heading, f.laneWidth))
	error := f.bus.Pos.Sub(ad.pointAt(f.s).Add(offset))
	pointAt := func(distance float64) sim.Vec2 {
		return ad.pointAt(f.s + distance).Add(offset).Add(error.Mul(math.Max(0, 1-distance/80)))
	}
	// Apply this exemption only to a path moving farther away from the queue.
	separation := delta.Dot(right)
	movement := pointAt(math.Min(120, ahead)).Sub(f.bus.Pos).Dot(right)
	if separation*movement >= 0 || math.Abs(movement) < 8 {
		return false
	}
	other := sim.OBB{Center: obs.Pos, Heading: obs.Heading, HalfLen: obs.Length/2 + 3, HalfWid: obs.Width/2 + 3}
	current := sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length/2 + 4, HalfWid: f.bus.Width/2 + 4}
	if hit, _, _ := sim.CheckOBBCollision(current, other); hit {
		return false
	}
	for distance := 0.0; distance <= ahead+obs.Length/2; distance += 5 {
		position := pointAt(distance)
		tangent := pointAt(distance + 2).Sub(position)
		// A corner or crossing is handled by the normal obstacle rules.
		if tangent.Normalize().Dot(forward) < 0.8 {
			return false
		}
		body := current
		body.Center = position
		body.Heading = math.Atan2(tangent.Y, tangent.X)
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			return false
		}
	}
	return true
}
