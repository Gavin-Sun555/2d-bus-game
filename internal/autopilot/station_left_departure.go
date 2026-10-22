package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// When a left-lane taper begins immediately after the bay, the ordinary
// pull-out would join it almost sideways. Join the first settled approach
// instead, retaining forward tangents at both ends. Never consume the turn arc
// or compress the curve into less than twice its lateral displacement.
func closeLeftDeparture(raw []sim.Vec2, forward sim.Vec2, lw float64) ([]sim.Vec2, int) {
	if len(raw) < 5 || math.Abs(raw[1].Sub(raw[0]).Dot(forward)) > 10 || raw[2].Sub(raw[1]).Dot(forward) > 160 {
		return nil, 0
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	for i := 2; i < len(raw)-1; i++ {
		tangent := raw[i+1].Sub(raw[i]).Normalize()
		if tangent.Dot(forward) < .5 {
			break
		}
		if tangent.Dot(forward) < .995 || raw[i].Sub(raw[1]).Dot(right) > -.8*lw {
			continue
		}
		delta := raw[i].Sub(raw[0])
		distance := delta.Dot(forward)
		if distance < math.Max(240, 2*math.Abs(delta.Dot(right))) {
			continue
		}
		// A U-turn has a separate departure/turn controller. Keep its prefix
		// intact, including when a saved bus resumes partway through the arc.
		if !leftQuarterTurnAhead(raw[i:], forward, right) {
			return nil, 0
		}
		points := make([]sim.Vec2, 0, 25)
		for step := 0; step <= 24; step++ {
			u := float64(step) / 24
			lateral := delta.Dot(right) * u * u * (3 - 2*u)
			points = append(points, raw[0].Add(forward.Mul(distance*u)).Add(right.Mul(lateral)))
		}
		return points, i + 1
	}
	return nil, 0
}

// Recognize the first settled outgoing road rather than a single tangent in an arc.
func leftQuarterTurnAhead(path []sim.Vec2, forward, right sim.Vec2) bool {
	for i := 0; i < len(path)-1; i++ {
		segment := path[i+1].Sub(path[i])
		tangent := segment.Normalize()
		if tangent.Dot(forward) < -.7 {
			return false
		}
		if segment.Length() > 100 && tangent.Dot(right) < -.98 {
			return true
		}
	}
	return false
}
