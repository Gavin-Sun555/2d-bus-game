package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A road-lane return can consume the space reserved by the station taper.
// Join the two maneuvers before their overlap rather than driving backwards
// to the taper start. The preceding turn and its separated exit lane stay intact.
func joinFoldedArrival(w *sim.World, path []sim.Vec2, destination sim.Vec2, heading, lw float64) ([]sim.Vec2, float64, bool) {
	n := len(path)
	if n < 5 || path[n-1].Distance(destination) > .01 {
		return path, 0, false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	fold := path[n-3].Sub(path[n-4])
	if fold.Dot(u) >= -1 || math.Abs(fold.Dot(r)) > .01 || math.Abs(path[n-2].Sub(destination).Dot(r)) > .01 {
		return path, 0, false
	}
	runway := 330.
	for attempt := 0; attempt < 3; attempt++ {
		for i := n - 4; i > 0; i-- {
			a, b := path[i-1], path[i]
			delta := b.Sub(a)
			before, after := destination.Sub(a).Dot(u), destination.Sub(b).Dot(u)
			if before < runway || before-after < 1 || delta.Normalize().Dot(u) < .995 {
				continue
			}
			run := math.Max(runway, after)
			anchor := a.Add(delta.Mul((before - run) / (before - after)))
			needed := math.Max(330, math.Abs(destination.Sub(anchor).Dot(r))*2.2)
			if needed > run+.01 {
				runway = needed
				break
			}
			// Do not cut a turn or start the lateral arrival with the tail in its box.
			if w != nil && w.InsideJunction(anchor, lw, 150) {
				return path, 0, false
			}
			prefix := 0.
			for j := 1; j < i; j++ {
				prefix += path[j].Distance(path[j-1])
			}
			prefix += anchor.Distance(a)
			result := append([]sim.Vec2(nil), path[:i]...)
			result = append(result, gentleDockDeparture(anchor, anchor, destination, u)...)
			return dedupePath(result), prefix, true
		}
	}
	return path, 0, false
}
