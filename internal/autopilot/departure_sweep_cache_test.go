package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// Compare against the original full articulated prediction for every obstacle,
// including repeated queries, both sampling grids and a fresh changed pose.
func TestDepartureSweepCachePreservesFullCollisionResult(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
		for _, tail := range []float64{-.5, 0, .5} {
			b := sim.NewBus(20, -30, .2)
			b.SetModel(model)
			b.TrailerHeading = tail
			for pass := 0; pass < 2; pass++ {
				b.Pos.X += float64(pass) * 7
				var cache departureSweepCache
				f := autopilotFrame{bus: b, departureCache: &cache}
				for kind := 0; kind < 2; kind++ {
					for i := 0; i <= 12; i++ {
						distance := float64(i) * 5
						if kind == 1 {
							distance = float64(i) * 4
						}
						front := sim.OBB{Center: b.Pos.Add(sim.Vec2{X: distance, Y: math.Sin(distance/80) * 20}), Heading: b.Heading + distance/400, HalfLen: b.Length/2 + 2, HalfWid: b.Width/2 + 2}
						for x := -160.; x <= 160; x += 40 {
							for y := -60.; y <= 100; y += 20 {
								other := sim.OBB{Center: sim.Vec2{X: x, Y: y}, Heading: tail + .4, HalfLen: 25, HalfWid: 13}
								want, _, _ := sim.CheckBusAtBody(b, front, other)
								if got := departureBodyCollision(f, kind, i, front, other); got != want {
									t.Fatalf("model%d tail%.2f grid%d sample%d obstacle%v got%v want%v", model, tail, kind, i, other.Center, got, want)
								}
							}
						}
					}
				}
			}
		}
	}
}
