package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSkippedParkedBayUsesPhysicalDeparture(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for model := sim.BusStandard; model < sim.BusModelCount; model++ {
			t.Run(fmt.Sprintf("heading%.2f/model%d", h, model), func(t *testing.T) {
				w, u, _ := serviceTestWorld(h, 52)
				stop := w.Stops[1]
				p := stop.FrontBerthPosition()
				b := sim.NewBus(p.X, p.Y, h)
				b.SetModel(model)
				if !stop.ContainsBus(b) {
					t.Fatal("fixture is not inside the platform")
				}
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.ContinuePastSkippedStop(b, stop, 52)
				if ad.skippedDeparture != nil || !ad.departureFromDock {
					t.Fatal("parked bus was treated as already on the roadway")
				}
				r := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 2}
				start := b.Pos
				for tick := 0; tick < 1200; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(b, r, w, nil, nil, 52, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if result := b.CheckBoundaries(w, 52); result.Zone == sim.ZoneHardBarrier {
						t.Fatalf("departure hit a barrier at tick%d: %+v", tick, result)
					}
					if b.Pos.Sub(start).Dot(u) > 650 {
						return
					}
				}
				t.Fatalf("skipped bay did not depart: distance%.1f", b.Pos.Distance(start))
			})
		}
	}
}
