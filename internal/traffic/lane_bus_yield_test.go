package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLaneMergePriorityDoesNotStopUnrelatedOrUnsafeCars(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, scenario := range []string{"waiting behind", "approaching safely", "alongside", "ahead", "fast and close", "other lane", "opposite", "expired", "manual takeover"} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, scenario), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				bus := sim.NewBus(r.X*26, r.Y*26, h)
				bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: r.Mul(78), Heading: h, Remaining: .3}
				car := &sim.TrafficCar{Pos: u.Mul(-110).Add(r.Mul(78)), Heading: h, Length: 48, Width: 22}
				switch scenario {
				case "approaching safely":
					car.Pos = u.Mul(-400).Add(r.Mul(78))
					car.Speed = 100
				case "alongside":
					car.Pos = r.Mul(78)
				case "ahead":
					car.Pos = u.Mul(110).Add(r.Mul(78))
				case "fast and close":
					car.Speed = 200
				case "other lane":
					car.Pos = car.Pos.Add(r.Mul(52))
				case "opposite":
					car.Heading += math.Pi
				case "expired":
					bus.LaneMergeRequest.Remaining = 0
				case "manual takeover":
					bus.Update(.4, 0, 0, 0, false, false)
				}
				f := carFrame{car: car, world: &sim.World{}, buses: []*sim.Bus{bus}, laneWidth: 52}
				target := (&TrafficManager{}).busMergeYieldSpeed(f, 100)
				wantYield := scenario == "waiting behind" || scenario == "approaching safely"
				if wantYield {
					want := 0.0
					if scenario == "approaching safely" {
						want = 100 // Close the excess gap while retaining merge priority.
					}
					if target != want || car.YieldingToBus != bus {
						t.Fatal("eligible rear car did not reserve the merge gap")
					}
					bus.LaneMergeRequest = nil
					if (&TrafficManager{}).busMergeYieldSpeed(f, 100) != 100 || car.YieldingToBus != nil {
						t.Fatal("yield persisted after merge")
					}
				} else if target != 100 || car.YieldingToBus != nil {
					t.Fatalf("unrelated/unsafe car forced to yield: target%.1f", target)
				}
			})
		}
	}
}
