package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStationAccessShoulderKeepsNormalSpeed(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/model%d", lw, h, model), func(t *testing.T) {
					u := sim.UnitFromAngle(h)
					right := sim.Vec2{X: -u.Y, Y: u.X}
					stop := sim.NewBusStop(0, "station", sim.Vec2{}, h, 6, sim.RGBA(0, 0, 0, 255), lw)
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 6}}, Stops: []*sim.BusStop{stop}}
					for _, tc := range []struct {
						name  string
						along float64
						side  float64
						slow  bool
					}{{"approach", -stop.Length/2 - 80, 1, false}, {"exit", stop.Length/2 + 260 + 60, 1, false}, {"far shoulder", 1200, 1, true}, {"opposite shoulder", 500, -1, true}} {
						t.Run(tc.name, func(t *testing.T) {
							pos := u.Mul(tc.along).Add(right.Mul(tc.side * (3*lw + 15)))
							b := sim.NewBus(pos.X, pos.Y, h)
							b.SetModel(model)
							b.Speed = 200
							b.OnShoulder = true
							g := &Game{World: w, Bus: b}
							if zone := b.CheckBoundaries(w, lw).Zone; zone == sim.ZoneHardBarrier {
								t.Fatalf("fixture hits hard boundary at%v", pos)
							}
							g.applyRoadBoundaries(lw)
							if b.OnShoulder != tc.slow {
								t.Fatalf("shoulder drag%v want%v", b.OnShoulder, tc.slow)
							}
							reference := *b
							reference.OnShoulder = false
							b.Update(.1, .5, 0, 0, false, false)
							reference.Update(.1, .5, 0, 0, false, false)
							if !tc.slow && b.Speed != reference.Speed {
								t.Fatalf("station access lost speed: %.3f vs normal%.3f", b.Speed, reference.Speed)
							}
							if tc.slow && b.Speed >= reference.Speed {
								t.Fatal("ordinary shoulder resistance removed")
							}
						})
					}
				})
			}
		}
	}
}
