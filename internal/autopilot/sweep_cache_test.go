package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestSweepBodiesMatchesFullTrailerPrediction(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
		b := sim.NewBus(10, -30, .2)
		b.SetModel(model)
		for i := 0; i < 40; i++ {
			angle := float64(i)*.07 - .8
			front := sim.OBB{Center: b.Pos.Add(sim.Vec2{X: float64(i) * 4, Y: math.Sin(angle) * 80}), Heading: angle, HalfLen: b.Length/2 + 6, HalfWid: b.Width/2 + 4}
			bodies := sweepBodies(b, front)
			for x := -180.; x < 260; x += 13 {
				for y := -160.; y < 160; y += 17 {
					ob := VehicleObstacle{Pos: sim.Vec2{X: x, Y: y}, Heading: angle + .4, Length: 46, Width: 22}
					expected, _, _ := sim.CheckBusAtBody(b, front, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: 23, HalfWid: 11})
					if actual := sweepBodiesCollision(bodies, ob, 0, 0, 0); actual != expected {
						t.Fatalf("model%d sample%d obstacle%v got%v want%v", model, i, ob.Pos, actual, expected)
					}
				}
			}
		}
	}
}
