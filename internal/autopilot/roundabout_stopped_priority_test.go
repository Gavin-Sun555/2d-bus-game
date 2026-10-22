package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestEntryYieldsOutsideRingToStoppedCirculatingCar(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("berserk%v/h%.2f", berserk, h), func(t *testing.T) {
				w := sim.NewWorld(52, sim.MapThemeMetropolis)
				rb := w.Roundabouts[0]
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", rb.Center.Sub(u.Mul(800)), h, 4, sim.RGBA(0, 0, 0, 255), 52), sim.NewBusStop(1, "to", rb.Center.Add(u.Mul(800)), h, 4, sim.RGBA(0, 0, 0, 255), 52)}
				p := rb.Center.Sub(u.Mul(360)).Add(r.Mul(78))
				bus := sim.NewBus(p.X, p.Y, h)
				bus.Speed = 90
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.DisallowOncomingOvertake = true
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				a := h + math.Pi - math.Asin(78./214) - .3
				ob := VehicleObstacle{Pos: rb.Center.Add(sim.Vec2{X: 214 * math.Cos(a), Y: 214 * math.Sin(a)}), Heading: a - math.Pi/2, Length: 46, Width: 22}
				for tick := 0; tick < 180; tick++ {
					th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{ob}, 52, 1./60)
					bus.Update(1./60, th, br, st, hb, d)
					yaw := bus.Heading - h
					front := bus.Length/2*math.Abs(math.Cos(yaw)) + bus.Width/2*math.Abs(math.Sin(yaw))
					if bus.Pos.Sub(rb.Center).Dot(u)+front > -rb.Radius-10 {
						t.Fatalf("bus entered blocked mouth: %v", bus.Pos)
					}
				}
				for tick := 0; tick < 1800; tick++ {
					th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, nil, 52, 1./60)
					bus.Update(1./60, th, br, st, hb, d)
					if bus.Pos.Sub(rb.Center).Dot(u) > 400 {
						return
					}
				}
				t.Fatal("bus did not resume after stopped ring car left")
			})
		}
	}
}
