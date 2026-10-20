package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestOuterRoundaboutExitUsesAvailableRoadMouth(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, overshoot := range []float64{.13, .18} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/overshoot%.2f", lw, heading, overshoot), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					rb := &w.Roundabouts[0]
					in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					out := sim.Vec2{X: -in.Y, Y: in.X}
					from := sim.NewBusStop(0, "from", rb.Center.Sub(in.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", rb.Center.Add(out.Mul(800)), heading+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.PrepareLeg(w, from, to, lw)
					index := ad.roundaboutExitIndex(rb, 0)
					if index < 0 {
						t.Fatal("missing route exit")
					}
					exit := ad.drivePath[index].Sub(rb.Center)
					angle := math.Atan2(exit.Y, exit.X) - overshoot
					pos := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(roundaboutOuterLaneRadius(rb)))
					bus := sim.NewBus(pos.X, pos.Y, angle-math.Pi/2+.6)
					bus.Speed = 60

					original := append([]sim.Vec2(nil), ad.drivePath...)
					blocked := VehicleObstacle{Pos: rb.Center.Add(out.Mul(300)), Heading: heading + math.Pi/2, Length: 200, Width: lw * 5}
					if ad.tryDirectRoundaboutExit(bus, rb, []VehicleObstacle{blocked}, index, lw) {
						t.Fatal("exit crossed traffic blocking the road mouth")
					}
					if len(ad.drivePath) != len(original) || ad.drivePath[0] != original[0] {
						t.Fatal("rejected exit changed route")
					}

					if !ad.retryRoundaboutExit(bus, rb, nil, lw, ad.cumLen[index], 60) || ad.directRoundaboutExit == nil {
						t.Fatal("clear outer-lane exit unnecessarily replaced with a full lap")
					}
					if bus.Pos != pos {
						t.Fatal("exit replanning moved the bus")
					}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					lastAngle, traveled := angle, 0.0
					docked := false
					for step := 0; step < 1600; step++ {
						th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, nil, lw, 1.0/60)
						bus.Update(1.0/60, th, br, st, hb, doors)
						if bus.Pos.Distance(rb.Center) < rb.Radius+20 {
							current := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
							traveled += math.Max(0, sim.NormalizeAngle(lastAngle-current))
							lastAngle = current
						}
						body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
						if !ad.roundaboutExitBodyOnRoad(body, rb, lw) {
							t.Fatalf("bus left road at step%d pos%v", step, bus.Pos)
						}
						if dock {
							docked = true
							break
						}
					}
					if !docked || traveled > 1.5 {
						t.Fatalf("unnecessary lap or failed departure: docked%v traveled%.2f pos%v", docked, traveled, bus.Pos)
					}
				})
			}
		}
	}
}
