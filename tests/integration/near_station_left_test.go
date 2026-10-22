package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestUniversityDepartureLeftTurn(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("width%.0f/berserk%v", lw, berserk), func(t *testing.T) {
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				from := w.Stops[7]
				bus := sim.NewBus(from.Pos.X, from.Pos.Y, from.Heading)
				ad := autopilot.NewAutopilotDriver()
				ad.BerserkMode = berserk
				route := &sim.Route{StopIDs: []int{7, 9}, CurrentIndex: 1}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				for _, it := range tls.Intersections {
					it.Phase = sim.PhaseNSGreen
					if it.Center.Y == 1200*sim.MapGridScale {
						it.Phase = sim.PhaseNSLeftGreen
					}
				}
				g := w.JunctionAt(sim.Vec2{X: from.RoadCenter.X, Y: 1200 * sim.MapGridScale}, lw)
				leader := sim.NewBus(from.RoadCenter.X-lw/2, 1200*sim.MapGridScale-g.HalfY-sim.JunctionStopInset-65, math.Pi/2)
				passed := false
				for tick := 0; tick < 7200; tick++ {
					var obstacles []autopilot.VehicleObstacle
					if tick < 1800 {
						obstacles = []autopilot.VehicleObstacle{{Pos: leader.Pos, Heading: leader.Heading, Length: leader.Length, Width: leader.Width}}
					}
					th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, d)
					if tick < 1800 {
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: leader.Pos, Heading: leader.Heading, HalfLen: leader.Length / 2, HalfWid: leader.Width / 2}); hit {
							t.Fatalf("contact with queued bus tick%d", tick)
						}
					}
					if w.CheckBoundaries(bus.Pos, bus.Width/2, lw).Zone == sim.ZoneHardBarrier {
						t.Fatalf("barrier tick%d pos%v yaw%.2f", tick, bus.Pos, bus.Heading)
					}
					if bus.Pos.X > from.RoadCenter.X+700 && math.Cos(bus.Heading) > .9 {
						passed = true
						break
					}
				}
				if !passed {
					t.Fatalf("stalled pos%v heading%.2f target%.1f", bus.Pos, bus.Heading, ad.LastTargetSpeed)
				}
			})
		}
	}
}
