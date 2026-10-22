package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// Reproduce the reported University Park -> Eco Wetlands leg. Pause the real
// bus in the transverse part of its turn, then release it without resetting its
// path, heading or steering actuator. It must leave on the correct road.
func TestUniversityWetlandsUTurnResumesAfterStop(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, cruise := range []float64{150, 445, 666.6666667} {
			for _, pause := range []bool{false, true} {
				t.Run(fmt.Sprintf("width%.0f/speed%.0f/pause%v", lw, cruise, pause), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					// Keep the original northbound University Park departure for this U-turn regression.
					w.Stops[7].RoadCenter.Y = 900 * sim.MapGridScale
					w.Stops[7].Heading = -math.Pi / 2
					w.Stops[7].UpdateGeometry(lw)
					from := w.Stops[7]
					route := &sim.Route{StopIDs: []int{7, 9}, CurrentIndex: 1}
					bus := sim.NewBus(from.Pos.X, from.Pos.Y, from.Heading)
					bus.MaxSpeedFwd = cruise + 20
					ad := autopilot.NewAutopilotDriver()
					ad.BerserkMode = cruise > 150
					ad.BerserkMaxSpeed = cruise
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					for _, inter := range tls.Intersections {
						inter.Phase = sim.PhaseNSLeftGreen
					}
					center := sim.Vec2{X: from.RoadCenter.X, Y: 600 * sim.MapGridScale}
					g := w.JunctionAt(center, lw)
					bus.Pos = center.Add(sim.Vec2{X: lw / 2, Y: g.HalfY + sim.JunctionStopInset + 150})
					bus.Heading = -math.Pi / 2

					paused, turned, exited := false, false, false
					hold := 0
					idle := 0
					for step := 0; step < 9000; step++ {
						if pause && !paused && bus.Pos.Distance(center) < w.JunctionAt(center, lw).HalfY && math.Abs(sim.NormalizeAngle(bus.Heading-math.Pi)) < .25 {
							paused = true
							hold = 180
						}
						if hold > 0 {
							bus.Speed = 0
							hold--
						}
						th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1.0/60)
						if hold > 0 {
							th = 0
							br = 1
						}
						bus.Update(1.0/60, th, br, st, hb, d)
						if bus.Pos.Distance(center) < w.JunctionAt(center, lw).HalfY && math.Cos(bus.Heading) > -.3 && math.Sin(bus.Heading) > .7 {
							turned = true
						}
						if turned && bus.Pos.Y > center.Y+w.JunctionAt(center, lw).HalfY+bus.Length {
							if bus.Pos.X > center.X {
								t.Fatalf("exit on wrong side: %v", bus.Pos)
							}
							exited = true
							break
						}
						if hold == 0 && bus.Speed < 1 && bus.Pos.Distance(center) < w.JunctionAt(center, lw).HalfY-60 {
							idle++
						} else {
							idle = 0
						}
						if idle > 180 {
							t.Fatalf("stuck in U-turn: pos%v heading%.2f target%.1f", bus.Pos, bus.Heading, ad.LastTargetSpeed)
						}
						if result := w.CheckBoundaries(bus.Pos, bus.Width/2, lw); result.Zone == sim.ZoneHardBarrier {
							t.Fatalf("hit barrier step%d: %v", step, bus.Pos)
						}
					}
					if !turned || !exited || (pause && !paused) {
						t.Fatalf("did not complete turn: paused%v turned%v exited%v pos%v", paused, turned, exited, bus.Pos)
					}
				})
			}
		}
	}
}
