package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestEightToFourLaneBusTurnsStayOnRoad(t *testing.T) {
	for _, kind := range []sim.RoadType{sim.RoadBusBoulevard8Lane} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, turn := range []int{-1, 1} {
				for _, berserk := range []bool{false, true} {
					t.Run(fmt.Sprintf("type%d/h%.2f/turn%d/berserk%v", kind, heading, turn, berserk), func(t *testing.T) {
						const lw = 52.0
						w := &sim.World{Roads: []sim.RoadSegment{
							{P1: sim.Vec2{X: -2400}, P2: sim.Vec2{X: 2400}, Lanes: 8, Type: kind},
							{P1: sim.Vec2{Y: -2400}, P2: sim.Vec2{Y: 2400}, Lanes: 8, Type: kind}}}
						cross := 1
						if math.Abs(math.Sin(heading)) > .5 {
							cross = 0
						}
						w.Roads[cross].Lanes = 4
						w.Roads[cross].Type = sim.RoadAvenue4Lane
						w.RebuildIntersections()
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						outHeading := heading + float64(turn)*math.Pi/2
						out := sim.Vec2{X: math.Cos(outHeading), Y: math.Sin(outHeading)}
						start := sim.NewBusStop(0, "start", in.Mul(-1700), heading, 8, sim.RGBA(0, 0, 0, 255), lw, 0)
						end := sim.NewBusStop(1, "end", out.Mul(1700), outHeading, 4, sim.RGBA(0, 0, 0, 255), lw, 0)
						w.Stops = []*sim.BusStop{start, end}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						bus := sim.NewBus(start.Pos.X, start.Pos.Y, heading)
						ad := autopilot.NewAutopilotDriver()
						ad.BerserkMode = berserk
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						phase := sim.PhaseEWGreen
						if math.Abs(in.Y) > .5 {
							phase = sim.PhaseNSGreen
						}
						if turn == -1 {
							if math.Abs(in.Y) > .5 {
								phase = sim.PhaseNSLeftGreen
							} else {
								phase = sim.PhaseEWLeftGreen
							}
						}
						for _, inter := range tls.Intersections {
							inter.Phase = phase
						}
						line := tls.Intersections[0].GetStopLineDistForHeading(heading, lw)
						crossed := false
						for step := 0; step < 6500; step++ {
							previous := bus.Pos
							th, br, st, hb, doors, docked := ad.ComputeControls(bus, route, w, tls, nil, lw, .016)
							bus.Update(.016, th, br, st, hb, doors)
							if bus.Pos.Distance(previous) > 12 {
								t.Fatal("bus teleported")
							}
							if bus.Pos.Dot(in)+bus.Length/2 >= -line {
								crossed = true
							}
							fwd := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
							right := sim.Vec2{X: -fwd.Y, Y: fwd.X}
							if bus.Pos.Dot(in) > -1100 && bus.Pos.Dot(out) < 1100 {
								for _, a := range []float64{-1, 1} {
									for _, b := range []float64{-1, 1} {
										corner := bus.Pos.Add(fwd.Mul(a * bus.Length / 2)).Add(right.Mul(b * bus.Width / 2))
										if zone := w.CheckBoundaries(corner, 0, lw); zone.Zone == sim.ZoneHardBarrier {
											t.Fatalf("step%d bus%v heading%.2f offset%.1f/%.1f pursuit%v hit %s at%v", step, bus.Pos, bus.Heading, ad.LaneOffset, ad.TargetLaneOffset, ad.LastPursuitPt, zone.ObstacleName, corner)
										}
									}
								}

							}

							if docked {
								if !crossed {
									t.Fatal("bus did not reach intersection")
								}
								return
							}
						}
						t.Fatalf("bus did not reach stop: at%v heading%.2f remaining%.1f", bus.Pos, bus.Heading, ad.LastRemaining)
					})
				}
			}
		}
	}
}

func TestMetropolisBusLaneTurnToUniversityStaysOnRoad(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, offset := range []float64{0, -1, -2} {
			t.Run(fmt.Sprintf("width%.0f/offset%.0f", lw, offset), func(t *testing.T) {
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				from, to := w.Stops[11], w.Stops[7]
				route := &sim.Route{StopIDs: []int{11, 7}, CurrentIndex: 1}
				ad := autopilot.NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, from, to, lw)
				center := sim.Vec2{X: 2600 * sim.MapGridScale, Y: 1200 * sim.MapGridScale}
				bus := sim.NewBus(center.X-(3.5+offset)*lw, center.Y-700, math.Pi/2)
				bus.Speed = 90
				ad.LaneOffset = offset * lw
				ad.TargetLaneOffset = ad.LaneOffset
				ad.OvertakeState = autopilot.OvertakePassing
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				for _, it := range tls.Intersections {
					it.Phase = sim.PhaseNSGreen
				}
				for step := 0; step < 1800; step++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, .016)
					bus.Update(.016, th, br, st, hb, doors)
					if bus.Pos.X < center.X-1100 {
						return
					}
					forward := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					for _, a := range []float64{-1, 1} {
						for _, b := range []float64{-1, 1} {
							corner := bus.Pos.Add(forward.Mul(a * bus.Length / 2)).Add(right.Mul(b * bus.Width / 2))
							if zone := w.CheckBoundaries(corner, 0, lw); zone.Zone == sim.ZoneHardBarrier {
								t.Fatalf("step%d pos%v heading%.2f offset%.1f/%.1f pursuit%v hit%s", step, bus.Pos, bus.Heading, ad.LaneOffset, ad.TargetLaneOffset, ad.LastPursuitPt, zone.ObstacleName)
							}
						}
					}
				}
				t.Fatalf("turn did not finish: pos%v heading%.2f", bus.Pos, bus.Heading)
			})
		}
	}
}
