package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestEightLaneBusesUseLegalTurnAndThroughLanes(t *testing.T) {
	testEightLaneBusMovements(t, false)
}

func TestEightLaneBusesTurnRightOnRedWithoutFine(t *testing.T) {
	testEightLaneBusMovements(t, true)
}

func testEightLaneBusMovements(t *testing.T, rightOnRed bool) {
	t.Helper()
	for _, kind := range []sim.RoadType{sim.RoadBoulevard8Lane, sim.RoadBusBoulevard8Lane} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, turn := range []int{-1, 0, 1} {
				if rightOnRed && turn != 1 {
					continue
				}
				for _, berserk := range []bool{false, true} {
					t.Run(fmt.Sprintf("type%d/h%.2f/turn%d/berserk%v", kind, heading, turn, berserk), func(t *testing.T) {
						const lw = 52.0
						w := &sim.World{Roads: []sim.RoadSegment{
							{P1: sim.Vec2{X: -2400}, P2: sim.Vec2{X: 2400}, Lanes: 8, Type: kind},
							{P1: sim.Vec2{Y: -2400}, P2: sim.Vec2{Y: 2400}, Lanes: 8, Type: kind}}}
						w.RebuildIntersections()
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						outHeading := heading + float64(turn)*math.Pi/2
						out := sim.Vec2{X: math.Cos(outHeading), Y: math.Sin(outHeading)}
						start := sim.NewBusStop(0, "start", in.Mul(-1700), heading, 8, sim.RGBA(0, 0, 0, 255), lw, 0)
						end := sim.NewBusStop(1, "end", out.Mul(1700), outHeading, 8, sim.RGBA(0, 0, 0, 255), lw, 0)
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
						if rightOnRed {
							phase = sim.PhaseNSGreen
							if math.Abs(in.Y) > .5 {
								phase = sim.PhaseEWGreen
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
							if bad, notice := tls.CheckPlayerViolation(bus, lw); bad {
								t.Fatalf("lawful bus movement fined: %s", notice)
							}
							if bus.Pos.Distance(previous) > 12 {
								t.Fatal("bus teleported")
							}
							if !crossed && bus.Pos.Dot(in)+bus.Length/2 >= -line {
								crossed = true
								lateral := bus.Pos.Dot(sim.Vec2{X: -in.Y, Y: in.X}) / lw
								if turn == 1 && (lateral < 3 || lateral > 4) {
									t.Fatalf("right turn from lane offset %.2f", lateral)
								}
								if turn == -1 && lateral > 1 && kind != sim.RoadBusBoulevard8Lane {
									t.Fatalf("left turn from lane offset %.2f", lateral)
								}
								if turn == 0 && (lateral < 1 || lateral > 3) && kind != sim.RoadBusBoulevard8Lane {
									t.Fatalf("through movement uses a turn lane: %.2f", lateral)
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
