package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func wideLeftExitWorld(lw, heading float64, reserved bool) (*sim.World, sim.Vec2, sim.Vec2, float64) {
	in := sim.Vec2{X: math.Round(math.Cos(heading)), Y: math.Round(math.Sin(heading))}
	right := sim.Vec2{X: -in.Y, Y: in.X}
	out := right.Mul(-1)
	rotate := func(p sim.Vec2) sim.Vec2 { return in.Mul(p.X).Add(right.Mul(p.Y)) }
	typ := sim.RoadBoulevard8Lane
	offset := 2.5 * lw
	if reserved {
		typ = sim.RoadBusBoulevard8Lane
		offset = 3.5 * lw
	}
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: rotate(sim.Vec2{X: -3000}), P2: rotate(sim.Vec2{X: 3000}), Lanes: 8, Type: typ},
		{P1: rotate(sim.Vec2{Y: -3000}), P2: rotate(sim.Vec2{Y: 3000}), Lanes: 8, Type: typ},
	}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "from", in.Mul(-2200), heading, 8, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "to", out.Mul(2200), heading-math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw),
	}
	return w, in, out, offset
}

// A left-turn exit must be aligned before meeting the adjacent through queue.
// The old route exited in lane zero and crossed two/three lanes outside the box.
func TestEightLaneLeftExitAlignedBeforeMouth(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, reserved := range []bool{false, true} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/reserved%v", lw, h, reserved), func(t *testing.T) {
					w, _, out, offset := wideLeftExitWorld(lw, h, reserved)
					path := buildAutopilotDrivePath(w, w.Stops[0], w.Stops[1], lw, true, true)
					g := w.JunctionAt(sim.Vec2{}, lw)
					mouth := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
					right := sim.Vec2{X: -out.Y, Y: out.X}
					checked := false
					for i := 1; i < len(path); i++ {
						p := path[i]
						d := p.Dot(out)
						if d < mouth || d > mouth+400 {
							continue
						}
						checked = true
						tangent := p.Sub(path[i-1]).Normalize()
						if math.Abs(p.Dot(right)-offset) > 1 || tangent.Dot(out) < .995 {
							t.Fatalf("exit still crossing lanes at %.1f: lateral %.1f want %.1f tangent %v", d, p.Dot(right), offset, tangent)
						}
					}
					if !checked {
						t.Fatal("missing exit samples")
					}
				})
			}
		}
	}
}

func TestEightLaneLeftPassesAdjacentStoppedCar(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, reserved := range []bool{false, true} {
				for _, berserk := range []bool{false, true} {
					t.Run(fmt.Sprintf("w%.0f/h%.2f/reserved%v/berserk%v", lw, h, reserved, berserk), func(t *testing.T) {
						w, in, out, offset := wideLeftExitWorld(lw, h, reserved)
						rightIn := sim.Vec2{X: -in.Y, Y: in.X}
						rightOut := sim.Vec2{X: -out.Y, Y: out.X}
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						it := tls.Intersections[0]
						it.Phase = sim.PhaseEWLeftGreen
						if math.Abs(in.Y) > .5 {
							it.Phase = sim.PhaseNSLeftGreen
						}
						g := w.JunctionAt(sim.Vec2{}, lw)
						mouth := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
						entryOffset := .5 * lw
						if reserved {
							entryOffset = 3.5 * lw
						}
						pos := in.Mul(-it.GetStopLineDistForHeading(h, lw) - 75).Add(rightIn.Mul(entryOffset))
						bus := sim.NewBus(pos.X, pos.Y, h)
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						ad.DisallowOncomingOvertake = true
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						car := VehicleObstacle{Pos: out.Mul(mouth + 220).Add(rightOut.Mul(offset - lw)), Heading: h - math.Pi/2, Length: 48, Width: 22}
						cleared := false
						for tick := 0; tick < 2400; tick++ {
							tls.Update(1. / 60)
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{car}, lw, 1./60)
							bus.Update(1./60, th, br, st, hb, doors)
							if d := bus.Pos.Dot(out); d >= mouth+bus.Length/2 && d < mouth+120 {
								if lateral := bus.Pos.Dot(rightOut); math.Abs(lateral-offset) > 6 || math.Cos(bus.Heading-car.Heading) < .99 {
									t.Fatalf("bus left junction before aligning: pos%v lateral%.1f want%.1f heading%.2f", bus.Pos, lateral, offset, bus.Heading)
								}
							}
							body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							other := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
							if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
								t.Fatalf("left exit struck adjacent car at %v", bus.Pos)
							}
							if bus.Pos.Dot(out) > mouth+650 {
								cleared = true
								break
							}
						}
						if !cleared {
							t.Fatalf("left exit stuck at %v heading%.2f diagnosis %+v", bus.Pos, bus.Heading, ad.DiagnoseStop(bus, route, w, tls, []VehicleObstacle{car}, lw))
						}
					})
				}
			}
		}
	}
}

// The median left-turn lane and reserved curb lane both have a left arrow.
// Three curb buses must not reserve the median bus's separate outgoing lane.
func TestEightLaneParallelLeftWithThreeCurbBuses(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, choose := range []bool{false, true} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/choose%v", lw, h, choose), func(t *testing.T) {
					w, in, out, _ := wideLeftExitWorld(lw, h, true)
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.LeftGreenDuration = 30
					green := sim.PhaseEWLeftGreen
					it.Phase = sim.PhaseEWGreen
					if math.Abs(in.Y) > .5 {
						green = sim.PhaseNSLeftGreen
						it.Phase = sim.PhaseNSGreen
					}
					line := it.GetStopLineDistForHeading(h, lw)
					right := sim.Vec2{X: -in.Y, Y: in.X}
					g := w.JunctionAt(sim.Vec2{}, lw)
					mouth := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
					var buses []*sim.Bus
					var drivers []*AutopilotDriver
					redTicks := 180
					if choose {
						redTicks = 600
					}
					for i := 0; i < 4; i++ {
						offset := 3.5 * lw
						distance := line + 64 + float64(max(0, i-1))*135
						if i == 0 {
							offset = .5 * lw
							if choose {
								offset = 3.5 * lw
								distance = line + 600
							}
						}
						p := in.Mul(-distance).Add(right.Mul(offset))
						b := sim.NewBus(p.X, p.Y, h)
						ad := NewAutopilotDriver()
						ad.DisallowOncomingOvertake = true
						if i == 0 {
							ad.BerserkMode = true
							ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
							ad.ordinaryBusLeft = !choose
							ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
						}
						buses = append(buses, b)
						drivers = append(drivers, ad)
					}
					routes := make([]*sim.Route, len(buses))
					for i := range routes {
						routes[i] = &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					}
					route := routes[0]
					overlapTicks := 0
					beganWithQueue := false
					cleared := false
					for tick := 0; tick < 3000; tick++ {
						if tick == redTicks {
							if !drivers[0].ordinaryBusLeft || buses[0].Pos.Dot(right) > lw {
								obs := []VehicleObstacle{}
								for _, b := range buses[1:] {
									obs = append(obs, VehicleObstacle{Pos: b.Pos, Heading: b.Heading, Speed: b.Speed, Length: b.Length, Width: b.Width, JunctionCorridor: b.JunctionCorridor})
								}
								t.Fatalf("did not join clear median left lane: pos%v offset%.1f ordinary%v diagnosis%+v", buses[0].Pos, drivers[0].LaneOffset, drivers[0].ordinaryBusLeft, drivers[0].DiagnoseStop(buses[0], route, w, tls, obs, lw))
							}
							it.Phase = green
							it.Timer = 0
						}
						if tick >= redTicks {
							tls.Update(1. / 60)
						}
						for i, b := range buses {
							obs := make([]VehicleObstacle, 0, 3)
							for j, other := range buses {
								if i != j {
									obs = append(obs, VehicleObstacle{Pos: other.Pos, Heading: other.Heading, Speed: other.Speed, Length: other.Length, Width: other.Width, JunctionCorridor: other.JunctionCorridor, YieldingTo: other.YieldingToBus})
								}
							}
							th, br, st, hb, doors, _ := drivers[i].ComputeControls(b, routes[i], w, tls, obs, lw, 1./60)
							b.Update(1./60, th, br, st, hb, doors)
							if tick < redTicks && b.Pos.Dot(in)+b.Length/2 > -line+1 {
								t.Fatal("left turn crossed on through green")
							}
						}
						for i, b := range buses {
							for j := i + 1; j < len(buses); j++ {
								other := buses[j]
								if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}); hit {
									t.Fatalf("parallel buses %d/%d collide at tick%d: %v/%v", i, j, tick, b.Pos, other.Pos)
								}
							}
						}
						if tick >= redTicks && w.InsideJunction(buses[0].Pos, lw, -60) && w.InsideJunction(buses[1].Pos, lw, -60) {
							overlapTicks++
						}
						if buses[0].Pos.Dot(in) > -line+80 && buses[3].Pos.Dot(out) < mouth {
							beganWithQueue = true
						}
						if buses[0].Pos.Dot(out) > mouth+150 {
							cleared = true
							break
						}
					}
					if !cleared || !beganWithQueue || overlapTicks < 10 {
						t.Fatalf("median bus waited for entire curb queue: cleared%v simultaneous%d pos%v heading%.2f diagnosis%+v", cleared, overlapTicks, buses[0].Pos, buses[0].Heading, drivers[0].DiagnoseStop(buses[0], route, w, tls, nil, lw))
					}
				})
			}
		}
	}
}

func TestBusLaneLeftRecentersWithRearCarAndBus(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, fleet := range []bool{false, true} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/fleet%v", lw, h, fleet), func(t *testing.T) {
					w, in, out, _ := wideLeftExitWorld(lw, h, true)
					right := sim.Vec2{X: -in.Y, Y: in.X}
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.Phase = sim.PhaseEWGreen
					green := sim.PhaseEWLeftGreen
					if math.Abs(in.Y) > .5 {
						it.Phase = sim.PhaseNSGreen
						green = sim.PhaseNSLeftGreen
					}
					it.LeftGreenDuration = 30
					line := it.GetStopLineDistForHeading(h, lw)
					pos := in.Mul(-line - 110).Add(right.Mul(3.1 * lw))
					bus := sim.NewBus(pos.X, pos.Y, h-.12)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.DisallowOncomingOvertake = fleet
					ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
					ad.holdLaneOffset(-.4 * lw)
					obs := []VehicleObstacle{
						{Pos: pos.Sub(in.Mul(95)).Add(right.Mul(-.6 * lw)), Heading: h, Length: 48, Width: 22},
						{Pos: pos.Sub(in.Mul(145)).Add(right.Mul(.4 * lw)), Heading: h, Length: 96, Width: 36},
					}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					cleared := false
					for tick := 0; tick < 2400; tick++ {
						if tick == 180 {
							it.Phase = green
							it.Timer = 0
						}
						if tick >= 180 {
							tls.Update(1. / 60)
						}
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						if tick < 180 && bus.Pos.Dot(in)+bus.Length/2 > -line+1 {
							t.Fatal("crossed on through green")
						}
						for _, ob := range obs {
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
								t.Fatal("recentering hit rear vehicle")
							}
						}
						if bus.Pos.Dot(out) > w.JunctionAt(sim.Vec2{}, lw).HalfY+150 {
							cleared = true
							break
						}
					}
					if !cleared {
						t.Fatalf("rear car/bus locked recentering: pos%v heading%.2f diag%+v", bus.Pos, bus.Heading, ad.DiagnoseStop(bus, route, w, tls, obs, lw))
					}
				})
			}
		}
	}
}

func TestMedianLeftReturnPassesBlockedAdjacentLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		t.Run(fmt.Sprintf("w%.0f", lw), func(t *testing.T) {
			w, _, out, _ := wideLeftExitWorld(lw, 0, true)
			right := sim.Vec2{X: -out.Y, Y: out.X}
			g := w.JunctionAt(sim.Vec2{}, lw)
			p := out.Mul(g.HalfY + 250).Add(right.Mul(1.5 * lw))
			bus := sim.NewBus(p.X, p.Y, -math.Pi/2)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.DisallowOncomingOvertake = true
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			ad.ordinaryBusLeft = true
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			car := VehicleObstacle{Pos: p.Add(out.Mul(60)).Add(right.Mul(lw)), Heading: -math.Pi / 2, Length: 48, Width: 22}
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			passed := false
			for tick := 0; tick < 1800; tick++ {
				th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{car}, lw, 1./60)
				bus.Update(1./60, th, br, st, hb, doors)
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
					t.Fatal("route return hit adjacent stopped car")
				}
				if bus.Pos.Sub(car.Pos).Dot(out) > 250 {
					passed = true
					break
				}
			}
			if !passed {
				t.Fatalf("blocked route taper stopped inner lane: pos%v diag%+v", bus.Pos, ad.DiagnoseStop(bus, route, w, tls, []VehicleObstacle{car}, lw))
			}
		})
	}
}

func TestCurbRecenteringKeepsActualBlockedBodyStopped(t *testing.T) {
	const lw = 52.
	w, in, _, _ := wideLeftExitWorld(lw, 0, true)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)
	pos := sim.Vec2{X: -line - 150, Y: 3.1 * lw}
	bus := sim.NewBus(pos.X, pos.Y, -.12)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	ad.holdLaneOffset(-.4 * lw)
	ob := VehicleObstacle{Pos: pos.Add(in.Mul(85)).Add(sim.Vec2{Y: .4 * lw}), Heading: 0, Length: 48, Width: 22}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for tick := 0; tick < 120; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{ob}, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: ob.Pos, Heading: 0, HalfLen: 24, HalfWid: 11}); hit {
			t.Fatal("recenter ignored actual blocked body")
		}
	}
	if bus.Pos.Sub(pos).Dot(in) > 12 {
		t.Fatalf("advanced into blocked merge: %v", bus.Pos)
	}
}
