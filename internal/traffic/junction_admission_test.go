package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestSeparatedOpposingLeftCarDoesNotReserveWholeBox(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f", lw, lanes, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					out := r.Mul(-1)
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: lanes}, {P1: r.Mul(-3000), P2: r.Mul(3000), Lanes: 4}}}
					w.RebuildIntersections()
					a := sim.NewBusStop(0, "from", u.Mul(-2200), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
					z := sim.NewBusStop(1, "to", out.Mul(2200), h-math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{a, z}
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.Phase = sim.PhaseEWLeftGreen
					if math.Abs(u.Y) > .5 {
						it.Phase = sim.PhaseNSLeftGreen
					}
					ad := autopilot.NewAutopilotDriver()
					g := w.JunctionAt(sim.Vec2{}, lw)
					p := u.Mul(lw/2 - g.TurnRadius + 5).Add(r.Mul(lw / 2))
					bus := sim.NewBus(p.X, p.Y, h)
					bus.Speed = 70
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
					if bus.JunctionCorridor == nil {
						t.Fatal("bus did not publish junction corridor")
					}
					stop := it.GetStopLineDistForHeading(h+math.Pi, lw)
					car := &sim.TrafficCar{Pos: u.Mul(stop + 35).Sub(r.Mul(lw / 2)), Heading: sim.NormalizeAngle(h + math.Pi), Length: 48, Width: 22, LaneIndex: 0, IsEW: math.Abs(u.X) > .5}
					f := carFrame{car: car, world: w, tls: tls, laneWidth: lw, buses: []*sim.Bus{bus}}
					if !carClearsBusInJunction(f, bus, it) {
						t.Fatal("separate opposing left corridors were treated as a conflict")
					}
					if got := (&TrafficManager{}).signalSpeed(f, 85); got != 85 {
						t.Fatalf("nonconflicting car stopped for bus: %.1f", got)
					}
					// With no route evidence, retain conservative yielding to a manual bus.
					bus.JunctionCorridor = nil
					if got := (&TrafficManager{}).signalSpeed(f, 85); got >= 85 {
						t.Fatal("unplanned crossing bus lost protection")
					}
					car.Speed = 60
					car.TargetSpeed = 85
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					concurrent := false
					completed := false
					for tick := 0; tick < 2400; tick++ {
						tm.StepCar(1./60, car, 0, bus, tls, 1, lw, w)
						ob := autopilot.VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: 48, Width: 22, JunctionTurn: car.JunctionTurn}
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, []autopilot.VehicleObstacle{ob}, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
							t.Fatalf("opposing routes touched at tick%d", tick)
						}
						if car.JunctionTurn != nil && bus.Pos.Dot(out) < g.HalfX+bus.Length/2 {
							concurrent = true
						}
						if bus.Pos.Dot(out) > g.HalfX+200 && car.Pos.Dot(r) > g.HalfX+200 {
							completed = true
							break
						}
					}
					if !concurrent || !completed {
						t.Fatalf("junction did not flow: simultaneous=%v completed=%v bus%v car%v", concurrent, completed, bus.Pos, car.Pos)
					}

				})
			}
		}
	}
}

func TestExitQueueWaitsOutsideAndReleases(t *testing.T) {
	for _, movement := range []string{"straight", "left", "right"} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, lw := range []float64{42, 52, 78} {
				t.Run(fmt.Sprintf("%s/h%.2f/w%.0f", movement, h, lw), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 4}}}
					w.RebuildIntersections()
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.Phase = sim.PhaseEWGreen
					if movement == "left" {
						it.Phase = sim.PhaseEWLeftGreen
					}
					if math.Abs(u.Y) > .5 {
						if movement == "left" {
							it.Phase = sim.PhaseNSLeftGreen
						} else {
							it.Phase = sim.PhaseNSGreen
						}
					}
					lane := 1
					if movement == "left" {
						lane = 0
					}
					if movement == "right" {
						lane = 3
					}
					line := it.GetStopLineDistForHeading(h, lw)
					p := u.Mul(-line - 150).Add(r.Mul((float64(lane) + .5) * lw))
					car := &sim.TrafficCar{Pos: p, Heading: h, Length: 48, Width: 22, LaneIndex: lane, IsEW: math.Abs(u.X) > .5, TargetSpeed: 100, Speed: 80}
					frame := carFrame{car: car, world: w, tls: tls, laneWidth: lw}
					bodies := carJunctionBodies(frame, it)
					exit := bodies[len(bodies)-1]
					out := sim.Vec2{X: math.Cos(exit.Heading), Y: math.Sin(exit.Heading)}
					g := w.JunctionAt(sim.Vec2{}, lw)
					half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
					q := exit.Center.Add(out.Mul(half + 50 - exit.Center.Dot(out)))
					queue := &sim.TrafficCar{Pos: q, Heading: exit.Heading, Length: 48, Width: 22}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car, queue}}
					for tick := 0; tick < 900; tick++ {
						tm.StepCar(1./60, car, 0, nil, tls, 2, lw, w)
						if car.Pos.Dot(u)+24 > -line+.1 {
							t.Fatal("car entered blocked junction")
						}
					}
					tm.Cars = tm.Cars[:1]
					for tick := 0; tick < 2000; tick++ {
						tm.StepCar(1./60, car, 0, nil, tls, 1, lw, w)
						if car.Pos.Dot(out) > half+150 {
							return
						}
					}
					t.Fatal("car did not release after outgoing queue cleared")
				})
			}
		}
	}
}

func TestStraightLaneReductionUsesSmoothPathsAndPriority(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: sim.Vec2{}, Lanes: 8}, {P1: sim.Vec2{}, P2: u.Mul(4000), Lanes: 4}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 4}}}
				w.RebuildIntersections()
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				it := tls.Intersections[0]
				it.Phase = sim.PhaseEWGreen
				if math.Abs(u.Y) > .5 {
					it.Phase = sim.PhaseNSGreen
				}
				line := it.GetStopLineDistForHeading(h, lw)
				cars := []*sim.TrafficCar{}
				for i, lane := range []int{1, 2} {
					cars = append(cars, &sim.TrafficCar{ID: i + 1, Pos: u.Mul(-line - 300).Add(r.Mul((float64(lane) + .5) * lw)), Heading: h, Speed: 140, TargetSpeed: 140, Length: 48, Width: 22, IsEW: math.Abs(u.X) > .5, LaneIndex: lane})
				}
				tm := &TrafficManager{Cars: cars}
				done := false
				for tick := 0; tick < 2400; tick++ {
					for i, c := range cars {
						before := c.Pos
						tm.StepCar(1./60, c, i, nil, tls, 2, lw, w)
						if c.Pos.Distance(before) > c.Speed/60+.01 {
							t.Fatal("straight lane reduction slid sideways")
						}
					}
					if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: cars[0].Pos, Heading: cars[0].Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: cars[1].Pos, Heading: cars[1].Heading, HalfLen: 24, HalfWid: 11}); hit {
						t.Fatalf("lane reduction contact tick%d", tick)
					}
					if cars[0].Pos.Dot(u) > line+250 && cars[1].Pos.Dot(u) > line+250 {
						done = true
						break
					}
				}
				if !done {
					t.Fatalf("lane reduction stalled: %v %v", cars[0].Pos, cars[1].Pos)
				}
				for _, c := range cars {
					if c.LaneIndex != 1 || math.Abs(c.Pos.Dot(r)-1.5*lw) > .1 || math.Abs(sim.NormalizeAngle(c.Heading-h)) > .001 {
						t.Fatal("wrong outgoing lane")
					}
				}
			})
		}
	}
}

func TestDistantJunctionDoesNotReserveLocalEntry(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 4}, {P1: sim.Vec2{Y: -10000}, P2: sim.Vec2{Y: 5000}, Lanes: 4}}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	inter := tls.Intersections[0]
	inter.Phase = sim.PhaseEWGreen
	car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -inter.GetStopLineDistForHeading(0, lw) - 100, Y: 78}, Heading: 0, Length: 48, Width: 22, Speed: 80, LaneIndex: 1, IsEW: true}
	distant := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: -26, Y: -4800}, Heading: math.Pi / 2, Length: 48, Width: 22, Speed: 70, JunctionTurn: &sim.CarJunctionTurn{Center: sim.Vec2{Y: -5000}, Points: []sim.Vec2{{X: 26, Y: -5000}, {X: -26, Y: -4800}}, ExitHeading: math.Pi / 2}}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car, distant}}
	frame := carFrame{car: car, world: w, tls: tls, index: 0, activeCount: 2, laneWidth: lw}
	if got := tm.junctionQueueSpeed(frame, 90); got != 90 {
		t.Fatal("distant committed turn blocked local junction")
	}
	if distant.JunctionCorridor != nil {
		t.Fatal("built a cross-map corridor for unrelated junction")
	}
}
