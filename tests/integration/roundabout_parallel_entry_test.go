package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Both controllers remain live, with no collision resolution, reset or removal.
// Adjacent entries should share the mouth rather than serialize an empty ring.
func TestParallelRoundaboutEntry(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, busMode := range []string{"cars", "berserkInner", "berserkOuter", "ordinaryOuter", "fleetInner", "fleetOuter"} {
				busLane := -1
				if busMode == "berserkInner" || busMode == "fleetInner" {
					busLane = 0
				} else if busMode != "cars" {
					busLane = 1
				}
				for _, stagger := range []float64{-30, 0, 30} {
					t.Run(fmt.Sprintf("width%.0f/heading%.2f/%s/stagger%.0f", lw, h, busMode, stagger), func(t *testing.T) {
						w := sim.NewWorld(lw, sim.MapThemeMetropolis)
						rb := w.Roundabouts[0]
						f := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -f.Y, Y: f.X}
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", rb.Center.Sub(f.Mul(800)), h, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", rb.Center.Add(f.Mul(800)), h, 4, sim.RGBA(0, 0, 0, 255), lw)}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						tm := &traffic.TrafficManager{}
						var bus *sim.Bus
						var ad *autopilot.AutopilotDriver
						for lane := 0; lane < 2; lane++ {
							p := rb.Center.Sub(f.Mul(rb.Radius + 180)).Add(r.Mul((float64(lane) + .5) * lw))
							if lane == 1 {
								p = p.Add(f.Mul(stagger))
							}
							if lane == busLane {
								bus = sim.NewBus(p.X, p.Y, h)
								bus.Speed = 90
								ad = autopilot.NewAutopilotDriver()
								ad.BerserkMode = busMode != "ordinaryOuter"
								ad.LaneOffset = float64(lane-1) * lw
								ad.TargetLaneOffset = ad.LaneOffset
								if lane == 0 {
									ad.OvertakeState = autopilot.OvertakePassing
								}
								continue
							}
							rc := rb.Center.Y
							if math.Abs(f.X) < .5 {
								rc = rb.Center.X
							}
							tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: lane + 1, Pos: p, Heading: h, Speed: 90, TargetSpeed: 90, Length: 48, Width: 22, LaneIndex: lane, IsEW: math.Abs(f.X) > .5, RoadCenter: rc})
						}
						player := bus
						if busMode == "fleetInner" || busMode == "fleetOuter" {
							player = nil
							tm.AIBusMgr = &traffic.AIBusManager{Buses: []*traffic.AIBus{{Bus: bus}}}
						}
						entered := make([]bool, len(tm.Cars))
						exited := make([]bool, len(tm.Cars))
						stopped := make([]int, len(tm.Cars))
						simultaneous := false
						busExited := bus == nil
						for step := 0; step < 1800; step++ {
							bodies := []sim.OBB{}
							for i, c := range tm.Cars {
								tm.StepCar(1.0/60, c, i, player, nil, len(tm.Cars), lw, w)
								if c.InRoundabout && c.Pos.Sub(rb.Center).Dot(f) < 0 && c.Speed < 2 {
									stopped[i]++
								} else {
									stopped[i] = 0
								}
								if stopped[i] > 30 {
									if bus != nil {
										t.Logf("bus pos%v h%.2f speed%.1f target%.1f offset%.1f entry%v carcurve%v", bus.Pos, bus.Heading, bus.Speed, ad.LastTargetSpeed, ad.LaneOffset, bus.RoundaboutEntry, c.RoundaboutCurve != nil)
									}
									t.Fatalf("car%d stopped after entering mouth step%d: pos%v heading%.2f", i, step, c.Pos, c.Heading)
								}
								entered[i] = entered[i] || c.InRoundabout
								exited[i] = exited[i] || (entered[i] && !c.InRoundabout)
								bodies = append(bodies, sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2})
							}
							if bus != nil {
								obs := []autopilot.VehicleObstacle{}
								for _, c := range tm.Cars {
									obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
								}
								th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, nil, obs, lw, 1.0/60)
								bus.Update(1.0/60, th, br, st, hb, d)
								bodies = append(bodies, sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2})
								busExited = busExited || bus.Pos.Sub(rb.Center).Dot(f) > rb.Radius+140
							}
							if hit, _, _ := sim.CheckOBBCollision(bodies[0], bodies[1]); hit {
								t.Fatalf("collision step%d: %+v %+v", step, bodies[0], bodies[1])
							}
							inside := 0
							for _, b := range bodies {
								if b.Center.Distance(rb.Center) < rb.Radius && b.Center.Sub(rb.Center).Dot(f) < 0 {
									inside++
								}
							}
							simultaneous = simultaneous || inside == 2
							all := busExited
							for _, done := range exited {
								all = all && done
							}
							if all {
								if stagger == 0 && !simultaneous {
									t.Fatal("empty two-lane mouth was serialized")
								}
								return
							}
						}
						t.Fatalf("entry stalled: cars%+v bus%+v simultaneous%v", tm.Cars[0], bus, simultaneous)
					})
				}
			}
		}
	}
}
