package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Include the route handoff before the stop: a new leg must not start from
// the skipped bay and pull the bus toward it, or brake for its artificial turn.
func TestSkipStopMaintainsCruiseAndCurrentLane(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, lanes := range []int{2, 4, 6, 8} {
					for lane := 0; lane < lanes/2; lane++ {
						for _, occupied := range []bool{false, true} {
							t.Run(fmt.Sprintf("berserk%v/width%.0f/heading%.2f/lanes%d/lane%d/occupied%v", berserk, lw, heading, lanes, lane, occupied), func(t *testing.T) {
								forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
								right := sim.Vec2{X: -forward.Y, Y: forward.X}
								w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-2000), P2: forward.Mul(2500), Lanes: lanes}}}
								if lanes == 8 {
									w.Roads[0].Type = sim.RoadBusBoulevard8Lane
								}
								for i, x := range []float64{-1200, 0, 1400} {
									w.Stops = append(w.Stops, sim.NewBusStop(i, "test", forward.Mul(x), heading, lanes, sim.RGBA(0, 0, 0, 255), lw))
								}
								offset := (float64(lane) + .5) * lw
								pos := forward.Mul(-800).Add(right.Mul(offset))
								bus := sim.NewBus(pos.X, pos.Y, heading)
								ad := NewAutopilotDriver()
								ad.BerserkMode = berserk
								ad.CanSkipTargetStop = true
								cruise := apCruiseSpeed
								if berserk {
									cruise = ad.GetBerserkCruiseSpeed()
								}
								bus.Speed = cruise
								bus.MaxSpeedFwd = cruise + 20
								ad.LaneOffset = offset - (float64(lanes)/2-.5)*lw
								ad.TargetLaneOffset = ad.LaneOffset
								if ad.LaneOffset != 0 {
									ad.OvertakeState = OvertakePassing
								}
								route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
								var obstacles []VehicleObstacle
								if occupied {
									obstacles = []VehicleObstacle{{Pos: w.Stops[1].Pos, Heading: heading, Length: 96, Width: 36}}
								}
								skipped := false
								for step := 0; step < 900; step++ {
									if !skipped && bus.Pos.Dot(forward) > -370 {
										route.AdvanceToNextStop()
										ad.ContinuePastSkippedStop(bus, w.Stops[1], lw)
										ad.CanSkipTargetStop = false
										skipped = true
									}
									th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
									if br > .001 || hb || doors || dock {
										t.Fatalf("stop-related control while skipping step%d brake%.2f target%.1f", step, br, ad.LastTargetSpeed)
									}
									bus.Update(1.0/60, th, br, st, hb, doors)
									if deviation := math.Abs(bus.Pos.Dot(right) - offset); deviation > 2 {
										t.Fatalf("moved toward station/lane step%d deviation%.1f pos%v", step, deviation, bus.Pos)
									}
									if bus.Speed < cruise-5 {
										t.Fatalf("slowed for skipped station %.1f want%.1f", bus.Speed, cruise)
									}
									for _, ob := range obstacles {
										if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
											t.Fatal("hit bus in skipped bay")
										}
									}
									if skipped && bus.Pos.Dot(forward) > 80 {
										return
									}
								}
								t.Fatal("did not pass skipped stop")
							})
						}
					}
				}
			}
		}
	}
}

func TestSkipPermissionChangeRebuildsStationApproach(t *testing.T) {
	const lw = 52.0
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}}}
	for i, x := range []float64{-1000, 0, 1000} {
		w.Stops = append(w.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw))
	}
	ad := NewAutopilotDriver()
	for _, skip := range []bool{true, false, true} {
		ad.CanSkipTargetStop = skip
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		end := ad.drivePath[len(ad.drivePath)-1]
		expected := w.Stops[1].FrontBerthPosition()
		if skip {
			expected = stopRoadPoint(w.Stops[1], lw)
		}
		if end.Distance(expected) > .01 {
			t.Fatalf("skip%v cached wrong endpoint%v want%v", skip, end, expected)
		}
	}
	for _, berserk := range []bool{false, true} {
		ad.Reset()
		ad.BerserkMode = berserk
		ad.CanSkipTargetStop = true
		ad.stationDock = &stationDockPlan{stopID: 1, target: w.Stops[1].Pos}
		ad.State = AutopilotStateApproaching
		bus := sim.NewBus(-300, 1.5*lw, 0)
		bus.Speed = 100
		_, br, _, _, doors, dock := ad.ComputeControls(bus, &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}, w, nil, nil, lw, 1.0/60)
		if br != 0 || doors || dock || ad.stationDock != nil || ad.State != AutopilotStateCruising {
			t.Fatalf("did not cancel docking after becoming eligible: berserk%v brake%.2f dock%v", berserk, br, dock)
		}
	}
}

func TestSkipStopStillRespectsSignalAndRoadObstacle(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, signal := range []bool{false, true} {
			t.Run(fmt.Sprintf("berserk%v/signal%v", berserk, signal), func(t *testing.T) {
				const lw = 52.0
				w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}, {P1: sim.Vec2{Y: -2000}, P2: sim.Vec2{Y: 2000}, Lanes: 4}}}
				w.RebuildIntersections()
				for i, x := range []float64{-1400, 700, 1400} {
					w.Stops = append(w.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw))
				}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				for _, inter := range tls.Intersections {
					inter.Phase = sim.PhaseNSGreen
				}
				line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)
				bus := sim.NewBus(-line-80, 1.5*lw, 0)
				bus.Speed = 150
				var obstacles []VehicleObstacle
				if !signal {
					tls = nil
					obstacles = []VehicleObstacle{{Pos: bus.Pos.Add(sim.Vec2{X: 90}), Length: 48, Width: 22}}
				}
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.CanSkipTargetStop = true
				_, br, _, _, _, _ := ad.ComputeControls(bus, &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}, w, tls, obstacles, lw, 1.0/60)
				if br <= 0 {
					t.Fatal("flying stop disabled necessary braking")
				}
			})
		}
	}
}

func TestSkippedBayVehicleIntrudingIntoRoadStillRequiresBraking(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		const lw = 42.0
		w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -1500}, P2: sim.Vec2{X: 1500}, Lanes: 4}}}
		for i, x := range []float64{-900, 0, 900} {
			w.Stops = append(w.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw))
		}
		bus := sim.NewBus(-110, 1.5*lw, 0)
		bus.Speed = 100
		obstacle := VehicleObstacle{Pos: w.Stops[1].Pos.Sub(sim.Vec2{Y: 15}), Length: 96, Width: 36}
		ad := NewAutopilotDriver()
		ad.BerserkMode = berserk
		ad.CanSkipTargetStop = true
		_, br, _, _, _, _ := ad.ComputeControls(bus, &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}, w, nil, []VehicleObstacle{obstacle}, lw, 1.0/60)
		if br <= 0 {
			t.Fatalf("ignored bay vehicle intruding into passing corridor: berserk%v", berserk)
		}
	}
}
