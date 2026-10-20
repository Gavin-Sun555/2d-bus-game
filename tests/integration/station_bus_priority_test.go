package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestCurbTrafficCreatesGapForBusDepartureAndArrival(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, arrival := range []bool{false, true} {
					if arrival && lanes == 2 {
						continue
					}
					for _, fleet := range []bool{false, true} {
						t.Run(fmt.Sprintf("width%.0f/lanes%d/h%.2f/arrival%v/fleet%v", lw, lanes, heading, arrival, fleet), func(t *testing.T) {
							forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
							right := sim.Vec2{X: -forward.Y, Y: forward.X}
							w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-2500), P2: forward.Mul(3000), Lanes: lanes}}}
							for id, along := range []float64{0, 1100} {
								w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", forward.Mul(along), heading, lanes, sim.RGBA(0, 0, 0, 255), lw, 10))
							}
							curb := (float64(lanes/2) - .5) * lw
							pos := w.Stops[0].Pos
							if arrival {
								pos = forward.Mul(800).Add(right.Mul(curb - lw))
							}
							bus := sim.NewBus(pos.X, pos.Y, heading)
							ad := autopilot.NewAutopilotDriver()
							ad.BerserkMode = arrival
							if arrival {
								bus.Speed = 60
							} else {
								ad.State = autopilot.AutopilotStateDeparting
							}
							route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
							carPos := forward.Mul(bus.Pos.Dot(forward) - 400).Add(right.Mul(curb))
							car := &sim.TrafficCar{ID: 1, Pos: carPos, Heading: heading, Speed: 90, TargetSpeed: 90, Length: 48, Width: 22, LaneIndex: lanes/2 - 1, IsEW: math.Abs(forward.X) > .5}
							following := *car
							following.ID = 2
							following.Pos = car.Pos.Sub(forward.Mul(150))
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car, &following}}
							player := bus
							if fleet {
								player = nil
								tm.AIBusMgr = &traffic.AIBusManager{Buses: []*traffic.AIBus{{Bus: bus, Driver: ad, Route: route}}}
							}
							yielded, finished := false, false
							for step := 0; step < 1800; step++ {
								var obstacles []autopilot.VehicleObstacle
								for _, c := range tm.Cars {
									obstacles = append(obstacles, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: 48, Width: 22})
								}
								th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obstacles, lw, .016)
								bus.Update(.016, th, br, st, hb, doors)
								for i, c := range tm.Cars {
									tm.StepCar(.016, c, i, player, nil, len(tm.Cars), lw, w)
								}
								yielded = yielded || car.YieldingToBus == bus
								for _, c := range tm.Cars {
									if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 24, HalfWid: 11}); hit {
										t.Fatalf("station yield collision at step%d", step)
									}
								}
								if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: following.Pos, Heading: following.Heading, HalfLen: 24, HalfWid: 11}); hit {
									t.Fatalf("following cars collided at step%d", step)
								}

								if arrival && dock {
									if !w.Stops[1].ContainsBus(bus) {
										t.Fatal("arrival stayed outside the available bay")
									}
									finished = true
									break
								}
								if !arrival && bus.Pos.Dot(forward) > 500 {
									finished = true
									break
								}
							}
							if !yielded || !finished {
								t.Fatalf("station priority failed: yielded%v finished%v bus%v speed%.1f car%v speed%.1f", yielded, finished, bus.Pos, bus.Speed, car.Pos, car.Speed)
							}
						})
					}
				}
			}
		}
	}
}
