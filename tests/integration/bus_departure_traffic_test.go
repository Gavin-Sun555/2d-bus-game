package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Run the actual fleet and car controllers together. Passing this test may not
// depend on either vehicle being reset, disappearing, or passing through the other.
func TestAIBusDepartureAndPassingCarDoNotDeadlock(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, along := range []float64{-140, -70, 0, 70, 140} {
					for _, speed := range []float64{0, 90} {
						t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f/along%.0f/speed%.0f", lw, lanes, heading, along, speed), func(t *testing.T) {
							forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
							right := sim.Vec2{X: -forward.Y, Y: forward.X}
							w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-2000), P2: forward.Mul(2500), Lanes: lanes}}}
							for i, x := range []float64{0, 1100} {
								w.Stops = append(w.Stops, sim.NewBusStop(i, "test", forward.Mul(x), heading, lanes, sim.RGBA(0, 0, 0, 255), lw, 10))
							}
							start := w.Stops[0]
							bus := sim.NewBus(start.Pos.X, start.Pos.Y, heading)
							driver := autopilot.NewAutopilotDriver()
							driver.State = autopilot.AutopilotStateDeparting
							ab := &traffic.AIBus{ID: 1, Bus: bus, Route: &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}, Driver: driver, LastExchangedStopID: -1}
							mgr := &traffic.AIBusManager{Buses: []*traffic.AIBus{ab}}
							lane := float64(lanes/2) - .5
							pos := forward.Mul(along).Add(right.Mul(lane * lw))
							car := &sim.TrafficCar{ID: 1, Pos: pos, Heading: heading, Speed: speed, TargetSpeed: 90, Length: 48, Width: 22, LaneIndex: lanes/2 - 1, IsEW: math.Abs(forward.X) > .5}
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}, AIBusMgr: mgr}
							for step := 0; step < 1200; step++ {
								oldBus, oldCar := bus.Pos, car.Pos
								tm.Update(1.0/60, nil, nil, 1, lw, w)
								mgr.Update(1.0/60, nil, w, nil, tm, lw, 1)
								expectedBus := oldBus.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(bus.Speed / 60))
								expectedCar := oldCar.Add(sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}.Mul(car.Speed / 60))
								if bus.Pos.Distance(expectedBus) > .05 || car.Pos.Distance(expectedCar) > .05 {
									t.Fatalf("vehicle reset/teleport at step%d bus%v ->%v speed%.1f car%v ->%v speed%.1f", step, oldBus, bus.Pos, bus.Speed, oldCar, car.Pos, car.Speed)
								}
								a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
								b := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}
								if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
									t.Fatalf("departure collision step%d bus%v h%.2f v%.1f car%v v%.1f", step, bus.Pos, bus.Heading, bus.Speed, car.Pos, car.Speed)
								}
								if bus.Pos.Dot(forward) > 500 && car.Pos.Dot(forward) > 500 {
									return
								}
							}
							t.Fatalf("departure deadlock: bus%v h%.2f v%.1f state%v car%v v%.1f", bus.Pos, bus.Heading, bus.Speed, driver.State, car.Pos, car.Speed)
						})
					}
				}
			}
		}
	}
}

func TestPartlyDepartedAIBusAndNearbyCarResumeWithoutReset(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, scenario := range []struct{ lateral, ahead float64 }{{40, -80}, {40, -20}, {40, 20}, {40, 80}, {32, -80}} {
				ahead := scenario.ahead
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/lateral%.0f/ahead%.0f", lw, heading, scenario.lateral, ahead), func(t *testing.T) {
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-2000), P2: forward.Mul(2500), Lanes: 4}}}
					for i, x := range []float64{0, 1100} {
						w.Stops = append(w.Stops, sim.NewBusStop(i, "test", forward.Mul(x), heading, 4, sim.RGBA(0, 0, 0, 255), lw, 10))
					}
					bp := forward.Mul(95).Add(right.Mul(1.5*lw + scenario.lateral))
					bus := sim.NewBus(bp.X, bp.Y, heading-.18)
					ad := autopilot.NewAutopilotDriver()
					ad.State = autopilot.AutopilotStateDeparting
					ab := &traffic.AIBus{ID: 1, Bus: bus, Route: &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}, Driver: ad, LastExchangedStopID: -1}
					mgr := &traffic.AIBusManager{Buses: []*traffic.AIBus{ab}}
					cp := forward.Mul(95 + ahead).Add(right.Mul(1.5 * lw))
					car := &sim.TrafficCar{ID: 1, Pos: cp, Heading: heading, TargetSpeed: 90, Length: 48, Width: 22, LaneIndex: 1, IsEW: math.Abs(forward.X) > .5}
					tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}, AIBusMgr: mgr}
					for step := 0; step < 1200; step++ {
						beforeBus, beforeCar := bus.Pos, car.Pos
						tm.Update(1.0/60, nil, nil, 1, lw, w)
						mgr.Update(1.0/60, nil, w, nil, tm, lw, 1)
						expectedBus := beforeBus.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(bus.Speed / 60))
						expectedCar := beforeCar.Add(sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}.Mul(car.Speed / 60))
						if bus.Pos.Distance(expectedBus) > .05 || car.Pos.Distance(expectedCar) > .05 {
							t.Fatalf("relied on collision separation/reset at step%d", step)
						}
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
							t.Fatalf("collision at step%d", step)
						}
						if bus.Pos.Dot(forward) > 500 && car.Pos.Dot(forward) > 500 {
							return
						}
					}
					t.Fatalf("partly merged bus deadlocked bus%v h%.2f v%.1f car%v v%.1f", bus.Pos, bus.Heading, bus.Speed, car.Pos, car.Speed)
				})
			}
		}
	}
}

func TestCarStillYieldsWhenBusActuallyOccupiesItsLane(t *testing.T) {
	for _, inBay := range []bool{false, true} {
		t.Run(fmt.Sprintf("inBay%v", inBay), func(t *testing.T) {
			const lw = 52.0
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}}}
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "test", sim.Vec2{}, 0, 4, sim.RGBA(0, 0, 0, 255), lw, 0)}
			bus := sim.NewBus(95, 1.5*lw+16, -.18)
			if inBay {
				bus.Pos = sim.Vec2{X: 50, Y: w.Stops[0].Pos.Y}
				bus.Heading = -.5
			}
			mgr := &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 1, Bus: bus}}}
			car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -120, Y: 1.5 * lw}, Heading: 0, Speed: 90, TargetSpeed: 90, Length: 48, Width: 22, LaneIndex: 1, IsEW: true}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}, AIBusMgr: mgr}
			for step := 0; step < 360; step++ {
				tm.StepCar(1.0/60, car, 0, nil, nil, 1, lw, w)
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
					t.Fatalf("passed through blocking bus step%d", step)
				}
			}
			if car.Speed > 1 {
				t.Fatalf("did not yield to blocking bus: speed%.1f", car.Speed)
			}
		})
	}
}
