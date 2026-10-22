package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestBerserkBusReturnsToLaneBeforeIntersectionOn2LaneRoad verifies that when a bus
// is borrowing the oncoming lane on a 2-lane road in Berserk mode, it returns to the
// designated travel lane before reaching an intersection (e.g. for a left turn),
// civilian traffic yields to let the bus merge, and the bus smoothly executes the turn
// without crashing into barriers or entering the intersection on the wrong side.
func TestBerserkBusReturnsToLaneBeforeIntersectionOn2LaneRoad(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, turnDir := range []string{"left", "straight", "right"} {
			t.Run(fmt.Sprintf("lw%.0f/turn_%s", lw, turnDir), func(t *testing.T) {
				// 2-lane East-West road intersecting a 2-lane North-South road
				w := &sim.World{
					Roads: []sim.RoadSegment{
						{P1: sim.Vec2{X: -2500, Y: 0}, P2: sim.Vec2{X: 2500, Y: 0}, Lanes: 2},
						{P1: sim.Vec2{X: 0, Y: -2500}, P2: sim.Vec2{X: 0, Y: 2500}, Lanes: 2},
					},
				}
				w.RebuildIntersections()

				// Bus travels Westbound towards the intersection at (0, 0)
				// Westbound lane is at Y = -0.5 * lw.
				// Oncoming eastbound lane is at Y = +0.5 * lw.
				h := math.Pi
				from := sim.NewBusStop(0, "EastApproach", sim.Vec2{X: 1800, Y: 0}, h, 2, sim.RGBA(0, 0, 0, 255), lw)

				var to *sim.BusStop
				switch turnDir {
				case "left":
					// Left turn at intersection (0, 0): turns South onto NS road (center X = 0)
					to = sim.NewBusStop(1, "SouthExit", sim.Vec2{X: 0, Y: 1800}, math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw)
				case "right":
					// Right turn at intersection (0, 0): turns North onto NS road (center X = 0)
					to = sim.NewBusStop(1, "NorthExit", sim.Vec2{X: 0, Y: -1800}, -math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw)
				case "straight":
					// Straight through intersection (0, 0): continues West (center Y = 0)
					to = sim.NewBusStop(1, "WestExit", sim.Vec2{X: -1800, Y: 0}, math.Pi, 2, sim.RGBA(0, 0, 0, 255), lw)
				}
				w.Stops = []*sim.BusStop{from, to}

				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				inter := tls.Intersections[0]
				// Green light for EW traffic
				inter.Phase = sim.PhaseEWGreen
				stopLineDist := inter.GetStopLineDistForHeading(h, lw)

				// Bus is at X = 500 (distance to stop line ~450px)
				// Bus was overtaking in oncoming lane: Y = +0.5 * lw (LaneOffset = -lw = -52)
				startPos := sim.Vec2{X: stopLineDist + 450, Y: 0.5 * lw}
				bus := sim.NewBus(startPos.X, startPos.Y, h)
				bus.Speed = 160.0 // Fast Berserk cruising speed

				ad := autopilot.NewAutopilotDriver()
				ad.BerserkMode = true
				ad.BerserkMaxSpeed = 667
				ad.PrepareLeg(w, from, to, lw)
				ad.LaneOffset = -lw
				ad.TargetLaneOffset = -lw
				ad.OvertakeState = autopilot.OvertakePassing

				// Civilian car is traveling in the Westbound base lane (Y = -0.5 * lw), behind the bus
				carPos := sim.Vec2{X: startPos.X + 110, Y: -0.5 * lw}
				car := &sim.TrafficCar{
					ID:          1,
					Pos:         carPos,
					Heading:     h,
					Speed:       80.0,
					TargetSpeed: 100.0,
					Length:      48,
					Width:       22,
					LaneIndex:   0,
					IsEW:        true,
				}
				tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}

				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

				yielded := false
				merged := false
				passedJunction := false

				for step := 0; step < 1800; step++ {
					// 1. Traffic cars update and observe bus merge request
					tm.StepCar(0.016, car, 0, bus, tls, 1, lw, w)
					if car.YieldingToBus == bus {
						yielded = true
					}

					// 2. Bus controller updates
					obs := []autopilot.VehicleObstacle{
						{
							Pos:        car.Pos,
							Heading:    car.Heading,
							Speed:      car.Speed,
							Length:     car.Length,
							Width:      car.Width,
							YieldingTo: car.YieldingToBus,
						},
					}

					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 0.016)
					bus.Update(0.016, th, br, st, hb, doors)

					// Check boundary collisions
					fwd := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
					normal := sim.Vec2{X: -fwd.Y, Y: fwd.X}
					for _, a := range []float64{-1, 1} {
						for _, z := range []float64{-1, 1} {
							corner := bus.Pos.Add(fwd.Mul(a * bus.Length / 2)).Add(normal.Mul(z * bus.Width / 2))
							if w.CheckBoundaries(corner, 0, lw).Zone == sim.ZoneHardBarrier {
								t.Fatalf("step%d bus hit road edge/barrier at%v corner%v heading%.2f", step, bus.Pos, corner, bus.Heading)
							}
						}
					}

					// Check car collision
					busOBB := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + 1, HalfWid: bus.Width/2 + 1}
					carOBB := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length/2 + 1, HalfWid: car.Width/2 + 1}
					if hit, _, _ := sim.CheckOBBCollision(busOBB, carOBB); hit {
						t.Fatalf("step%d bus/car collision at bus%v car%v", step, bus.Pos, car.Pos)
					}

					// Verify merge into base travel lane before entering intersection
					if math.Abs(ad.LaneOffset) < 1.0 && math.Abs(bus.Pos.Y-(-0.5*lw)) < 8.0 {
						merged = true
					}

					// When reaching stop line (X <= stopLineDist), bus MUST NOT be in opposing lane (Y > 0)
					if bus.Pos.X <= stopLineDist && bus.Pos.X >= stopLineDist-25 && !passedJunction {
						if bus.Pos.Y > 0.1*lw {
							t.Fatalf("step%d bus entered intersection from opposing lane! Pos%v offset%.1f", step, bus.Pos, ad.LaneOffset)
						}
					}

					// Check if bus successfully completed the turn / passed junction
					switch turnDir {
					case "left":
						if bus.Pos.Y > 300.0 && bus.Heading > 1.2 {
							passedJunction = true
						}
					case "right":
						if bus.Pos.Y < -300.0 && bus.Heading < -1.2 {
							passedJunction = true
						}
					case "straight":
						if bus.Pos.X < -300.0 {
							passedJunction = true
						}
					}

					if passedJunction && merged && yielded {
						return // Success!
					}
				}

				t.Fatalf("test timed out: merged=%v, yielded=%v, passedJunction=%v, busPos=%v, offset=%.1f",
					merged, yielded, passedJunction, bus.Pos, ad.LaneOffset)
			})
		}
	}
}
