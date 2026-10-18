package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Circulating traffic keeps its speed and priority: the entering bus must find
// a gap without relying on the other vehicle to brake or disappear.
func TestBerserkEntryYieldsToCirculatingTraffic(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, speed := range []float64{0, 45, 90, 150} {
			for _, phase := range []float64{0.35, 1.2, 2.0} {
				for _, length := range []float64{48, 96} {
					t.Run(fmt.Sprintf("heading%.2f/speed%.0f/phase%.2f/length%.0f", heading, speed, phase, length), func(t *testing.T) {
						const lw = 52.0
						w := sim.NewWorld(lw, sim.MapThemeMetropolis)
						rb := w.Roundabouts[0]
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						w.Stops = []*sim.BusStop{
							sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw),
							sim.NewBusStop(1, "to", rb.Center.Add(forward.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw),
						}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						pos := rb.Center.Sub(forward.Mul(350)).Add(right.Mul(lw / 2))
						bus := sim.NewBus(pos.X, pos.Y, heading)
						bus.Speed = 120
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.LaneOffset, ad.TargetLaneOffset = -lw, -lw
						ad.OvertakeState = OvertakePassing
						radius := roundaboutOuterLaneRadius(&rb) + roundaboutLaneOffset(&rb, bus, -lw)
						initialAngle := heading + math.Pi + phase
						if speed == 0 {
							initialAngle = heading + math.Pi - 0.35
						}
						yielded, exited := false, false
						for step := 0; step < 1800; step++ {
							angle := initialAngle - speed/radius*float64(step)/60
							obstacle := VehicleObstacle{Pos: rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius)), Heading: angle - math.Pi/2, Speed: speed, Length: length, Width: 24}
							obstacles := []VehicleObstacle{obstacle}
							// A stopped car is removed only after verifying that the bus waits.
							if speed == 0 && step >= 300 {
								obstacles = nil
							}
							throttle, brake, steer, handbrake, doors, _ := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60)
							bus.Update(1.0/60, throttle, brake, steer, handbrake, doors)
							if bus.Speed < 10 {
								yielded = true
							}
							if len(obstacles) > 0 {
								nextAngle := angle - speed/radius/60
								other := sim.OBB{Center: rb.Center.Add(sim.Vec2{X: math.Cos(nextAngle), Y: math.Sin(nextAngle)}.Mul(radius)), Heading: nextAngle - math.Pi/2, HalfLen: length / 2, HalfWid: 12}
								body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
								if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
									t.Fatalf("collision step=%d bus=%v speed=%.1f car=%v", step, bus.Pos, bus.Speed, other.Center)
								}
							}
							if bus.Pos.Sub(rb.Center).Dot(forward) > rb.Radius+150 {
								exited = true
								break
							}
						}
						if (speed == 0 || (speed == 45 && phase == 0.35)) && !yielded {
							t.Error("bus did not yield")
						}
						if !exited {
							t.Fatalf("bus failed to resume and exit: pos=%v speed=%.1f", bus.Pos, bus.Speed)
						}
					})
				}
			}
		}
	}
}

func TestBerserkEntryDoesNotYieldToFollowingApproachCar(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "from", rb.Center.Add(sim.Vec2{X: -800}), 0, 4, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "to", rb.Center.Add(sim.Vec2{X: 800}), 0, 4, sim.RGBA(0, 0, 0, 255), lw),
	}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	bus := sim.NewBus(rb.Center.X-350, rb.Center.Y+lw/2, 0)
	bus.Speed = 120
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.LaneOffset, ad.TargetLaneOffset = -lw, -lw
	ad.OvertakeState = OvertakePassing
	following := VehicleObstacle{Pos: bus.Pos.Sub(sim.Vec2{X: 120}), Heading: 0, Speed: 120, Length: 48, Width: 24}
	throttle, brake, _, _, _, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{following}, lw, 1.0/60)
	if brake > 0 || throttle <= 0 {
		t.Fatalf("yielded to car following on approach: throttle=%.2f brake=%.2f", throttle, brake)
	}
}
