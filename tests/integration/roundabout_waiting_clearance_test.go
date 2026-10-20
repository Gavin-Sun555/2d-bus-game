package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

// A player can hug the outside edge rather than the nominal lane center. Cars
// at either entrance lane must leave room for the full bus body as it passes.
func TestWaitingRoundaboutCarsLeaveBusSweptClearance(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, lane := range []int{0, 1} {
				for _, speed := range []float64{40, 100, 200} {
					for _, aiBus := range []bool{false, true} {
						t.Run(fmt.Sprintf("width%.0f/h%.2f/lane%d/speed%.0f/AI%v", lw, heading, lane, speed, aiBus), func(t *testing.T) {
							center := sim.Vec2{X: 100, Y: -200}
							rb := sim.Roundabout{Center: center, Radius: 240, InnerRadius: 120}
							w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{
								{P1: center.Add(sim.Vec2{X: -2000}), P2: center.Add(sim.Vec2{X: 2000}), Lanes: 4},
								{P1: center.Add(sim.Vec2{Y: -2000}), P2: center.Add(sim.Vec2{Y: 2000}), Lanes: 4}}}
							w.RebuildIntersections()
							forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
							right := sim.Vec2{X: -forward.Y, Y: forward.X}
							car := &sim.TrafficCar{Pos: center.Sub(forward.Mul(650)).Add(right.Mul((float64(lane) + .5) * lw)), Heading: heading, Speed: speed, TargetSpeed: speed, Length: 48, Width: 22, LaneIndex: lane, IsEW: math.Abs(forward.X) > .5}
							car.RoadCenter = center.X
							if car.IsEW {
								car.RoadCenter = center.Y
							}
							bus := sim.NewBus(0, 0, 0)
							bus.Length = 120
							bus.Width = 42
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
							player := bus
							if aiBus {
								player = nil
								tm.AIBusMgr = &traffic.AIBusManager{Buses: []*traffic.AIBus{{Bus: bus}}}
							}
							initialAngle := heading + math.Pi + .75
							radius := rb.Radius - 3
							for step := 0; step < 1200; step++ {
								elapsed := math.Max(0, float64(step)*.016-14)
								angle := initialAngle - 90/radius*elapsed
								bus.Pos = center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(radius))
								bus.Heading = angle - math.Pi/2
								bus.Speed = 0
								if elapsed > 0 {
									bus.Speed = 90
								}
								before, oldSpeed := car.Pos, car.Speed
								tm.StepCar(.016, car, 0, player, nil, 1, lw, w)
								if car.Pos.Distance(before) > math.Max(oldSpeed, car.Speed)*.016+.05 {
									t.Fatal("car position jumped")
								}
								if !car.InRoundabout && oldSpeed-car.Speed > 280*.016+.01 {
									t.Fatal("late emergency speed snap instead of planned braking")
								}
								a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
								b := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
								if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
									t.Fatalf("waiting car hit passing bus step%d car%v bus%v", step, car.Pos, bus.Pos)
								}
								if step == 870 {
									noseDistance := center.Sub(car.Pos).Dot(forward) - car.Length/2
									if noseDistance < rb.Radius+38 {
										t.Fatalf("waiting nose too close: %.1f, ring radius%.1f", noseDistance, rb.Radius)
									}
									if car.InRoundabout || car.Speed > 1 {
										t.Fatal("car did not wait for occupied ring")
									}
								}
							}
							tm.AIBusMgr = nil
							for step := 0; step < 1200; step++ {
								tm.StepCar(.016, car, 0, nil, nil, 1, lw, w)
								if car.InRoundabout {
									return
								}
							}
							t.Fatal("car did not resume after the bus cleared")
						})
					}
				}
			}
		}
	}
}
