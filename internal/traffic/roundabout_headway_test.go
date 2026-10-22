package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRoundaboutCarFollowsMovingBusContinuously(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, fleet := range []bool{false, true} {
			t.Run(fmt.Sprintf("heading%.2f/fleet%v", heading, fleet), func(t *testing.T) {
				rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
				radius := 214.
				angle := heading + math.Pi
				car := &sim.TrafficCar{Pos: sim.Vec2{X: math.Cos(angle) * radius, Y: math.Sin(angle) * radius}, Heading: angle - math.Pi/2, Speed: 90, TargetSpeed: 90, Length: 48, Width: 22, InRoundabout: true, RoundaboutCenter: rb.Center, RoundaboutRadius: radius, RoundaboutTargetR: radius, RoundaboutAngle: angle, RoundaboutOuterRadius: rb.Radius, RoundaboutExitAngle: sim.NormalizeAngle(angle - 3)}
				bus := sim.NewBus(0, 0, 0)
				bus.Speed = 85
				w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
				tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
				player := bus
				if fleet {
					player = nil
					tm.AIBusMgr = &AIBusManager{Buses: []*AIBus{{Bus: bus}}}
				}
				for step := 0; step < 240; step++ {
					a := angle - .5 - 85/radius*float64(step)/60
					bus.Pos = sim.Vec2{X: math.Cos(a) * radius, Y: math.Sin(a) * radius}
					bus.Heading = a - math.Pi/2
					tm.StepCar(1.0/60, car, 0, player, nil, 1, 52, w)
					if car.Speed < 20 {
						t.Fatalf("moving leader caused a stop at step%d: speed%.1f", step, car.Speed)
					}
					first := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}
					second := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}
					if hit, _, _ := sim.CheckOBBCollision(first, second); hit {
						t.Fatalf("following contact at step%d", step)
					}
				}
			})
		}
	}
}

func TestRoundaboutHeadwayStillStopsForStationaryBus(t *testing.T) {
	r := 214.
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -r}, Heading: math.Pi / 2, Length: 48, Width: 22, RoundaboutRadius: r, RoundaboutTargetR: r, RoundaboutAngle: math.Pi}
	a := math.Pi - .4
	bus := sim.NewBus(r*math.Cos(a), r*math.Sin(a), a-math.Pi/2)
	if speed := roundaboutBusHeadway(car, bus); speed != 0 {
		t.Fatalf("stationary leader not respected: %.1f", speed)
	}
	bus.Speed = 85
	if speed := roundaboutBusHeadway(car, bus); speed <= 20 {
		t.Fatalf("moving leader incorrectly stops follower: %.1f", speed)
	}
}
