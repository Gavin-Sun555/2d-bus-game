package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestCarHeadwayUsesArticulatedRearSection(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, scenario := range []struct {
			speed float64
			brake bool
		}{{0, false}, {100, false}, {100, true}} {
			leadSpeed := scenario.speed
			t.Run(fmt.Sprintf("heading%.2f/lead%.0f/brake%v", h, leadSpeed, scenario.brake), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				leader := sim.NewBus(0, 0, h)
				leader.SetModel(sim.BusArticulated)
				leader.Speed = leadSpeed
				car := &sim.TrafficCar{Pos: u.Mul(-387), Heading: h, Speed: 180, TargetSpeed: 180, Length: 46, Width: 22}
				tm := &TrafficManager{}
				minGap := math.Inf(1)
				for tick := 0; tick < 1800; tick++ {
					if scenario.brake && tick == 900 {
						leader.Speed = 0
					}
					leader.Pos = leader.Pos.Add(u.Mul(leader.Speed / 60))
					target := tm.busFollowingSpeed(carFrame{car: car, buses: []*sim.Bus{leader}}, 180)
					if car.Speed < target {
						car.Speed = math.Min(target, car.Speed+120./60)
					} else {
						car.Speed = math.Max(target, car.Speed-280./60)
					}
					car.Pos = car.Pos.Add(u.Mul(car.Speed / 60))
					gap := leader.Pos.Sub(car.Pos).Dot(u) - 144 - car.Length/2
					minGap = math.Min(minGap, gap)
					if hit, _, _ := leader.CollidesBody(sim.OBB{Center: car.Pos, Heading: h, HalfLen: car.Length / 2, HalfWid: car.Width / 2}); hit {
						t.Fatalf("tail contact tick%d", tick)
					}
				}
				if minGap < 24 || (leader.Speed > 0 && minGap < 50) {
					t.Fatalf("tail headway too short %.2f", minGap)
				}
				if leader.Speed == 0 && (car.Speed > .1 || minGap > 40) {
					t.Fatalf("did not queue normally: gap%.2f speed%.2f", minGap, car.Speed)
				}
				if leader.Speed > 0 && math.Abs(car.Speed-leader.Speed) > 5 {
					t.Fatalf("unstable moving following speed %.2f", car.Speed)
				}
				t.Logf("minimum bumper gap %.2fpx", minGap)
			})
		}
	}
}
