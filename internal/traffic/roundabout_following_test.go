package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRoundaboutEntryDoesNotYieldToFollowingBus(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lane := range []int{0, 1} {
			t.Run(fmt.Sprintf("h%.2f/lane%d", h, lane), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
				w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
				pos := u.Mul(-math.Sqrt(215*215 - math.Pow((float64(lane)+.5)*52, 2))).Add(r.Mul((float64(lane) + .5) * 52))
				car := &sim.TrafficCar{Pos: pos, Heading: h, Length: 48, Width: 22, LaneIndex: lane, IsEW: math.Abs(u.X) > .5}
				b := pos.Sub(u.Mul(100))
				bus := sim.NewBus(b.X, b.Y, h)
				f := carFrame{car: car, bus: bus, buses: []*sim.Bus{bus}, world: w, laneWidth: 52, activeCount: 1}
				tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
				if target := tm.roundaboutSpeed(f, 100); target == 0 || !car.InRoundabout {
					t.Fatalf("lead car yielded to its own queue: speed%.1f entered%v", target, car.InRoundabout)
				}
			})
		}
	}
}

func TestCirculatingLeaderDoesNotStopForFollowingVehicles(t *testing.T) {
	const radius = 214.
	for _, busBehind := range []bool{false, true} {
		car := &sim.TrafficCar{Pos: sim.Vec2{X: radius}, Heading: -math.Pi / 2, Length: 48, Width: 22, InRoundabout: true, RoundaboutCenter: sim.Vec2{}, RoundaboutAngle: 0, RoundaboutTargetR: radius, RoundaboutOuterRadius: 240}
		w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 240, InnerRadius: 120}}}
		angle := .26
		if busBehind {
			angle = .39
		}
		pos := sim.Vec2{X: radius * math.Cos(angle), Y: radius * math.Sin(angle)}
		tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
		f := carFrame{car: car, world: w, laneWidth: 52, activeCount: 1}
		if busBehind {
			f.bus = sim.NewBus(pos.X, pos.Y, angle-math.Pi/2)
		} else {
			tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: pos, Heading: angle - math.Pi/2, Length: 48, Width: 22, InRoundabout: true, RoundaboutCenter: sim.Vec2{}, RoundaboutAngle: angle, RoundaboutTargetR: radius})
			f.activeCount = 2
		}
		if speed := tm.roundaboutSpeed(f, 100); speed <= 0 {
			t.Fatalf("circulating leader stopped for follower, bus=%v", busBehind)
		}
	}
}

func TestRoundaboutFollowerExemptionRequiresCompleteRearClearance(t *testing.T) {
	car := &sim.TrafficCar{Length: 48, Width: 22}
	for _, scenario := range []struct {
		name      string
		pos       sim.Vec2
		heading   float64
		following bool
	}{
		{"behind clear", sim.Vec2{X: -100}, 0, true},
		{"behind nose overlapping", sim.Vec2{X: -60}, 0, false},
		{"alongside", sim.Vec2{Y: 52}, 0, false},
		{"ahead", sim.Vec2{X: 100}, 0, false},
		{"crossing", sim.Vec2{X: -100}, math.Pi / 2, false},
	} {
		t.Run(scenario.name, func(t *testing.T) {
			if got := roundaboutVehicleFollowing(car, scenario.pos, scenario.heading, 96, 36); got != scenario.following {
				t.Fatalf("follower classification=%v want%v", got, scenario.following)
			}
		})
	}
}
