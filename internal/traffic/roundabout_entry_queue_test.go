package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCirculatingCarClearsWaitingEntranceQueue(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, busQueue := range []bool{false, true} {
			for _, a := range []float64{-.4, -.3} {
				t.Run(fmt.Sprintf("h%.2f/bus%v/angle%.1f", h, busQueue, a), func(t *testing.T) {
					rotate := func(p sim.Vec2) sim.Vec2 {
						return sim.Vec2{X: p.X*math.Cos(h) - p.Y*math.Sin(h), Y: p.X*math.Sin(h) + p.Y*math.Cos(h)}
					}
					rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
					w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
					car := &sim.TrafficCar{Pos: rotate(sim.Vec2{X: 214 * math.Cos(a), Y: 214 * math.Sin(a)}), Heading: a + h - math.Pi/2, Length: 46, Width: 22, TargetSpeed: 100, InRoundabout: true, RoundaboutRadius: 214, RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutAngle: a + h, RoundaboutExitAngle: sim.NormalizeAngle(h - 2), RoundaboutExitHeading: sim.NormalizeAngle(math.Pi + h), LaneIndex: 1}
					start := car.Pos
					p := sim.Vec2{X: 235, Y: -52}
					if a == -.3 {
						p = sim.Vec2{X: 240, Y: -78}
					}
					length, width := 46., 22.
					var bus *sim.Bus
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					if busQueue {
						p = sim.Vec2{X: 270, Y: -78}
						length, width = 96, 36
						v := rotate(p)
						bus = sim.NewBus(v.X, v.Y, math.Pi+h)
					} else {
						tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: rotate(p), Heading: math.Pi + h, Length: 46, Width: 22, InRoundabout: true, RoundaboutRadius: p.Length(), RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutAngle: math.Atan2(p.Y, p.X) + h, RoundaboutExitHeading: math.Pi + h, LaneIndex: 1})
					}
					// Hold the yielding entrance queue still. The circulating car must clear it
					// without needing the queue to advance, disappear, or reset.
					for tick := 0; tick < 180; tick++ {
						tm.StepCar(1./60, car, 0, bus, nil, len(tm.Cars), 52, w)
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: rotate(p), Heading: math.Pi + h, HalfLen: length / 2, HalfWid: width / 2}); hit {
							t.Fatalf("contact with waiting queue at tick %d", tick)
						}
					}
					if car.Pos.Distance(start) < 100 {
						t.Fatalf("circulating car stuck beside yielding queue: moved %.1f", car.Pos.Distance(start))
					}
				})
			}
		}
	}
}

func TestCirculatingCarStillStopsForBlockedPath(t *testing.T) {
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
	car := &sim.TrafficCar{Pos: sim.Vec2{X: 214}, Heading: -math.Pi / 2, Length: 46, Width: 22, InRoundabout: true, RoundaboutRadius: 214, RoundaboutTargetR: 214, RoundaboutOuterRadius: 240}
	a := -.2
	leader := &sim.TrafficCar{Pos: sim.Vec2{X: 214 * math.Cos(a), Y: 214 * math.Sin(a)}, Heading: a - math.Pi/2, Length: 46, Width: 22, InRoundabout: true, RoundaboutRadius: 214, RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutAngle: a}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car, leader}}
	f := carFrame{car: car, world: w, laneWidth: 52, activeCount: 2}
	if speed := tm.roundaboutSpeed(f, 100); speed != 0 {
		t.Fatalf("blocked circulating path allowed speed %.1f", speed)
	}
}
