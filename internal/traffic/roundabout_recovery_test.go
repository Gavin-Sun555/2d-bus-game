package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestDisplacedCirculatingCarClearsWaitingBus(t *testing.T) {
	for _, scene := range []struct {
		name                                string
		car, bus                            sim.Vec2
		carHeading, busHeading, tailHeading float64
	}{
		{"original", sim.Vec2{X: -242.38079204252932, Y: -68.33086077399285}, sim.Vec2{X: -176.84724595051296, Y: -86.89692662845641}, 1.8831325813283053, 2.277, 2.444},
	} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
				t.Run(fmt.Sprintf("%s/h%.2f/model%d", scene.name, h, model), func(t *testing.T) {
					rotate := func(p sim.Vec2) sim.Vec2 {
						return sim.Vec2{X: p.X*math.Cos(h) - p.Y*math.Sin(h), Y: p.X*math.Sin(h) + p.Y*math.Cos(h)}
					}
					rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
					p := rotate(scene.car)
					car := &sim.TrafficCar{ID: 683, Pos: p, Heading: scene.carHeading + h, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: 1, InRoundabout: true, RoundaboutRadius: 195.25, RoundaboutTargetR: 195.25, RoundaboutOuterRadius: 220, RoundaboutAngle: -2.8292563990563844 + h, RoundaboutTraveled: .994811966464977, RoundaboutExitHeading: math.Pi/2 + h, RoundaboutExitAngle: 1.9817544243405925 + h, RoundaboutExitOffset: 78, RoundaboutExitLane: 1}
					car.RoundaboutExitEW = math.Abs(math.Cos(car.RoundaboutExitHeading)) > .5
					bp := rotate(scene.bus)
					bus := sim.NewBus(bp.X, bp.Y, scene.busHeading+h)
					bus.SetModel(model)
					bus.TrailerHeading = scene.tailHeading + h
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -4000}), P2: rotate(sim.Vec2{X: 4000}), Lanes: 4}, {P1: rotate(sim.Vec2{Y: -4000}), P2: rotate(sim.Vec2{Y: 4000}), Lanes: 4}}}
					exited := false
					for tick := 0; tick < 1200; tick++ {
						before := car.Pos
						tm.StepCar(1./60, car, 0, bus, nil, 1, 52, w)
						bus.Speed = 0
						if step := car.Pos.Distance(before); step > math.Abs(car.Speed)/60*1.12+.01 {
							t.Fatalf("jump tick%d distance%.2f speed%.2f", tick, step, car.Speed)
						}
						if hit, _, _ := bus.CollidesBody(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 23, HalfWid: 11}); hit {
							t.Fatalf("collision tick%d pos%v", tick, car.Pos)
						}
						if !car.InRoundabout && car.Pos.Distance(rb.Center) > 350 {
							exited = true
							break
						}
					}
					if !exited {
						t.Fatalf("car still holds ring: pos%v speed%.1f", car.Pos, car.Speed)
					}
				})
			}
		}
	}
}

func TestRoundaboutLocalizationKeepsPhysicalPoseAndProgress(t *testing.T) {
	car := &sim.TrafficCar{InRoundabout: true, Pos: sim.Vec2{X: -249, Y: -74}, Heading: 1.85, RoundaboutRadius: 195, RoundaboutAngle: -2.83, RoundaboutTraveled: 1.002}
	before := *car
	localizeRoundaboutCar(car)
	if car.Pos != before.Pos || car.Heading != before.Heading || car.RoundaboutTraveled != before.RoundaboutTraveled {
		t.Fatal("localization moved or advanced the car")
	}
	if math.Abs(car.RoundaboutRadius-car.Pos.Length()) > 1e-9 {
		t.Fatal("stale radius")
	}
	valid := *car
	localizeRoundaboutCar(car)
	if *car != valid {
		t.Fatal("valid localization was modified")
	}
}
