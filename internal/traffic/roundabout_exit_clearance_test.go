package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// Reconstruct the inner-ring car which cut across the release soak's outer
// circulating bus. Rotate the complete encounter and exercise player/fleet buses.
func TestRoundaboutInnerExitYieldsToOuterBus(t *testing.T) {
	for _, rotation := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			for _, fleet := range []bool{false, true} {
				t.Run(fmt.Sprintf("h%.2f/model%d/fleet%v", rotation, model, fleet), func(t *testing.T) {
					rotate := func(p sim.Vec2) sim.Vec2 {
						return sim.Vec2{X: p.X*math.Cos(rotation) - p.Y*math.Sin(rotation), Y: p.X*math.Sin(rotation) + p.Y*math.Cos(rotation)}
					}
					rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
					// Captured encounter shortly before the inner car starts its exit curve.
					angle := .872
					p := rotate(sim.Vec2{X: math.Cos(angle) * 145.75, Y: math.Sin(angle) * 145.75})
					car := &sim.TrafficCar{ID: 316, Pos: p, Heading: angle - math.Pi/2 + rotation, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, InRoundabout: true, RoundaboutRadius: 145.75, RoundaboutTargetR: 145.75, RoundaboutAngle: angle + rotation, RoundaboutOuterRadius: 220, RoundaboutTraveled: 2, RoundaboutExitAngle: .17934757392108142 + rotation, RoundaboutExitHeading: rotation, RoundaboutExitOffset: 26}
					car.RoundaboutExitEW = math.Abs(math.Cos(rotation)) > .5
					bp := rotate(sim.Vec2{X: 148.09, Y: 126.56})
					bus := sim.NewBus(bp.X, bp.Y, -.86+rotation)
					bus.SetModel(model)
					bus.Speed = 85
					if bus.IsArticulated() {
						bus.TrailerHeading = -.33 + rotation
					}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					player := bus
					if fleet {
						player = nil
						tm.AIBusMgr = &AIBusManager{Buses: []*AIBus{{Bus: bus}}}
					}
					w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -2000}), P2: rotate(sim.Vec2{X: 2000}), Lanes: 4}}}
					yielded, exited := false, false
					for tick := 0; tick < 600; tick++ {
						tm.StepCar(1./60, car, 0, player, nil, 1, 52, w)
						yielded = yielded || car.Speed < 70
						// Hold an outer circular path with the real front/trailer integrator.
						steer := math.Atan(-bus.WheelBase/194) / bus.MaxSteerAngle
						bus.Update(1./60, 0, 0, steer, false, false)
						bus.Speed = 85
						body := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 23, HalfWid: 11}
						if hit, _, _ := bus.CollidesBody(body); hit {
							t.Fatalf("contact tick%d bus%v car%v", tick, bus.Pos, car.Pos)
						}
						if !car.InRoundabout && car.Pos.Length() > 350 {
							exited = true
							break
						}
					}
					if !yielded || !exited {
						t.Fatalf("yielded%v exited%v car%v speed%.1f", yielded, exited, car.Pos, car.Speed)
					}
				})
			}
		}
	}
}

func TestRoundaboutFollowerProtectsStoppedArticulatedTail(t *testing.T) {
	rb := sim.Roundabout{Radius: 220, InnerRadius: 105}
	bus := sim.NewBus(-158.2615, -113.7212, 2.19765)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = 2.740745
	angle := -.8
	car := &sim.TrafficCar{ID: 342, Pos: sim.Vec2{X: 195.25 * math.Cos(angle), Y: 195.25 * math.Sin(angle)}, Heading: angle - math.Pi/2, Speed: 90, TargetSpeed: 90, Length: 50, Width: 24, InRoundabout: true, RoundaboutRadius: 195.25, RoundaboutTargetR: 195.25, RoundaboutAngle: angle, RoundaboutOuterRadius: 220, RoundaboutTraveled: 1, RoundaboutExitAngle: -2.730635, RoundaboutExitHeading: math.Pi, RoundaboutExitOffset: 78, RoundaboutExitLane: 1, RoundaboutExitEW: true}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}, Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 4}}}
	stopped := false
	for tick := 0; tick < 600; tick++ {
		tm.StepCar(1./60, car, 0, bus, nil, 1, 52, w)
		if tick < 240 && car.Speed < 1 {
			stopped = true
		}
		if tick >= 240 {
			bus.Speed = 85
			bus.Update(1./60, 0, 0, math.Atan(-bus.WheelBase/195.25)/bus.MaxSteerAngle, false, false)
		}
		body := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 25, HalfWid: 12}
		if hit, _, _ := bus.CollidesBody(body); hit {
			t.Fatalf("tail contact tick%d car%v bus%v", tick, car.Pos, bus.Pos)
		}
	}
	if !stopped || car.InRoundabout {
		t.Fatalf("stopped%v exited%v speed%.1f", stopped, !car.InRoundabout, car.Speed)
	}
}
