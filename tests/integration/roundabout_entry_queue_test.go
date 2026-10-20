package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Keep the real car controller active: neither vehicle may disappear, be
// teleported, or rely on the deadlock reset to clear an adjacent entry queue.
func TestBerserkRoundaboutEntryBesideStoppedCar(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, exitTurn := range []float64{-math.Pi / 2, 0, math.Pi / 2} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, separation := range []float64{0, 20, -20} {
					t.Run(fmt.Sprintf("width%.0f/turn%.2f/heading%.2f/ahead%.0f", lw, exitTurn, heading, separation), func(t *testing.T) {
						w := sim.NewWorld(lw, sim.MapThemeMetropolis)
						rb := w.Roundabouts[0]
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						outgoing := sim.Vec2{X: math.Cos(heading + exitTurn), Y: math.Sin(heading + exitTurn)}
						w.Stops = []*sim.BusStop{
							sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(800)), heading, 4, sim.RGBA(0, 0, 0, 255), lw),
							sim.NewBusStop(1, "to", rb.Center.Add(outgoing.Mul(800)), heading+exitTurn, 4, sim.RGBA(0, 0, 0, 255), lw),
						}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						pos := rb.Center.Sub(forward.Mul(225)).Add(right.Mul(lw / 2))
						bus := sim.NewBus(pos.X, pos.Y, heading+.15)
						ad := autopilot.NewAutopilotDriver()
						ad.BerserkMode = true
						ad.LaneOffset, ad.TargetLaneOffset = -lw, -lw
						ad.OvertakeState = autopilot.OvertakePassing
						cp := rb.Center.Sub(forward.Mul(225 - separation)).Add(right.Mul(lw * 1.5))
						car := &sim.TrafficCar{ID: 1, Pos: cp, Heading: heading, TargetSpeed: 100, Length: 48, Width: 22, LaneIndex: 1, IsEW: math.Abs(forward.X) > .5}
						if car.IsEW {
							car.RoadCenter = rb.Center.Y
						} else {
							car.RoadCenter = rb.Center.X
						}
						tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
						entered, carCleared, busCleared := false, false, false
						for step := 0; step < 1800; step++ {
							tm.StepCar(1.0/60, car, 0, bus, nil, 1, lw, w)
							obs := autopilot.VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width}
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []autopilot.VehicleObstacle{obs}, lw, 1.0/60)
							bus.Update(1.0/60, th, br, st, hb, doors)
							a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							b := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
							if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
								t.Fatalf("collision step%d bus%v v%.1f car%v v%.1f", step, bus.Pos, bus.Speed, car.Pos, car.Speed)
							}
							entered = entered || car.InRoundabout
							carCleared = carCleared || (entered && !car.InRoundabout && car.Pos.Sub(rb.Center).Dot(forward) > rb.Radius+60)
							busCleared = busCleared || bus.Pos.Sub(rb.Center).Dot(outgoing) > rb.Radius+140
							if carCleared && busCleared {
								return
							}
						}
						t.Fatalf("entry deadlock: bus%v v%.1f target%.1f car%v v%.1f entered%v carCleared%v busCleared%v headings%.2f/%.2f", bus.Pos, bus.Speed, ad.LastTargetSpeed, car.Pos, car.Speed, entered, carCleared, busCleared, bus.Heading, car.Heading)
					})
				}
			}
		}
	}
}

func TestStoppedRoundaboutCarDoesNotSlideOrRotate(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	pos := rb.Center.Add(sim.Vec2{X: -225, Y: 78})
	car := &sim.TrafficCar{Pos: pos, Heading: .15, Length: 48, Width: 22, InRoundabout: true, RoundaboutCenter: rb.Center, RoundaboutRadius: pos.Distance(rb.Center), RoundaboutTargetR: 210, RoundaboutAngle: math.Atan2(pos.Y-rb.Center.Y, pos.X-rb.Center.X), RoundaboutOuterRadius: rb.Radius}
	tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
	for i := 0; i < 120; i++ {
		tm.StepCar(1.0/60, car, 0, nil, nil, 1, 52, w)
	}
	if car.Pos.Distance(pos) > 1e-6 || car.Heading != .15 {
		t.Fatalf("stopped car moved: %v heading%.3f", car.Pos, car.Heading)
	}
}

func TestRoundaboutCarStillYieldsToBusBlockingEntry(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			const lw = 52.0
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			rb := w.Roundabouts[0]
			forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			bp := rb.Center.Sub(forward.Mul(213)).Add(right.Mul(70))
			bus := sim.NewBus(bp.X, bp.Y, heading+1.25)
			cp := rb.Center.Sub(forward.Mul(290)).Add(right.Mul(78))
			car := &sim.TrafficCar{Pos: cp, Heading: heading, TargetSpeed: 100, Length: 48, Width: 22, LaneIndex: 1, IsEW: math.Abs(forward.X) > .5}
			if car.IsEW {
				car.RoadCenter = rb.Center.Y
			} else {
				car.RoadCenter = rb.Center.X
			}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
			for step := 0; step < 180; step++ {
				tm.StepCar(1.0/60, car, 0, bus, nil, 1, lw, w)
				a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}
				b := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}
				if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
					t.Fatalf("car entered occupied mouth at step%d", step)
				}
				if car.InRoundabout {
					t.Fatal("car must yield to bus occupying its entry sweep")
				}
			}
			if car.Speed > 1 {
				t.Fatalf("car failed to stop: speed%.1f", car.Speed)
			}
		})
	}
}
