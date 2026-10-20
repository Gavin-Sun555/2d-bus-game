package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Exercise the actual car motion and bus bicycle model together, with no
// cross-vehicle yielding that could conceal intersecting turning corridors.
func TestBerserkBusAndOpposingCarLeftTurns(t *testing.T) {
	for _, ew := range []int{2, 4, 6, 8} {
		for _, ns := range []int{2, 4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2} {
				// Civilian cars use dedicated left-turn lanes at four-way junctions.
				if (heading == 0 && ew == 2) || (heading != 0 && ns == 2) {
					continue
				}
				for _, delay := range []int{0, 90, 180} {
					t.Run(fmt.Sprintf("%dx%d/h%.2f/delay%d", ew, ns, heading, delay), func(t *testing.T) {
						w := &sim.World{Roads: []sim.RoadSegment{
							{P1: sim.Vec2{X: -2200}, P2: sim.Vec2{X: 2200}, Lanes: ew},
							{P1: sim.Vec2{Y: -2200}, P2: sim.Vec2{Y: 2200}, Lanes: ns},
						}}
						w.RebuildIntersections()
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						inLanes, outLanes := ew, ns
						phase := sim.PhaseEWLeftGreen
						if heading != 0 {
							inLanes, outLanes = ns, ew
							phase = sim.PhaseNSLeftGreen
						}
						for _, it := range tls.Intersections {
							it.Phase = phase
						}
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						start := sim.NewBusStop(0, "start", in.Mul(1400), heading+math.Pi, inLanes, sim.RGBA(0, 0, 0, 255), 52, 0)
						end := sim.NewBusStop(1, "end", right.Mul(1400), heading+math.Pi/2, outLanes, sim.RGBA(0, 0, 0, 255), 52, 0)
						w.Stops = []*sim.BusStop{start, end}
						bus := sim.NewBus(start.Pos.X, start.Pos.Y, start.Heading)
						ad := autopilot.NewAutopilotDriver()
						ad.BerserkMode = true
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						cp := in.Mul(-1050).Add(right.Mul(26))
						car := &sim.TrafficCar{Pos: cp, Heading: heading, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, IsEW: heading == 0, LaneIndex: 0}
						tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
						busDone, carDone := false, false
						for step := 0; step < 2400; step++ {
							if step >= delay {
								tm.StepCar(.016, car, 0, nil, tls, 1, 52, w)
							}
							previous := bus.Pos
							th, br, st, hb, doors, docked := ad.ComputeControls(bus, route, w, tls, nil, 52, .016)
							bus.Update(.016, th, br, st, hb, doors)
							if bus.Pos.Distance(previous) > 12 {
								t.Fatal("bus position jumped")
							}
							busDone = busDone || docked
							carDone = carDone || (!car.IsEW && heading == 0 && car.Pos.Y < -350) || (car.IsEW && heading != 0 && car.Pos.X > 350)
							aa := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + 2, HalfWid: bus.Width/2 + 2}
							bb := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length/2 + 2, HalfWid: car.Width/2 + 2}
							if hit, _, _ := sim.CheckOBBCollision(aa, bb); hit {
								t.Fatalf("step%d bus/car collision at%v/%v", step, bus.Pos, car.Pos)
							}
							if busDone && carDone {
								return
							}
						}
						t.Fatalf("turns did not finish: bus%v at%v car%v at%v heading%.2f", busDone, bus.Pos, carDone, car.Pos, car.Heading)
					})
				}
			}
		}
	}
}
