package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCarLeftTurnJoinsDestinationLaneWithoutSliding(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, incoming := range []int{4, 6, 8} {
			for _, destination := range []struct {
				lanes   int
				busLane bool
			}{{2, false}, {4, false}, {6, false}, {8, false}, {8, true}} {
				for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("width%.0f/in%d/out%d/bus%v/h%.2f", lw, incoming, destination.lanes, destination.busLane, heading), func(t *testing.T) {
						ew, ns := incoming, destination.lanes
						if math.Abs(math.Sin(heading)) > .5 {
							ew, ns = ns, ew
						}
						w := &sim.World{Roads: []sim.RoadSegment{
							{P1: sim.Vec2{X: -2200}, P2: sim.Vec2{X: 2200}, Lanes: ew},
							{P1: sim.Vec2{Y: -2200}, P2: sim.Vec2{Y: 2200}, Lanes: ns},
						}}
						if destination.busLane {
							index := 1
							if math.Abs(math.Sin(heading)) > .5 {
								index = 0
							}
							w.Roads[index].Type = sim.RoadBusBoulevard8Lane
						}
						w.RebuildIntersections()
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						phase := sim.PhaseEWLeftGreen
						if math.Abs(math.Sin(heading)) > .5 {
							phase = sim.PhaseNSLeftGreen
						}
						for _, inter := range tls.Intersections {
							inter.Phase = phase
						}
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						rightIn := sim.Vec2{X: -in.Y, Y: in.X}
						out := sim.Vec2{X: in.Y, Y: -in.X}
						rightOut := sim.Vec2{X: -out.Y, Y: out.X}
						radius := w.JunctionAt(sim.Vec2{}, lw).TurnRadius
						car := &sim.TrafficCar{Pos: in.Mul(-radius - 80).Add(rightIn.Mul(lw / 2)), Heading: heading, Speed: 85, TargetSpeed: 85, Length: 46, Width: 22, LaneIndex: 0, IsEW: math.Abs(in.X) > .5}
						tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
						turning, finished := false, false
						after := 0
						const dt = .016
						for step := 0; step < 1600; step++ {
							previous := car.Pos
							tm.StepCar(dt, car, 0, nil, tls, 1, lw, w)
							if car.Pos.Distance(previous) > car.Speed*dt+.001 {
								t.Fatalf("sideways position jump at step%d: moved%.3f speed%.1f", step, car.Pos.Distance(previous), car.Speed)
							}
							if car.JunctionTurn != nil {
								turning = true
							}
							if turning && car.JunctionTurn == nil {
								finished = true
								if car.LaneIndex != min(1, destination.lanes/2-1) {
									t.Fatalf("wrong destination lane: %d", car.LaneIndex)
								}
								expected := (float64(car.LaneIndex) + .5) * lw
								if math.Abs(car.Pos.Dot(rightOut)-expected) > .001 {
									t.Fatalf("turn ended outside target lane: offset%.3f lane%d expects%.3f", car.Pos.Dot(rightOut), car.LaneIndex, expected)
								}
								if after > 0 && math.Abs(car.Pos.Sub(previous).Dot(rightOut)) > .001 {
									t.Fatal("car slid sideways after its left turn")
								}
								after++
								if after >= 90 {
									break
								}
							}
						}
						if !finished || after < 90 {
							t.Fatalf("left turn did not finish: %+v", car)
						}
					})
				}
			}
		}
	}
}
