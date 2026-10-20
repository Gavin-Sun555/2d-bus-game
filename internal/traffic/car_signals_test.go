package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCivilianRightTurnOnRed(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f", lw, lanes, heading), func(t *testing.T) {
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					// Narrow roads exercise a forced right turn; eight-lane roads exercise
					// their dedicated right-turn lane at a full crossroads.
					end := sim.Vec2{}
					if lanes == 8 {
						end = forward.Mul(2200)
					}
					w := &sim.World{Roads: []sim.RoadSegment{
						{P1: forward.Mul(-2200), P2: end, Lanes: lanes},
						{P1: sim.Vec2{}, P2: right.Mul(2200), Lanes: lanes},
					}}
					w.RebuildIntersections()
					phase := sim.PhaseNSGreen
					if math.Abs(forward.Y) > .5 {
						phase = sim.PhaseEWGreen
					}
					inter := &sim.TrafficIntersection{Phase: phase, IsMajor: lanes >= 4, ApproachLanes: [4]int{lanes, lanes, lanes, lanes}}
					tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
					stop := inter.GetStopLineDistForHeading(heading, lw)
					lane := lanes/2 - 1
					car := &sim.TrafficCar{ID: 3, Pos: forward.Mul(-stop - 170).Add(right.Mul((float64(lane) + .5) * lw)), Heading: heading, Speed: 80, TargetSpeed: 90, Length: 46, Width: 22, IsEW: math.Abs(forward.X) > .5, LaneIndex: lane}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					f := carFrame{car: car, world: w, tls: tls, laneWidth: lw}
					// Permission to turn on red must not bypass the occupied-box rule.
					approach := car.Pos
					car.Pos = forward.Mul(-stop - 30).Add(right.Mul((float64(lane) + .5) * lw))
					f.buses = []*sim.Bus{sim.NewBus(0, 0, heading+math.Pi/2)}
					if tm.signalSpeed(f, 90) >= 90 {
						t.Fatal("right turn ignored crossing bus")
					}
					car.Pos = approach
					f.buses = nil
					car.IsBraking = false
					if tm.signalSpeed(f, 90) != 90 || car.IsBraking {
						t.Fatal("clear right turn still waits for red light")
					}
					completed := false
					for step := 0; step < 1200; step++ {
						tm.StepCar(.016, car, 0, nil, tls, 1, lw, w)
						if car.Speed < 1 {
							t.Fatalf("right-turning car stopped for red at step%d", step)
						}
						if car.Pos.Dot(right) > 500 {
							if math.Abs(sim.NormalizeAngle(car.Heading-heading-math.Pi/2)) > .01 {
								t.Fatal("car did not follow its right turn")
							}
							completed = true
							break
						}
					}
					if !completed {
						t.Fatalf("right turn did not complete: %+v", car)
					}
				})
			}
		}
	}
}

func TestCivilianThroughAndLeftStillStopOnRed(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lane := range []int{0, 1} {
			w, tls := eightLaneWorld(sim.RoadBoulevard8Lane)
			inter := tls.Intersections[0]
			inter.Phase = sim.PhaseNSGreen
			if math.Abs(math.Sin(heading)) > .5 {
				inter.Phase = sim.PhaseEWGreen
			}
			in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			right := sim.Vec2{X: -in.Y, Y: in.X}
			stop := inter.GetStopLineDistForHeading(heading, 52)
			car := &sim.TrafficCar{Pos: in.Mul(-stop - 30).Add(right.Mul((float64(lane) + .5) * 52)), Heading: heading, Length: 46, Width: 22, LaneIndex: lane, IsEW: math.Abs(in.X) > .5}
			f := carFrame{car: car, world: w, tls: tls, laneWidth: 52}
			if (&TrafficManager{}).signalSpeed(f, 90) != 0 || !car.IsBraking {
				t.Fatalf("lane%d heading%.2f did not stop at red", lane, heading)
			}
		}
	}
}
