package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func forcedLeftWorld(lanes int, heading float64) (*sim.World, *sim.TrafficLightSystem) {
	in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	left := sim.Vec2{X: in.Y, Y: -in.X}
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: in.Mul(-2400), P2: sim.Vec2{}, Lanes: lanes},
		{P1: sim.Vec2{}, P2: left.Mul(2400), Lanes: lanes},
	}}
	// This bend is internal to the city, rather than an outward map exit.
	w.Roads = append(w.Roads, sim.RoadSegment{P1: in.Mul(3000).Add(left.Mul(3000)), P2: in.Mul(3400).Add(left.Mul(3000)), Lanes: 2})
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	phase := sim.PhaseEWGreen
	if math.Abs(in.Y) > .5 {
		phase = sim.PhaseNSGreen
	}
	for _, inter := range tls.Intersections {
		inter.Phase = phase
	}
	return w, tls
}

func TestThroughLaneCannotForceLeftAtRoadEnd(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for lane := 1; lane < lanes/2; lane++ {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/lane%d/h%.2f", lw, lanes, lane, heading), func(t *testing.T) {
						w, tls := forcedLeftWorld(lanes, heading)
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						pos := in.Mul(-40).Add(right.Mul((float64(lane) + .5) * lw))
						car := &sim.TrafficCar{Pos: pos, Heading: heading, LaneIndex: lane, IsEW: math.Abs(in.X) > .5}
						f := carFrame{car: car, world: w, tls: tls, laneWidth: lw}
						if left, _ := carIntersectionTurns(f, tls.Intersections[0]); left {
							t.Fatal("through lane selected an illegal left turn at road end")
						}
						(&TrafficManager{}).turnCarAtIntersection(f)
						if car.JunctionTurn != nil {
							t.Fatal("through lane began a left-turn path")
						}
					})
				}
			}
		}
	}
}

func TestForcedLeftApproachMergesBeforeLineAndWaitsForArrow(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, blocked := range []bool{false, true} {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/h%.2f/blocked%v", lw, lanes, heading, blocked), func(t *testing.T) {
						w, tls := forcedLeftWorld(lanes, heading)
						inter := tls.Intersections[0]
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						out := right.Mul(-1)
						line := inter.GetStopLineDistForHeading(heading, lw)
						car := &sim.TrafficCar{Pos: in.Mul(-line - 950).Add(right.Mul((float64(lanes/2-1) + .5) * lw)), Heading: heading, Speed: 85, TargetSpeed: 85, Length: 46, Width: 22, LaneIndex: lanes/2 - 1, IsEW: math.Abs(in.X) > .5}
						tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
						merged, stopped, finished := false, false, false
						for step := 0; step < 3800; step++ {
							var bus *sim.Bus
							if blocked && step < 700 {
								p := car.Pos.Sub(right.Mul(lw))
								bus = sim.NewBus(p.X, p.Y, heading)
								bus.Speed = car.Speed
							}
							if step == 2000 {
								inter.Phase = sim.PhaseEWLeftGreen
								if math.Abs(in.Y) > .5 {
									inter.Phase = sim.PhaseNSLeftGreen
								}
							}
							previous := car.Pos
							tm.StepCar(.016, car, 0, bus, tls, 1, lw, w)
							if car.Pos.Distance(previous) > car.Speed*.016+.001 {
								t.Fatal("approach merge moved sideways instead of following its path")
							}
							if blocked && step < 700 && car.JunctionTurn != nil {
								t.Fatal("merged into occupied lane")
							}
							if car.JunctionTurn != nil && !car.JunctionTurn.LaneChange {
								if step < 2000 {
									t.Fatal("left turn started under a red left arrow")
								}
								if car.LaneIndex != 0 {
									t.Fatal("left turn started from through lane")
								}
							}
							if step < 2000 && car.Pos.Dot(in)+car.Length/2 > -line+.1 {
								t.Fatal("approach crossed stop line before left green")
							}
							if car.LaneIndex == 0 && car.JunctionTurn == nil && step < 2000 {
								merged = true
								stopped = stopped || car.Speed < 1
							}
							if car.Pos.Dot(out) > 1000 && car.JunctionTurn == nil {
								finished = true
								break
							}
						}
						if !merged || !stopped || !finished {
							t.Fatalf("merge/stop/turn failed: merged%v stopped%v finished%v car%+v", merged, stopped, finished, car)
						}
					})
				}
			}
		}
	}
}

func TestThroughCarsKeepHeadingUnderGreenWithRedLeftArrow(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for lane := 1; lane < min(3, lanes/2); lane++ {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/lane%d/h%.2f", lw, lanes, lane, heading), func(t *testing.T) {
						in := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						w, tls := forcedLeftWorld(lanes, heading)
						w.Roads[0].P2 = in.Mul(2400)
						w.Roads[1].P1 = right.Mul(2400)
						w.RebuildIntersections()
						tls.BuildForWorld(w)
						phase := sim.PhaseEWGreen
						if math.Abs(in.Y) > .5 {
							phase = sim.PhaseNSGreen
						}
						for _, inter := range tls.Intersections {
							inter.Phase = phase
						}
						line := tls.Intersections[0].GetStopLineDistForHeading(heading, lw)
						car := &sim.TrafficCar{ID: 1, Pos: in.Mul(-line - 200).Add(right.Mul((float64(lane) + .5) * lw)), Heading: heading, Speed: 85, TargetSpeed: 85, Length: 46, Width: 22, LaneIndex: lane, IsEW: math.Abs(in.X) > .5}
						tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
						for step := 0; step < 1500 && car.Pos.Dot(in) < line+200; step++ {
							tm.StepCar(.016, car, 0, nil, tls, 1, lw, w)
							if (car.JunctionTurn != nil && !car.JunctionTurn.Straight) || car.LaneIndex != lane || math.Abs(sim.NormalizeAngle(car.Heading-heading)) > .001 {
								t.Fatal("through-green car turned or changed lanes")
							}
						}
						if car.Pos.Dot(in) < line+200 {
							t.Fatal("through-green car did not clear junction")
						}
					})
				}
			}
		}
	}
}
