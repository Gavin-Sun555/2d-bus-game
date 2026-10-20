package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func eightLaneWorld(kind sim.RoadType) (*sim.World, *sim.TrafficLightSystem) {
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -2200}, P2: sim.Vec2{X: 2200}, Lanes: 8, Type: kind},
		{P1: sim.Vec2{Y: -2200}, P2: sim.Vec2{Y: 2200}, Lanes: 8, Type: kind}}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	return w, tls
}

func TestEightLaneTrafficSpawningExcludesBusLane(t *testing.T) {
	for _, kind := range []sim.RoadType{sim.RoadBoulevard8Lane, sim.RoadBusBoulevard8Lane} {
		w, _ := eightLaneWorld(kind)
		tm := &TrafficManager{}
		tm.BuildForWorld(w, 200, 52)
		outer := false
		for index, car := range tm.Cars {
			if car.LaneIndex == 3 {
				outer = true
			}
			if kind == sim.RoadBusBoulevard8Lane && car.LaneIndex > 2 {
				t.Fatal("car spawned in reserved lane")
			}
			tm.DespawnAndResetCar(index, w, 52, nil)
			if kind == sim.RoadBusBoulevard8Lane && car.LaneIndex > 2 {
				t.Fatal("car reset into reserved lane")
			}
		}
		if kind == sim.RoadBoulevard8Lane && !outer {
			t.Fatal("normal road's fourth lane is unused")
		}
	}
}

func TestEightLaneCivilianRightTurns(t *testing.T) {
	for _, kind := range []sim.RoadType{sim.RoadBoulevard8Lane, sim.RoadBusBoulevard8Lane} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, lane := range []int{1, 2, 3} {
				if kind == sim.RoadBusBoulevard8Lane && lane == 3 {
					continue
				}
				t.Run(fmt.Sprintf("type%d/h%.2f/lane%d", kind, heading, lane), func(t *testing.T) {
					w, tls := eightLaneWorld(kind)
					phase := sim.PhaseEWGreen
					if math.Abs(math.Sin(heading)) > .5 {
						phase = sim.PhaseNSGreen
					}
					for _, inter := range tls.Intersections {
						inter.Phase = phase
					}
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					car := &sim.TrafficCar{ID: 3, Pos: forward.Mul(-1300).Add(right.Mul((float64(lane) + .5) * 52)), Heading: heading, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: lane, IsEW: math.Abs(forward.X) > .5}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					for step := 0; step < 2200; step++ {
						tm.StepCar(.016, car, 0, nil, tls, 1, 52, w)
						if kind == sim.RoadBusBoulevard8Lane && car.LaneIndex == 3 && car.JunctionTurn == nil && car.Pos.Dot(forward) < -tls.Intersections[0].GetStopLineDistForHeading(heading, 52)-425 {
							t.Fatal("car cruises in bus lane")
						}
						if lane == 1 && car.Pos.Dot(forward) > 600 {
							if car.LaneIndex != 1 {
								t.Fatal("through car used turn lane")
							}
							return
						}
						if lane != 1 && car.Pos.Dot(right) > 700 {
							if math.Abs(sim.NormalizeAngle(car.Heading-heading-math.Pi/2)) > .01 || car.LaneIndex > 2 {
								t.Fatal("right turn exits into wrong lane")
							}
							return
						}
					}
					t.Fatalf("car did not complete its movement: %+v", car)
				})
			}
		}
	}
}

func TestEightLaneRightMergeYieldsToBus(t *testing.T) {
	w, tls := eightLaneWorld(sim.RoadBusBoulevard8Lane)
	car := &sim.TrafficCar{ID: 3, Pos: sim.Vec2{X: -650, Y: 130}, Heading: 0, Speed: 80, Length: 46, Width: 22, IsEW: true, LaneIndex: 2}
	bus := sim.NewBus(-670, 182, 0)
	bus.Speed = 100
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	frame := carFrame{car: car, world: w, tls: tls, laneWidth: 52, buses: []*sim.Bus{bus}}
	tm.prepareEightLaneApproach(frame)
	if car.LaneIndex != 2 {
		t.Fatal("car merged in front of a bus")
	}
	bus.Pos.X = -1200
	tm.prepareEightLaneApproach(frame)
	if car.LaneIndex != 3 {
		t.Fatal("car did not prepare its right turn after the lane cleared")
	}
}
