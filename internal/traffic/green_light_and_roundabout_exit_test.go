package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestGreenLightQueueVehiclesDoNotStopForCrossRedTraffic verifies that:
// 1. Cars stopping at a red light do not get premature JunctionTurn assignments.
// 2. Cars queued on a green light (e.g. 4th and 5th cars) do not halt at the stop line
//    due to cross-street vehicles waiting at a red light.
func TestGreenLightQueueVehiclesDoNotStopForCrossRedTraffic(t *testing.T) {
	lw := 52.0
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 2},
			{P1: sim.Vec2{X: 0, Y: -2000}, P2: sim.Vec2{X: 0, Y: 2000}, Lanes: 2},
		},
	}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	inter := tls.Intersections[0]
	// Set EW Green (NS is Red)
	inter.Phase = sim.PhaseEWGreen
	inter.Timer = 2.0
	inter.GreenDuration = 15.0
	inter.YellowDuration = 3.0

	tm := &TrafficManager{}

	// Cross-street car on NS facing RED, decelerating to a stop at the stop line
	stopDistNS := inter.GetStopLineDistForHeading(math.Pi/2, lw)
	crossCar := &sim.TrafficCar{
		ID:        10,
		Pos:       sim.Vec2{X: -lw / 2, Y: -stopDistNS - 10},
		Heading:   math.Pi / 2,
		Speed:     12.0,
		Length:    48,
		Width:     22,
		LaneIndex: 0,
		IsEW:      false,
	}
	crossFrame := carFrame{car: crossCar, world: w, tls: tls, laneWidth: lw, index: 0, activeCount: 1}

	// 1. Verify turnCarAtIntersection does not set JunctionTurn for red-light car
	tm.turnCarAtIntersection(crossFrame)
	if crossCar.JunctionTurn != nil {
		t.Fatalf("turnCarAtIntersection must not create JunctionTurn for car facing red light")
	}

	// Now place cross car right at the stop line (gap <= 0)
	crossCar.Pos = sim.Vec2{X: -lw / 2, Y: -stopDistNS}
	crossCar.Speed = 0

	// Create EW green cars:
	// Car 1 has passed the intersection, Car 2 is exiting, Car 3 is in junction,
	// Car 4 is approaching the stop line, Car 5 is behind Car 4.
	stopDistEW := inter.GetStopLineDistForHeading(0, lw)
	car4 := &sim.TrafficCar{
		ID:          4,
		Pos:         sim.Vec2{X: -stopDistEW - 30, Y: lw / 2},
		Heading:     0,
		Speed:       70.0,
		TargetSpeed: 85.0,
		Length:      48,
		Width:       22,
		LaneIndex:   0,
		IsEW:        true,
	}
	car5 := &sim.TrafficCar{
		ID:          5,
		Pos:         sim.Vec2{X: -stopDistEW - 90, Y: lw / 2},
		Heading:     0,
		Speed:       65.0,
		TargetSpeed: 85.0,
		Length:      48,
		Width:       22,
		LaneIndex:   0,
		IsEW:        true,
	}

	tm.Cars = []*sim.TrafficCar{crossCar, car4, car5}
	f4 := carFrame{car: car4, world: w, tls: tls, laneWidth: lw, index: 1, activeCount: 3}
	f5 := carFrame{car: car5, world: w, tls: tls, laneWidth: lw, index: 2, activeCount: 3}

	// 2. Verify signalSpeed does not stop Car 4 or Car 5 on green
	spd4 := tm.signalSpeed(f4, 85.0)
	if spd4 < 50.0 || car4.IsBraking {
		t.Fatalf("Car 4 on green must not brake for red-light cross traffic, got spd=%.1f braking=%v", spd4, car4.IsBraking)
	}

	spd5 := tm.signalSpeed(f5, 85.0)
	if spd5 < 50.0 || car5.IsBraking {
		t.Fatalf("Car 5 on green must not brake for red-light cross traffic, got spd=%.1f braking=%v", spd5, car5.IsBraking)
	}

	// 3. Verify junctionQueueSpeed does not halt Car 4 or Car 5
	qSpd4 := tm.junctionQueueSpeed(f4, 85.0)
	if qSpd4 < 50.0 {
		t.Fatalf("junctionQueueSpeed must not stop Car 4 at stop line when light is green and cross traffic is red, got %.1f", qSpd4)
	}

	qSpd5 := tm.junctionQueueSpeed(f5, 85.0)
	if qSpd5 < 50.0 {
		t.Fatalf("junctionQueueSpeed must not stop Car 5 at stop line, got %.1f", qSpd5)
	}
}

// TestRoundaboutExitDoesNotStopForAdjacentEntranceTraffic verifies that an exiting car
// does not brake or stop due to oncoming vehicles in the adjacent entrance lane.
func TestRoundaboutExitDoesNotStopForAdjacentEntranceTraffic(t *testing.T) {
	lw := 52.0
	rbCenter := sim.Vec2{X: 0, Y: 0}
	rb := sim.Roundabout{
		Center:      rbCenter,
		Radius:      240,
		InnerRadius: 120,
	}
	w := &sim.World{
		Roundabouts: []sim.Roundabout{rb},
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: 0, Y: -2000}, P2: sim.Vec2{X: 0, Y: 2000}, Lanes: 2},
		},
	}

	// Exiting car heading North (heading = -math.Pi/2) along the exit lane (X = +lw/2 = +26 in right-hand traffic)
	exitingCar := &sim.TrafficCar{
		ID:                    1,
		Pos:                   sim.Vec2{X: lw / 2, Y: -rb.Radius - 20},
		Heading:               -math.Pi / 2,
		Speed:                 90.0,
		TargetSpeed:           100.0,
		Length:                48,
		Width:                 22,
		InRoundabout:          true,
		RoundaboutCenter:      rbCenter,
		RoundaboutOuterRadius: rb.Radius,
		RoundaboutExitHeading: -math.Pi / 2,
		RoundaboutExitOffset:  lw / 2, // In road exit coordinates
		RoundaboutExitRC:      0,
		RoundaboutExitLane:    0,
		RoundaboutCurve: &sim.CarRoundaboutCurve{
			Exiting: true,
		},
	}

	// Oncoming car in the entrance lane (X = -lw/2 = -26), heading South (+math.Pi/2) towards roundabout
	oncomingCar := &sim.TrafficCar{
		ID:          2,
		Pos:         sim.Vec2{X: -lw / 2, Y: -rb.Radius - 60},
		Heading:     math.Pi / 2,
		Speed:       40.0,
		Length:      48,
		Width:       22,
		LaneIndex:   0,
		RoadCenter:  0,
	}

	tm := &TrafficManager{
		Cars: []*sim.TrafficCar{exitingCar, oncomingCar},
	}
	f := carFrame{
		car:         exitingCar,
		world:       w,
		laneWidth:   lw,
		index:       0,
		activeCount: 2,
	}

	spd := tm.roundaboutSpeed(f, 100.0)
	if spd < 90.0 || exitingCar.IsBraking {
		t.Fatalf("Exiting car must not brake for oncoming vehicle in adjacent entrance lane, got spd=%.1f braking=%v", spd, exitingCar.IsBraking)
	}

	// Now place another car DIRECTLY ahead in the SAME exit lane (X = +lw/2, Y = -rb.Radius - 70)
	leadCar := &sim.TrafficCar{
		ID:          3,
		Pos:         sim.Vec2{X: lw / 2, Y: -rb.Radius - 70},
		Heading:     -math.Pi / 2,
		Speed:       30.0,
		Length:      48,
		Width:       22,
		LaneIndex:   0,
	}
	tm.Cars = []*sim.TrafficCar{exitingCar, leadCar}
	f.activeCount = 2

	spdBehindLead := tm.roundaboutSpeed(f, 100.0)
	if spdBehindLead >= 100.0 || !exitingCar.IsBraking {
		t.Fatalf("Exiting car SHOULD brake behind a slower car directly ahead in its own exit lane, got spd=%.1f braking=%v", spdBehindLead, exitingCar.IsBraking)
	}
}
