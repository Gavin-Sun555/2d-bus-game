package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// The rear corner is already beside the car's nose. Rejecting all clipped
// points behind that nose loses the obstacle until after physical contact.
func TestCarBrakesForArticulatedTailAlongsideNose(t *testing.T) {
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(angle), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(angle) - v.Y*math.Sin(angle), Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle)}
			}
			p := rotate(sim.Vec2{X: -271.465363, Y: -3498.193479})
			b := sim.NewBus(p.X, p.Y, .020746+angle)
			b.SetModel(sim.BusArticulated)
			b.TrailerHeading = -.186370 + angle
			c := &sim.TrafficCar{Pos: rotate(sim.Vec2{X: -404.684828, Y: -3522}), Heading: angle, Speed: 72, Length: 46, Width: 22}
			tm := &TrafficManager{}
			target := tm.busFollowingSpeed(carFrame{car: c, buses: []*sim.Bus{b}}, 150)
			if target != 0 || !c.IsBraking {
				t.Fatalf("tail corridor not contained: speed%.2f brake%v", target, c.IsBraking)
			}
		})
	}
}

func TestCarPassesClearParallelArticulatedSections(t *testing.T) {
	b := sim.NewBus(0, 52, 0)
	b.SetModel(sim.BusArticulated)
	c := &sim.TrafficCar{Pos: sim.Vec2{X: -100}, Heading: 0, Speed: 90, Length: 46, Width: 22}
	tm := &TrafficManager{}
	if speed := tm.busFollowingSpeed(carFrame{car: c, buses: []*sim.Bus{b}}, 150); speed != 150 {
		t.Fatalf("clear neighboring lane blocked: %.2f", speed)
	}
}

func TestLeadingCarClearsStoppedArticulatedMerge(t *testing.T) {
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(angle), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(angle) - v.Y*math.Sin(angle), Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle)}
			}
			b := sim.NewBus(0, 0, 2.957261+angle)
			b.SetModel(sim.BusArticulated)
			b.TrailerHeading = 3.131766 + angle
			c := &sim.TrafficCar{Pos: rotate(sim.Vec2{X: -31.255812, Y: 43.535086}), Heading: math.Pi + angle, Length: 46, Width: 22}
			tm := &TrafficManager{}
			if v := tm.busFollowingSpeed(carFrame{car: c, buses: []*sim.Bus{b}}, 150); v != 150 {
				t.Fatalf("leading car cannot release merge: %.2f", v)
			}
			// Moving the same car into the real front section must still stop it.
			c.Pos = rotate(sim.Vec2{X: -31.255812, Y: 15})
			if carClearsStoppedBusAhead(c, b) {
				t.Fatal("overlapping path admitted")
			}
			c.Pos = rotate(sim.Vec2{X: -31.255812, Y: 43.535086})
			b.Speed = 20
			if carClearsStoppedBusAhead(c, b) {
				t.Fatal("moving bus granted stationary clearance")
			}
		})
	}
}

func TestLeadingCarClearsStoppedStationMergeBetweenTwoArticulatedBuses(t *testing.T) {
	player := sim.NewBus(-13359.862722606757, -42.01754412813188, -3.0277862000510196)
	player.SetModel(sim.BusArticulated)
	player.TrailerHeading = -3.066952296879403
	player.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 74, Remaining: 0.283333}

	ai0 := sim.NewBus(-13324.14873746123, -89.70680791808735, -3.0274233117392746)
	ai0.SetModel(sim.BusArticulated)
	ai0.TrailerHeading = -3.0843070880569625
	ai0.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 74, Remaining: 0.283333}

	car := &sim.TrafficCar{
		Pos:     sim.Vec2{X: -13398.240526926289, Y: -77.99999999999993},
		Heading: math.Pi,
		Length:  46,
		Width:   22,
	}

	if !carClearsStoppedBusAhead(car, player) || !carClearsStoppedBusAhead(car, ai0) {
		t.Fatal("leading car ahead of both merging articulated buses must clear to release mutual station merge jam")
	}
	tm := &TrafficManager{}
	if got := tm.busFollowingSpeed(carFrame{car: car, buses: []*sim.Bus{player, ai0}}, 166.7); got != 166.7 {
		t.Fatalf("expected leading car to keep target speed 166.7, got %.2f", got)
	}
}

func TestCarCreepsPastClearParallelStoppedBusCab(t *testing.T) {
	b := sim.NewBus(-7744.72, 4169.19, -1.5677)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = -1.5577
	car := &sim.TrafficCar{
		ID:      5958,
		Pos:     sim.Vec2{X: -7774.0, Y: 4187.33},
		Heading: -math.Pi / 2,
		Length:  46,
		Width:   22,
	}
	tm := &TrafficManager{}
	if got := tm.busFollowingSpeed(carFrame{car: car, buses: []*sim.Bus{b}}, 150); got != 45 {
		t.Fatalf("expected clear parallel stopped car to creep at 45, got %.2f", got)
	}
	// An active lane merge still lets an already-alongside non-yielding car clear first.
	b77 := sim.NewBus(-7952.5, 2643.8, 1.564)
	b77.SetModel(sim.BusArticulated)
	b77.TrailerHeading = 1.532
	b77.LaneMergeRequest = &sim.BusLaneMergeRequest{Heading: math.Pi / 2, Target: sim.Vec2{X: -7982}, Remaining: 0.3}
	c9757 := &sim.TrafficCar{
		ID:      9757,
		Pos:     sim.Vec2{X: -7982.0, Y: 2632.8},
		Heading: math.Pi / 2,
		Length:  46,
		Width:   22,
	}
	if got := tm.busFollowingSpeed(carFrame{car: c9757, buses: []*sim.Bus{b77}}, 150); got != 45 {
		t.Fatalf("expected alongside non-yielding car to clear merging bus at 45, got %.2f", got)
	}
	// If the car overlaps laterally (same-lane obstruction), it must still stop.
	car.Pos.X = -7760.0
	if got := tm.busFollowingSpeed(carFrame{car: car, buses: []*sim.Bus{b}}, 150); got != 0 {
		t.Fatalf("expected colliding car to stay stopped, got %.2f", got)
	}
}

func TestCarAlongsideStationArrivalCanClearOppositeSide(t *testing.T) {
	bus := sim.NewBus(-35.5054, 0, 1.78278)
	bus.StationMergeRequest = &sim.BusStationMergeRequest{Remaining: .3}
	bus.LaneMergeRequest = &sim.BusLaneMergeRequest{
		Target: sim.Vec2{X: -47.12, Y: 88}, Heading: math.Pi / 2, Remaining: .3,
	}
	car := &sim.TrafficCar{Pos: sim.Vec2{Y: 21.5338}, Heading: math.Pi / 2, Length: 46, Width: 22}
	tm := &TrafficManager{}
	frame := carFrame{car: car, buses: []*sim.Bus{bus}}
	if got := tm.busFollowingSpeed(frame, 150); got != 45 {
		t.Fatalf("expected clear car beside the arriving bus to creep, speed limit %.2f", got)
	}
	bus.LaneMergeRequest.Target.X = car.Pos.X
	if got := tm.busFollowingSpeed(frame, 150); got == 45 {
		t.Fatal("car used the arrival exception on the side the bus is entering")
	}
}

func TestCarAlongsideStraightWaitingTrailerCanClear(t *testing.T) {
	bus := sim.NewBus(0, -29.21, math.Pi)
	bus.SetModel(sim.BusArticulated)
	bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Heading: math.Pi, Remaining: .3, LeftTurn: true}
	car := &sim.TrafficCar{Pos: sim.Vec2{X: 82.18}, Heading: math.Pi, Length: 46, Width: 22}
	tm := &TrafficManager{}
	frame := carFrame{car: car, buses: []*sim.Bus{bus}}
	if got := tm.busFollowingSpeed(frame, 150); got != 45 {
		t.Fatalf("expected clear car beside the trailer to creep, speed limit %.2f", got)
	}
	car.Pos.Y = -1
	if got := tm.busFollowingSpeed(frame, 150); got != 0 {
		t.Fatalf("expected car in the body's swept path to hold, speed limit %.2f", got)
	}
}

func TestAlongsideCarsClearOppositeSideOfStationDeparture(t *testing.T) {
	bus := sim.NewBus(0, 29.44524, .00252)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -.04738
	bus.StationMergeRequest = &sim.BusStationMergeRequest{Departing: true, Remaining: .3}
	bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 47.12}, Remaining: .3}
	car := &sim.TrafficCar{Pos: sim.Vec2{X: 32.75765}, Length: 46, Width: 22}
	tm := &TrafficManager{}
	frame := carFrame{car: car, buses: []*sim.Bus{bus}}
	if got := tm.busFollowingSpeed(frame, 150); got != 45 {
		t.Fatalf("leading car should clear the side of the stopped bus at a crawl, got %.2f", got)
	}
	car.Pos.X = -31.10782
	if got := tm.busFollowingSpeed(frame, 150); got != 45 {
		t.Fatalf("alongside following car should also clear at a crawl, got %.2f", got)
	}
	car.Pos.X = 32.75765
	car.Pos.Y = 2
	if got := tm.busFollowingSpeed(frame, 150); got != 0 {
		t.Fatalf("car overlapping the bus should remain stopped, got %.2f", got)
	}
}

func TestLeadingCarClearsBusCorrectingIntoAdjacentTurnLane(t *testing.T) {
	bus := sim.NewBus(0, 0, -.3893)
	bus.SetModel(sim.BusDoubleDecker)
	bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: -24.3}, LeftTurn: true, Remaining: .3}
	car := &sim.TrafficCar{Pos: sim.Vec2{X: 38.77, Y: 27.69}, Length: 46, Width: 22}
	tm := &TrafficManager{}
	frame := carFrame{car: car, buses: []*sim.Bus{bus}}
	if got := tm.busFollowingSpeed(frame, 150); got != 45 {
		t.Fatalf("clear car already ahead should leave at a crawl, got %.2f", got)
	}
	bus.LaneMergeRequest.Target.Y = car.Pos.Y
	if carClearsParallelStoppedBus(car, bus) {
		t.Fatal("car used the escape on the side the bus was entering")
	}
}

func TestCarClearsWhenStoppedBusStraddlesDivider(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -1000}, P2: sim.Vec2{X: 1000}, Lanes: 2}}}
	for _, tc := range []struct{ side, yaw, tail, along float64 }{
		{side: -12.05, yaw: .04975, tail: .08076, along: 9.80},
		{side: -8.59, yaw: -.36758, tail: -.70071, along: 21.35},
	} {
		bus := sim.NewBus(0, tc.side, tc.yaw)
		bus.SetModel(sim.BusArticulated)
		bus.TrailerHeading = tc.tail
		car := &sim.TrafficCar{Pos: sim.Vec2{X: tc.along, Y: 26}, Length: 46, Width: 22, IsEW: true}
		tm := &TrafficManager{}
		f := carFrame{car: car, world: w, laneWidth: 52, buses: []*sim.Bus{bus}}
		if got := tm.busFollowingSpeed(f, 150); got != 45 {
			t.Fatalf("clear path beside the straddling cab: speed %.2f", got)
		}
		car.Pos.Y = 12
		if carClearsStoppedBusAcrossDivider(f, bus) {
			t.Fatal("accepted a path that intersects the angled bus")
		}
	}
}
