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
