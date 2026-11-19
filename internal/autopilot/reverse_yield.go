package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type rearReverseYield struct {
	cars  []*sim.TrafficCar
	buses []*sim.Bus
}

func rearReverseYieldFrame(f autopilotFrame) (autopilotFrame, rearReverseYield) {
	return rearReverseYieldFrameWithHeading(f, approachRoadHeading(f.bus.Heading), false)
}

func rearReverseYieldFrameWithHeading(f autopilotFrame, roadHeading float64, includeSideBus bool) (autopilotFrame, rearReverseYield) {
	u := sim.UnitFromAngle(roadHeading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	trial := f
	trial.obstacles = nil
	var requests rearReverseYield
	for _, ob := range f.obstacles {
		d := ob.Pos.Sub(f.bus.Pos)
		maxAhead := -f.bus.Length / 2
		minCos := .92
		if includeSideBus && ob.Bus != nil {
			maxAhead = 0
			minCos = .75
		}
		behind := math.Abs(ob.Speed) <= 1 &&
			(math.Cos(ob.Heading-f.bus.Heading) > .9 || math.Cos(ob.Heading-roadHeading) > minCos) &&
			d.Dot(u) < maxAhead && d.Dot(u) > -f.bus.OverallLength()-ob.Length-100 && math.Abs(d.Dot(r)) < f.laneWidth*2.5
		if behind && ob.Car != nil && ob.Car.JunctionTurn == nil {
			requests.cars = append(requests.cars, ob.Car)
		} else if behind && ob.Bus != nil && !ob.Bus.DoorsOpen && ob.Bus.PassengerServiceRemaining() == 0 {
			requests.buses = append(requests.buses, ob.Bus)
		} else {
			trial.obstacles = append(trial.obstacles, ob)
		}
	}
	return trial, requests
}

func (r rearReverseYield) empty() bool { return len(r.cars)+len(r.buses) == 0 }

func (r rearReverseYield) send() {
	for _, c := range r.cars {
		if c.YieldReverseRequest <= 0 && c.ReverseRemaining <= 0 && c.ReverseHold <= 0 {
			c.YieldReverseDelay = .6
		}
		c.YieldReverseRequest = .6
	}
	for _, b := range r.buses {
		b.ReverseYieldRequest = .6
	}
}

// Only fleet automation accepts a relay. A vehicle never starts backing until
// its complete real reverse sweep is clear; a hypothetical sweep can merely ask
// stationary rear traffic to open a gap, with the same safety checks downstream.
func (ad *AutopilotDriver) requestedReverseYieldDistance(f autopilotFrame) float64 {
	dist, _ := ad.requestedReverseYieldPlan(f)
	return dist
}

func (ad *AutopilotDriver) requestedReverseYieldPlan(f autopilotFrame) (float64, float64) {
	if !ad.FleetBus || f.bus.ReverseYieldRequest <= 0 || f.world == nil || f.speedAbs > 1 || f.bus.DoorsOpen ||
		f.bus.PassengerServiceRemaining() > 0 || ad.State == AutopilotStateDocked || f.nearRoundabout ||
		f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) || ad.trafficLightSpeed(f, 30) <= .5 {
		return 0, 0
	}
	heading := approachRoadHeading(f.bus.Heading)
	maxYaw, maxTailYaw := .3, .3
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > maxYaw || math.Abs(sim.NormalizeAngle(f.bus.TailHeading()-heading)) > maxTailYaw {
		return 0, 0
	}
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	steers := []float64{0}
	if math.Abs(yaw) > .08 {
		s := sim.Clamp(2*yaw, -.8, .8)
		steers = append(steers, s, -s)
	}
	for _, distance := range []float64{60, 40, 24, 16, 8} {
		for _, st := range steers {
			if _, safe := ad.simulateReversePose(f, distance, st); safe {
				return distance, st
			}
		}
	}
	trial, requests := rearReverseYieldFrame(f)
	if !requests.empty() {
		for _, distance := range []float64{60, 40, 24, 16, 8} {
			for _, st := range steers {
				if _, safe := ad.simulateReversePose(trial, distance, st); safe {
					requests.send()
					return 0, 0
				}
			}
		}
	}
	return 0, 0
}
