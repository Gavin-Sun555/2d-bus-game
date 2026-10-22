package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Let an outer-lane car clear a waiting neighbor only when its complete swept
// body clears that bus. Entering cars still yield to circulating buses; cars
// already on the ring may pass a stationary bus behind them in another lane.
func canPassWaitingRoundaboutBus(f carFrame, bus *sim.Bus) bool {
	if bus == nil || f.world == nil || math.Abs(bus.Speed) >= 5 {
		return false
	}
	car := f.car
	if car.LaneIndex == 0 || car.JunctionTurn != nil {
		return false
	}
	incoming := car.Heading
	if car.InRoundabout {
		incoming = car.RoundaboutExitHeading
	}
	travelHeading := incoming
	if car.InRoundabout {
		travelHeading = car.Heading
	}
	if math.Cos(bus.Heading-travelHeading) < .6 {
		return false
	}
	for _, rb := range f.world.Roundabouts {
		radial := bus.Pos.Sub(rb.Center)
		radius := radial.Length()
		if radius < rb.InnerRadius+bus.Width/2 || radius > rb.Radius+100 || car.Pos.Distance(rb.Center) > rb.Radius+100 {
			continue
		}
		tangent := sim.Vec2{X: radial.Y, Y: -radial.X}.Normalize()
		if !car.InRoundabout && (sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}).Dot(tangent) > .7 {
			continue
		}
		forward := sim.Vec2{X: math.Cos(incoming), Y: math.Sin(incoming)}
		if rb.Center.Sub(bus.Pos).Dot(forward) <= 0 {
			continue
		}
		body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + 2, HalfWid: bus.Width/2 + 2}
		predicted := *car
		if car.RoundaboutCurve != nil {
			curve := *car.RoundaboutCurve
			predicted.RoundaboutCurve = &curve
		}
		predicted.Speed = math.Max(70, car.Speed)
		// Cover the initial straight movement, radial join and the full mouth.
		for elapsed := 0.0; elapsed <= 2.4; elapsed += .04 {
			other := sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length/2 + 2, HalfWid: car.Width/2 + 2}
			if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
				return false
			}
			if !predicted.InRoundabout && predicted.Pos.Distance(rb.Center) <= rb.Radius+12 {
				predicted.InRoundabout = true
				predicted.RoundaboutCenter = rb.Center
				predicted.RoundaboutRadius = predicted.Pos.Distance(rb.Center)
				predicted.RoundaboutAngle = math.Atan2(predicted.Pos.Y-rb.Center.Y, predicted.Pos.X-rb.Center.X)
				predicted.RoundaboutTargetR = (rb.InnerRadius + 16 + 3*rb.Radius) / 4
				predicted.RoundaboutExitHeading = incoming
				predicted.RoundaboutTraveled = 0
				beginRoundaboutEntry(&predicted, rb, f.laneWidth)
			}
			moveCar(&predicted, .04)
		}
		return true
	}
	return false
}
