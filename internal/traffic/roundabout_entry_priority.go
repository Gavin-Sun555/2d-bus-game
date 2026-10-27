package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Joining the approach curve does not grant priority over circulating traffic.
// Re-evaluate while outside, then let an admitted car clear the mouth.
func (tm *TrafficManager) roundaboutEntryPrioritySpeed(f carFrame, rb sim.Roundabout) float64 {
	car := f.car
	curve := car.RoundaboutCurve
	if curve == nil || curve.Exiting {
		return math.Inf(1)
	}
	forward := sim.UnitFromAngle(car.RoundaboutExitHeading)
	yaw := car.Heading - car.RoundaboutExitHeading
	front := car.Length/2*math.Abs(math.Cos(yaw)) + car.Width/2*math.Abs(math.Sin(yaw))
	gap := rb.Center.Sub(car.Pos).Dot(forward) - front - rb.YieldLineDistance()
	if gap < -2 {
		return math.Inf(1)
	}
	conflict := func(body sim.OBB, speed float64) bool {
		if !sim.HasRoundaboutPriority(rb, body) {
			return false
		}
		predicted := *car
		c := *curve
		predicted.RoundaboutCurve = &c
		predicted.Speed = sim.RoundaboutSpeedLimit
		remaining := curve.ArcLengths[64] - curve.Distance + 24
		for distance := 0.; distance <= remaining; distance += 8 {
			own := sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length/2 + 5, HalfWid: car.Width/2 + 4}
			arrival := sim.RoundaboutEntryArrival(distance, math.Abs(car.Speed))
			for time := math.Max(0, arrival-.6); time <= arrival+.8; time += .1 {
				other := sim.PredictRoundaboutBody(rb, body, speed, time)
				other.HalfLen += 4
				other.HalfWid += 2
				if hit, _, _ := sim.CheckOBBCollision(own, other); hit {
					return true
				}
			}
			moveCarRoundabout(&predicted, 8/sim.RoundaboutSpeedLimit)
		}
		return false
	}
	for i, other := range tm.Cars {
		if i != f.index && i < f.activeCount && conflict(sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}, other.Speed) {
			return math.Sqrt(2 * 220 * math.Max(0, gap-8))
		}
	}
	for _, bus := range f.buses {
		if bus != nil && conflict(bus.BodyEnvelope(), bus.Speed) {
			return math.Sqrt(2 * 220 * math.Max(0, gap-8))
		}
	}
	return math.Inf(1)
}
