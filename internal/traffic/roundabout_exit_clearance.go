package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// An inner-ring exit crosses the outer circulating lane. Check that crossing
// before the ordinary exit-road headway filters discard a turning bus whose
// heading or lane differs from the destination road. Include both bus sections.
func roundaboutExitBusSpeed(f carFrame, rb sim.Roundabout) float64 {
	car := f.car
	if !car.InRoundabout {
		return math.Inf(1)
	}
	speed := math.Max(20, car.Speed)
	horizon := math.Max(.8, speed/200+.45)
	if car.RoundaboutCurve != nil && !car.RoundaboutCurve.Exiting {
		return math.Inf(1)
	}
	if car.RoundaboutCurve == nil {
		gap := sim.NormalizeAngle(car.RoundaboutAngle - car.RoundaboutExitAngle)
		if gap < 0 {
			gap += 2 * math.Pi
		}
		if car.RoundaboutTraveled+speed*horizon/math.Max(1, car.RoundaboutRadius) < 1 || gap > .65+speed*horizon/math.Max(1, car.RoundaboutRadius) {
			return math.Inf(1)
		}
	}
	limit := math.Inf(1)
	for _, bus := range f.buses {
		if bus == nil || car.Pos.Distance(bus.Pos) > speed*horizon+bus.OverallLength()+math.Abs(bus.Speed)*horizon+80 {
			continue
		}
		predicted := *car
		if car.RoundaboutCurve != nil {
			curve := *car.RoundaboutCurve
			predicted.RoundaboutCurve = &curve
		}
		predicted.Speed = speed
		for elapsed := .04; elapsed <= horizon; elapsed += .04 {
			moveCarRoundabout(&predicted, .04)
			if !predicted.InRoundabout {
				break
			}
			// Only the outward crossing needs this reservation. Ordinary circulation
			// retains its existing headway/parallel-lane rules.
			if predicted.RoundaboutCurve == nil || !predicted.RoundaboutCurve.Exiting {
				continue
			}
			body := sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length/2 + 3, HalfWid: car.Width/2 + 3}
			blocked := false
			for _, part := range bus.BodyParts() {
				if part.HalfLen <= 0 {
					continue
				}
				other := sim.PredictRoundaboutBody(rb, part, math.Max(0, bus.Speed), elapsed)
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					blocked = true
					break
				}
			}
			if blocked {
				limit = math.Min(limit, math.Sqrt(2*200*math.Max(0, speed*elapsed-14)))
				break
			}
		}
	}
	return limit
}
