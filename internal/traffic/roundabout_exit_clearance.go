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
			margin := 3.0
			if math.Abs(bus.Speed) < 0.5 {
				margin = 0.5
				if bus.IsArticulated() {
					curBody := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length/2 + 2.5, HalfWid: car.Width/2 + 2.5}
					if hit, _, _ := bus.CollidesBody(curBody); !hit {
						margin = 2.5
					}
				}
			}
			body := sim.OBB{Center: predicted.Pos, Heading: predicted.Heading, HalfLen: car.Length/2 + margin, HalfWid: car.Width/2 + margin}
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

// Exit-road headway may ignore a circulating car whose heading differs from
// the exit. It still owns its actual body where an outward curve crosses it.
func (tm *TrafficManager) roundaboutExitCarSpeed(f carFrame) float64 {
	car := f.car
	if !car.InRoundabout || (car.RoundaboutCurve != nil && !car.RoundaboutCurve.Exiting) {
		return math.Inf(1)
	}
	speed := math.Max(20, car.Speed)
	horizon := math.Max(.8, speed/200+.45)
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
	for _, other := range tm.carsForFrame(f) {
		if other == nil || other == car || !other.InRoundabout || other.RoundaboutCenter != car.RoundaboutCenter ||
			(other.RoundaboutCurve != nil && other.RoundaboutCurve.Exiting) || other.Pos.Distance(car.Pos) > speed*horizon+math.Abs(other.Speed)*horizon+car.Length+other.Length+30 {
			continue
		}
		own, obstacle := *car, *other
		if car.RoundaboutCurve != nil {
			curve := *car.RoundaboutCurve
			own.RoundaboutCurve = &curve
		}
		if other.RoundaboutCurve != nil {
			curve := *other.RoundaboutCurve
			obstacle.RoundaboutCurve = &curve
		}
		own.Speed = speed
		for elapsed := .025; elapsed <= horizon; elapsed += .025 {
			moveCarRoundabout(&own, .025)
			if !own.InRoundabout {
				break
			}
			if math.Abs(obstacle.Speed) > .01 {
				moveCar(&obstacle, .025)
			}
			if own.RoundaboutCurve == nil || !own.RoundaboutCurve.Exiting {
				continue
			}
			margin := .5
			if math.Abs(other.Speed) > 1 {
				margin = 2
			}
			a := sim.OBB{Center: own.Pos, Heading: own.Heading, HalfLen: car.Length/2 + margin, HalfWid: car.Width/2 + margin}
			b := sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}
			if hit, _, _ := sim.CheckOBBCollision(a, b); hit {
				limit = math.Min(limit, math.Sqrt(2*200*math.Max(0, speed*elapsed-14)))
				break
			}
		}
	}
	return limit
}
