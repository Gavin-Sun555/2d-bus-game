package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Check the whole turning envelope before the adjacent-lane filters used on
// straight roads. Circulating cars have priority even when currently beside us.
func (ad *AutopilotDriver) innerEntryObstacleSpeed(f autopilotFrame) float64 {
	if !ad.BerserkMode || f.world == nil || len(f.obstacles) == 0 {
		return math.Inf(1)
	}
	limit := math.Inf(1)
	for i := range f.world.Roundabouts {
		rb := &f.world.Roundabouts[i]
		lookahead := math.Max(180, f.speedAbs*f.speedAbs/(2*135)+100)
		if f.bus.Pos.Distance(rb.Center) > rb.Radius+lookahead {
			continue
		}
		offset := roundaboutLaneOffset(rb, f.bus, ad.LaneOffset)
		points, forward, ok := ad.innerRoundaboutEntryCurve(f, rb, offset)
		if !ok {
			continue
		}
		s := entryPosition(points, f.bus.Pos, forward)
		// Hold with the whole body outside circulating traffic. Once committed,
		// continue through the gap instead of stopping in front of a following car.
		yieldDistance := -rb.Radius - f.bus.Length/2 - 16 - f.bus.Pos.Sub(rb.Center).Dot(forward)
		traffic := make([]sim.OBB, 0, len(f.obstacles))
		for _, obstacle := range f.obstacles {
			if math.Abs(obstacle.Speed) < 5 {
				traffic = append(traffic, roundaboutObstacleAt(rb, obstacle, 0))
			}
		}
		if yieldDistance >= -2 && entryHasMovingConflict(f, rb, points, forward, s) {
			limit = math.Min(limit, math.Sqrt(2*135*math.Max(0, yieldDistance-20)))
		}

		startTangent := entryPointAt(points, forward, s+2).Sub(entryPointAt(points, forward, s))
		positionError := f.bus.Pos.Sub(entryPointAt(points, forward, s))
		headingError := sim.NormalizeAngle(f.bus.Heading - math.Atan2(startTangent.Y, startTangent.X))
		for distance := 0.0; distance <= lookahead; distance += 5 {
			position := entryPointAt(points, forward, s+distance)
			tangent := entryPointAt(points, forward, s+distance+2).Sub(position)
			if tangent.Length() < 0.5 {
				break
			}
			body := sim.OBB{Center: position, Heading: math.Atan2(tangent.Y, tangent.X), HalfLen: f.bus.Length/2 + 10, HalfWid: f.bus.Width/2 + 8}
			blocked := false
			for _, other := range traffic {
				// Pursuit can lag the curve while turning. Preserve the current
				// yaw error as a second envelope, including when braking to rest.
				actual := body
				actual.Heading += headingError
				actual.Center = actual.Center.Add(positionError)
				hit, _, _ := sim.CheckOBBCollision(body, other)
				laggedHit, _, _ := sim.CheckOBBCollision(actual, other)
				if hit || laggedHit {
					blocked = true
					break
				}
			}
			if blocked {
				limit = math.Min(limit, math.Sqrt(2*135*math.Max(0, distance-25)))
				break
			}
		}
	}
	return limit
}

// Compare arrival times at the merge, rather than treating the entire future
// arc as permanently occupied. This lets the bus use the gap after a car passes.
func entryHasMovingConflict(f autopilotFrame, rb *sim.Roundabout, points []sim.Vec2, forward sim.Vec2, s float64) bool {
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += points[i].Distance(points[i-1])
	}
	for distance := 0.0; s+distance < total-2; distance += 8 {
		position := entryPointAt(points, forward, s+distance)
		tangent := entryPointAt(points, forward, s+distance+2).Sub(position)
		body := sim.OBB{Center: position, Heading: math.Atan2(tangent.Y, tangent.X), HalfLen: f.bus.Length/2 + 12, HalfWid: f.bus.Width/2 + 10}
		arrival := roundaboutEntryArrival(distance, f.speedAbs)
		for _, obstacle := range f.obstacles {
			// Only circulating traffic has priority here. A following vehicle
			// on the approach must not make the bus stop at the yield line.
			radius := obstacle.Pos.Distance(rb.Center)
			if math.Abs(obstacle.Speed) < 5 || radius < rb.InnerRadius || radius > rb.Radius+10 {
				continue
			}
			for time := math.Max(0, arrival-0.6); time <= arrival+0.8; time += 0.08 {
				other := roundaboutObstacleAt(rb, obstacle, time)
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					return true
				}
			}
		}
	}
	return false
}

// Predict circulating traffic along its arc; approach traffic follows its heading.
func roundaboutObstacleAt(rb *sim.Roundabout, obstacle VehicleObstacle, elapsed float64) sim.OBB {
	radial := obstacle.Pos.Sub(rb.Center)
	radius := radial.Length()
	forward := sim.Vec2{X: math.Cos(obstacle.Heading), Y: math.Sin(obstacle.Heading)}
	tangent := sim.Vec2{X: radial.Y, Y: -radial.X}.Normalize()
	alignment := forward.Dot(tangent)
	position, heading := obstacle.Pos, obstacle.Heading
	if radius >= rb.InnerRadius && radius <= rb.Radius+10 && math.Abs(alignment) > 0.7 {
		turn := -math.Copysign(1, alignment) * obstacle.Speed * elapsed / radius
		c, s := math.Cos(turn), math.Sin(turn)
		position = rb.Center.Add(sim.Vec2{X: radial.X*c - radial.Y*s, Y: radial.X*s + radial.Y*c})
		heading += turn
	} else {
		position = position.Add(forward.Mul(obstacle.Speed * elapsed))
	}
	return sim.OBB{Center: position, Heading: heading, HalfLen: obstacle.Length/2 + 4, HalfWid: obstacle.Width/2 + 2}
}

// Use the low cornering speed and allow for acceleration from a waiting start.
func roundaboutEntryArrival(distance, speed float64) float64 {
	const cruise = 90.0
	const acceleration = 100.0
	speed = math.Min(speed, cruise)
	ramp := (cruise - speed) / acceleration
	rampDistance := (speed + cruise) * ramp / 2
	if distance < rampDistance {
		return (math.Sqrt(speed*speed+2*acceleration*distance) - speed) / acceleration
	}
	return ramp + (distance-rampDistance)/cruise
}
