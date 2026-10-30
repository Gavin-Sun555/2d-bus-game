package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Check the whole turning envelope before the adjacent-lane filters used on
// straight roads. Circulating cars have priority even when currently beside us.
func (ad *AutopilotDriver) roundaboutEntryObstacleSpeed(f autopilotFrame) float64 {
	if f.world == nil || len(f.obstacles) == 0 {
		return math.Inf(1)
	}
	// No borrowed observation survives this synchronous scan.
	defer func() {
		if ad == nil {
			return
		}
		if scratch := ad.temporaryGeometry; scratch != nil {
			scratch.ringIndices = scratch.ringIndices[:0]
			if cap(scratch.ringIndices) > maxTemporaryObstacles {
				scratch.ringIndices = nil
			}
			if cap(scratch.ringBodies) > maxTemporaryObstacles {
				scratch.ringBodies = nil
			}
		}
	}()
	limit := math.Inf(1)
	for i := range f.world.Roundabouts {
		rb := &f.world.Roundabouts[i]
		lookahead := math.Max(180, f.speedAbs*f.speedAbs/(2*135)+100)
		if f.bus.Pos.Distance(rb.Center) > rb.Radius+lookahead {
			continue
		}
		offset := roundaboutLaneOffset(rb, f.bus, ad.LaneOffset)
		points, forward, ok := ad.roundaboutEntryCurve(f, rb, offset)
		if !ok {
			continue
		}
		s := entryPosition(points, f.bus.Pos, forward)
		staticRange := roundaboutStaticEntryRange(f, points, forward, s)
		// Hold with the whole body outside circulating traffic. Once committed,
		// continue through the gap instead of stopping in front of a following car.
		yieldDistance := -rb.Radius - f.bus.Length/2 - 16 - f.bus.Pos.Sub(rb.Center).Dot(forward)
		scratch := ad.geometryScratch()
		traffic := scratch.ringBodies[:0]
		if cap(traffic) < len(f.obstacles) {
			traffic = make([]sim.OBB, 0, len(f.obstacles))
		}
		selected := scratch.ringIndices[:0]
		if cap(selected) < len(f.obstacles) {
			selected = make([]int, 0, len(f.obstacles))
		}
		parallel := ad.parallelRoundaboutJoin(f, rb)
		var separation sim.RoundaboutEntrySeparation
		if parallel {
			var geometry *sim.RoundaboutEntryGeometry
			if !ad.disableRoundaboutEntryGeometry {
				geometry = &ad.roundaboutEntryGeometry
			}
			separation = sim.PrepareRoundaboutEntrySeparation(*rb, forward, f.laneWidth, sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length / 2, HalfWid: f.bus.Width / 2}, true, geometry)
		}
		for obstacleIndex, obstacle := range f.obstacles {
			// Keep local body clearance while an adjacent car clears the mouth,
			// including the first frames after it starts moving.
			if parallel && separation.Clears(sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}, false) {
				continue
			}
			body := roundaboutObstacleAt(rb, obstacle, 0)
			if !staticRange.excludes(body) {
				traffic = append(traffic, body)
			}
			selected = append(selected, obstacleIndex)
		}
		scratch.ringBodies, scratch.ringIndices = traffic, selected
		if yieldDistance >= -2 && entryHasMovingConflict(f, rb, points, forward, s, selected) {
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
func entryHasMovingConflict(f autopilotFrame, rb *sim.Roundabout, points []sim.Vec2, forward sim.Vec2, s float64, selection ...[]int) bool {
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += points[i].Distance(points[i-1])
	}
	if !(s < total-2) {
		return false
	}
	var priorityStorage [128]int
	priorityReady := false
	if len(points) >= 2 {
		if priority, ok := roundaboutMovingCandidates(f, rb, priorityStorage[:0], selection...); ok {
			if len(priority) == 0 {
				return false
			}
			selection = [][]int{priority}
			priorityReady = true
		}
	}
	for distance := 0.0; s+distance < total-2; distance += 8 {
		position := entryPointAt(points, forward, s+distance)
		tangent := entryPointAt(points, forward, s+distance+2).Sub(position)
		body := sim.OBB{Center: position, Heading: math.Atan2(tangent.Y, tangent.X), HalfLen: f.bus.Length/2 + 12, HalfWid: f.bus.Width/2 + 10}
		arrival := roundaboutEntryArrival(distance, f.speedAbs)
		count := len(f.obstacles)
		if len(selection) > 0 {
			count = len(selection[0])
		}
		for i := 0; i < count; i++ {
			index := i
			if len(selection) > 0 {
				index = selection[0][i]
			}
			obstacle := f.obstacles[index]
			// Only circulating traffic has priority here. A following vehicle
			// on the approach must not make the bus stop at the yield line.
			if !priorityReady {
				obstacleBody := sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}
				if !sim.HasRoundaboutPriority(*rb, obstacleBody) {
					continue
				}
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
	body := sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length/2 + 4, HalfWid: obstacle.Width/2 + 2}
	return sim.PredictRoundaboutBody(*rb, body, obstacle.Speed, elapsed)
}

// Use the low cornering speed and allow for acceleration from a waiting start.
func roundaboutEntryArrival(distance, speed float64) float64 {
	return sim.RoundaboutEntryArrival(distance, speed)
}

func separateRoundaboutEntry(f autopilotFrame, rb *sim.Roundabout, forward sim.Vec2, obs VehicleObstacle) bool {
	return sim.SeparateRoundaboutEntries(*rb, forward, f.laneWidth,
		sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length / 2, HalfWid: f.bus.Width / 2}, true,
		sim.OBB{Center: obs.Pos, Heading: obs.Heading, HalfLen: obs.Length / 2, HalfWid: obs.Width / 2}, false)
}
