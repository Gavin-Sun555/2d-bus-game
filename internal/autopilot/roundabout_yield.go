package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Check the whole turning envelope before the adjacent-lane filters used on
// straight roads. Circulating cars have priority even when currently beside us.
func (ad *AutopilotDriver) roundaboutEntryObstacleSpeed(f autopilotFrame) float64 {
	return ad.roundaboutEntryObstacleLimit(f, false)
}

func (ad *AutopilotDriver) roundaboutEntryDriveSpeed(f autopilotFrame) float64 {
	return ad.roundaboutEntryObstacleLimit(f, f.bus != nil && f.bus.IsArticulated())
}

func (ad *AutopilotDriver) roundaboutEntryObstacleLimit(f autopilotFrame, articulatedHold bool) float64 {
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
		var curveBuf [33]sim.Vec2
		points, forward, ok := ad.roundaboutEntryCurveInto(curveBuf[:0], f, rb, offset)
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
		var ringStopped [128]bool
		for obstacleIndex, obstacle := range f.obstacles {
			body := roundaboutObstacleAt(rb, obstacle, 0)
			inStatic := !staticRange.excludes(body)
			hasPriority := yieldDistance >= -2 && sim.HasRoundaboutPriority(*rb, sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2})
			if !inStatic && !hasPriority {
				continue
			}
			// Keep local body clearance while an adjacent car clears the mouth,
			// including the first frames after it starts moving.
			if parallel && roundaboutEntrySeparated(&separation, obstacle) {
				continue
			}
			if !hasPriority && roundaboutStoppedRearClearsEntry(f, obstacle, points, forward, s, lookahead) {
				continue
			}
			if inStatic {
				if idx := len(traffic); idx < len(ringStopped) {
					ringStopped[idx] = hasPriority && math.Abs(obstacle.Speed) < 10
				}
				traffic = append(traffic, body)
			}
			if hasPriority {
				selected = append(selected, obstacleIndex)
			}
		}
		scratch.ringBodies, scratch.ringIndices = traffic, selected
		if yieldDistance >= -2 && articulatedHold && !rb.SupportsParallelArticulated() && roundaboutOccupiedByArticulatedBus(rb, f.bus, f.obstacles) {
			limit = math.Min(limit, math.Sqrt(2*135*math.Max(0, yieldDistance-20)))
		}
		if yieldDistance >= -2 && entryHasMovingConflict(f, rb, points, forward, s, articulatedHold, selected) {
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
			blockedByStoppedRing := false
			for idx, other := range traffic {
				// Pursuit can lag the curve while turning. Preserve the current
				// yaw error as a second envelope, including when braking to rest.
				actual := body
				actual.Heading += headingError
				actual.Center = actual.Center.Add(positionError)
				hit, _, _ := sim.CheckOBBCollision(body, other)
				laggedHit, _, _ := sim.CheckOBBCollision(actual, other)
				if hit || laggedHit {
					blocked = true
					if idx < len(ringStopped) && ringStopped[idx] {
						blockedByStoppedRing = true
					}
					break
				}
			}
			if blocked {
				if articulatedHold && yieldDistance >= -2 && distance <= 190 && blockedByStoppedRing {
					// Wait outside the yield line rather than pulling the front section
					// inside while leaving the trailer across upstream ring traffic.
					limit = math.Min(limit, math.Sqrt(2*135*math.Max(0, yieldDistance-20)))
				} else {
					limit = math.Min(limit, math.Sqrt(2*135*math.Max(0, distance-25)))
				}
				break
			}
		}
	}
	return limit
}

// A compact front bus can be physically clear of a staggered, stopped rear
// bus while the padded entry boxes still touch. Prove the complete local
// entry against the real rear bodies; circulating traffic retains priority.
func roundaboutStoppedRearClearsEntry(f autopilotFrame, ob VehicleObstacle, points []sim.Vec2, forward sim.Vec2, s, lookahead float64) bool {
	if f.bus.IsArticulated() || ob.Bus == nil || math.Abs(ob.Speed) > 1 || f.speedAbs > 30 ||
		ob.Bus.Pos.Sub(f.bus.Pos).Dot(forward) > -f.bus.Length/2 || math.Cos(ob.Bus.Heading-math.Atan2(forward.Y, forward.X)) < .98 {
		return false
	}
	start := entryPointAt(points, forward, s)
	dir := entryPointAt(points, forward, s+2).Sub(start)
	if dir.Length() < .5 {
		return false
	}
	positionError := f.bus.Pos.Sub(start)
	yawError := sim.NormalizeAngle(f.bus.Heading - math.Atan2(dir.Y, dir.X))
	for distance := 0.; distance <= lookahead; distance += 4 {
		p := entryPointAt(points, forward, s+distance)
		tangent := entryPointAt(points, forward, s+distance+2).Sub(p)
		if tangent.Length() < .5 {
			break
		}
		pose := f.bus.AtPose(p.Add(positionError), math.Atan2(tangent.Y, tangent.X)+yawError)
		if sweepBodiesCollision(pose.BodyParts(), ob, 0, 1.5, 1.5) {
			return false
		}
	}
	return true
}

// Compare arrival times at the merge, rather than treating the entire future
// arc as permanently occupied. This lets the bus use the gap after a car passes.
func entryHasMovingConflict(f autopilotFrame, rb *sim.Roundabout, points []sim.Vec2, forward sim.Vec2, s float64, extendTrailerWindow bool, selection ...[]int) bool {
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
			maxTime := arrival + 0.8
			if extendTrailerWindow && distance <= 140 {
				maxTime = arrival + 1.3
			}
			for time := math.Max(0, arrival-0.6); time <= maxTime; time += 0.08 {
				other := roundaboutObstacleAt(rb, obstacle, time)
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					return true
				}
			}
		}
	}
	return false
}

// roundaboutOccupiedByArticulatedBus reports whether another articulated bus is
// currently inside a roundabout whose radial ring cannot carry two side-by-side
// articulated buses cleanly, ignoring buses already clearing onto an exit leg.
func roundaboutOccupiedByArticulatedBus(rb *sim.Roundabout, ego *sim.Bus, obstacles []VehicleObstacle) bool {
	if rb == nil {
		return false
	}
	for _, ob := range obstacles {
		other := ob.Bus
		if other == nil || other == ego || !other.IsArticulated() {
			continue
		}
		frontDist := other.Pos.Distance(rb.Center)
		inside := frontDist < rb.Radius-2
		if !inside {
			for _, part := range other.BodyParts() {
				if part.HalfLen > 0 && part.Center.Distance(rb.Center) < rb.Radius-8 {
					inside = true
					break
				}
			}
		}
		if !inside {
			continue
		}
		if ob.Speed > 15 && frontDist > rb.Radius-14 {
			radial := other.Pos.Sub(rb.Center)
			if radial.Dot(sim.UnitFromAngle(other.Heading)) > 0.35*frontDist {
				continue
			}
		}
		return true
	}
	return false
}

// Predict circulating traffic along its arc; approach traffic follows its heading.
// An articulated bus bends at its joint, so each rigid section follows its
// own entrance lane. Sweeping the straight whole-body envelope along the curve
// would swing it across the neighbouring lane and lock two buses waiting
// side by side out of the ring. Every section must stay separated.
func roundaboutEntrySeparated(separation *sim.RoundaboutEntrySeparation, obstacle VehicleObstacle) bool {
	if bus := obstacle.Bus; bus != nil && bus.IsArticulated() {
		parts := 0
		for _, part := range bus.BodyParts() {
			if part.HalfLen <= 0 {
				continue
			}
			if !separation.Clears(part, true) {
				return false
			}
			parts++
		}
		return parts > 0
	}
	return separation.Clears(sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}, false)
}

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
