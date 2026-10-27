package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Frozen pre-change scans for equivalence tests; no production fallback toggle.
func (ad *AutopilotDriver) originalTemporaryAdmission(f autopilotFrame) float64 {
	corridor := f.bus.JunctionCorridor
	if f.world == nil || f.tls == nil || corridor == nil || corridor.Remaining <= 0 {
		return math.Inf(1)
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	var inter *sim.TrafficIntersection
	for _, it := range f.tls.Intersections {
		if it.Center.Distance(corridor.Center) < 1 {
			inter = it
			break
		}
	}
	if inter == nil {
		return math.Inf(1)
	}
	gap := corridor.Center.Sub(f.bus.Pos).Dot(u) - inter.GetStopLineDistForHeading(heading, f.laneWidth) - f.bus.Length/2
	// A vehicle already admitted clears the box instead of waiting at its exit.
	if gap <= 0 {
		return math.Inf(1)
	}
	g := f.world.JunctionAt(corridor.Center, f.laneWidth)
	for _, ob := range f.obstacles {
		body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
		if _, ok := sim.JunctionLeaderGap(corridor, ob.JunctionCorridor, body, f.bus.Length, f.bus.Width); ok {
			continue
		}
		conflict := false
		waitingAtRed := waitingOutsideRedEntry(ob, inter, g)
		if !waitingAtRed && ob.JunctionCorridor != nil && ob.JunctionCorridor.Remaining > 0 && ob.JunctionCorridor.Entered && ob.JunctionCorridor.Center.Distance(corridor.Center) < 1 {
			sameLeader := ob.Speed > 15 && math.Cos(ob.Heading-heading) > .98 && ob.Pos.Sub(f.bus.Pos).Dot(u) > 0 && math.Abs(ob.Pos.Sub(f.bus.Pos).X*u.Y-ob.Pos.Sub(f.bus.Pos).Y*u.X) < f.laneWidth*.35
			conflict = !sameLeader && !sim.SeparatedJunctionCorridors(corridor, ob.JunctionCorridor)
		}
		if !waitingAtRed && ob.JunctionTurn != nil && !ob.JunctionTurn.LaneChange && ob.JunctionTurn.Center.Distance(corridor.Center) < 1 {
			points := ob.JunctionTurn.Points[ob.JunctionTurn.Segment:]
			otherBodies := sim.AppendPathBodies(nil, points, ob.Length, ob.Width)
			delta := ob.Pos.Sub(f.bus.Pos)
			sameLeader := ob.Speed > 15 && math.Cos(ob.Heading-heading) > .98 && delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.35
			conflict = conflict || (!sameLeader && !sim.CorridorsSeparated(corridor.Bodies, otherBodies))
		}
		if conflict {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
		if sim.ExitQueueBlocksEntry(corridor.Bodies, corridor.Center, g, f.bus.Length, f.bus.Width, body, ob.Speed) {
			return math.Sqrt(2 * 130 * math.Max(0, gap-18))
		}
	}
	return math.Inf(1)
}

func (ad *AutopilotDriver) originalTemporaryRoundabout(f autopilotFrame) float64 {
	if f.world == nil || len(f.obstacles) == 0 {
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
		points, forward, ok := ad.roundaboutEntryCurve(f, rb, offset)
		if !ok {
			continue
		}
		s := entryPosition(points, f.bus.Pos, forward)
		// Hold with the whole body outside circulating traffic. Once committed,
		// continue through the gap instead of stopping in front of a following car.
		yieldDistance := -rb.Radius - f.bus.Length/2 - 16 - f.bus.Pos.Sub(rb.Center).Dot(forward)
		traffic := make([]sim.OBB, 0, len(f.obstacles))
		entryFrame := f
		entryFrame.obstacles = make([]VehicleObstacle, 0, len(f.obstacles))
		parallel := ad.parallelRoundaboutJoin(f, rb)
		var separation sim.RoundaboutEntrySeparation
		if parallel {
			var geometry *sim.RoundaboutEntryGeometry
			if !ad.disableRoundaboutEntryGeometry {
				geometry = &ad.roundaboutEntryGeometry
			}
			separation = sim.PrepareRoundaboutEntrySeparation(*rb, forward, f.laneWidth, sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length / 2, HalfWid: f.bus.Width / 2}, true, geometry)
		}
		for _, obstacle := range f.obstacles {
			// Keep local body clearance while an adjacent car clears the mouth,
			// including the first frames after it starts moving.
			if parallel && separation.Clears(sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}, false) {
				continue
			}
			traffic = append(traffic, roundaboutObstacleAt(rb, obstacle, 0))
			entryFrame.obstacles = append(entryFrame.obstacles, obstacle)
		}
		if yieldDistance >= -2 && entryHasMovingConflict(entryFrame, rb, points, forward, s) {
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
