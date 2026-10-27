package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func carJunctionBodies(f carFrame, inter *sim.TrafficIntersection) []sim.OBB {
	car := f.car
	scratch := f.junctionScratch()
	cache := car.JunctionCorridor
	if cache == nil && scratch != nil && scratch.corridor != nil {
		cache = scratch.corridor
		cache.Remaining = 0
		car.JunctionCorridor = cache
	}
	key := carJunctionKey(f, inter)
	if cache != nil && (scratch == nil || (scratch.bodyValid && scratch.bodyKey == key)) && cache.Remaining > 0 && cache.Center.Distance(inter.Center) < 1 && cache.CachePosition == car.Pos && cache.CacheHeading == car.Heading && cache.CacheTurn == car.JunctionTurn {
		return cache.Bodies
	}
	if cache == nil {
		cache = &sim.JunctionCorridor{}
		car.JunctionCorridor = cache
	}
	if scratch != nil {
		scratch.corridor = cache
		scratch.bodyKey, scratch.bodyValid = key, true
	}
	cache.Center = inter.Center
	cache.Remaining = .25
	cache.CachePosition = car.Pos
	cache.CacheHeading = car.Heading
	cache.CacheTurn = car.JunctionTurn
	cache.EntryHeading = car.Heading
	if car.JunctionTurn != nil && !car.JunctionTurn.LaneChange {
		cache.EntryHeading = car.JunctionTurn.EntryHeading
	}
	u0 := sim.UnitFromAngle(car.Heading)
	toCenter := inter.Center.Sub(car.Pos)
	cache.Entered = (car.JunctionTurn != nil && !car.JunctionTurn.LaneChange && car.JunctionTurn.Center.Distance(inter.Center) < 1) || (toCenter.Dot(u0) > 0 && toCenter.Dot(u0)-car.Length/2 <= inter.GetStopLineDistForHeading(car.Heading, f.laneWidth))

	var points []sim.Vec2
	if scratch != nil {
		points = scratch.points[:0]
	}
	points = append(points, car.Pos)
	turn := car.JunctionTurn
	if turn != nil && !turn.LaneChange {
		points = append(points, turn.Points[turn.Segment+1:]...)
	} else {
		turn = planCarJunctionTurn(f, inter)
		if turn != nil {
			u := sim.UnitFromAngle(car.Heading)
			for _, p := range turn.Points {
				if p.Sub(car.Pos).Dot(u) > 0 {
					points = append(points, p)
				}
			}
		}
	}
	heading := car.Heading
	if turn != nil {
		heading = turn.ExitHeading
	}
	u := sim.UnitFromAngle(heading)
	g := f.world.JunctionAt(inter.Center, f.laneWidth)
	half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	last := points[len(points)-1]
	distance := half + car.Length + 50 - last.Sub(inter.Center).Dot(u)
	if distance > 0 {
		points = append(points, last.Add(u.Mul(distance)))
	}
	if scratch != nil {
		scratch.points = points
	}
	if scratch != nil {
		cache.Bodies = scratch.appendBodies(cache.Bodies[:0], points, turn, car.Length, car.Width)
	} else {
		cache.Bodies = sim.AppendPathBodies(cache.Bodies[:0], points, car.Length, car.Width)
	}
	cache.RebuildBounds()
	return cache.Bodies
}

func carClearsBusInJunction(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection) bool {
	if canFollowCommittedThroughBus(f, bus, inter) {
		return true
	}
	corridor := bus.JunctionCorridor
	if f.world != nil && !f.world.InsideJunction(bus.Pos, f.laneWidth, 0) && (corridor == nil || corridor.Remaining <= 0) {
		return true
	}
	if f.world == nil || corridor == nil || corridor.Remaining <= 0 || corridor.Center.Distance(inter.Center) > 1 || len(corridor.Bodies) < 2 {
		return false
	}
	carJunctionBodies(f, inter)
	if _, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, corridor, bus.BodyEnvelope(), f.car.Length, f.car.Width); ok {
		return true
	}
	actual := bus.BodyEnvelope()
	actual.HalfLen += 4
	actual.HalfWid += 3
	actualCorridor := sim.JunctionCorridor{Bodies: []sim.OBB{actual}}
	actualCorridor.RebuildBounds()
	if !sim.SeparatedJunctionCorridors(f.car.JunctionCorridor, &actualCorridor) {
		return false
	}
	// A same-lane moving leader does not own the whole box; normal headway
	// handles it. A turning bus must still reserve its full swept corridor.
	last := corridor.Bodies[len(corridor.Bodies)-1]
	u := sim.UnitFromAngle(f.car.Heading)
	delta := bus.Pos.Sub(f.car.Pos)
	if bus.Speed > 15 && math.Cos(bus.Heading-f.car.Heading) > .98 && math.Cos(last.Heading-f.car.Heading) > .98 && delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.35 {
		return true
	}
	return sim.SeparatedJunctionCorridors(f.car.JunctionCorridor, corridor)
}

func (tm *TrafficManager) junctionQueueSpeed(f carFrame, target float64) float64 {
	if target <= .5 || f.world == nil || f.tls == nil || f.car.InRoundabout || (f.car.JunctionTurn != nil && !f.car.JunctionTurn.LaneChange) {
		return target
	}
	u := sim.UnitFromAngle(f.car.Heading)
	for _, inter := range f.nearbyJunctions() {
		if f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		delta := inter.Center.Sub(f.car.Pos)
		gap := delta.Dot(u) - inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth) - f.car.Length/2
		horizon := math.Max(180, f.car.Speed*f.car.Speed/(2*220)+60)
		if gap <= 0 || gap > horizon || math.Abs(delta.X*u.Y-delta.Y*u.X) > inter.ApproachRoadHalfWidth(f.car.Heading, f.laneWidth) {
			continue
		}
		g := f.world.JunctionAt(inter.Center, f.laneWidth)
		var bodies []sim.OBB
		blocked := func(pos sim.Vec2, heading, length, width, speed float64) bool {
			if math.Abs(speed) > 5 || pos.Distance(inter.Center) > 2*(g.HalfX+g.HalfY)+200 {
				return false
			}
			if bodies == nil {
				bodies = carJunctionBodies(f, inter)
			}
			return sim.ExitQueueBlocksEntry(bodies, inter.Center, g, f.car.Length, f.car.Width, sim.OBB{Center: pos, Heading: heading, HalfLen: length / 2, HalfWid: width / 2}, speed)
		}
		occupied := false
		approachReach := junctionApproachReach(inter, f.laneWidth)
		ids, indexed := tm.junctionScanCandidates(f, inter, approachReach, 2*(g.HalfX+g.HalfY)+200)
		count := len(tm.Cars)
		if indexed {
			count = len(ids)
		}
		for candidate := 0; candidate < count; candidate++ {
			i := candidate
			if indexed {
				i = ids[candidate]
			}
			other := tm.Cars[i]
			if i == f.index || i >= f.activeCount || other.InRoundabout {
				continue
			}
			if !tm.disableJunctionBroadPhase && !carCanReachJunction(other, inter.Center, approachReach) {
				continue
			}
			if !otherHasJunctionPriority(f, inter, other, i, gap) {
				continue
			}
			delta := other.Pos.Sub(f.car.Pos)
			if math.Cos(other.Heading-f.car.Heading) > .85 && delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.75 {
				continue
			}
			if bodies == nil {
				bodies = carJunctionBodies(f, inter)
			}
			otherFrame := f
			otherFrame.car = other
			otherFrame.scratch = tm.scratchForCar(other, i)
			carJunctionBodies(otherFrame, inter)
			if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, other.JunctionCorridor, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}, f.car.Length, f.car.Width); ok {
				target = math.Min(target, sim.JunctionFollowingSpeed(gap, other.Speed, f.car.Speed, 220))
				continue
			}
			if !sim.SeparatedJunctionCorridors(f.car.JunctionCorridor, other.JunctionCorridor) {
				occupied = true
				break
			}
		}
		for candidate := 0; candidate < count; candidate++ {
			i := candidate
			if indexed {
				i = ids[candidate]
			}
			other := tm.Cars[i]
			if i == f.index || i >= f.activeCount {
				continue
			}
			if blocked(other.Pos, other.Heading, other.Length, other.Width, other.Speed) {
				occupied = true
				break
			}
		}
		for _, bus := range f.buses {
			if bus == nil {
				continue
			}
			body := bus.BodyEnvelope()
			if blocked(body.Center, body.Heading, body.HalfLen*2, body.HalfWid*2, bus.Speed) {
				occupied = true
				break
			}
		}
		if occupied {
			f.car.IsBraking = true
			target = math.Min(target, math.Sqrt(2*220*math.Max(0, gap-14)))
		}
	}
	return target
}

// Vehicles already inside clear first. Before entry, green movements precede
// right-on-red, then arrival time and a stable ID resolve a simultaneous merge.
func otherHasJunctionPriority(f carFrame, inter *sim.TrafficIntersection, other *sim.TrafficCar, index int, ownGap float64) bool {
	if other.JunctionTurn != nil {
		if other.JunctionTurn.LaneChange || other.JunctionTurn.Center.Distance(inter.Center) >= 1 {
			return false
		}
		// Entry permission is decided before attaching a junction turn. Once
		// committed, protect the remaining path even after the light changes.
		// The changing yaw is not a new approach or a new turn intention.
		return true
	}
	u := sim.UnitFromAngle(other.Heading)
	delta := inter.Center.Sub(other.Pos)
	if math.Abs(delta.X*u.Y-delta.Y*u.X) > inter.ApproachRoadHalfWidth(other.Heading, f.laneWidth) {
		return false
	}
	gap := delta.Dot(u) - inter.GetStopLineDistForHeading(other.Heading, f.laneWidth) - other.Length/2
	if delta.Dot(u) <= 0 || gap > math.Max(180, other.Speed*other.Speed/(2*220)+60) {
		return false
	}
	// The entry line precedes the turn tangent. A vehicle between those two
	// points can already be crossing even without an attached turn. A stopped
	// nose short of the actual box must not reserve its entire future route.
	if gap <= -sim.JunctionStopInset || (gap <= 0 && other.Speed > 5) {
		return true
	}
	otherFrame := f
	otherFrame.car = other
	left, right := carIntersectionTurns(otherFrame, inter)
	signal := inter.GetSignalForHeading(other.Heading, left)
	if signal == sim.SignalRed && !right {
		return false
	}
	ownLeft, ownRight := carIntersectionTurns(f, inter)
	ownSignal := inter.GetSignalForHeading(f.car.Heading, ownLeft)
	if right && signal == sim.SignalRed && ownSignal != sim.SignalRed {
		return false
	}
	if ownRight && ownSignal == sim.SignalRed && signal != sim.SignalRed {
		return true
	}
	if !ownLeft && !ownRight && left && math.Cos(other.Heading-f.car.Heading) < -0.85 {
		// Opposing oncoming vehicle turning left yields to straight-through vehicle on green
		return false
	}

	arrival := gap / math.Max(25, other.Speed)
	ownArrival := ownGap / math.Max(25, f.car.Speed)
	if math.Abs(arrival-ownArrival) > .25 {
		return arrival < ownArrival
	}
	if other.ID != f.car.ID {
		return other.ID < f.car.ID
	}
	return index < f.index
}

// Entry precedes the tangent of a wide turn. Publish that short straight portion
// too, so buses do not wait until the car starts rotating to recognize priority.
func publishCarJunctionCorridor(f carFrame) {
	if f.world == nil || f.tls == nil || f.car.InRoundabout {
		return
	}
	for _, inter := range f.nearbyJunctions() {
		if f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		if turn := f.car.JunctionTurn; turn != nil && !turn.LaneChange && turn.Center.Distance(inter.Center) < 1 {
			carJunctionBodies(f, inter)
			return
		}
		u := sim.UnitFromAngle(f.car.Heading)
		delta := inter.Center.Sub(f.car.Pos)
		forward := delta.Dot(u)
		if forward > 0 && forward-f.car.Length/2 <= inter.GetStopLineDistForHeading(f.car.Heading, f.laneWidth) && math.Abs(delta.X*u.Y-delta.Y*u.X) < inter.ApproachRoadHalfWidth(f.car.Heading, f.laneWidth) {
			carJunctionBodies(f, inter)
			return
		}
	}
}

// A non-committed approach is bounded by its stopping horizon plus the line
// and roadway extents. Using their sum is conservative for every yaw. Keep
// committed turns regardless of distance, as their original corridor can
// remain relevant after the vehicle travels past the approach region.
func junctionApproachReach(inter *sim.TrafficIntersection, lw float64) float64 {
	ew, _, ns, _ := inter.GetApproachDimensions(lw)
	lanes := 0
	for _, n := range inter.ApproachLanes {
		lanes = max(lanes, n)
	}
	return math.Max(ew, ns) + float64(lanes)*lw/2 + 12
}
func carCanReachJunction(car *sim.TrafficCar, center sim.Vec2, approachReach float64) bool {
	if car.JunctionTurn != nil {
		return true
	}
	radius := approachReach + car.Length/2 + math.Max(180, car.Speed*car.Speed/(2*220)+60)
	d := car.Pos.Sub(center)
	return d.Dot(d) <= radius*radius
}
