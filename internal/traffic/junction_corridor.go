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
	pathEnd := len(points)
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
		cache.Bodies = scratch.appendBodies(cache.Bodies[:0], points, pathEnd, turn, car.Length, car.Width)
	} else {
		cache.Bodies = sim.AppendPathBodies(cache.Bodies[:0], points, car.Length, car.Width)
	}
	cache.RebuildBounds()
	return cache.Bodies
}

func waitingUTurnBlocksCar(f carFrame, bus *sim.Bus, inter *sim.TrafficIntersection) bool {
	c := bus.JunctionCorridor
	if f.world == nil || c == nil || !c.UTurnReady || c.Center.Distance(inter.Center) > 1 {
		return false
	}
	carJunctionBodies(f, inter)
	return sim.UTurnPrecedesEntry(c, f.car.JunctionCorridor, inter)
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
	var actualBodies [1]sim.OBB
	actualBodies[0] = actual
	actualCorridor := sim.JunctionCorridor{Bodies: actualBodies[:]}
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
	horizon := math.Max(180, f.car.Speed*f.car.Speed/(2*220)+60)
	for _, inter := range f.nearbyJunctions() {
		if f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		delta := inter.Center.Sub(f.car.Pos)
		stopLine, halfWidth := inter.ApproachGeometry(u, f.laneWidth)
		gap := delta.Dot(u) - stopLine - f.car.Length/2
		if gap <= 0 || gap > horizon || math.Abs(delta.X*u.Y-delta.Y*u.X) > halfWidth {
			continue
		}
		g := f.world.JunctionAt(inter.Center, f.laneWidth)
		var bodies []sim.OBB
		occupied := false
		exitCandidates := tm.junctionExitCandidates[:0]
		approachReach := junctionApproachReach(inter, f.laneWidth)
		exitReach := 2*(g.HalfX+g.HalfY) + 200
		ids, indexed := tm.junctionScanCandidates(f, inter, approachReach, exitReach)
		exitReachSquared := exitReach * exitReach
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
			if i == f.index || i >= f.activeCount {
				continue
			}
			if nearExitQueue(inter.Center, other.Pos, other.Speed, exitReachSquared) {
				exitCandidates = append(exitCandidates, i)
			}
			if other.InRoundabout {
				continue
			}
			if !tm.disableJunctionBroadPhase && !carCanReachJunction(other, inter.Center, approachReach) {
				continue
			}
			delta := other.Pos.Sub(f.car.Pos)
			if delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < f.laneWidth*.75 && math.Cos(other.Heading-f.car.Heading) > .85 {
				continue
			}
			otherFrame := f
			otherFrame.car = other
			otherFrame.scratch = tm.scratchForCar(other, i)
			if !otherHasJunctionPriority(f, inter, other, i, gap, otherFrame.scratch) {
				continue
			}
			if bodies == nil {
				bodies = carJunctionBodies(f, inter)
			}
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
		tm.junctionExitCandidates = exitCandidates
		var exitQueue sim.JunctionExitQueue
		exitReady := false
		blocksExit := func(body sim.OBB) bool {
			if !exitReady {
				if bodies == nil {
					bodies = carJunctionBodies(f, inter)
				}
				exitQueue = sim.PrepareJunctionExitQueue(bodies, inter.Center, g, f.car.Length, f.car.Width)
				exitReady = true
			}
			return exitQueue.Blocks(body)
		}
		if !occupied {
			for _, i := range exitCandidates {
				other := tm.Cars[i]
				if blocksExit(sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}) {
					occupied = true
					break
				}
			}
		}
		for _, bus := range f.buses {
			if occupied {
				break
			}
			if bus == nil || math.Abs(bus.Speed) > 5 {
				continue
			}
			body := bus.BodyEnvelope()
			if nearExitQueue(inter.Center, body.Center, bus.Speed, exitReachSquared) && blocksExit(body) {
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
func otherHasJunctionPriority(f carFrame, inter *sim.TrafficIntersection, other *sim.TrafficCar, index int, ownGap float64, otherScratch ...*carJunctionScratch) bool {
	if other.JunctionTurn != nil {
		dc := other.JunctionTurn.Center.Sub(inter.Center)
		if other.JunctionTurn.LaneChange || math.Abs(dc.X) >= 1 || math.Abs(dc.Y) >= 1 || dc.Dot(dc) >= 1 {
			return false
		}
		// Entry permission is decided before attaching a junction turn. Once
		// committed, protect the remaining path even after the light changes.
		// The changing yaw is not a new approach or a new turn intention.
		return true
	}
	u := sim.UnitFromAngle(other.Heading)
	delta := inter.Center.Sub(other.Pos)
	stopLine, halfWidth := inter.ApproachGeometry(u, f.laneWidth)
	if math.Abs(delta.X*u.Y-delta.Y*u.X) > halfWidth {
		return false
	}
	gap := delta.Dot(u) - stopLine - other.Length/2
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
	if len(otherScratch) > 0 {
		otherFrame.scratch = otherScratch[0]
	}
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

func nearExitQueue(center, pos sim.Vec2, speed, reachSquared float64) bool {
	delta := pos.Sub(center)
	return !(math.Abs(speed) > 5 || delta.Dot(delta) > reachSquared)
}
