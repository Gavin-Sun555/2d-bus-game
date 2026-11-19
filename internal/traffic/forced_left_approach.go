package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A road ending in a left-only bend does not make its through lanes legal
// left-turn lanes. Merge before the line, or wait for a gap outside the box.
func (tm *TrafficManager) prepareForcedLeftApproach(f carFrame) float64 {
	car := f.car
	if f.world == nil || f.tls == nil || car.InRoundabout || car.JunctionTurn != nil {
		return math.Inf(1)
	}
	leftLane := 0
	if road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos); ok {
		leftLane = road.FirstGeneralTrafficLane()
		if car.LaneIndex <= leftLane && road.GeneralTrafficLanes() < 2 {
			return math.Inf(1)
		}
	}
	if car.LaneIndex <= leftLane {
		return tm.forcedLeftQueueYield(f, leftLane)
	}
	forward := roundaboutApproachDirection(car)
	heading := math.Atan2(forward.Y, forward.X)
	next, nearest := forcedLeftJunctionAhead(f, forward, heading)
	if next == nil {
		return math.Inf(1)
	}
	line := next.GetStopLineDistForHeading(heading, f.laneWidth)
	if nearest > line+1000 {
		return math.Inf(1)
	}
	lineRoom := nearest - line - car.Length/2
	room := lineRoom - 20
	// One adjacent lane at a time, with zero lateral slope at both endpoints.
	// Path following uses traveled arc length, so the car rotates through the
	// merge instead of sliding sideways or moving farther than speed*dt.
	// Budget all remaining lane changes before committing the first one.
	// A late merge may use a shorter smooth taper, but cannot consume the
	// road needed to finish the next merge and leave the car stranded.
	steps := float64(car.LaneIndex - leftLane)
	standardLength := math.Max(160, 3*f.laneWidth)
	minLength := math.Max(70, 1.3*f.laneWidth)
	length := math.Min(standardLength, (room-20)/steps)
	lateClose := false
	if length < minLength && (lineRoom-2)/steps >= minLength {
		length = math.Min(standardLength, (lineRoom-2)/steps)
		lateClose = true
	}
	if length >= minLength && tm.laneMergeClear(f, forward, -1) {
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		points := make([]sim.Vec2, 33)
		for i := range points {
			u := float64(i) / float64(len(points)-1)
			ease := u * u * u * (10 - 15*u + 6*u*u)
			points[i] = car.Pos.Add(forward.Mul(length * u)).Sub(right.Mul(f.laneWidth * ease))
		}
		car.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true, Points: points, ExitHeading: heading, ExitEW: car.IsEW, ExitRoadCenter: car.RoadCenter, ExitLane: car.LaneIndex - 1, SpeedLimit: 65}
		if lateClose {
			return math.Sqrt(2 * 220 * math.Max(0, lineRoom-2))
		}
		return math.Sqrt(2 * 220 * math.Max(0, room))
	}
	// Keep enough road ahead to complete the merge after the gap opens;
	// stopping right at the line would leave no room to try again.
	if lateClose {
		return math.Sqrt(2 * 220 * math.Max(0, lineRoom-minLength*steps-2))
	}
	return math.Sqrt(2 * 220 * math.Max(0, room-standardLength*steps-20))
}

func forcedLeftJunctionAhead(f carFrame, forward sim.Vec2, heading float64) (*sim.TrafficIntersection, float64) {
	car := f.car
	var next *sim.TrafficIntersection
	nearest := math.Inf(1)
	for _, inter := range f.tls.Intersections {
		distance := inter.Center.Sub(car.Pos).Dot(forward)
		coord := inter.Center.Y
		if !car.IsEW {
			coord = inter.Center.X
		}
		if distance <= 0 || distance >= nearest || math.Abs(coord-car.RoadCenter) > 25 || f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		nearest, next = distance, inter
	}
	if next == nil || !next.ApproachHasDedicatedLanesForHeading(heading) ||
		hasRoadInDirection(f.world, next.Center, heading) || isMapExitAhead(f.world, next.Center, heading) ||
		hasRoadInDirection(f.world, next.Center, heading+math.Pi/2) || !hasRoadInDirection(f.world, next.Center, heading-math.Pi/2) {
		return nil, 0
	}
	return next, nearest
}

// A dense dedicated lane can keep consuming the gaps needed by cars forced
// to merge into it. Leave a slot behind the leading waiting car and let
// vehicles already beside that car continue through.
func (tm *TrafficManager) forcedLeftQueueYield(f carFrame, leftLane int) float64 {
	car := f.car
	if car.LaneIndex != leftLane || car.Speed > 35 {
		return math.Inf(1)
	}
	forward := roundaboutApproachDirection(car)
	heading := math.Atan2(forward.Y, forward.X)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	gap := math.Inf(1)
	for _, other := range tm.carsForFrame(f) {
		if other == car || other.LaneIndex != leftLane+1 || other.IsEW != car.IsEW || other.RoadCenter != car.RoadCenter ||
			other.InRoundabout || other.JunctionTurn != nil || math.Abs(other.Speed) > 1 || sim.Cos(other.Heading-heading) < .98 {
			continue
		}
		delta := other.Pos.Sub(car.Pos)
		if math.Abs(delta.Dot(right)-f.laneWidth) > 8 {
			continue
		}
		open := delta.Dot(forward) - (car.Length+other.Length)/2 - 42
		if open >= 0 && open < gap {
			gap = open
		}
	}
	if gap > 85 {
		return math.Inf(1)
	}
	inter, distance := forcedLeftJunctionAhead(f, forward, heading)
	if inter == nil || distance > inter.GetStopLineDistForHeading(heading, f.laneWidth)+650 {
		return math.Inf(1)
	}
	return math.Sqrt(2 * 220 * math.Max(0, gap-4))
}

func tooCloseToForcedLeftBend(w *sim.World, pos sim.Vec2, heading, roadCoord float64, isEW bool, steps int, laneWidth float64) bool {
	if w == nil || steps <= 0 {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	nearest := math.Inf(1)
	var center sim.Vec2
	for _, ipt := range w.Intersections {
		coord := ipt.Y
		if !isEW {
			coord = ipt.X
		}
		d := ipt.Sub(pos).Dot(forward)
		if d <= 0 || d >= nearest || math.Abs(coord-roadCoord) > 25 || w.IsAtRoundabout(ipt) {
			continue
		}
		nearest, center = d, ipt
	}
	if math.IsInf(nearest, 1) {
		return false
	}
	if (hasRoadInDirection(w, center, heading) || isMapExitAhead(w, center, heading)) ||
		hasRoadInDirection(w, center, heading+math.Pi/2) || !hasRoadInDirection(w, center, heading-math.Pi/2) {
		return false
	}
	g := w.JunctionAt(center, laneWidth)
	stopDist := g.HalfY + sim.JunctionStopInset
	if isEW {
		stopDist = g.HalfX + sim.JunctionStopInset
	}
	minLength := math.Max(70, 1.3*laneWidth)
	return nearest < stopDist+minLength*float64(steps)+40
}
