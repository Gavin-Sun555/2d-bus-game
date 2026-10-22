package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A road ending in a left-only bend does not make its through lanes legal
// left-turn lanes. Merge before the line, or wait for a gap outside the box.
func (tm *TrafficManager) prepareForcedLeftApproach(f carFrame) float64 {
	car := f.car
	if f.world == nil || f.tls == nil || car.InRoundabout || car.JunctionTurn != nil || car.LaneIndex == 0 {
		return math.Inf(1)
	}
	forward := roundaboutApproachDirection(car)
	heading := math.Atan2(forward.Y, forward.X)
	var next *sim.TrafficIntersection
	nearest := math.Inf(1)
	for _, inter := range f.tls.Intersections {
		delta := inter.Center.Sub(car.Pos)
		distance := delta.Dot(forward)
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
		(hasRoadInDirection(f.world, next.Center, heading) || isMapExitAhead(f.world, next.Center, heading)) || hasRoadInDirection(f.world, next.Center, heading+math.Pi/2) ||
		!hasRoadInDirection(f.world, next.Center, heading-math.Pi/2) {
		return math.Inf(1)
	}
	line := next.GetStopLineDistForHeading(heading, f.laneWidth)
	if nearest > line+1000 {
		return math.Inf(1)
	}
	room := nearest - line - car.Length/2 - 20
	// One adjacent lane at a time, with zero lateral slope at both endpoints.
	// Path following uses traveled arc length, so the car rotates through the
	// merge instead of sliding sideways or moving farther than speed*dt.
	// Budget all remaining lane changes before committing the first one.
	// A late merge may use a shorter smooth taper, but cannot consume the
	// road needed to finish the next merge and leave the car stranded.
	standardLength := math.Max(160, 3*f.laneWidth)
	length := math.Min(standardLength, (room-20)/float64(car.LaneIndex))
	if length >= math.Max(70, 1.3*f.laneWidth) && tm.laneMergeClear(f, forward, -1) {
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		points := make([]sim.Vec2, 33)
		for i := range points {
			u := float64(i) / float64(len(points)-1)
			ease := u * u * u * (10 - 15*u + 6*u*u)
			points[i] = car.Pos.Add(forward.Mul(length * u)).Sub(right.Mul(f.laneWidth * ease))
		}
		car.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true, Points: points, ExitHeading: heading, ExitEW: car.IsEW, ExitRoadCenter: car.RoadCenter, ExitLane: car.LaneIndex - 1, SpeedLimit: 65}
		return math.Sqrt(2 * 220 * math.Max(0, room))
	}
	// Keep enough road ahead to complete the merge after the gap opens;
	// stopping right at the line would leave no room to try again.
	return math.Sqrt(2 * 220 * math.Max(0, room-standardLength*float64(car.LaneIndex)-20))
}
