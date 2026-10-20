package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Only the outer general-purpose lane supplies right turns into the shared
// bus lane. Select early enough to finish merging before the solid lane lines.
func (tm *TrafficManager) prepareEightLaneApproach(f carFrame) {
	car := f.car
	if f.world == nil || car.InRoundabout || car.JunctionTurn != nil {
		return
	}
	road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos)
	if !ok || road.LaneCount() != 8 {
		return
	}
	forward := sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}
	wantRight := false
	if f.tls != nil {
		nearest := math.Inf(1)
		for _, inter := range f.tls.Intersections {
			delta := inter.Center.Sub(car.Pos)
			d := delta.Dot(forward)
			if d <= 0 || d >= nearest || math.Abs(delta.X*forward.Y-delta.Y*forward.X) > f.laneWidth*4.2 || f.world.IsAtRoundabout(inter.Center) {
				continue
			}
			nearest = d
			stop := inter.GetStopLineDistForHeading(car.Heading, f.laneWidth)
			wantRight = d < stop+420 && hasRoadInDirection(f.world, inter.Center, car.Heading+math.Pi/2)
		}
	}
	if car.LaneIndex == 2 && car.ID%3 == 0 && wantRight && car.Speed > 1 && tm.rightMergeClear(f, forward) {
		car.LaneIndex = 3
	}
	// Reserved lanes are never a cruising lane for civilian cars. At junctions
	// the lane remains reserved for a committed right turn until its arc starts.
	if car.LaneIndex == 3 && !wantRight {
		if road.HasBusLane() || !tm.eightLaneRightExitAhead(f, forward) {
			car.LaneIndex = 2
		}
	}
}

func (tm *TrafficManager) eightLaneRightExitAhead(f carFrame, forward sim.Vec2) bool {
	if f.tls == nil {
		return false
	}
	for _, inter := range f.tls.Intersections {
		delta := inter.Center.Sub(f.car.Pos)
		if delta.Dot(forward) > 0 && math.Abs(delta.X*forward.Y-delta.Y*forward.X) < f.laneWidth*4.2 && hasRoadInDirection(f.world, inter.Center, f.car.Heading+math.Pi/2) {
			return true
		}
	}
	return false
}

func (tm *TrafficManager) rightMergeClear(f carFrame, forward sim.Vec2) bool {
	return tm.laneMergeClear(f, forward, 1)
}

func (tm *TrafficManager) laneMergeClear(f carFrame, forward sim.Vec2, direction float64) bool {
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	target := f.car.Pos.Add(right.Mul(f.laneWidth * direction))
	clear := func(pos sim.Vec2, length, width, speed float64) bool {
		delta := pos.Sub(target)
		along := delta.Dot(forward)
		return math.Abs(delta.Dot(right)) > (width+f.car.Width)/2+4 || along > (length+f.car.Length)/2+math.Max(35, f.car.Speed*.8) || along < -(length+f.car.Length)/2-math.Max(40, speed*1.2)
	}
	active := tm.Cars
	if f.activeCount > 0 && f.activeCount < len(active) {
		active = active[:f.activeCount]
	}
	for _, other := range active {
		if other != f.car && !clear(other.Pos, other.Length, other.Width, other.Speed) {
			return false
		}
	}
	for _, bus := range f.buses {
		if bus != nil && !clear(bus.Pos, bus.Length, bus.Width, bus.Speed) {
			return false
		}
	}
	return true
}
