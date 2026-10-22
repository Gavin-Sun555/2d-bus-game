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
	mergeRoom := 0.0
	mergeLength := carLaneChangeLength(car, f.laneWidth)
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
			wantRight = d < stop+math.Max(420, mergeLength+200) && hasRoadInDirection(f.world, inter.Center, car.Heading+math.Pi/2)
			mergeRoom = d - stop - car.Length/2 - 40
		}
	}
	if car.LaneIndex == 2 && car.ID%3 == 0 && wantRight && car.Speed > 1 && mergeRoom >= mergeLength && tm.rightMergeClear(f, forward) {
		// Keep the original lane identity until the arc-length follower has
		// completed the merge; navigation must not slide the body sideways.
		tm.beginCarLaneChange(f, 3)
		return
	}
	// Reserved lanes are never a cruising lane for civilian cars. At junctions
	// the lane remains reserved for a committed right turn until its arc starts.
	// Cars already leaving the map should keep their exit corridor; a new
	// lane-change path would delay rear-body boundary removal.
	roadAhead := math.Max(road.P1.Sub(car.Pos).Dot(forward), road.P2.Sub(car.Pos).Dot(forward))
	if car.LaneIndex == 3 && !wantRight && roadAhead >= mergeLength+car.Length/2+40 {
		if (road.HasBusLane() || !tm.eightLaneRightExitAhead(f, forward)) && tm.laneMergeClear(f, forward, -1) {
			tm.beginCarLaneChange(f, 2)
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
	var ours []sim.OBB
	for _, other := range active {
		if other != f.car && !clear(other.Pos, other.Length, other.Width, other.Speed) {
			return false
		}
		if other == f.car || other.JunctionTurn == nil || !other.JunctionTurn.LaneChange {
			continue
		}
		if other.Pos.Distance(f.car.Pos) > 600 {
			continue
		}
		// A follower paused behind us must not reserve our escape into the
		// empty lane ahead. Its normal headway keeps it behind our rear bumper.
		if math.Cos(other.JunctionTurn.ExitHeading-f.car.Heading) > .98 && other.Pos.Sub(f.car.Pos).Dot(forward) < -(f.car.Length+other.Length)/2-12 {
			continue
		}
		if ours == nil {
			length := carLaneChangeLength(f.car, f.laneWidth)
			points := make([]sim.Vec2, 33)
			for i := range points {
				t := float64(i) / 32
				ease := t * t * t * (10 - 15*t + 6*t*t)
				points[i] = f.car.Pos.Add(forward.Mul(length * t)).Add(right.Mul(f.laneWidth * direction * ease))
			}
			ours = sim.AppendPathBodies(nil, points, f.car.Length, f.car.Width)
		}
		turn := other.JunctionTurn
		remaining := append([]sim.Vec2{other.Pos}, turn.Points[turn.Segment+1:]...)
		theirs := sim.AppendPathBodies(nil, remaining, other.Length, other.Width)
		if !sim.CorridorsSeparated(ours, theirs) {
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
