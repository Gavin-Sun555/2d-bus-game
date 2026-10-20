package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func (tm *TrafficManager) turnCarAtIntersection(f carFrame) {
	if f.tls == nil || f.car.InRoundabout || f.car.JunctionTurn != nil {
		return
	}
	car := f.car
	in := sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}
	for _, inter := range f.tls.Intersections {
		if f.world != nil && f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		delta := inter.Center.Sub(car.Pos)
		forward := delta.Dot(in)
		lateral := math.Abs(delta.X*in.Y - delta.Y*in.X)
		if forward <= 0 || lateral > f.laneWidth*4.2 {
			continue
		}
		left := sim.NormalizeAngle(car.Heading - math.Pi/2)
		right := sim.NormalizeAngle(car.Heading + math.Pi/2)
		turnLeft, turnRight := carIntersectionTurns(f, inter)
		if !turnLeft && !turnRight {
			continue
		}
		geometry := sim.NewJunctionGeometry(f.laneWidth+6, f.laneWidth+6, f.laneWidth)
		if f.world != nil {
			geometry = f.world.JunctionAt(inter.Center, f.laneWidth)
		}
		heading := right
		if turnLeft {
			heading = left
		}
		out := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
		newEW := math.Abs(out.X) > .5
		roadCenter := inter.Center.Y
		if !newEW {
			roadCenter = inter.Center.X
		}
		lanes := max(2, inter.ApproachLanes[inter.GetApproachIndexForHeading(heading+math.Pi)])
		if f.world != nil {
			lanes = f.world.GetRoadLanes(roadCenter, newEW, inter.Center.Add(out.Mul(geometry.TurnRadius+60)))
		}
		exitLane := min(1, lanes/2-1)
		inOffset, outOffset, radius := f.laneWidth/2, (float64(exitLane)+.5)*f.laneWidth, geometry.TurnRadius
		if turnRight {
			inOffset = math.Abs(car.Pos.Y - car.RoadCenter)
			if !car.IsEW {
				inOffset = math.Abs(car.Pos.X - car.RoadCenter)
			}
			outOffset, radius = f.laneWidth/2, 90
			if lanes >= 4 {
				exitLane = min(2, lanes/2-1)
				outOffset = (float64(exitLane) + .5) * f.laneWidth
			}
		}
		// QuarterTurnPath starts at this tangent. Check it before allocating
		// and sampling an arc: distant or stopped cars do not need a new path.
		rightIn := sim.Vec2{X: -in.Y, Y: in.X}
		rightOut := sim.Vec2{X: -out.Y, Y: out.X}
		start := inter.Center.Add(rightIn.Mul(inOffset)).Add(rightOut.Mul(outOffset)).Sub(in.Mul(radius))
		if start.Sub(car.Pos).Dot(in) > 0 {
			continue
		}
		path := sim.QuarterTurnPath(inter.Center, in, out, inOffset, outOffset, radius)
		// Cars may be spawned or pushed past the ideal tangent. Discard the
		// passed prefix instead of driving backwards to the beginning of the arc.
		remaining := []sim.Vec2{car.Pos}
		for _, point := range path[1:] {
			if point.Sub(car.Pos).Dot(in) > 0 {
				remaining = append(remaining, point)
			}
		}
		if len(remaining) == 1 {
			goal := path[len(path)-1]
			goal = goal.Add(in.Mul(math.Max(0, -goal.Sub(car.Pos).Dot(in))))
			remaining = append(remaining, goal)
		}
		path = remaining
		car.JunctionTurn = &sim.CarJunctionTurn{Points: path, ExitHeading: heading, ExitEW: newEW, ExitRoadCenter: roadCenter, ExitLane: exitLane}
		return
	}
}

// Signals and motion must use the same turn intent, including forced turns at
// T junctions and the shared right-turn/bus lane on eight-lane roads.
func carIntersectionTurns(f carFrame, inter *sim.TrafficIntersection) (left, right bool) {
	car := f.car
	straight := hasRoadInDirection(f.world, inter.Center, car.Heading)
	hasLeft := hasRoadInDirection(f.world, inter.Center, car.Heading-math.Pi/2)
	hasRight := hasRoadInDirection(f.world, inter.Center, car.Heading+math.Pi/2)
	dedicated := inter.ApproachHasDedicatedLanesForHeading(car.Heading)
	left = hasLeft && ((!straight && (!hasRight || car.LaneIndex == 0)) || (car.LaneIndex == 0 && dedicated))
	if dedicated && car.LaneIndex != 0 {
		left = false
	}
	lanes := inter.ApproachLanes[inter.GetApproachIndexForHeading(car.Heading)]
	right = !left && hasRight && ((lanes >= 8 && car.LaneIndex == 3) || !straight)
	return
}

func moveCarThroughJunction(car *sim.TrafficCar, dt float64) {
	turn := car.JunctionTurn
	distance := math.Max(0, car.Speed) * dt
	for distance > 0 && turn.Segment < len(turn.Points)-1 {
		a, b := turn.Points[turn.Segment], turn.Points[turn.Segment+1]
		length := a.Distance(b)
		remaining := length - turn.Progress
		if remaining <= distance {
			distance -= remaining
			turn.Segment++
			turn.Progress = 0
			car.Pos = b
			continue
		}
		turn.Progress += distance
		car.Pos = a.Add(b.Sub(a).Mul(turn.Progress / length))
		car.Heading = math.Atan2(b.Y-a.Y, b.X-a.X)
		distance = 0
	}
	if turn.Segment >= len(turn.Points)-1 {
		car.Heading = turn.ExitHeading
		car.IsEW = turn.ExitEW
		car.RoadCenter = turn.ExitRoadCenter
		car.LaneIndex = turn.ExitLane
		car.JunctionTurn = nil
		car.Pos = car.Pos.Add(sim.Vec2{X: math.Cos(car.Heading) * distance, Y: math.Sin(car.Heading) * distance})
	}
}
