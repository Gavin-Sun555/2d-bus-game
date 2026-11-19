package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func (tm *TrafficManager) turnCarAtIntersection(f carFrame) {
	if f.tls == nil || f.car.InRoundabout || f.car.JunctionTurn != nil || f.car.Speed <= 0 {
		return
	}
	car := f.car
	in := sim.UnitFromAngle(car.Heading)
	for _, inter := range f.nearbyJunctions() {
		if f.world != nil && f.world.IsAtRoundabout(inter.Center) {
			continue
		}
		delta := inter.Center.Sub(car.Pos)
		forward := delta.Dot(in)
		lateral := math.Abs(delta.X*in.Y - delta.Y*in.X)
		if forward <= 0 || lateral > f.laneWidth*4.2 {
			continue
		}
		if forward > inter.GetStopLineDistForHeading(car.Heading, f.laneWidth)+car.Length {
			continue
		}
		left, right := carIntersectionTurns(f, inter)
		signal := inter.GetSignalForHeading(car.Heading, left)
		stopLineDist := inter.GetStopLineDistForHeading(car.Heading, f.laneWidth)
		if signal == sim.SignalRed && !right && forward > stopLineDist {
			continue
		}
		if inter.HoldingUTurnClearance() && forward-car.Length/2 > stopLineDist {
			continue
		}
		if signal == sim.SignalYellow && inter.YellowRemaining() <= 0 && forward > stopLineDist {
			continue
		}
		plan := planCarJunctionTurn(f, inter)
		if plan == nil {
			continue
		}
		path := plan.Points
		if path[0].Sub(car.Pos).Dot(in) > 0 {
			continue
		}

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
		committed := *plan
		committed.Points = path
		car.JunctionTurn = &committed
		return
	}
}

// Signals and motion must use the same turn intent, including forced turns at
// T junctions and the shared right-turn/bus lane on eight-lane roads.
func carIntersectionTurns(f carFrame, inter *sim.TrafficIntersection) (left, right bool) {
	car := f.car
	scratch := f.junctionScratch()
	var key carTurnIntentKey
	if scratch != nil {
		key = carTurnIntentKey{
			world:      f.world,
			center:     inter.Center,
			pos:        car.Pos,
			heading:    car.Heading,
			roadCenter: car.RoadCenter,
			lane:       car.LaneIndex,
			id:         car.ID,
			ew:         car.IsEW,
			lanes:      inter.ApproachLanes,
			busLane:    inter.ApproachBusLane,
			dedicated:  inter.ApproachIs4Lane,
		}
		if f.world != nil {
			key.revision = f.world.GeometryRevision()
		}
		if scratch.intentValid && scratch.intentKey == key {
			return scratch.intentLeft, scratch.intentRight
		}
	}
	leftLane := 0
	if f.world != nil {
		if road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos); ok {
			leftLane = road.FirstGeneralTrafficLane()
		}
	}
	dedicated := inter.ApproachHasDedicatedLanesForHeading(car.Heading)
	hasLeft := false
	if !dedicated || car.LaneIndex == leftLane {
		hasLeft = hasRoadInDirection(f.world, inter.Center, car.Heading-math.Pi/2) || (dedicated && car.LaneIndex == leftLane && isMapExitAhead(f.world, inter.Center, car.Heading-math.Pi/2))
		if dedicated && car.LaneIndex == leftLane && hasLeft {
			if scratch != nil {
				scratch.intentKey, scratch.intentValid = key, true
				scratch.intentLeft, scratch.intentRight = true, false
			}
			return true, false
		}
	}
	// Boundary exits are legal movements too. Use the same destinations for
	// signal selection, swept-corridor admission and the committed path.
	straight := hasRoadInDirection(f.world, inter.Center, car.Heading) || isMapExitAhead(f.world, inter.Center, car.Heading)
	hasRight := hasRoadInDirection(f.world, inter.Center, car.Heading+math.Pi/2)
	singleLaneTurnLeft := !dedicated && (car.ID <= 0 || car.ID%2 != 0)
	left = hasLeft && ((!straight && (!hasRight || (dedicated && car.LaneIndex == leftLane) || singleLaneTurnLeft)) || (car.LaneIndex == leftLane && dedicated))
	if dedicated && car.LaneIndex != leftLane {
		left = false
	}
	approach := inter.GetApproachIndexForHeading(car.Heading)
	curbTurn := inter.ApproachLanes[approach] >= 8 && car.LaneIndex == 3 &&
		(inter.ApproachBusLane[approach] || car.ID%3 == 0)
	right = !left && hasRight && (curbTurn || !straight)
	if scratch != nil {
		scratch.intentKey, scratch.intentValid = key, true
		scratch.intentLeft, scratch.intentRight = left, right
	}
	return
}

func moveCarThroughJunction(car *sim.TrafficCar, dt float64) {
	turn := car.JunctionTurn
	if car.Speed < 0 && ((car.JunctionReverseActive && !turn.LaneChange) || (car.PassRetreatActive && turn.LaneChange)) {
		distance := -car.Speed * dt
		for distance > 0 {
			step := math.Min(distance, turn.Progress)
			turn.Progress -= step
			distance -= step
			if distance <= 0 || turn.Segment == 0 {
				break
			}
			turn.Segment--
			turn.Progress = turn.Points[turn.Segment].Distance(turn.Points[turn.Segment+1])
		}
		a, b := turn.Points[turn.Segment], turn.Points[turn.Segment+1]
		car.Pos = a.Add(b.Sub(a).Normalize().Mul(turn.Progress))
		car.Heading = math.Atan2(b.Y-a.Y, b.X-a.X)
		return
	}
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

func buildCarJunctionTurn(f carFrame, inter *sim.TrafficIntersection) *sim.CarJunctionTurn {
	car := f.car
	in := sim.UnitFromAngle(car.Heading)
	left := sim.NormalizeAngle(car.Heading - math.Pi/2)
	right := sim.NormalizeAngle(car.Heading + math.Pi/2)
	turnLeft, turnRight := carIntersectionTurns(f, inter)
	if !turnLeft && !turnRight {
		return planCarStraight(f, inter)
	}
	geometry := sim.NewJunctionGeometry(f.laneWidth+6, f.laneWidth+6, f.laneWidth)
	if f.world != nil {
		geometry = f.world.JunctionAt(inter.Center, f.laneWidth)
	}
	heading := right
	if turnLeft {
		heading = left
	}
	out := sim.UnitFromAngle(heading)
	newEW := math.Abs(out.X) > .5
	roadCenter := inter.Center.Y
	if !newEW {
		roadCenter = inter.Center.X
	}
	lanes := max(2, inter.ApproachLanes[inter.GetApproachIndexForHeading(heading+math.Pi)])
	curbIsGeneral, medianExit, medianEntry := false, false, false
	if f.world != nil {
		probe := inter.Center.Add(out.Mul(geometry.TurnRadius + 60))
		lanes = f.world.GetRoadLanes(roadCenter, newEW, probe)
		if road, ok := f.world.RoadAt(roadCenter, newEW, probe); ok {
			curbIsGeneral = road.LaneCount() == 8 && !road.HasBusLane()
			medianExit = road.HasMedianBusLane()
		}
	}
	exitLane := min(1, lanes/2-1)
	inOffset, outOffset, radius := f.laneWidth/2, (float64(exitLane)+.5)*f.laneWidth, geometry.TurnRadius
	if turnLeft && f.world != nil {
		if road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos); ok && road.HasMedianBusLane() {
			inOffset = 1.5 * f.laneWidth
			medianEntry = true
		}
	}
	if turnRight {
		inOffset = math.Abs(car.Pos.Y - car.RoadCenter)
		if !car.IsEW {
			inOffset = math.Abs(car.Pos.X - car.RoadCenter)
		}
		outOffset, radius = f.laneWidth/2, 90
		if lanes >= 4 {
			exitLane = min(2, lanes/2-1)
			if curbIsGeneral {
				exitLane = 3
			}
			outOffset = (float64(exitLane) + .5) * f.laneWidth
		}
	}
	var path []sim.Vec2
	incoming := inter.ApproachLanes[inter.GetApproachIndexForHeading(car.Heading)]
	if turnLeft && car.LaneIndex == 0 && incoming >= 4 && lanes >= 4 && !medianEntry && !medianExit {
		// Keep the inner exit lane for the inner approach. A bus turning
		// beside it can use the outside curve and a separate receiving lane.
		exitLane = 0
		outOffset = (float64(exitLane) + .5) * f.laneWidth
		path = sim.InnerLeftTurnPath(inter.Center, in, out, inOffset, outOffset, geometry, incoming, lanes)
	} else {
		path = sim.QuarterTurnPath(inter.Center, in, out, inOffset, outOffset, radius)
	}
	// The arc ends at its outgoing tangent, usually still inside the box.
	// Retain the original approach and committed lane until the rear clears;
	// otherwise a phase change admits crossing traffic too early.
	half := math.Abs(out.X)*geometry.HalfX + math.Abs(out.Y)*geometry.HalfY
	end := path[len(path)-1]
	clearance := half + car.Length + 50 - end.Sub(inter.Center).Dot(out)
	if clearance > 0 {
		path = append(path, end.Add(out.Mul(clearance)))
	}
	return &sim.CarJunctionTurn{Center: inter.Center, EntryHeading: car.Heading, Points: path, ExitHeading: heading, ExitEW: newEW, ExitRoadCenter: roadCenter, ExitLane: exitLane}
}

// Preserve a straight lane across the box, or taper into a narrower outgoing
// carriageway while still inside it. Navigation never snaps a lane at the exit.
func planCarStraight(f carFrame, inter *sim.TrafficIntersection) *sim.CarJunctionTurn {
	exiting := isMapExitAhead(f.world, inter.Center, f.car.Heading)
	if f.world == nil || (!hasRoadInDirection(f.world, inter.Center, f.car.Heading) && !exiting) {
		return nil
	}
	car := f.car
	u := sim.UnitFromAngle(car.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	g := f.world.JunctionAt(inter.Center, f.laneWidth)
	half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	ew := math.Abs(u.X) > .5
	coord := inter.Center.Y
	if !ew {
		coord = inter.Center.X
	}
	road, ok := f.world.RoadAt(coord, ew, inter.Center.Add(u.Mul(half+100)))
	if !ok && !exiting {
		return nil
	}
	lane := car.LaneIndex
	if !exiting {
		lane = road.ClampGeneralTrafficLane(lane)
	}
	start := -inter.GetStopLineDistForHeading(car.Heading, f.laneWidth) - car.Length/2 + 4
	finish := half + car.Length + 60
	lateral := car.Pos.Sub(inter.Center).Dot(r)
	target := (float64(lane) + .5) * f.laneWidth
	points := make([]sim.Vec2, 33)
	for i := range points {
		t := float64(i) / 32
		ease := t * t * t * (10 - 15*t + 6*t*t)
		points[i] = inter.Center.Add(u.Mul(start + (finish-start)*t)).Add(r.Mul(lateral + (target-lateral)*ease))
	}
	return &sim.CarJunctionTurn{Center: inter.Center, EntryHeading: car.Heading, Straight: true, Points: points, ExitHeading: car.Heading, ExitEW: ew, ExitRoadCenter: coord, ExitLane: lane}
}
