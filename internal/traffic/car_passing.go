package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Civilian passing uses adjacent same-direction general lanes only. Leave enough
// approach space to return to the original movement before its lane arrows.
func (tm *TrafficManager) prepareCarPassing(f carFrame) float64 {
	car := f.car
	if !car.CanOvertake || f.world == nil || car.InRoundabout || car.JunctionTurn != nil {
		return math.Inf(1)
	}
	road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos)
	if !ok || road.GeneralTrafficLanes() < 2 {
		return math.Inf(1)
	}
	u := sim.UnitFromAngle(car.Heading)
	length := carLaneChangeLength(car, f.laneWidth)
	approach := math.Inf(1)
	var nextJunction *sim.TrafficIntersection
	if f.tls != nil {
		for _, inter := range f.tls.Intersections {
			delta := inter.Center.Sub(car.Pos)
			along := delta.Dot(u)
			if along > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < road.GetWidth(f.laneWidth)/2+10 {
				gap := along - inter.GetStopLineDistForHeading(car.Heading, f.laneWidth) - car.Length/2
				if gap < approach {
					approach = gap
					nextJunction = inter
				}
				if gap < 1300 && !hasRoadInDirection(f.world, inter.Center, car.Heading) && !hasRoadInDirection(f.world, inter.Center, car.Heading+math.Pi/2) && hasRoadInDirection(f.world, inter.Center, car.Heading-math.Pi/2) {
					// Do not leave a required left-turn queue to pass and then
					// compete for the same lane again at the end of the road.
					return math.Inf(1)
				}
			}
		}
	}
	for _, rb := range f.world.Roundabouts {
		delta := rb.Center.Sub(car.Pos)
		if delta.Dot(u) > 0 && math.Abs(delta.X*u.Y-delta.Y*u.X) < rb.Radius {
			approach = math.Min(approach, delta.Dot(u)-rb.Radius-150)
		}
	}
	reserve := math.Max(700, car.TargetSpeed*2+length+200)
	if car.Passing {
		passed := true
		fullFrame := f
		fullFrame.candidates = nil
		fullFrame.localCarRadius = 0 // Explicit leader-ID lookup covers all cars.
		for _, other := range tm.carsForFrame(fullFrame) {
			if other.ID == car.PassingLeadID {
				passed = other.Pos.Sub(car.Pos).Dot(u) < -(car.Length+other.Length)/2-math.Max(25, car.Speed*.4)
				break
			}
		}
		if leader := car.PassingLeadBus; leader != nil {
			passed = leader.Pos.Sub(car.Pos).Dot(u) < -(car.Length+leader.Length)/2-math.Max(25, car.Speed*.4)
		}
		if passed || approach < reserve {
			direction := float64(car.PassingBaseLane - car.LaneIndex)
			if math.Abs(direction) < .5 {
				car.Passing = false
				return math.Inf(1)
			}
			if tm.passingLaneClear(f, u, direction) {
				tm.beginCarLaneChange(f, car.PassingBaseLane)
				car.Passing = false
			} else if approach < reserve {
				// An optional passing return must not stop two adjacent
				// queues waiting to swap lanes. Keep this legal lane and let
				// its normal signal/turn planner choose the next movement.
				if nextJunction != nil && carCurrentLaneHasExit(f, nextJunction) {
					car.Passing = false
					car.PassingLeadBus = nil
					car.PassingLeadID = 0
					return math.Inf(1)
				}
				return math.Sqrt(2 * 220 * math.Max(0, approach-length-100))
			}
		}
		return math.Inf(1)
	}
	if approach < reserve {
		return math.Inf(1)
	}
	var leader *sim.TrafficCar
	nearest := math.Max(300, car.Speed*car.Speed/(2*220)+car.Speed+80)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	for _, other := range tm.carsForFrame(f) {
		if other == car || other.InRoundabout || sim.Cos(other.Heading-car.Heading) < .98 {
			continue
		}
		delta := other.Pos.Sub(car.Pos)
		along := delta.Dot(u)
		if along > (car.Length+other.Length)/2+12 && along < nearest && math.Abs(delta.Dot(right)) < (car.Width+other.Width)/2+5 && other.Speed < car.TargetSpeed*.85 {
			leader = other
			nearest = along
		}
	}
	var leadBus *sim.Bus
	for _, bus := range f.buses {
		if bus == nil || sim.Cos(bus.Heading-car.Heading) < .98 {
			continue
		}
		body := bus.BodyEnvelope()
		delta := body.Center.Sub(car.Pos)
		along := delta.Dot(u)
		if along > car.Length/2+body.HalfLen+12 && along < nearest && math.Abs(delta.Dot(right)) < car.Width/2+body.HalfWid+5 && bus.Speed < car.TargetSpeed*.85 {
			leadBus = bus
			nearest = along
		}
	}
	if leader == nil && leadBus == nil {
		return math.Inf(1)
	}
	// Prefer passing on the left, using only general traffic lanes.
	highest := road.FirstGeneralTrafficLane() + road.GeneralTrafficLanes() - 1
	for _, direction := range []int{-1, 1} {
		target := car.LaneIndex + direction
		if target < road.FirstGeneralTrafficLane() || target > highest || !tm.passingLaneClear(f, u, float64(direction)) {
			continue
		}
		car.Passing = true
		car.PassingBaseLane = car.LaneIndex
		car.PassingLeadBus = leadBus
		if leadBus == nil {
			car.PassingLeadID = leader.ID
		}
		tm.beginCarLaneChange(f, target)
		break
	}
	return math.Inf(1)
}

func (tm *TrafficManager) carsForFrame(f carFrame) []*sim.TrafficCar {
	if f.candidates != nil {
		return f.candidates
	}
	if f.localCarRadius > 0 && tm.spatialActive {
		if !tm.localCarsReady {
			tm.localCars(f.car, f.localCarRadius)
			tm.localCarsReady = true
		}
		return tm.nearbyCars
	}
	if f.activeCount > 0 && f.activeCount < len(tm.Cars) {
		return tm.Cars[:f.activeCount]
	}
	return tm.Cars
}

func carLaneChangeLength(car *sim.TrafficCar, lw float64) float64 {
	return math.Max(math.Max(220, 4*lw), car.Speed*1.5)
}

func (tm *TrafficManager) beginCarLaneChange(f carFrame, target int) {
	tm.beginCarLaneChangeWithLength(f, target, carLaneChangeLength(f.car, f.laneWidth))
}

func (tm *TrafficManager) beginCarLaneChangeWithLength(f carFrame, target int, length float64) {
	car := f.car
	heading := math.Atan2(math.Sin(car.Heading), sim.Cos(car.Heading))
	u := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	shift := float64(target-car.LaneIndex) * f.laneWidth
	points := make([]sim.Vec2, 33)
	for i := range points {
		t := float64(i) / float64(len(points)-1)
		ease := t * t * t * (10 - 15*t + 6*t*t)
		points[i] = car.Pos.Add(u.Mul(length * t)).Add(right.Mul(shift * ease))
	}
	car.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true, Points: points, ExitHeading: heading, ExitEW: car.IsEW, ExitRoadCenter: car.RoadCenter, ExitLane: target}
}

func (tm *TrafficManager) passingLaneClear(f carFrame, u sim.Vec2, direction float64) bool {
	if !tm.laneMergeClear(f, u, direction) {
		return false
	}
	right := sim.Vec2{X: -u.Y, Y: u.X}
	passShift := direction * f.laneWidth
	target := f.car.Pos.Add(right.Mul(passShift))
	runway := carLaneChangeLength(f.car, f.laneWidth)
	for _, bus := range f.buses {
		if bus == nil || bus.DoorsOpen {
			continue
		}
		request := bus.LaneMergeRequest
		if request == nil || request.Joined || request.Remaining <= 0 || sim.UnitFromAngle(request.Heading).Dot(u) < .98 {
			continue
		}
		// An optional pass reserves both its source and destination until
		// the tail leaves the source. A bus moving into either lane needs
		// the same space, particularly while pulling out of a stop bay.
		shift := request.Target.Sub(bus.Pos).Dot(right)
		if math.Abs(shift) < 4 || math.Abs(shift) >= f.laneWidth {
			continue
		}
		delta := bus.Pos.Sub(f.car.Pos)
		padding := (bus.Width+f.car.Width)/2 + 4
		x := delta.Dot(right)
		if math.Min(x, x+shift) > math.Max(0, passShift)+padding ||
			math.Max(x, x+shift) < math.Min(0, passShift)-padding {
			continue
		}
		along := delta.Dot(u)
		if along > -bus.Length/2-f.car.Length/2-math.Max(35, bus.Speed) &&
			along < runway+bus.OverallLength()+f.car.Length/2 {
			return false
		}
	}
	for _, other := range tm.carsForFrame(f) {
		if other == f.car || other.JunctionTurn == nil || !other.JunctionTurn.LaneChange {
			continue
		}
		p, _ := other.JunctionTurn.Predict(math.Max(1, other.Speed) * 1.5)
		delta := p.Sub(target)
		if math.Abs(delta.Dot(right)) < (other.Width+f.car.Width)/2+8 && math.Abs(delta.Dot(u)) < runway+(f.car.Length+other.Length)/2+40 {
			return false
		}
	}
	return tm.stoppedTrafficClearsPassingCurve(f, u, direction)
}

func carCurrentLaneHasExit(f carFrame, inter *sim.TrafficIntersection) bool {
	left, right := carIntersectionTurns(f, inter)
	heading := f.car.Heading
	if left {
		heading -= math.Pi / 2
	} else if right {
		heading += math.Pi / 2
	}
	return hasRoadInDirection(f.world, inter.Center, heading) || isMapExitAhead(f.world, inter.Center, heading)
}

const carRecoveryLaneChangeSpeed = 45.

// A passing gap can close after the car starts moving laterally. If the
// remaining curve would hit a stopped bus, use a checked return to the source
// lane instead of pinning both vehicles across their lane boundary.
func (tm *TrafficManager) retreatBlockedCarPass(f carFrame) {
	car, turn := f.car, f.car.JunctionTurn
	if turn == nil || !turn.LaneChange || !car.CanOvertake ||
		f.world == nil || math.Abs(car.Speed) > 1 || f.world.InsideJunction(car.Pos, f.laneWidth, car.Length) {
		return
	}
	replanStalledRecovery := turn.SpeedLimit == carRecoveryLaneChangeSpeed && turn.Segment == 0 && turn.Progress == 0
	if turn.SpeedLimit > 0 && !replanStalledRecovery {
		return
	}
	road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos)
	if !ok || car.LaneIndex < road.FirstGeneralTrafficLane() || car.LaneIndex >= road.LaneCount()/2 {
		return
	}
	if replanStalledRecovery {
		if tm.recoveryTurnClearsTraffic(f, turn, math.Max(120, turn.Points[0].Distance(turn.Points[len(turn.Points)-1])+12)) {
			return
		}
	} else if math.IsInf(tm.carLaneChangeConflict(f, 100, true), 1) {
		return
	}
	heading := turn.ExitHeading
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: car.RoadCenter}
	if !car.IsEW {
		origin = sim.Vec2{X: car.RoadCenter}
	}
	offset := car.Pos.Sub(origin).Dot(r)
	source := (float64(car.LaneIndex) + .5) * f.laneWidth
	if math.Abs(offset-source) > .9*f.laneWidth {
		return // The car has almost finished entering the destination lane.
	}
	yaw := sim.NormalizeAngle(car.Heading - heading)
	if math.Abs(yaw) > .65 {
		return
	}
	baseLen := math.Max(60, 2*math.Abs(offset-source))
	for _, length := range []float64{baseLen, math.Max(85, baseLen*1.4), math.Max(105, baseLen*1.75), math.Max(130, baseLen*2.1)} {
		points := make([]sim.Vec2, 25)
		for i := range points {
			t := float64(i) / float64(len(points)-1)
			shift := (source-offset)*t*t*(3-2*t) + math.Tan(yaw)*length*t*(1-t)*(1-t)
			points[i] = car.Pos.Add(u.Mul(length * t)).Add(r.Mul(shift))
		}
		candidate := &sim.CarJunctionTurn{
			LaneChange:     true,
			Points:         points,
			ExitHeading:    heading,
			ExitEW:         car.IsEW,
			ExitRoadCenter: car.RoadCenter,
			ExitLane:       car.LaneIndex,
			SpeedLimit:     carRecoveryLaneChangeSpeed,
		}
		if !tm.recoveryTurnClearsTraffic(f, candidate, math.Max(120, length+12)) {
			continue
		}
		car.JunctionTurn = candidate
		car.Passing, car.PassingLeadBus, car.PassingLeadID = false, nil, 0
		if !replanStalledRecovery {
			car.DeadlockTimer = 0
			car.DeadlockTracker = sim.StationaryTracker{}
		}
		return
	}
}

func (tm *TrafficManager) recoveryTurnClearsTraffic(f carFrame, turn *sim.CarJunctionTurn, horizon float64) bool {
	car := f.car
	if turn == nil || len(turn.Points) < 2 || f.world == nil {
		return false
	}
	cars := tm.carsForFrame(f)
	heading := turn.ExitHeading
	for distance := 0.; distance <= horizon; distance += 1.0 {
		p, h := turn.Predict(distance)
		if f.world.InsideJunction(p, f.laneWidth, car.Length/2+4) {
			return false
		}
		body := sim.OBB{Center: p, Heading: h, HalfLen: car.Length/2 + 1, HalfWid: car.Width/2 + 1}
		for _, corner := range sim.BodyCorners(sim.OBB{Center: p, Heading: h, HalfLen: car.Length / 2, HalfWid: car.Width / 2}) {
			if f.world.CheckBoundaries(corner, .5, f.laneWidth).Zone == sim.ZoneHardBarrier {
				return false
			}
		}
		for _, bus := range f.buses {
			if bus == nil || bus.Pos.Distance(body.Center) > bus.OverallLength()+car.Length+30 {
				continue
			}
			check := body
			if math.Abs(bus.Speed) < 1 && sim.Cos(bus.Heading-heading) > .98 {
				check.HalfLen, check.HalfWid = car.Length/2+.15, car.Width/2+.15
			}
			if hit, _, _ := bus.CollidesBody(check); hit {
				return false
			}
		}
		for _, other := range cars {
			if other == car || other.Pos.Distance(body.Center) > car.Length+other.Length {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}); hit {
				return false
			}
		}
	}
	return true
}

// Recheck a live lane change after traffic has advanced. The current heading
// alone misses vehicles that intersect the remaining lateral curve.
func (tm *TrafficManager) carLaneChangeConflict(f carFrame, horizon float64, stoppedOnly bool) float64 {
	car, turn := f.car, f.car.JunctionTurn
	cars := tm.carsForFrame(f)
	invSpeed := 1 / math.Max(20, car.Speed)
	for distance := 4.; distance <= horizon; distance += 4 {
		p, h := turn.Predict(distance)
		elapsed := distance * invSpeed
		body := sim.OBB{Center: p, Heading: h, HalfLen: car.Length/2 + 1, HalfWid: car.Width/2 + 1}
		for _, other := range cars {
			if other == car || (stoppedOnly && other.Speed > 3) {
				continue
			}
			reach := car.Length + other.Length + math.Abs(other.Speed)*elapsed
			d := other.Pos.Sub(p)
			if d.Dot(d) > reach*reach {
				continue
			}
			pos, heading := other.Pos, other.Heading
			if other.JunctionTurn != nil && other.Speed > 0 {
				pos, heading = other.JunctionTurn.Predict(other.Speed * elapsed)
			} else if other.Speed != 0 {
				pos = pos.Add(sim.UnitFromAngle(other.Heading).Mul(other.Speed * elapsed))
			}
			if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: pos, Heading: heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}); hit {
				return distance
			}
		}
		for _, bus := range f.buses {
			if bus == nil || (stoppedOnly && math.Abs(bus.Speed) > 2) {
				continue
			}
			reach := car.Length + bus.OverallLength() + math.Abs(bus.Speed)*elapsed
			d := bus.Pos.Sub(p)
			if d.Dot(d) > reach*reach {
				continue
			}
			var parts [2]sim.OBB
			if bus.Speed == 0 {
				parts = bus.BodyParts()
			} else {
				var pos sim.Vec2
				var heading float64
				if prediction, ok := sim.PredictJunctionBody(bus.JunctionCorridor, math.Max(0, bus.Speed)*elapsed); ok {
					pos, heading = prediction.Center, prediction.Heading
				} else {
					pos, heading = bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Speed*elapsed)), bus.Heading
				}
				parts = bus.BodyPartsAtPose(pos, heading)
			}
			for _, part := range parts {
				if part.HalfLen <= 0 {
					continue
				}
				shape := body
				if !stoppedOnly && turn.SpeedLimit == carRecoveryLaneChangeSpeed && math.Abs(bus.Speed) < 1 {
					shape.HalfLen, shape.HalfWid = car.Length/2, car.Width/2
				}
				if hit, _, _ := sim.CheckOBBCollision(shape, part); hit {
					return distance
				}
			}
		}
	}
	return math.Inf(1)
}

// A stopped car beside an articulated rear can be the vehicle preventing the
// bus from straightening. Give it a safe adjacent exit from that deadlock even
// if its normal cruising tier does not initiate passing.
func (tm *TrafficManager) releaseCarBesideBusTail(f carFrame) {
	car := f.car
	if car.JunctionTurn != nil || car.InRoundabout || math.Abs(car.Speed) > 1 || f.world == nil || car.DeadlockTimer < 2 ||
		f.world.InsideJunction(car.Pos, f.laneWidth, car.Length) {
		return
	}
	road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos)
	if !ok || car.LaneIndex <= road.FirstGeneralTrafficLane() {
		return
	}
	u := sim.UnitFromAngle(car.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for _, bus := range f.buses {
		if bus == nil || !bus.IsArticulated() || math.Abs(bus.Speed) > 1 || bus.DoorsOpen || sim.Cos(bus.Heading-car.Heading) < .9 {
			continue
		}
		d := bus.Pos.Sub(car.Pos)
		tail := bus.TrailerBody()
		if d.Dot(u) <= car.Length/2 || d.Dot(r) < 0 || math.Abs(car.Pos.Sub(tail.Center).Dot(u)) > tail.HalfLen+car.Length/2+12 ||
			math.Abs(car.Pos.Sub(tail.Center).Dot(r)) > tail.HalfWid+car.Width/2+14 {
			continue
		}
		for _, length := range []float64{1.5 * f.laneWidth, 2 * f.laneWidth, 3 * f.laneWidth} {
			tm.beginCarLaneChangeWithLength(f, car.LaneIndex-1, length)
			car.JunctionTurn.SpeedLimit = carRecoveryLaneChangeSpeed
			if math.IsInf(tm.carLaneChangeConflict(f, length, false), 1) {
				car.Passing, car.PassingLeadBus, car.PassingLeadID = false, nil, 0
				car.DeadlockTimer = 0
				car.DeadlockTracker = sim.StationaryTracker{}
				return
			}
		}
		car.JunctionTurn = nil
		return
	}
}

// The destination lane being clear is insufficient: the departure half of
// a long lane-change curve can still run through a stopped source-lane leader.
func (tm *TrafficManager) stoppedTrafficClearsPassingCurve(f carFrame, forward sim.Vec2, direction float64) bool {
	car := f.car
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	length := carLaneChangeLength(car, f.laneWidth)
	shift := direction * f.laneWidth
	cars := tm.carsForFrame(f)
	for i := 0; i <= 64; i++ {
		t := float64(i) / 64
		ease := t * t * t * (10 - 15*t + 6*t*t)
		tangent := forward.Mul(length).Add(right.Mul(shift * 30 * t * t * (1 - t) * (1 - t)))
		body := sim.OBB{Center: car.Pos.Add(forward.Mul(length * t)).Add(right.Mul(shift * ease)), Heading: math.Atan2(tangent.Y, tangent.X), HalfLen: car.Length/2 + 2, HalfWid: car.Width/2 + 1}
		for _, other := range cars {
			if other == nil || other == car || math.Abs(other.Speed) > 3 || other.Pos.Distance(body.Center) > car.Length+other.Length {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}); hit {
				return false
			}
		}
		for _, bus := range f.buses {
			if bus == nil || math.Abs(bus.Speed) > 1 || bus.Pos.Distance(body.Center) > car.Length+bus.OverallLength() {
				continue
			}
			if hit, _, _ := bus.CollidesBody(body); hit {
				return false
			}
		}
	}
	return true
}
