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
		if other == car || other.InRoundabout || math.Cos(other.Heading-car.Heading) < .98 {
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
		if bus == nil || math.Cos(bus.Heading-car.Heading) < .98 {
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
	heading := math.Atan2(math.Sin(car.Heading), math.Cos(car.Heading))
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
	target := f.car.Pos.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(direction * f.laneWidth))
	right := sim.Vec2{X: -u.Y, Y: u.X}
	for _, other := range tm.carsForFrame(f) {
		if other == f.car || other.JunctionTurn == nil || !other.JunctionTurn.LaneChange {
			continue
		}
		p, _ := other.JunctionTurn.Predict(math.Max(1, other.Speed) * 1.5)
		delta := p.Sub(target)
		if math.Abs(delta.Dot(right)) < (other.Width+f.car.Width)/2+8 && math.Abs(delta.Dot(u)) < carLaneChangeLength(f.car, f.laneWidth)+(f.car.Length+other.Length)/2+40 {
			return false
		}
	}
	return true
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
	if turn == nil || !turn.LaneChange || turn.SpeedLimit > 0 || !car.CanOvertake ||
		f.world == nil || math.Abs(car.Speed) > 1 || f.world.InsideJunction(car.Pos, f.laneWidth, car.Length) {
		return
	}
	road, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos)
	if !ok || car.LaneIndex < road.FirstGeneralTrafficLane() || car.LaneIndex >= road.LaneCount()/2 ||
		math.IsInf(tm.carLaneChangeConflict(f, 100, true), 1) {
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
	length := math.Max(60, 2*math.Abs(offset-source))
	yaw := sim.NormalizeAngle(car.Heading - heading)
	if math.Abs(yaw) > .65 {
		return
	}
	var points [25]sim.Vec2
	for i := range points {
		t := float64(i) / float64(len(points)-1)
		shift := (source-offset)*t*t*(3-2*t) + math.Tan(yaw)*length*t*(1-t)*(1-t)
		points[i] = car.Pos.Add(u.Mul(length * t)).Add(r.Mul(shift))
		if i == 0 {
			continue
		}
		d := points[i].Sub(points[i-1])
		body := sim.OBB{Center: points[i], Heading: math.Atan2(d.Y, d.X), HalfLen: car.Length/2 + 1, HalfWid: car.Width/2 + 1}
		for _, bus := range f.buses {
			if bus == nil || bus.Pos.Distance(body.Center) > bus.OverallLength()+car.Length+30 {
				continue
			}
			check := body
			if math.Abs(bus.Speed) < 1 && math.Cos(bus.Heading-heading) > .98 {
				check.HalfLen, check.HalfWid = car.Length/2, car.Width/2
			}
			if hit, _, _ := bus.CollidesBody(check); hit {
				return
			}
		}
		for _, other := range tm.carsForFrame(f) {
			if other == car || other.Pos.Distance(body.Center) > car.Length+other.Length {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}); hit {
				return
			}
		}
	}
	car.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true, Points: points[:], ExitHeading: heading, ExitEW: car.IsEW, ExitRoadCenter: car.RoadCenter, ExitLane: car.LaneIndex, SpeedLimit: carRecoveryLaneChangeSpeed}
	car.Passing, car.PassingLeadBus, car.PassingLeadID = false, nil, 0
	car.DeadlockTimer = 0
	car.DeadlockTracker = sim.StationaryTracker{}
}

// Recheck a live lane change after traffic has advanced. The current heading
// alone misses vehicles that intersect the remaining lateral curve.
func (tm *TrafficManager) carLaneChangeConflict(f carFrame, horizon float64, stoppedOnly bool) float64 {
	car, turn := f.car, f.car.JunctionTurn
	cars := tm.carsForFrame(f)
	for distance := 4.; distance <= horizon; distance += 4 {
		p, h := turn.Predict(distance)
		elapsed := distance / math.Max(20, car.Speed)
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
			pos := other.Pos.Add(sim.UnitFromAngle(other.Heading).Mul(other.Speed * elapsed))
			heading := other.Heading
			if other.JunctionTurn != nil && other.Speed > 0 {
				pos, heading = other.JunctionTurn.Predict(other.Speed * elapsed)
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
			pos := bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Speed * elapsed))
			heading := bus.Heading
			parts := bus.BodyParts()
			if bus.Speed != 0 {
				if prediction, ok := sim.PredictJunctionBody(bus.JunctionCorridor, math.Max(0, bus.Speed)*elapsed); ok {
					pos, heading = prediction.Center, prediction.Heading
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
		if bus == nil || !bus.IsArticulated() || math.Abs(bus.Speed) > 1 || bus.DoorsOpen || math.Cos(bus.Heading-car.Heading) < .9 {
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
