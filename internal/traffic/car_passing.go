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
	// Prefer passing on the left; the bus lane and dedicated right-turn lane
	// are excluded from ordinary cruising by the lane count below.
	highest := road.GeneralTrafficLanes() - 1
	if road.LaneCount() == 8 && !road.HasBusLane() {
		highest = 2
	}
	for _, direction := range []int{-1, 1} {
		target := car.LaneIndex + direction
		if target < 0 || target > highest || !tm.passingLaneClear(f, u, float64(direction)) {
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
	car := f.car
	heading := math.Atan2(math.Sin(car.Heading), math.Cos(car.Heading))
	u := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	length := carLaneChangeLength(car, f.laneWidth)
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
