package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A blocked optional pass can have no travelled curve to reverse along.
// Cancel it in place while still aligned in the source lane, so the ordinary
// rear-clearance logic can give the bus space.
func (tm *TrafficManager) cancelUnstartedCarPass(f carFrame) {
	car := f.car
	turn := car.JunctionTurn
	if !car.Passing || turn == nil || !turn.LaneChange || turn.SpeedLimit != 0 || !car.CanOvertake ||
		car.PassRetreatActive || math.Abs(car.Speed) >= 1 || car.DeadlockTimer < carDeadlockReverseTriggerSec ||
		f.world == nil || turn.Segment != 0 || turn.Progress != 0 || len(turn.Points) < 2 ||
		car.Pos.Distance(turn.Points[0]) > .01 || math.Cos(car.Heading-turn.ExitHeading) < .999 ||
		f.world.InsideJunction(car.Pos, f.laneWidth, car.Length/2+8) {
		return
	}
	if tm.carLaneChangeConflict(f, 100, true) > 16 {
		return
	}
	car.JunctionTurn, car.JunctionCorridor = nil, nil
	car.Passing, car.PassingLeadBus, car.PassingLeadID = false, nil, 0
}

// A stopped optional lane change can already be physically aligned with its
// destination while a few sampled tail points remain. Finish only that
// bookkeeping so ordinary following and a checked rear-slot yield can work.
func finishAlignedCarPass(f carFrame) bool {
	c := f.car
	tr := c.JunctionTurn
	if tr == nil || !tr.LaneChange || tr.SpeedLimit != 0 || c.PassRetreatActive || c.ReverseRemaining > 0 || math.Abs(c.Speed) >= 1 ||
		f.world == nil || len(tr.Points) < 2 ||
		f.world.InsideJunction(c.Pos, f.laneWidth, c.Length/2+8) || math.Cos(c.Heading-tr.ExitHeading) < .9999 {
		return false
	}
	u := sim.UnitFromAngle(tr.ExitHeading)
	goal := tr.Points[len(tr.Points)-1].Sub(c.Pos)
	pastEnd := tr.Segment == len(tr.Points)-2 && goal.Dot(u) <= 0 && goal.Dot(u) >= -c.Length
	if !pastEnd && (c.DeadlockTimer < carDeadlockReverseTriggerSec && c.YieldReverseRequest <= 0) {
		return false
	}
	if (!pastEnd && goal.Dot(u) < 0) || goal.Dot(u) > c.Length+8 || math.Abs(goal.X*u.Y-goal.Y*u.X) > .3 {
		return false
	}
	for i := tr.Segment; i < len(tr.Points)-1; i++ {
		if tr.Points[i+1].Sub(tr.Points[i]).Normalize().Dot(u) < .9999 {
			return false
		}
	}
	if _, ok := f.world.RoadAt(tr.ExitRoadCenter, tr.ExitEW, c.Pos); !ok {
		return false
	}
	c.IsEW, c.RoadCenter, c.LaneIndex = tr.ExitEW, tr.ExitRoadCenter, tr.ExitLane
	c.JunctionTurn, c.JunctionCorridor = nil, nil
	return true
}

func carTurnHistory(turn *sim.CarJunctionTurn) float64 {
	if turn == nil || turn.Segment < 0 || turn.Segment >= len(turn.Points)-1 {
		return 0
	}
	d := turn.Progress
	for i := 0; i < turn.Segment; i++ {
		d += turn.Points[i].Distance(turn.Points[i+1])
	}
	return d
}
func carTurnRemaining(turn *sim.CarJunctionTurn) float64 {
	if turn == nil || turn.Segment < 0 || turn.Segment >= len(turn.Points)-1 {
		return 0
	}
	d := math.Max(0, turn.Points[turn.Segment].Distance(turn.Points[turn.Segment+1])-turn.Progress)
	for i := turn.Segment + 1; i < len(turn.Points)-1; i++ {
		d += turn.Points[i].Distance(turn.Points[i+1])
	}
	return d
}
func carReverseAttemptLimit(car *sim.TrafficCar) int {
	if car.PassRetreatActive {
		return 8
	}
	return carDeadlockReverseMaxAttempts
}
func (tm *TrafficManager) blockedOptionalPass(f carFrame) bool {
	if f.car.PassRetreatActive {
		horizon := math.Max(120, carTurnRemaining(f.car.JunctionTurn)+60)
		if f.car.ReverseRemaining <= 0 && math.Abs(f.car.Speed) < .5 &&
			math.IsInf(tm.carLaneChangeConflict(f, horizon, true), 1) && !tm.carBlockedByStoppedBus(f, false) {
			f.car.PassRetreatActive = false
			f.car.ReverseHold = 0
			return false
		}
		return true
	}
	tr := f.car.JunctionTurn
	hist := carTurnHistory(tr)
	return tr != nil && tr.LaneChange && tr.SpeedLimit == 0 && f.car.CanOvertake && hist >= 4 && hist <= 260 &&
		math.Cos(f.car.Heading-tr.ExitHeading) > .75 &&
		(!math.IsInf(tm.carLaneChangeConflict(f, 100, true), 1) || tm.carBlockedByStoppedBus(f, true))
}
func (tm *TrafficManager) canRetreatOptionalPass(f carFrame) bool {
	tr := f.car.JunctionTurn
	return tr != nil && tr.LaneChange && tr.SpeedLimit == 0 && carTurnHistory(tr) <= 260 &&
		math.Cos(f.car.Heading-tr.ExitHeading) > .75 && (f.car.PassRetreatActive || f.car.YieldReverseRequest > 0 || tm.blockedOptionalPass(f))
}
func (tm *TrafficManager) finishPassRetreat(car *sim.TrafficCar) {
	tr := car.JunctionTurn
	if !car.PassRetreatActive || tr == nil || !tr.LaneChange || carTurnHistory(tr) > .5 || math.Abs(car.Speed) > 1 {
		return
	}
	car.JunctionTurn, car.JunctionCorridor = nil, nil
	car.Heading = tr.ExitHeading
	car.Passing, car.PassingLeadBus, car.PassingLeadID = false, nil, 0
	car.PassRetreatActive = false
	car.ReverseRemaining = 0
	car.ReverseHold = carDeadlockReverseHoldSec
	car.ReverseCooldown = carDeadlockReverseCooldown
	car.ReverseAttempts = 0
}

// Roll back the already travelled optional curve, reserving the same 12px
// stopping gap as straight-road reverse. Never invent a new lateral path.
func (tm *TrafficManager) availablePassRetreatDistance(f carFrame, maximum float64) (float64, *sim.TrafficCar, bool) {
	car := f.car
	maximum = math.Min(maximum, carTurnHistory(car.JunctionTurn))
	safe := 0.
	for step := 0; step <= int(math.Ceil(maximum/2)); step++ {
		d := math.Min(float64(step)*2, maximum)
		pose := *car
		turn := *car.JunctionTurn
		pose.JunctionTurn = &turn
		pose.PassRetreatActive = true
		pose.Speed = -d
		moveCarThroughJunction(&pose, 1)
		if f.world == nil || f.world.InsideJunction(pose.Pos, f.laneWidth, pose.Length/2+8) || f.world.VehicleOnLevelCrossing(pose.Pos, pose.Heading, pose.Length+12) {
			return safe, nil, false
		}
		box := sim.OBB{Center: pose.Pos.Sub(sim.UnitFromAngle(pose.Heading).Mul(6)), Heading: pose.Heading, HalfLen: pose.Length/2 + 6, HalfWid: pose.Width/2 + 1}
		for _, corner := range sim.BodyCorners(box) {
			if f.world.CheckBoundaries(corner, .5, f.laneWidth).Zone == sim.ZoneHardBarrier {
				return safe, nil, false
			}
		}
		for _, other := range tm.carsForFrame(f) {
			if other == nil || other == car || other.Pos.Distance(pose.Pos) > car.Length+other.Length+100+math.Abs(other.Speed)*2 {
				continue
			}
			reserve := math.Abs(other.Speed)*.4 + other.Speed*other.Speed/(2*220)
			ob := sim.OBB{Center: other.Pos.Add(sim.UnitFromAngle(other.Heading).Mul(math.Copysign(reserve/2, other.Speed))), Heading: other.Heading, HalfLen: other.Length/2 + reserve/2, HalfWid: other.Width / 2}
			if hit, _, _ := sim.CheckOBBCollision(box, ob); hit {
				return safe, other, other.Speed < -1 || other.ReverseRemaining > 0
			}
		}
		for _, bus := range f.buses {
			if bus == nil || bus.Pos.Distance(pose.Pos) > bus.OverallLength()+car.Length+100+math.Abs(bus.Speed)*2 {
				continue
			}
			for _, part := range bus.BodyParts() {
				if part.HalfLen <= 0 {
					continue
				}
				part.HalfLen += math.Abs(bus.Speed)*.4 + bus.Speed*bus.Speed/(2*160)
				if hit, _, _ := sim.CheckOBBCollision(box, part); hit {
					return safe, nil, false
				}
			}
		}
		safe = d
	}
	return safe, nil, false
}
