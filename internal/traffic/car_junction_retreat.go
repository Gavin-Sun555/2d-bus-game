package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A vehicle already committed to the box may back along its own traversed path
// to release a mutually wedged crossing vehicle. Both forward paths must be
// physically blocked; ordinary signal queues never qualify.
func (tm *TrafficManager) stepJunctionRetreat(f carFrame) (bool, float64) {
	c := f.car
	tr := c.JunctionTurn
	if tr == nil || tr.LaneChange || c.InRoundabout || f.world == nil {
		c.JunctionReverseYieldID = 0
		c.JunctionReverseYieldBus = false
		c.JunctionReverseActive = false
		return false, 0
	}
	if tr.Straight && math.Cos(tr.ExitHeading-tr.EntryHeading) < .99 {
		c.JunctionReverseYieldID = 0
		c.JunctionReverseYieldBus = false
		c.JunctionReverseActive = false
		return false, 0
	}
	if !c.JunctionReverseActive {
		if math.Abs(c.Speed) > .5 || (c.DeadlockTimer < 5 && c.YieldReverseRequest <= 0) {
			return false, 0
		}
		if c.DeadlockTimer >= 5 {
			for _, other := range tm.carsForFrame(f) {
				if other == nil {
					continue
				}
				ot := other.JunctionTurn
				if other == c || ot == nil || ot.LaneChange || other.JunctionReverseActive || math.Abs(other.Speed) > .5 || ot.Center.Distance(tr.Center) > 1 ||
					math.Cos(ot.EntryHeading-tr.EntryHeading) > .2 || (tr.Straight && ot.Straight && math.Cos(ot.EntryHeading-tr.EntryHeading) < -.8) {
					continue
				}
				if ot.Straight && math.Cos(ot.ExitHeading-ot.EntryHeading) < .99 {
					continue
				}
				yieldFirst := (tr.Straight && !ot.Straight) || (tr.Straight == ot.Straight && c.ID > other.ID)
				if !yieldFirst && c.DeadlockTimer < 6.5 {
					continue
				}
				if !junctionPathHits(c, other, 42) || !junctionPathHits(other, c, 42) {
					continue
				}
				for _, d := range []float64{16, 24, 32, 48, 60, 72} {
					if !tm.junctionReverseClear(f, d+3) {
						continue
					}
					probe := *c
					pTurn := *tr
					probe.JunctionTurn = &pTurn
					probe.JunctionReverseActive = true
					probe.Speed = -d
					moveCarThroughJunction(&probe, 1)
					if junctionPathHits(other, &probe, 120) {
						continue
					}
					c.JunctionReverseYieldID = other.ID
					c.JunctionReverseYieldBus = false
					c.JunctionReverseActive = true
					c.ReverseRemaining = d
					c.DeadlockTimer = 0
					break
				}
				if c.JunctionReverseActive {
					break
				}
			}
		}
		if !c.JunctionReverseActive && tr.Straight && c.DeadlockTimer >= 5 {
			for _, bus := range f.buses {
				if bus == nil || math.Abs(bus.Speed) > .5 {
					continue
				}
				jc := bus.JunctionCorridor
				if jc == nil || !jc.Entered || jc.Remaining <= 0 || jc.Center.Distance(tr.Center) > 1 {
					continue
				}
				if math.Cos(jc.EntryHeading-tr.EntryHeading) >= .5 && math.Cos(bus.Heading-tr.EntryHeading) >= .85 {
					continue
				}
				if !junctionCarHitsBus(c, bus, 56) || (!junctionBusCorridorHitsCar(jc, c, 24) && c.Pos.Distance(bus.Pos) > bus.Length/2+c.Length/2+32) {
					continue
				}
				rearBlocked := false
				for _, d := range []float64{24, 36, 48, 64, 80, 96, 112, 128, 144, 160, 180} {
					if !tm.junctionReverseClear(f, d+3) {
						rearBlocked = true
						continue
					}
					probe := *c
					pTurn := *tr
					probe.JunctionTurn = &pTurn
					probe.JunctionReverseActive = true
					probe.Speed = -d
					moveCarThroughJunction(&probe, 1)
					if junctionBusCorridorHitsCar(jc, &probe, 6) || junctionCarHitsBus(&probe, bus, 2) {
						continue
					}
					c.JunctionReverseYieldBus = true
					c.JunctionReverseYieldID = 0
					c.JunctionReverseActive = true
					c.ReverseRemaining = d
					c.DeadlockTimer = 0
					break
				}
				if c.JunctionReverseActive {
					break
				}
				if rearBlocked {
					tm.relayJunctionReverseToRear(f)
				}
			}
		}
		if !c.JunctionReverseActive && tr.Straight && c.YieldReverseRequest > 0 {
			rearBlocked := false
			for _, d := range []float64{36, 24, 16} {
				if !tm.junctionReverseClear(f, d+3) {
					rearBlocked = true
					continue
				}
				c.JunctionReverseYieldBus = true
				c.JunctionReverseYieldID = -1
				c.JunctionReverseActive = true
				c.ReverseRemaining = d
				c.DeadlockTimer = 0
				c.YieldReverseRequest = 0
				break
			}
			if rearBlocked && !c.JunctionReverseActive {
				tm.relayJunctionReverseToRear(f)
			}
		}
	}
	if !c.JunctionReverseActive {
		return false, 0
	}
	c.IsBraking = true
	if c.ReverseRemaining > .25 {
		// Recheck the complete remaining maneuver against current observations.
		if !tm.junctionReverseClear(f, c.ReverseRemaining+3) {
			tm.relayJunctionReverseToRear(f)
			return true, 0
		}
		return true, -math.Min(10, math.Sqrt(2*120*c.ReverseRemaining))
	}
	c.ReverseRemaining = 0
	if c.Speed < -.1 {
		return true, 0
	}
	if c.JunctionReverseYieldBus {
		if c.JunctionReverseYieldID == -1 {
			fwd := sim.UnitFromAngle(c.Heading)
			for _, other := range tm.carsForFrame(f) {
				if other != nil && other != c && other.JunctionReverseActive &&
					other.Pos.Sub(c.Pos).Dot(fwd) > 0 && other.Pos.Distance(c.Pos) < 110 {
					return true, 0
				}
			}
		} else {
			for _, bus := range f.buses {
				if bus == nil || bus.Pos.Distance(c.Pos) >= 170 {
					continue
				}
				if jc := bus.JunctionCorridor; jc != nil && jc.Entered && jc.Remaining > 0 && jc.Center.Distance(tr.Center) <= 1 {
					return true, 0
				}
			}
		}
	} else {
		for _, other := range tm.carsForFrame(f) {
			if other != nil && other.ID == c.JunctionReverseYieldID && other.Pos.Distance(c.Pos) < 110 {
				return true, 0
			}
		}
	}
	c.JunctionReverseYieldID = 0
	c.JunctionReverseYieldBus = false
	c.JunctionReverseActive = false
	return false, 0
}

func junctionPathHits(moving, fixed *sim.TrafficCar, distance float64) bool {
	target := sim.OBB{Center: fixed.Pos, Heading: fixed.Heading, HalfLen: fixed.Length/2 + 1, HalfWid: fixed.Width/2 + 1}
	for d := 0.; d <= distance; d += 2 {
		p, h := moving.JunctionTurn.Predict(d)
		box := sim.OBB{Center: p, Heading: h, HalfLen: moving.Length / 2, HalfWid: moving.Width / 2}
		if hit, _, _ := sim.CheckOBBCollision(box, target); hit {
			return true
		}
	}
	return false
}

func junctionCarHitsBus(car *sim.TrafficCar, bus *sim.Bus, distance float64) bool {
	if car == nil || car.JunctionTurn == nil || bus == nil {
		return false
	}
	parts := bus.BodyParts()
	for d := 0.; d <= distance; d += 2 {
		p, h := car.JunctionTurn.Predict(d)
		box := sim.OBB{Center: p, Heading: h, HalfLen: car.Length/2 + 3, HalfWid: car.Width/2 + 3}
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(box, part); hit {
				return true
			}
		}
	}
	return false
}

func junctionBusCorridorHitsCar(jc *sim.JunctionCorridor, car *sim.TrafficCar, pad float64) bool {
	if jc == nil || car == nil || len(jc.Bodies) == 0 {
		return false
	}
	target := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length/2 + pad, HalfWid: car.Width/2 + pad}
	for _, body := range jc.Bodies {
		if body.HalfLen <= 0 {
			continue
		}
		if hit, _, _ := sim.CheckOBBCollision(body, target); hit {
			return true
		}
	}
	return false
}

func (tm *TrafficManager) relayJunctionReverseToRear(f carFrame) {
	c := f.car
	fwd := sim.UnitFromAngle(c.Heading)
	var nearest *sim.TrafficCar
	bestAlong := -math.Inf(1)
	for _, other := range tm.carsForFrame(f) {
		if other == nil || other == c || math.Abs(other.Speed) >= 2.0 || math.Cos(other.Heading-c.Heading) <= 0.85 {
			continue
		}
		delta := other.Pos.Sub(c.Pos)
		along := delta.Dot(fwd)
		lat := math.Abs(-delta.X*fwd.Y + delta.Y*fwd.X)
		if along >= 0 || along < -180 || lat > (c.Width+other.Width)*0.5+6 {
			continue
		}
		if along > bestAlong {
			bestAlong = along
			nearest = other
		}
	}
	if nearest == nil {
		return
	}
	if ot := nearest.JunctionTurn; ot != nil && ot.Straight && !ot.LaneChange && math.Cos(ot.ExitHeading-ot.EntryHeading) >= .99 {
		nearest.YieldReverseRequest = carDeadlockYieldRequestTTL
		return
	}
	tm.requestCarYieldReverse(f, nearest, false)
}

func (tm *TrafficManager) junctionReverseClear(f carFrame, distance float64) bool {
	c := f.car
	tr := c.JunctionTurn
	if tr.Straight {
		traversed := tr.Progress
		for i := 0; i < tr.Segment; i++ {
			delta := tr.Points[i+1].Sub(tr.Points[i])
			if delta.Normalize().Dot(sim.UnitFromAngle(c.Heading)) < .999 {
				return false
			}
			traversed += delta.Length()
		}
		if traversed < distance+2 {
			return false
		}
	} else if carTurnHistory(tr) < distance+2 {
		return false
	}
	for d := 0.; d <= distance; d += 1 {
		pose := *c
		pTurn := *tr
		pose.JunctionTurn = &pTurn
		pose.JunctionReverseActive = true
		pose.Speed = -d
		moveCarThroughJunction(&pose, 1)
		pos, heading := pose.Pos, pose.Heading
		if !f.world.InsideJunction(pos, f.laneWidth, 0) || f.world.IsAtRoundabout(pos) || f.world.VehicleOnLevelCrossing(pos, heading, c.Length+6) {
			return false
		}
		box := sim.OBB{Center: pos, Heading: heading, HalfLen: c.Length/2 + 1, HalfWid: c.Width/2 + 1}
		for _, corner := range sim.BodyCorners(box) {
			if f.world.CheckBoundaries(corner, .5, f.laneWidth).Zone == sim.ZoneHardBarrier {
				return false
			}
		}
		for _, other := range tm.carsForFrame(f) {
			if other == nil || other == c || other.Pos.Distance(pos) > 180+math.Abs(other.Speed)*2 {
				continue
			}
			// Include approaching traffic's braking reserve, not just its current body.
			reserve := math.Abs(other.Speed)*.4 + other.Speed*other.Speed/(2*220)
			ob := sim.OBB{Center: other.Pos.Add(sim.UnitFromAngle(other.Heading).Mul(math.Copysign(reserve/2, other.Speed))), Heading: other.Heading, HalfLen: other.Length/2 + reserve/2 + 1, HalfWid: other.Width/2 + 1}
			if hit, _, _ := sim.CheckOBBCollision(box, ob); hit {
				return false
			}
		}
		for _, b := range f.buses {
			if b == nil {
				continue
			}
			for _, part := range b.BodyParts() {
				if part.HalfLen <= 0 {
					continue
				}
				part.HalfLen += math.Abs(b.Speed)*.4 + b.Speed*b.Speed/(2*160) + 1
				part.HalfWid++
				if hit, _, _ := sim.CheckOBBCollision(box, part); hit {
					return false
				}
			}
		}
	}
	return true
}
