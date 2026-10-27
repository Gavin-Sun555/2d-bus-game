package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Prefer an earlier service position when the front queue leaves too little
// road to steer a long bus into the adjacent lane after closing the doors.
// This is a berth preference; departure still needs live physical sweeps.
func outerBerthExitRoom(bus *sim.Bus, target sim.Vec2, heading float64, obstacles []VehicleObstacle) bool {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for _, ob := range obstacles {
		if math.Abs(ob.Speed) >= 3 || math.Cos(ob.Heading-heading) < .98 {
			continue
		}
		d := ob.Pos.Sub(target)
		if d.Dot(u) <= 0 || math.Abs(d.Dot(r)) > (bus.Width+ob.Width)/2+5 {
			continue
		}
		if d.Dot(u)-(bus.Length+ob.Length)/2 < 80 {
			return false
		}
	}
	return true
}

// Roadside service may finish just behind a stopped queue. Use the existing
// actuator-checked crawl into an empty adjacent lane, rather than requiring
// forward progress in the blocked lane before starting a lane change.
func (ad *AutopilotDriver) prepareServedStationMerge(f autopilotFrame) {
	passage := ad.skippedDeparture
	if passage == nil || !passage.served || !ad.BerserkMode || !f.bus.IsArticulated() || ad.midLaneRecovery != nil || ad.State != AutopilotStateCruising || f.world == nil || f.speedAbs > 5 || f.bus.DoorsOpen || f.bus.Pos.Distance(passage.stop.Pos) > passage.stop.Length+f.bus.OverallLength() {
		return
	}
	h := passage.stop.Heading
	if math.Cos(f.bus.Heading-h) < .98 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		return
	}
	u := sim.UnitFromAngle(h)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	blocked := false
	for _, ob := range f.obstacles {
		d := ob.Pos.Sub(f.bus.Pos)
		if math.Abs(ob.Speed) < 3 && math.Cos(ob.Heading-h) > .98 && d.Dot(u) > 0 && d.Dot(u) < 220 && math.Abs(d.Dot(r)) < (f.bus.Width+ob.Width)/2+2 {
			blocked = true
			break
		}
	}
	if !blocked {
		return
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, h, f.laneWidth)
	if !multi && !six {
		return
	}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lateral := f.bus.Pos.Sub(origin).Dot(r)
	lane := math.Floor(lateral / f.laneWidth)
	if lane < 1 {
		return
	} // Never borrow an opposing lane for this escape.
	destination := lane - 1
	if f.laneWidth < f.bus.Width+12 && lane >= 2 {
		// On narrow lanes, one lane leaves little room for trailer offtracking
		// beside a stopped bus. Reserve both crossed lanes before moving.
		destination = lane - 2
	}
	target := origin.Add(r.Mul((destination + .5) * f.laneWidth))
	base := ad.pointAt(f.s)
	if !requiredMergeLaneClear(f, h, f.bus.Pos.Sub(base).Dot(r), target.Sub(base).Dot(r)) {
		return
	}
	ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: target, Heading: h, Remaining: .3}
}

func (ad *AutopilotDriver) clearsServedStationQueue(f autopilotFrame, ob VehicleObstacle) bool {
	if ad.skippedDeparture == nil || !ad.skippedDeparture.served || ad.midLaneRecovery == nil || !ad.midLaneSweepClear || math.Abs(ob.Speed) >= 3 {
		return false
	}
	request := ad.midLaneRecovery
	if math.Cos(ob.Heading-request.Heading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(request.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	delta := ob.Pos.Sub(f.bus.Pos)
	if delta.Dot(u) <= 0 || delta.Dot(u) > 220 || request.Target.Sub(ad.skippedDeparture.position).Dot(r) >= -3 {
		return false
	}
	// Only relax the straight following margin. Live actuator/body sweeps,
	// signals and target-lane reservations remain authoritative every tick.
	return math.Abs(ob.Pos.Sub(request.Target).Dot(r)) > (f.bus.Width+ob.Width)/2+2
}
