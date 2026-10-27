package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A queue-based lane choice happens after leaving the previous stop. Reusing
// that stop's full-leg taper can put its beginning behind the bus, making
// pursuit aim sharply sideways. Join the median from the current physical pose.
func (ad *AutopilotDriver) joinMedianLeftApproach(f autopilotFrame, heading float64, center sim.Vec2) bool {
	in := sim.UnitFromAngle(heading)
	right := sim.Vec2{X: -in.Y, Y: in.X}
	g := f.world.JunctionAt(center, f.laneWidth)
	half := math.Abs(in.X)*g.HalfX + math.Abs(in.Y)*g.HalfY
	finish := center.Sub(in.Mul(half + 80)).Add(right.Mul(.5 * f.laneWidth))
	delta := finish.Sub(f.bus.Pos)
	length, shift := delta.Dot(in), delta.Dot(right)
	if length < math.Max(200, 2*math.Abs(shift)) {
		return false
	}
	// Leave the curb queue decisively before catching its tail; spreading the
	// same lateral movement all the way to the line can trap the bus behind it.
	length = math.Min(length, math.Max(240, 2*math.Abs(shift)))
	entry := -1
	for i, p := range ad.drivePath {
		d := p.Sub(center)
		if d.Dot(in) >= -half-1 && math.Abs(d.Dot(right)-.5*f.laneWidth) < 1 {
			entry = i
			break
		}
	}
	if entry < 0 {
		return false
	}
	points := make([]sim.Vec2, 0, 25+len(ad.drivePath)-entry)
	for i := 0; i <= 24; i++ {
		u := float64(i) / 24
		ease := u * u * u * (10 - 15*u + 6*u*u)
		points = append(points, f.bus.Pos.Add(in.Mul(length*u)).Add(right.Mul(shift*ease)))
	}
	ad.drivePath = append(points, ad.drivePath[entry:]...)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	ad.WaypointIdx = 0
	ad.pathSynced = false
	ad.holdLaneOffset(0)
	return true
}

// A saved/late median choice may be frozen across the curb marking with no
// runway left to cross the occupied through lanes. Resume the nearby legal
// curb-left corridor instead of retaining an impossible turn-hold forever.
func (ad *AutopilotDriver) recoverBlockedMedianChoice(f autopilotFrame, heading, roadCenter float64, ew bool, dt float64) bool {
	if f.speedAbs > 5 || ad.busJunctionEntered || math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .25 {
		return false
	}
	if ad.busLaneJunction == nil || f.tls == nil {
		return false
	}
	forward := sim.UnitFromAngle(heading)
	nearLine := false
	for _, it := range f.tls.Intersections {
		if it.Center.Distance(*ad.busLaneJunction) < 1 {
			gap := it.Center.Sub(f.bus.Pos).Dot(forward) - it.GetStopLineDistForHeading(heading, f.laneWidth) - f.bus.Length/2
			nearLine = gap >= 0 && gap <= 100
			break
		}
	}
	if !nearLine {
		return false
	}
	right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: roadCenter}
	if !ew {
		origin = sim.Vec2{X: roadCenter}
	}
	lateral := f.bus.Pos.Sub(origin).Dot(right)
	if lateral <= 3*f.laneWidth || lateral > 3.5*f.laneWidth {
		return false
	}
	var from, to *sim.BusStop
	for _, stop := range f.world.Stops {
		if stop.ID == ad.pathFromID {
			from = stop
		}
		if stop.ID == ad.pathToID {
			to = stop
		}
	}
	if from == nil || to == nil {
		return false
	}
	ad.ordinaryBusLeft = false
	ad.PrepareLeg(f.world, from, to, f.laneWidth)
	f.s, _ = ad.trackPath(f.bus.Pos)
	ad.holdLaneOffset(f.bus.Pos.Sub(ad.pointAt(f.s)).Dot(right))
	ad.busTurnHoldDistance = 0
	ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: origin.Add(right.Mul(3.5 * f.laneWidth)), Heading: heading, Remaining: .3}
	ad.advanceMidLaneRecovery(f, dt)
	return true
}
