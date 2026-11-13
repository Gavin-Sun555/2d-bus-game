package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A left-turn lane choice happens after leaving the previous stop. Reusing
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
	if math.Abs(shift) > 4 && length < math.Max(200, 2*math.Abs(shift)) {
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
	if math.Abs(shift) < 4 && ad.drivePath[entry].Sub(f.bus.Pos).Dot(in) > 0 {
		ad.drivePath = append([]sim.Vec2{f.bus.Pos}, ad.drivePath[entry:]...)
		ad.cumLen = make([]float64, len(ad.drivePath))
		for i := 1; i < len(ad.drivePath); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
		}
		ad.WaypointIdx = 0
		ad.pathSynced = false
		ad.holdLaneOffset(0)
		return true
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

// RecoverStrandedMedianApproach handles a fleet bus stuck on the opposing
// side of the road during a pre-junction median merge. After a sustained
// stop, reseat it at the same road position only if a legal lane is clear.
func (ad *AutopilotDriver) RecoverStrandedMedianApproach(bus *sim.Bus, w *sim.World, obstacles []VehicleObstacle, lw, dt float64, signals ...*sim.TrafficLightSystem) bool {
	if ad == nil || bus == nil || w == nil {
		return false
	}
	if handled, recovered := ad.recoverStraddledLeftApproach(bus, w, obstacles, lw, dt, signals...); handled {
		return recovered
	}
	if !ad.ordinaryBusLeft || ad.busLaneJunction == nil || ad.busJunctionEntered ||
		ad.State != AutopilotStateCruising || math.Abs(bus.Speed) > 35 || bus.DoorsOpen ||
		bus.DoorProgress > .05 || bus.PassengerService.Active || len(ad.drivePath) < 2 {
		ad.strandedMedianWait = 0
		return false
	}
	center, ew, _, _ := sim.CurrentRoadInfo(w, bus.Pos, bus.Heading, lw)
	road, ok := w.RoadAt(center, ew, bus.Pos)
	if !ok || !road.HasBusLane() || road.HasMedianBusLane() {
		ad.strandedMedianWait = 0
		return false
	}
	junction := *ad.busLaneJunction
	forward := sim.Vec2{X: math.Copysign(1, junction.X-bus.Pos.X)}
	across := junction.Y
	if !ew {
		forward = sim.Vec2{Y: math.Copysign(1, junction.Y-bus.Pos.Y)}
		across = junction.X
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	side := bus.Pos.Sub(junction).Dot(right)
	if math.Abs(across-center) > 1 || math.Abs(side) > road.GetWidth(lw)/2+sim.ShoulderWidth ||
		side >= -lw/2 || junction.Sub(bus.Pos).Dot(forward) < 900 ||
		ad.pointAt(ad.LastPathS+100).Sub(ad.pointAt(ad.LastPathS)).Normalize().Dot(forward) < .9 ||
		w.InsideJunction(bus.Pos, lw, bus.OverallLength()/2) {
		ad.strandedMedianWait = 0
		return false
	}
	// Small curb rebounds can leave the bus reporting a crawl speed even
	// when it makes no progress along the road.
	if ad.strandedMedianWait == 0 || ad.strandedMedianStart.Distance(bus.Pos) > 12 {
		ad.strandedMedianStart = bus.Pos
		ad.strandedMedianWait = 0
	}
	ad.strandedMedianWait += dt
	if ad.strandedMedianWait < 10 {
		return false
	}
	ad.strandedMedianWait = 9 // Retry a blocked landing once per second.
	for lane := 0; lane < road.LaneCount()/2; lane++ {
		pose := *bus
		pose.Pos = bus.Pos.Add(right.Mul((float64(lane)+.5)*lw - side))
		pose.Heading = math.Atan2(forward.Y, forward.X)
		pose.ResetArticulation()
		if pose.CheckBoundaries(w, lw).Zone != sim.ZoneRoad {
			continue
		}
		clear := true
		bodies := pose.BodyParts()
		for _, rail := range w.Railways() {
			if math.Abs(pose.Pos.Y-rail.Y) < rail.HalfWidth+pose.OverallLength()+sim.RailGateMargin {
				clear = false
				break
			}
		}
		if !clear {
			continue
		}
		for _, ob := range obstacles {
			movingNear := math.Abs(ob.Speed) > 3 && ob.Pos.Distance(pose.Pos) < pose.OverallLength()+ob.Length+math.Abs(ob.Speed)*.5
			if movingNear || sweepBodiesCollision(bodies, ob, 0, 12, 3) {
				clear = false
				break
			}
		}
		if !clear {
			continue
		}
		if ad.BeforeRecovery != nil {
			ad.BeforeRecovery()
		}
		bus.Pos, bus.Heading = pose.Pos, pose.Heading
		bus.ResetArticulation()
		bus.Speed, bus.SteeringAngle = 0, 0
		bus.JunctionCorridor, bus.LaneMergeRequest, bus.StationMergeRequest, bus.YieldingToBus = nil, nil, nil, nil
		ad.headonRecovery, ad.approachMerge, ad.midLaneRecovery = nil, nil, nil
		ad.headonDeadlockTimer, ad.busTurnHoldDistance, ad.strandedMedianWait = 0, 0, 0
		ad.pathSynced = false
		s, _ := ad.trackPath(bus.Pos)
		ad.holdLaneOffset(bus.Pos.Sub(ad.pointAt(s)).Dot(right))
		return true
	}
	return false
}
