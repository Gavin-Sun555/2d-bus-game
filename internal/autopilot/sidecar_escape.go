package autopilot

import (
	"context"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"runtime/trace"
)

// A stationary parallel car may touch only the padded sweep of a yawed bus.
// Search a slow, real actuator trajectory; never waive physical body checks or
// use the opposing half of the road to escape that artificial stop.
func (ad *AutopilotDriver) sidecarEscape(f autopilotFrame, desired, normalSpeed float64) (float64, bool) {
	if trace.IsEnabled() {
		defer trace.StartRegion(context.Background(), "autopilot.sidecarEscape").End()
	}
	ad.sidecarEscapeTarget = 18
	curbApproach := ad.busLaneJunction != nil && !ad.busJunctionEntered
	if f.world == nil || !f.bus.IsArticulated() || f.speedAbs > 35 || (f.speedAbs > 25 && !curbApproach) || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.roadSafetyHold || ad.State == AutopilotStateDocked || f.bus.RoundaboutAdmissionHold != nil || ad.trafficLightSpeed(f, 30) <= .5 || (sidecarBodyInJunction(f) && !sidecarAtIncomingThroat(f)) || ad.junctionAdmissionSpeed(f) <= .5 || mergeYieldHold(f) {
		ad.sidecarPresteerValid = false
		ad.sidecarEscapeActive = false
		return 0, false
	}
	if ad.sidecarEscapeRetry > 0 {
		if ad.sidecarPresteerValid {
			return ad.sidecarPresteer, false
		}
		return 0, false
	}
	ad.sidecarPresteerValid = false
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			ad.sidecarEscapeActive, ad.sidecarPresteerValid = false, false
			return 0, false
		}
	}
	heading := approachRoadHeading(f.bus.Heading)
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	if (!multi && !six) || math.Cos(f.bus.Heading-heading) < .9 {
		ad.sidecarEscapeActive, ad.sidecarPresteerValid = false, false
		return 0, false
	}
	curbRetreat := false
	if curbApproach {
		road, ok := f.world.RoadAt(center, ew, f.bus.Pos)
		curbRetreat = ok && road.LaneCount() == 4
	}
	if f.speedAbs > 25 && !curbRetreat {
		ad.sidecarEscapeActive = false
		return 0, false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	// A latched approach crawl must hand steering back once the trailer
	// has cleared the adjacent queue and the normal checks allow progress.
	if c := f.bus.JunctionCorridor; curbRetreat && c != nil && c.Entered && normalSpeed > 32 {
		lane := sim.BusLaneMergeRequest{Target: c.Center.Add(r.Mul(1.5 * f.laneWidth)), Heading: heading}
		if wholeBusInMergeLane(f.bus, &lane, f.laneWidth) {
			ad.sidecarEscapeActive = false
			return 0, false
		}
	}
	away := 0.
	trailerBlocked := false
	var near [128]VehicleObstacle
	n := 0
	for _, ob := range f.obstacles {
		if offset, trailer := stoppedSidecarOffset(f, ob, u, r); offset != 0 {
			away = -math.Copysign(1, offset)
			trailerBlocked = trailer
		}
		d := ob.Pos.Sub(f.bus.Pos)
		reach := f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*2 + 100
		if d.Dot(d) <= reach*reach {
			if n < len(near) {
				near[n] = ob
				n++
			} else {
				n = -1
				break
			}
		}
	}
	if away == 0 && !curbRetreat {
		ad.sidecarEscapeActive = false
		return 0, false
	}
	if n >= 0 {
		f.obstacles = near[:n]
	}
	var traffic sidecarSweepTraffic
	if n > 0 && n <= maxCachedSidecarObstacles {
		scratch := ad.geometryScratch()
		size := n * sidecarSweepSamples
		if cap(scratch.sidecarTraffic) < size {
			scratch.sidecarTraffic = make([][2]sim.OBB, size)
		}
		traffic.poses = scratch.sidecarTraffic[:size]
	}
	ad.sidecarEscapeRetry = .2
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if curbRetreat {
		// Prove a short adjustment and its braking run, then replan
		// from the new physical pose on the next tick.
		lateral := f.bus.Pos.Sub(*ad.busLaneJunction).Dot(r)
		targetYaw := sim.Clamp((1.5*f.laneWidth-lateral)/90, -.14, .14)
		settle := sim.Clamp(4*(targetYaw-yaw), -1, 1)
		limit := math.Min(ad.trafficLightSpeed(f, 32), ad.junctionAdmissionSpeed(f))
		for _, speed := range []float64{math.Min(32, limit), math.Min(18, limit)} {
			for _, steer := range []float64{settle, settle * .5, 0, desired} {
				if ad.sidecarSweepClearStopping(f, steer, heading, center, ew, &traffic, 42, speed, 16) {
					ad.sidecarEscapeRetry = 0
					ad.sidecarEscapeActive = true
					ad.sidecarEscapeTarget = speed
					return steer, true
				}
			}
		}
		if away == 0 {
			ad.sidecarEscapeActive = false
			return 0, false
		}
	}
	if ad.midLaneRecovery != nil {
		// A lane retreat retains its steering target. Straightening along a
		// frozen divider would prevent it from ever settling into that lane.
		if ad.sidecarSweepClearSamples(f, desired, heading, center, ew, &traffic, midLaneRetreatSweepSamples) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return desired, true
		}
		return 0, false
	}
	// When the passing path is already steering away from the stopped car,
	// finish that lateral correction rather than crawling along its edge.
	if ad.OvertakeState == OvertakePassing && ad.TargetLaneOffset < 0 && desired < 0 && away < 0 && ad.sidecarSweepClear(f, desired, heading, center, ew, &traffic) {
		ad.sidecarEscapeRetry = 0
		// Resume normal steering and its braking caps when the verified
		// correction has a clear path. Nearby traffic alone is not a hold.
		if normalSpeed > 35 {
			ad.sidecarEscapeActive = false
			return 0, false
		}
		ad.sidecarEscapeActive = true
		return desired, true
	}
	for _, gain := range []float64{3, 5, 8, 1} {
		steer := sim.Clamp(-yaw*gain, -1, 1)
		if ad.sidecarSweepClear(f, steer, heading, center, ew, &traffic, gain) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
	}
	choices := []float64{away * .3, away * .1, 0, desired, away * .2, away * .5, away * .75}
	if trailerBlocked || (f.targetStop != nil && ad.stationDock != nil && ad.stationDock.stopID == f.targetStop.ID && ad.stationDock.rank == 0) {
		// The nose and the trailer can swing in opposite directions.
		// Check both directions before committing to the crawl.
		choices = append(choices, -away*.5, -away)
	}
	for _, input := range choices {
		steer := sim.Clamp(input, -1, 1)
		if ad.sidecarSweepClear(f, steer, heading, center, ew, &traffic) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
	}
	for _, gain := range []float64{3, 5, 8, 1} {
		steer := sim.Clamp(-yaw*gain, -1, 1)
		parked := *f.bus
		parked.SteeringAngle = steer * parked.MaxSteerAngle
		aligned := f
		aligned.bus = &parked
		if ad.sidecarSweepClear(aligned, steer, heading, center, ew, &traffic, gain) {
			ad.sidecarPresteer, ad.sidecarPresteerValid = steer, true
			ad.sidecarEscapeActive = true
			return steer, false
		}
	}

	// Steering at rest changes neither vehicle position nor trailer angle.
	// Prove the subsequent moving trajectory with the settled wheel first.
	for _, input := range choices {
		steer := sim.Clamp(input, -1, 1)
		parked := *f.bus
		parked.SteeringAngle = steer * parked.MaxSteerAngle
		aligned := f
		aligned.bus = &parked
		if ad.sidecarSweepClear(aligned, steer, heading, center, ew, &traffic) {
			ad.sidecarPresteer, ad.sidecarPresteerValid = steer, true
			ad.sidecarEscapeActive = true
			return steer, false
		}
	}
	return 0, false
}

// The articulated rear extends behind the front reference point. Expanding
// every junction by half the total length also excludes buses still fully
// before the mouth, disabling their checked retreat from an adjacent car.
func sidecarBodyInJunction(f autopilotFrame) bool {
	if !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()) {
		return false
	}
	parts := f.bus.BodyParts()
	for _, center := range f.world.Intersections {
		g := f.world.JunctionAt(center, f.laneWidth)
		delta := f.bus.Pos.Sub(center)
		if math.Abs(delta.X) > g.HalfX+f.bus.OverallLength() || math.Abs(delta.Y) > g.HalfY+f.bus.OverallLength() {
			continue
		}
		box := sim.OBB{Center: center, HalfLen: g.HalfX, HalfWid: g.HalfY}
		for _, part := range parts {
			if part.HalfLen > 0 {
				if hit, _, _ := sim.CheckOBBCollision(part, box); hit {
					return true
				}
			}
		}
	}
	return false
}

// A long bus can straddle the mouth while straightening past a sidecar.
// Permit the fully checked crawl only within one vehicle length of the entry,
// aligned with the incoming road. Mid-turn and departing buses retain the box
// guard; red signals and junction admission remain authoritative.
func sidecarAtIncomingThroat(f autopilotFrame) bool {
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	if math.Cos(f.bus.Heading-heading) < .9 {
		return false
	}
	for _, center := range f.world.Intersections {
		g := f.world.JunctionAt(center, f.laneWidth)
		delta := f.bus.Pos.Sub(center)
		half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
		along := delta.Dot(u)
		lateral := math.Abs(delta.X*u.Y - delta.Y*u.X)
		if along < -half+f.bus.OverallLength() && along > -half-f.bus.Length/2 && lateral < math.Max(g.HalfX, g.HalfY) {
			return true
		}
	}
	return false
}

func stoppedSidecarOffset(f autopilotFrame, ob VehicleObstacle, u, r sim.Vec2) (offset float64, trailer bool) {
	if ob.Bus != nil || ob.Length > 65 || math.Abs(ob.Speed) >= 1 || sim.UnitFromAngle(ob.Heading).Dot(u) < .9 {
		return 0, false
	}
	d := ob.Pos.Sub(f.bus.Pos)
	limit := (f.bus.Width+ob.Width)/2 + f.laneWidth*.5
	lateral := d.Dot(r)
	if math.Abs(d.Dot(u)) < f.bus.OverallLength()+ob.Length && math.Abs(lateral) > f.bus.Width/2 && math.Abs(lateral) < limit {
		return lateral, false
	}
	if f.bus.IsArticulated() {
		tail := f.bus.TrailerBody()
		tailForward := sim.UnitFromAngle(tail.Heading)
		tailRight := sim.Vec2{X: -tailForward.Y, Y: tailForward.X}
		d = ob.Pos.Sub(tail.Center)
		lateral = d.Dot(tailRight)
		if math.Abs(d.Dot(tailForward)) < tail.HalfLen+ob.Length/2 && math.Abs(lateral) > tail.HalfWid && math.Abs(lateral) < limit {
			return lateral, true
		}
	}
	return 0, false
}

const sidecarSweepSamples = 121
const maxCachedSidecarObstacles = 24

type sidecarSweepTraffic struct {
	poses    [][2]sim.OBB
	prepared int
}

// All steering candidates see the same traffic at the same sample time.
// Populate each sample on demand so early rejections leave later samples unused.
func (t *sidecarSweepTraffic) at(f autopilotFrame, heading float64, tick int) [][2]sim.OBB {
	if t == nil || len(t.poses) == 0 {
		return nil
	}
	count := len(f.obstacles)
	poses := t.poses[tick*count : (tick+1)*count]
	if tick >= t.prepared {
		for i, ob := range f.obstacles {
			poses[i] = sidecarObstaclePose(f, ob, heading, tick)
		}
		t.prepared = tick + 1
	}
	return poses
}

func sidecarObstaclePose(f autopilotFrame, ob VehicleObstacle, heading float64, tick int) [2]sim.OBB {
	padding := 2.
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if offset, trailer := stoppedSidecarOffset(f, ob, u, r); trailer {
		// At the trailer, a stopped car can already touch the safety margin.
		// Require separation of the actual bodies at every step of the crawl.
		padding = 0
	} else if offset != 0 {
		padding = .05
	}
	bodies := predictedObstacleBodies(ob, math.Max(0, ob.Speed)*float64(tick)/60)
	for i := range bodies {
		if bodies[i].HalfLen > 0 {
			bodies[i].HalfLen += padding
			bodies[i].HalfWid += padding
		}
	}
	return bodies
}

func (ad *AutopilotDriver) sidecarSweepClear(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, yawGain ...float64) bool {
	return ad.sidecarSweepClearSamples(f, steer, heading, center, ew, traffic, sidecarSweepSamples, yawGain...)
}

func (ad *AutopilotDriver) sidecarSweepClearSamples(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, samples int, yawGain ...float64) bool {
	return ad.sidecarSweepClearStopping(f, steer, heading, center, ew, traffic, samples, 18, samples, yawGain...)
}

func (ad *AutopilotDriver) sidecarSweepClearStopping(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, samples int, speed float64, brakeAt int, yawGain ...float64) bool {
	// This narrow recovery predicts forward traffic only. Defer to normal
	// avoidance while a nearby vehicle is reversing.
	for _, ob := range f.obstacles {
		if ob.Speed < 0 {
			return false
		}
	}
	b := *f.bus
	// Update expires these live reservations through pointers. The predicted
	// bus needs only its own motion state, so leave the originals untouched.
	b.JunctionCorridor, b.RoundaboutEntry = nil, nil
	b.LaneMergeRequest, b.StationMergeRequest = nil, nil
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	brakingRun := brakeAt < samples
	maxYaw := math.Max(.16, math.Abs(sim.NormalizeAngle(f.bus.Heading-heading))+.005)
	var poses [sidecarSweepSamples][2]sim.OBB
	for tick := range poses[:samples] {
		if tick > 0 {
			if len(yawGain) > 0 {
				steer = sim.Clamp(-sim.NormalizeAngle(b.Heading-heading)*yawGain[0], -1, 1)
			}
			th, br := ad.longitudinalControls(b.Speed, speed)
			if tick >= brakeAt {
				th, br = 0, 1
			}
			b.SetNextAccelerationScale(1)
			b.Update(1./60, th, br, steer, false, false)
		}
		if brakingRun && math.Abs(sim.NormalizeAngle(b.Heading-heading)) > maxYaw {
			return false
		}
		if math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
			return false
		}
		parts := b.BodyParts()
		poses[tick] = parts
		for _, part := range parts {
			if part.HalfLen > 0 {
				yaw := part.Heading - heading
				offset := part.Center.Sub(origin).Dot(r)
				extent := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
				if offset-extent < .5 || (brakingRun && offset+extent > 2*f.laneWidth) {
					return false
				}
			}
		}
		predicted := traffic.at(f, heading, tick)
		for i, ob := range f.obstacles {
			var bodies [2]sim.OBB
			if brakingRun && ob.Bus == nil && math.Abs(ob.Speed) < 3 && math.Cos(ob.Heading-heading) > .98 {
				// Check both actual bus sections against the adjacent car.
				// Its artificial skin can be tangent even when the bodies clear.
				bodies = predictedObstacleBodies(ob, math.Max(0, ob.Speed)*float64(tick)/60)
			} else if predicted != nil {
				bodies = predicted[i]
			} else {
				bodies = sidecarObstaclePose(f, ob, heading, tick)
			}
			for _, other := range bodies {
				if other.HalfLen <= 0 {
					continue
				}
				for _, part := range parts {
					if part.HalfLen <= 0 {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(part, other); hit && depth > 0 {
						return false
					}
				}
			}
		}
	}
	minProgress := 10.
	if brakingRun {
		if math.Abs(b.Speed) > .01 {
			return false
		}
		minProgress = .5
	}
	if b.Pos.Sub(f.bus.Pos).Dot(u) <= minProgress {
		return false
	}
	// Most rejected escape paths run into the adjacent traffic. Check road
	// geometry only for paths that clear traffic, using the same sampled poses.
	for _, parts := range poses[:samples] {
		if f.world.CheckBoundaries(parts[0].Center, parts[0].HalfWid, f.laneWidth).Zone == sim.ZoneHardBarrier {
			return false
		}
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if f.world.CheckBoundaries(corner, .5, f.laneWidth).Zone == sim.ZoneHardBarrier {
					return false
				}
			}
		}
	}
	return true
}

// mergeYieldHold reports a granted rear slot for a bus merging ahead. An
// escape crawl must not spend that slot: creeping beside the merging tail
// leaves both buses waiting on each other.
func mergeYieldHold(f autopilotFrame) bool {
	if f.bus.YieldingToBus == nil {
		return false
	}
	limit, ok := sim.BusMergeYieldLimit(f.bus, f.bus.YieldingToBus, f.laneWidth, true)
	return ok && limit <= .5
}
