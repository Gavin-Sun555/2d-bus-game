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
	if f.world == nil || !f.bus.IsArticulated() || f.speedAbs > 35 || (f.speedAbs > 25 && !curbApproach) || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.roadSafetyHold || ad.State == AutopilotStateDocked || f.bus.RoundaboutAdmissionHold != nil || mergeYieldHold(f) || ad.trafficLightSpeed(f, 30) <= .5 || (sidecarBodyInJunction(f) && !ad.sidecarAtIncomingThroatForDriver(f)) || ad.junctionAdmissionSpeed(f) <= .5 {
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
	turningThroat := ad.turningAtJunctionThroat(f, u)
	if turningThroat && normalSpeed >= 16 {
		ad.sidecarEscapeActive = false
		return 0, false
	}
	away := 0.
	trailerBlocked := false
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	maxSpeed := math.Max(f.speedAbs, 18)
	if curbRetreat {
		maxSpeed = math.Max(f.speedAbs, 32)
	}
	maxTravel := maxSpeed*2 + 6
	minAlong := -f.bus.OverallLength() - f.bus.Width - 4
	maxAlong := f.bus.Length + f.bus.Width + maxTravel
	maxLat := f.bus.Pos.Sub(origin).Dot(r) + f.bus.OverallLength() + f.bus.Width + maxTravel
	var near [128]VehicleObstacle
	n := 0
	for _, ob := range f.obstacles {
		d := ob.Pos.Sub(f.bus.Pos)
		reach := f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*2 + 100
		if d.Dot(d) > reach*reach {
			continue
		}
		offset, trailer := stoppedSidecarOffset(f, ob, u, r)
		if offset != 0 {
			away = -math.Copysign(1, offset)
			if trailer || sidecarBesideTrailer(f, ob) {
				trailerBlocked = true
			}
		}
		{
			if ob.Speed >= 0 {
				obRadius := 0.5*(ob.Length+ob.Width) + 4
				if ob.Bus != nil {
					obRadius = ob.Bus.OverallLength() + ob.Width + 4
				}
				lat0 := ob.Pos.Sub(origin).Dot(r)
				along0 := d.Dot(u)
				lat1, along1 := lat0, along0
				if ob.Speed > 0 {
					obU := sim.UnitFromAngle(ob.Heading)
					travel := ob.Speed * 2
					lat1 += obU.Dot(r) * travel
					along1 += obU.Dot(u) * travel
				}
				if math.Max(lat0, lat1)+obRadius < .5 || math.Min(lat0, lat1)-obRadius > maxLat || math.Max(along0, along1)+obRadius < minAlong || math.Min(along0, along1)-obRadius > maxAlong {
					continue
				}
			}
			if n < len(near) {
				near[n] = ob
				if offset != 0 && n > 0 {
					near[0], near[n] = near[n], near[0]
				}
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
	if n > 0 {
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
		for si, speed := range [...]float64{math.Min(32, limit), math.Min(18, limit)} {
			if si > 0 && speed == math.Min(32, limit) {
				continue
			}
			var curbTested [4]float64
			curbCount := 0
			for _, steer := range [...]float64{settle, settle * .5, 0, desired} {
				dup := false
				for _, prev := range curbTested[:curbCount] {
					if prev == steer {
						dup = true
						break
					}
				}
				if dup {
					continue
				}
				curbTested[curbCount] = steer
				curbCount++
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
		ad.sidecarEscapeActive = false
		return 0, false
	}
	if !ad.sidecarInitialPoseClear(f, heading, center, ew, &traffic) {
		ad.sidecarEscapeActive = false
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
	if turningThroat && math.Abs(desired) >= .15 {
		for _, scale := range [...]float64{1, .75, .5} {
			cand := sim.Clamp(desired*scale, -1, 1)
			if ad.sidecarSweepClear(f, cand, heading, center, ew, &traffic) {
				ad.sidecarEscapeRetry = 0
				ad.sidecarEscapeActive = true
				return cand, true
			}
		}
	}
	var tested [10]float64
	testedCount := 0
	saturatedGain := math.Inf(1)
	for gi, gain := range [...]float64{3, 5, 8, 1} {
		if gain > saturatedGain {
			continue
		}
		steer := sim.Clamp(-yaw*gain, -1, 1)
		if yaw == 0 && f.bus.SteeringAngle == 0 && gi > 0 {
			continue
		}
		clear, saturated := ad.sidecarSweepClearGain(f, steer, heading, center, ew, &traffic, gain)
		if clear {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
		if saturated {
			if gain < saturatedGain {
				saturatedGain = gain
			}
			dup := false
			for _, prev := range tested[:testedCount] {
				if prev == steer {
					dup = true
					break
				}
			}
			if !dup && testedCount < len(tested) {
				tested[testedCount] = steer
				testedCount++
			}
		}
		if yaw == 0 && f.bus.SteeringAngle == 0 && testedCount == 0 {
			tested[0] = 0
			testedCount = 1
		}
	}
	choices := []float64{away * .3, away * .1, 0, desired, away * .2, away * .5, away * .75}
	if trailerBlocked || (f.targetStop != nil && ad.stationDock != nil && ad.stationDock.stopID == f.targetStop.ID && ad.stationDock.rank == 0) {
		// The nose and the trailer can swing in opposite directions.
		// Check both gentle and full opposite directions before giving up.
		choices = append(choices, -away*.15, -away*.3, -away*.5, -away)
	}
	for _, input := range choices {
		if turningThroat && math.Abs(desired) >= .15 && input*desired < 0 {
			continue
		}
		steer := sim.Clamp(input, -1, 1)
		dup := false
		for _, prev := range tested[:testedCount] {
			if prev == steer {
				dup = true
				break
			}
		}
		if dup {
			continue
		}
		if testedCount < len(tested) {
			tested[testedCount] = steer
			testedCount++
		}
		if ad.sidecarSweepClear(f, steer, heading, center, ew, &traffic) {
			ad.sidecarEscapeRetry = 0
			ad.sidecarEscapeActive = true
			return steer, true
		}
	}
	var preTested [10]float64
	preTestedCount := 0
	preSaturatedGain := math.Inf(1)
	for _, gain := range [...]float64{3, 5, 8, 1} {
		if gain > preSaturatedGain {
			continue
		}
		steer := sim.Clamp(-yaw*gain, -1, 1)
		if steer*f.bus.MaxSteerAngle == f.bus.SteeringAngle {
			continue
		}
		parked := *f.bus
		parked.SteeringAngle = steer * parked.MaxSteerAngle
		aligned := f
		aligned.bus = &parked
		clear, saturated := ad.sidecarSweepClearGain(aligned, steer, heading, center, ew, &traffic, gain)
		if clear {
			ad.sidecarPresteer, ad.sidecarPresteerValid = steer, true
			ad.sidecarEscapeActive = true
			return steer, false
		}
		if saturated {
			if gain < preSaturatedGain {
				preSaturatedGain = gain
			}
			dup := false
			for _, prev := range preTested[:preTestedCount] {
				if prev == steer {
					dup = true
					break
				}
			}
			if !dup && preTestedCount < len(preTested) {
				preTested[preTestedCount] = steer
				preTestedCount++
			}
		}
	}

	// Steering at rest changes neither vehicle position nor trailer angle.
	// Prove the subsequent moving trajectory with the settled wheel first.
	for _, input := range choices {
		steer := sim.Clamp(input, -1, 1)
		dup := false
		if steer*f.bus.MaxSteerAngle == f.bus.SteeringAngle {
			for _, prev := range tested[:testedCount] {
				if prev == steer {
					dup = true
					break
				}
			}
		}
		for i := 0; !dup && i < preTestedCount; i++ {
			if preTested[i] == steer {
				dup = true
			}
		}
		if dup {
			continue
		}
		if preTestedCount < len(preTested) {
			preTested[preTestedCount] = steer
			preTestedCount++
		}
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
	ad.sidecarEscapeActive = false
	return 0, false
}

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

func (ad *AutopilotDriver) turningAtJunctionThroat(f autopilotFrame, u sim.Vec2) bool {
	if ad == nil || f.world == nil || f.bus == nil || len(ad.drivePath) < 2 {
		return false
	}
	if !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()) {
		return false
	}
	if ad.rightTurnJunction != nil {
		return true
	}
	ahead := ad.pointAt(f.s + 120).Sub(ad.pointAt(f.s))
	return ahead.Length() > 20 && ahead.Normalize().Dot(u) < .92
}

// A long bus can straddle the mouth while straightening past a sidecar.
// Permit the fully checked crawl only within one vehicle length of the entry,
// aligned with the incoming road. Mid-turn and departing buses retain the box
// guard; red signals and junction admission remain authoritative.
func sidecarAtIncomingThroat(f autopilotFrame) bool {
	return sidecarAtIncomingThroatWithin(f, f.bus.OverallLength())
}

func (ad *AutopilotDriver) sidecarAtIncomingThroatForDriver(f autopilotFrame) bool {
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	maxIntoBox := f.bus.OverallLength()
	if ad.turningAtJunctionThroat(f, u) {
		maxIntoBox = 36
	}
	return sidecarAtIncomingThroatWithin(f, maxIntoBox)
}

func sidecarAtIncomingThroatWithin(f autopilotFrame, maxIntoBox float64) bool {
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
		if along < -half+maxIntoBox && along > -half-f.bus.Length/2 && lateral < math.Max(g.HalfX, g.HalfY) {
			return true
		}
	}
	return false
}

func stoppedSidecarOffset(f autopilotFrame, ob VehicleObstacle, u, r sim.Vec2) (offset float64, trailer bool) {
	if ob.Bus != nil || ob.Length > 65 || math.Abs(ob.Speed) >= 1 || sim.UnitFromAngle(ob.Heading).Dot(u) < .9 {
		return 0, false
	}
	limit := (f.bus.Width+ob.Width)/2 + f.laneWidth*.5
	d := ob.Pos.Sub(f.bus.Pos)
	lateral := d.Dot(r)
	if math.Abs(d.Dot(u)) < f.bus.OverallLength()+ob.Length && math.Abs(lateral) > f.bus.Width/2 && math.Abs(lateral) < limit {
		return lateral, false
	}
	if trailerOffset := stoppedSidecarTrailerOffset(f, ob); trailerOffset != 0 {
		return trailerOffset, true
	}
	return 0, false
}

func sidecarBesideTrailer(f autopilotFrame, ob VehicleObstacle) bool {
	return f.bus.IsArticulated() && stoppedSidecarTrailerOffset(f, ob) != 0
}

func stoppedSidecarTrailerOffset(f autopilotFrame, ob VehicleObstacle) float64 {
	if !f.bus.IsArticulated() || ob.Bus != nil || ob.Length > 65 || math.Abs(ob.Speed) >= 1 {
		return 0
	}
	limit := (f.bus.Width+ob.Width)/2 + f.laneWidth*.5
	tail := f.bus.TrailerBody()
	tailForward := sim.UnitFromAngle(tail.Heading)
	tailRight := sim.Vec2{X: -tailForward.Y, Y: tailForward.X}
	td := ob.Pos.Sub(tail.Center)
	tlat := td.Dot(tailRight)
	if math.Abs(td.Dot(tailForward)) < tail.HalfLen+ob.Length/2 && math.Abs(tlat) > tail.HalfWid && math.Abs(tlat) < limit {
		return tlat
	}
	return 0
}

const sidecarSweepSamples = 121
const maxCachedSidecarObstacles = 24

type sidecarSweepTraffic struct {
	poses        [][2]sim.OBB
	prepared     int
	initialClear bool
	initialParts [2]sim.OBB
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
			if tick > 0 && t.prepared > 0 {
				if ob.Speed <= 0 {
					poses[i] = t.poses[i]
					continue
				}
				if ob.Bus == nil && ob.JunctionTurn == nil && ob.JunctionCorridor == nil {
					base := t.poses[i]
					base[0].Center = ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(ob.Speed * float64(tick) / 60))
					poses[i] = base
					continue
				}
			}
			poses[i] = sidecarObstaclePose(f, ob, heading, tick)
		}
		t.prepared = tick + 1
	}
	return poses
}

func (ad *AutopilotDriver) sidecarInitialPoseClear(f autopilotFrame, heading, center float64, ew bool, traffic *sidecarSweepTraffic) bool {
	for _, ob := range f.obstacles {
		if ob.Speed < 0 {
			return false
		}
	}
	b := f.bus
	if math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
		return false
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	parts := b.BodyParts()
	for _, part := range parts {
		if part.HalfLen > 0 {
			yaw := part.Heading - heading
			offset := part.Center.Sub(origin).Dot(r)
			extent := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
			if offset-extent < .5 {
				return false
			}
		}
	}
	predicted := traffic.at(f, heading, 0)
	for i, ob := range f.obstacles {
		var bodies [2]sim.OBB
		if predicted != nil {
			bodies = predicted[i]
		} else {
			bodies = sidecarObstaclePose(f, ob, heading, 0)
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
	if traffic != nil {
		traffic.initialClear = true
		traffic.initialParts = parts
	}
	return true
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

func (ad *AutopilotDriver) sidecarSweepClearGain(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, gain float64) (clear, saturated bool) {
	return ad.sidecarSweepClearStoppingWithSaturation(f, steer, heading, center, ew, traffic, sidecarSweepSamples, 18, sidecarSweepSamples, gain)
}

func (ad *AutopilotDriver) sidecarSweepClearSamples(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, samples int, yawGain ...float64) bool {
	return ad.sidecarSweepClearStopping(f, steer, heading, center, ew, traffic, samples, 18, samples, yawGain...)
}

func (ad *AutopilotDriver) sidecarSweepClearStopping(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, samples int, speed float64, brakeAt int, yawGain ...float64) bool {
	clear, _ := ad.sidecarSweepClearStoppingWithSaturation(f, steer, heading, center, ew, traffic, samples, speed, brakeAt, yawGain...)
	return clear
}

func (ad *AutopilotDriver) sidecarSweepClearStoppingWithSaturation(f autopilotFrame, steer, heading, center float64, ew bool, traffic *sidecarSweepTraffic, samples int, speed float64, brakeAt int, yawGain ...float64) (bool, bool) {
	brakingRun := brakeAt < samples
	skipInitial := !brakingRun && traffic != nil && traffic.initialClear
	// This narrow recovery predicts forward traffic only. Defer to normal
	// avoidance while a nearby vehicle is reversing.
	if !skipInitial {
		for _, ob := range f.obstacles {
			if ob.Speed < 0 {
				return false, false
			}
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
	maxYaw := math.Max(.16, math.Abs(sim.NormalizeAngle(f.bus.Heading-heading))+.005)
	steer0 := steer
	saturated := math.Abs(steer0) == 1
	var poses [sidecarSweepSamples][2]sim.OBB
	for tick := range poses[:samples] {
		if tick == 0 && skipInitial {
			poses[0] = traffic.initialParts
			continue
		}
		if tick > 0 {
			if len(yawGain) > 0 {
				steer = sim.Clamp(-sim.NormalizeAngle(b.Heading-heading)*yawGain[0], -1, 1)
				if steer != steer0 {
					saturated = false
				}
			}
			th, br := ad.longitudinalControls(b.Speed, speed)
			if tick >= brakeAt {
				th, br = 0, 1
			}
			b.SetNextAccelerationScale(1)
			b.Update(1./60, th, br, steer, false, false)
		}
		if brakingRun && math.Abs(sim.NormalizeAngle(b.Heading-heading)) > maxYaw {
			return false, saturated
		}
		if math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
			return false, saturated
		}
		parts := b.BodyParts()
		poses[tick] = parts
		for _, part := range parts {
			if part.HalfLen > 0 {
				yaw := part.Heading - heading
				offset := part.Center.Sub(origin).Dot(r)
				sinY, cosY := math.Sincos(yaw)
				extent := part.HalfLen*math.Abs(sinY) + part.HalfWid*math.Abs(cosY)
				if offset-extent < .5 || (brakingRun && offset+extent > 2*f.laneWidth) {
					return false, saturated
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
					r := part.HalfLen + part.HalfWid + other.HalfLen + other.HalfWid
					if math.Abs(part.Center.X-other.Center.X) > r || math.Abs(part.Center.Y-other.Center.Y) > r {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(part, other); hit && depth > 0 {
						return false, saturated
					}
				}
			}
		}
	}
	minProgress := 10.
	if brakingRun {
		if math.Abs(b.Speed) > .01 {
			return false, saturated
		}
		minProgress = .5
	}
	if b.Pos.Sub(f.bus.Pos).Dot(u) <= minProgress {
		return false, saturated
	}
	// Most rejected escape paths run into the adjacent traffic. Check road
	// geometry only for paths that clear traffic, using the same sampled poses.
	for tick, parts := range poses[:samples] {
		if tick != 0 && tick != samples-1 && tick%4 != 0 {
			continue
		}
		if f.world.CheckBoundaries(parts[0].Center, parts[0].HalfWid, f.laneWidth).Zone == sim.ZoneHardBarrier {
			return false, saturated
		}
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if f.world.CheckBoundaries(corner, .5, f.laneWidth).Zone == sim.ZoneHardBarrier {
					return false, saturated
				}
			}
		}
	}
	return true, saturated
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
