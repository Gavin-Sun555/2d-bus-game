package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A bay exit can stop with its nose almost touching the median. At a large
// yaw, even maximum forward steering moves that nose farther across the line.
// A short, checked reverse lets the wheels straighten within the carriageway.
func (ad *AutopilotDriver) roadReverseControls(f autopilotFrame) (throttle, brake, steer float64, active bool) {
	if f.world == nil || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 {
		if ad.roadReverseActive {
			f.bus.ShiftTo(sim.GearDrive)
			ad.roadReverseActive = false
		}
		return 0, 0, 0, false
	}
	heading := approachRoadHeading(f.bus.Heading)
	if len(ad.drivePath) >= 2 && len(ad.cumLen) == len(ad.drivePath) {
		p0 := ad.pointAt(f.s)
		p1 := ad.pointAt(f.s + 4)
		if d := p1.Sub(p0); d.Dot(d) > 1e-6 {
			heading = approachRoadHeading(math.Atan2(d.Y, d.X))
		}
	}
	yaw := sim.NormalizeAngle(f.bus.Heading - heading)
	if !ad.roadReverseActive && (!ad.roadSafetyHold || f.speedAbs > .5 || math.Abs(yaw) <= .4 || math.Abs(yaw) > 1.05) {
		return 0, 0, 0, false
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	clearance := roadMedianClearance(f.bus, f.bus.Pos, f.bus.Heading, heading, center, ew)
	wasActive := ad.roadReverseActive
	if !ad.roadReverseActive {
		if (!multi && !six) || clearance > 2 ||
			(clearance < 0 && f.bus.CheckBoundaries(f.world, f.laneWidth).Zone != sim.ZoneRoad) ||
			f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
			return 0, 0, 0, false
		}
		if busOverlapsLevelCrossing(f.bus, f.world) {
			return 0, 0, 0, false
		}
		ad.roadReverseActive = true
	}
	if clearance >= 8 && math.Abs(yaw) <= .4 {
		if f.speedAbs > .5 {
			return 0, 1, 0, true
		}
		f.bus.ShiftTo(sim.GearDrive)
		ad.roadReverseActive = false
		return 0, 0, 0, false
	}
	recoverySteer := math.Copysign(1, -yaw)
	if wasActive && f.bus.Gear == sim.GearDrive && math.Abs(yaw) <= .4 {
		if _, canRecover := ad.roadLaneRecovery(f, recoverySteer, heading, center, ew); canRecover {
			ad.roadReverseActive = false
			return 0, 0, 0, false
		}
	}
	if wasActive && f.bus.Gear == sim.GearDrive {
		cabLat := roadMedianClearance(f.bus, f.bus.Pos, heading, heading, center, ew) + f.bus.Width/2
		if cabLat > 8 && math.Abs(yaw) > .4 && ad.roadReverseForwardUnwindClear(f, heading, center, ew, clearance, yaw, recoverySteer) {
			if f.speedAbs <= .5 && f.bus.SteeringAngle*recoverySteer < f.bus.MaxSteerAngle*.5 {
				return 0, 1, recoverySteer, true
			}
			if f.bus.Speed > 22 {
				return 0, .2, recoverySteer, true
			}
			if f.bus.Speed < 18 {
				return .25, 0, recoverySteer, true
			}
			return .08, 0, recoverySteer, true
		}
		if f.speedAbs > .5 {
			return 0, 1, recoverySteer, true
		}
	}
	if wasActive && f.bus.Gear == sim.GearDrive && f.speedAbs <= .5 && clearance >= 1.0 {
		if math.Abs(yaw) > .2 && f.bus.SteeringAngle*recoverySteer < f.bus.MaxSteerAngle*.5 {
			return 0, 1, recoverySteer, true
		}
		ad.roadReverseActive = false
		return 0, 0, 0, false
	}
	steer = sim.Clamp(3*yaw, -.65, .65)
	steer, clear := ad.chooseRoadReverseSteer(f, heading, center, ew, steer, clearance)
	if !clear {
		if f.speedAbs <= .5 {
			trial, requests := rearReverseYieldFrameWithHeading(f, heading, f.bus.IsArticulated())
			if !requests.empty() {
				if _, trialClear := ad.chooseRoadReverseSteer(trial, heading, center, ew, steer, clearance); trialClear {
					requests.send()
				}
			}
		}
		if f.speedAbs <= .5 && clearance >= 1.0 {
			if math.Abs(yaw) > .2 && f.bus.SteeringAngle*recoverySteer < f.bus.MaxSteerAngle*.5 {
				f.bus.ShiftTo(sim.GearDrive)
				return 0, 1, recoverySteer, true
			}
			f.bus.ShiftTo(sim.GearDrive)
			ad.roadReverseActive = false
			return 0, 0, 0, false
		}
		if ad.roadReverseForwardUnwindClear(f, heading, center, ew, clearance, yaw, recoverySteer) {
			f.bus.ShiftTo(sim.GearDrive)
			if f.speedAbs <= .5 && f.bus.SteeringAngle*recoverySteer < f.bus.MaxSteerAngle*.5 {
				return 0, 1, recoverySteer, true
			}
			if f.bus.Speed > 22 {
				return 0, .2, recoverySteer, true
			}
			if f.bus.Speed < 18 {
				return .25, 0, recoverySteer, true
			}
			return .08, 0, recoverySteer, true
		}
		return 0, 1, steer, true
	}
	if f.bus.Speed > .5 {
		return 0, 1, steer, true
	}
	f.bus.ShiftTo(sim.GearReverse)
	throttle, brake = roadReversePedals(f.bus.Speed)
	return throttle, brake, steer, true
}

func roadReversePedals(speed float64) (throttle, brake float64) {
	if speed < -12 {
		return 0, .3
	}
	return sim.Clamp((12+speed)/15, .05, .35), 0
}

func roadReverseClear(f autopilotFrame, heading, center float64, ew bool) bool {
	for _, horizon := range []int{72, 48, 30} {
		if roadReverseClearHorizon(f, heading, center, ew, horizon) {
			return true
		}
	}
	return false
}

func roadReverseClearHorizon(f autopilotFrame, heading, center float64, ew bool, ticks int) bool {
	b := *f.bus
	b.JunctionCorridor, b.RoundaboutEntry, b.LaneMergeRequest, b.StationMergeRequest = nil, nil, nil, nil
	b.TireTracks = nil
	b.ShiftTo(sim.GearReverse)
	for _, ob := range f.obstacles {
		if ob.Bus != f.bus && ob.Speed < 0 {
			reach := b.OverallLength() + ob.Length + math.Abs(ob.Speed)*(float64(ticks)/60) + 40
			if ob.Pos.Distance(b.Pos) < reach {
				return false
			}
		}
	}
	normal := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	initial := roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew)
	brakeTick := ticks - 12
	for tick := 1; tick <= ticks; tick++ {
		throttle, brake := roadReversePedals(b.Speed)
		if tick > brakeTick {
			throttle, brake = 0, 1
		}
		steer := sim.Clamp(3*sim.NormalizeAngle(b.Heading-heading), -.65, .65)
		b.SetNextAccelerationScale(1)
		b.Update(1./60, throttle, brake, steer, false, false)
		if b.HitsHardBarrier(f.world, f.laneWidth) || busOverlapsLevelCrossing(&b, f.world) {
			return false
		}
		parts := b.BodyParts()
		minCorner := 0.0
		if initial < 0 {
			minCorner = initial - 4.0
		}
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if corner.Sub(origin).Dot(normal) < minCorner {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			if ob.Bus != f.bus && sweepBodiesCollision(parts, ob, math.Max(0, ob.Speed)*float64(tick)/60, 1, 1) {
				return false
			}
		}
	}
	return roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew) > initial+.1
}

func (ad *AutopilotDriver) chooseRoadReverseSteer(f autopilotFrame, heading, center float64, ew bool, steer, clearance float64) (float64, bool) {
	if roadReverseClear(f, heading, center, ew) {
		return steer, true
	}
	if f.bus != nil && (f.bus.IsArticulated() || clearance < 0) {
		for _, ticks := range []int{30, 18} {
			for _, cand := range []float64{steer * .5, 0} {
				if roadReverseClearSteer(f, heading, center, ew, cand, ticks) {
					return cand, true
				}
			}
		}
	}
	return steer, false
}

func roadReverseClearSteer(f autopilotFrame, heading, center float64, ew bool, fixedSteer float64, ticks int) bool {
	if f.world == nil || f.bus == nil {
		return false
	}
	b := *f.bus
	b.JunctionCorridor, b.RoundaboutEntry, b.LaneMergeRequest, b.StationMergeRequest = nil, nil, nil, nil
	b.TireTracks = nil
	b.ShiftTo(sim.GearReverse)
	normal := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	initial := roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew)
	minCorner := 0.0
	if initial < 0 {
		minCorner = initial - 4.0
	}
	for tick := 1; tick <= ticks; tick++ {
		throttle, brake := 1., 0.
		if math.Abs(b.Speed) > 35 {
			throttle, brake = 0, 1
		}
		b.SetNextAccelerationScale(1)
		b.Update(1./60, throttle, brake, fixedSteer, false, false)
		if b.HitsHardBarrier(f.world, f.laneWidth) || busOverlapsLevelCrossing(&b, f.world) {
			return false
		}
		parts := b.BodyParts()
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if corner.Sub(origin).Dot(normal) < minCorner {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			if ob.Bus != f.bus && sweepBodiesCollision(parts, ob, math.Max(0, ob.Speed)*float64(tick)/60, 1, 1) {
				return false
			}
		}
	}
	return roadMedianClearance(&b, b.Pos, b.Heading, heading, center, ew) > initial+.1
}

func (ad *AutopilotDriver) roadReverseForwardUnwindClear(f autopilotFrame, heading, center float64, ew bool, clearance, yaw, recoverySteer float64) bool {
	cabLat := roadMedianClearance(f.bus, f.bus.Pos, heading, heading, center, ew) + f.bus.Width/2
	if cabLat <= 8 || math.Abs(yaw) > .85 {
		return false
	}
	const crawlSpeed = 20.0
	const dt = .04
	pos, angle, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
	if f.speedAbs <= .5 && steering*recoverySteer < f.bus.MaxSteerAngle*.5 {
		steering = recoverySteer * f.bus.MaxSteerAngle * .5
	}
	for elapsed := dt; elapsed <= .9; elapsed += dt {
		steering += sim.Clamp(recoverySteer*f.bus.MaxSteerAngle-steering, -f.bus.SteerTurnSpeed*dt, f.bus.SteerTurnSpeed*dt)
		angle += crawlSpeed / f.bus.WheelBase * math.Tan(steering) * dt
		pos = pos.Add(sim.UnitFromAngle(angle).Mul(crawlSpeed * dt))
		if roadMedianClearance(f.bus, pos, heading, heading, center, ew)+f.bus.Width/2 <= 4 {
			return false
		}
		body := sim.OBB{Center: pos, Heading: angle, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
		for _, corner := range sim.BodyCorners(body) {
			if f.world.CheckBoundaries(corner, 0, f.laneWidth).Zone == sim.ZoneHardBarrier {
				return false
			}
		}
		for _, ob := range f.obstacles {
			if ob.Pos.Distance(f.bus.Pos) > 250+math.Abs(ob.Speed)*elapsed {
				continue
			}
			for _, part := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
				if part.HalfLen <= 0 {
					continue
				}
				part.HalfLen++
				part.HalfWid++
				if hit, _, _ := sim.CheckBusAtBody(f.bus, body, part); hit {
					return false
				}
			}
		}
	}
	return math.Abs(sim.NormalizeAngle(angle-heading)) < math.Abs(yaw)-.05
}

type barrierReverseState struct {
	revSteer   float64
	fwdSteer   float64
	longUnpin  bool
	revAligned bool
	presteer   bool
	fwdActive  bool
	fwdSec     float64
	fwdDist    float64
	fwdAnchor  sim.Vec2
	minSec     float64
	elapsedSec float64
}

// When a bus stops angled against a roadside barrier where even full-lock forward
// steering clips the barrier on the swept envelope, a brief checked reverse
// swings the nose back toward road alignment before pre-steering forward.
func (ad *AutopilotDriver) tryBarrierReverseRecovery(f autopilotFrame, steer, dt float64) bool {
	if ad.barrierReverse != nil || ad.barrierReverseCooldown > 0 || f.world == nil || f.bus == nil ||
		f.nearRoundabout || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 ||
		ad.State == AutopilotStateDocked || ad.roadReverseActive || f.speedAbs > .5 ||
		(f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) && len(ad.drivePath) >= 2 && f.bus.Pos.Distance(ad.pointAt(ad.LastPathS)) > f.laneWidth*1.2) ||
		f.bus.RoundaboutAdmissionHold != nil || ad.trafficLightSpeed(f, 30) <= .5 ||
		ad.junctionAdmissionSpeed(f) <= .5 || ad.followingSpeed(f) <= .5 || ad.obstacleSpeed(f, 30) <= .5 {
		return false
	}
	empty := f
	empty.obstacles = nil
	blocked := ad.roadSafetyHold || ad.controlSweptSpeed(empty, steer) <= .5 || (f.bus.IsArticulated() && ad.articulatedSweptSpeed(empty, steer) <= .5)
	if !blocked && ad.controlSweptSpeed(f, steer) <= .5 {
		stationary := f
		stationary.obstacles = nil
		for _, ob := range f.obstacles {
			if math.Abs(ob.Speed) < 3 {
				stationary.obstacles = append(stationary.obstacles, ob)
			}
		}
		if ad.controlSweptSpeed(stationary, steer) <= .5 {
			if _, _, clear := ad.compactControlUnwindSteer(stationary, steer); !clear {
				blocked = true
			}
		}
	}
	if !blocked && !ad.trailerSidecarSweepPinned(f, steer) {
		return false
	}
	yawErr := ad.barrierRecoveryYaw(f)
	if math.Abs(yawErr) >= math.Pi*.75 {
		return false
	}
	if math.Abs(yawErr) < .08 {
		if math.Abs(steer) > .05 {
			yawErr = math.Copysign(.1, steer)
		} else {
			yawErr = .1
		}
	}
	plan, ok := ad.planBarrierUnpinReverse(f, yawErr)
	if !ok {
		trial, requests := rearReverseYieldFrame(f)
		if !requests.empty() {
			if _, relayOk := ad.planBarrierUnpinReverse(trial, yawErr); relayOk {
				requests.send()
				ad.barrierYieldRelay = requests
				ad.barrierYieldRelayTimer = 2.4
				ad.barrierReverseCooldown = 144
				return false
			}
		}
		ad.barrierReverseCooldown = 30
		return false
	}
	ad.barrierYieldRelay = rearReverseYield{}
	ad.barrierYieldRelayTimer = 0
	ad.barrierReverse = &plan
	ad.departureEscapeActive = false
	ad.departurePresteerValid = false
	return true
}

func (ad *AutopilotDriver) planBarrierUnpinReverse(f autopilotFrame, yawErr float64) (barrierReverseState, bool) {
	if len(f.obstacles) > 0 && f.bus != nil {
		filtered := make([]VehicleObstacle, 0, min(len(f.obstacles), 32))
		for _, ob := range f.obstacles {
			reach := f.bus.OverallLength() + ob.Length + math.Abs(ob.Speed)*5 + 360
			d := ob.Pos.Sub(f.bus.Pos)
			if d.Dot(d) <= reach*reach {
				filtered = append(filtered, ob)
			}
		}
		f.obstacles = filtered
	}
	fwdSign := math.Copysign(1, yawErr)
	// Try road-aligned exit locks first, then gentler/opposite locks when the
	// barrier itself (such as a bay shelter) blocks the road-yaw side.
	fwdCandidates := []float64{fwdSign * 1.0, fwdSign * .65, fwdSign * .35, 0, -fwdSign * .35, -fwdSign * .65, -fwdSign * 1.0}
	revCandidates := []float64{-fwdSign, -fwdSign * .65, 0, fwdSign * .65, fwdSign}

	const preferredSteps = 160
	const minSteps = 30
	var best barrierReverseState
	bestTicks := 0
	bestSteps := 0
	searchPhase := func(maxReverseTicks int) bool {
		for _, revSteer := range revCandidates {
			b := *f.bus
			b.JunctionCorridor, b.RoundaboutEntry, b.LaneMergeRequest, b.StationMergeRequest = nil, nil, nil, nil
			b.TireTracks = nil
			b.SteeringAngle = revSteer * b.MaxSteerAngle
			b.Speed = 0
			b.ShiftTo(sim.GearReverse)
			for tick := 1; tick <= maxReverseTicks; tick++ {
				curRev := effectiveBarrierRevSteer(&b, revSteer)
				th, br := roadReversePedals(b.Speed)
				b.SetNextAccelerationScale(1)
				b.Update(1./60, th, br, curRev, false, false)
				if !ad.barrierReversePoseSafe(f, &b, tick) {
					break
				}
				if tick < 18 || (tick <= 180 && tick != 18 && tick%15 != 0 && tick != maxReverseTicks) || (tick > 180 && tick%30 != 0 && tick != maxReverseTicks) {
					continue
				}
				stopped := b
				stopped.ShiftTo(sim.GearDrive)
				for i := 0; i < 6 && math.Abs(stopped.Speed) > .5; i++ {
					stopRev := effectiveBarrierRevSteer(&stopped, revSteer)
					stopped.SetNextAccelerationScale(1)
					stopped.Update(1./60, 0, 1, stopRev, false, false)
					if !ad.barrierReversePoseSafe(f, &stopped, tick+i+1) {
						stopped.Speed = -999
						break
					}
				}
				if stopped.Speed == -999 {
					continue
				}
				stopped.Speed = 0
				for _, fwdSteer := range fwdCandidates {
					if steps, ok := ad.barrierForwardClearAfterPresteer(f, &stopped, fwdSteer, preferredSteps, minSteps); ok {
						cand := barrierReverseState{
							revSteer:  revSteer,
							fwdSteer:  fwdSteer,
							longUnpin: f.bus.Pos.Distance(stopped.Pos) > 30 && fwdSteer != 0,
							fwdSec:    float64(steps) / 60.0,
							minSec:    float64(tick) / 60.0,
						}
						if steps > bestSteps || (steps == bestSteps && tick < bestTicks) || bestTicks == 0 {
							best = cand
							bestTicks = tick
							bestSteps = steps
						}
						if bestSteps >= preferredSteps {
							return true
						}
					}
				}
			}
		}
		return false
	}
	if searchPhase(75) {
		return best, true
	}
	if ad.obstacleSpeed(f, 80) > 10 {
		if searchPhase(180) {
			return best, true
		}
		if searchPhase(270) {
			return best, true
		}
		searchPhase(420)
	}
	return best, bestTicks > 0
}

func (ad *AutopilotDriver) barrierReversePoseSafe(f autopilotFrame, b *sim.Bus, tick int) bool {
	if b.HitsHardBarrier(f.world, f.laneWidth) {
		return false
	}
	parts := b.BodyParts()
	if busOverlapsLevelCrossing(b, f.world) {
		return false
	}
	if f.world != nil && f.bus != nil && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) && !f.world.InsideJunction(b.Pos, f.laneWidth, 0) {
		roadHeading := ad.currentRoadHeading(f)
		if center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, roadHeading, f.laneWidth); multi || six {
			if busCenterlineClearance(f.bus, roadHeading, center, ew) >= .5 && busCenterlineClearance(b, roadHeading, center, ew) < .5 {
				return false
			}
		}
	}
	elapsed := float64(tick) / 60.0
	for _, ob := range f.obstacles {
		if ob.Bus != f.bus && sweepBodiesCollision(parts, ob, math.Max(0, ob.Speed)*elapsed, 1.5, 1.5) {
			return false
		}
	}
	return true
}

func busCenterlineClearance(b *sim.Bus, roadHeading, center float64, ew bool) float64 {
	u := sim.UnitFromAngle(roadHeading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	minClear := math.Inf(1)
	for _, part := range b.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - roadHeading
		offset := part.Center.Sub(origin).Dot(r)
		extent := part.HalfLen*math.Abs(math.Sin(yaw)) + part.HalfWid*math.Abs(math.Cos(yaw))
		minClear = math.Min(minClear, offset-extent)
	}
	return minClear
}

func effectiveBarrierRevSteer(b *sim.Bus, revSteer float64) float64 {
	if b != nil && b.IsArticulated() {
		bend := sim.NormalizeAngle(b.Heading - b.TailHeading())
		if math.Abs(bend) > .62 {
			return math.Copysign(1, bend)
		}
	}
	return revSteer
}

func longBarrierUnpinSteer(f autopilotFrame, b *sim.Bus, roadHeading, fwdSteer float64) float64 {
	if f.world == nil || fwdSteer == 0 {
		return fwdSteer
	}
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, b.Pos, roadHeading, f.laneWidth)
	u := sim.UnitFromAngle(roadHeading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	curLat := b.Pos.Sub(origin).Dot(right)
	targetLat := .5 * f.laneWidth
	if fwdSteer < 0 && curLat > 2*f.laneWidth {
		targetLat = (math.Floor((curLat-.1)/f.laneWidth) - .5) * f.laneWidth
	} else if fwdSteer > 0 {
		flare := f.world.BRTStationFlareAt(b.Pos.Add(u.Mul(80)), f.laneWidth)
		targetLat = 1.5*f.laneWidth + flare
		if road, ok := f.world.RoadAt(center, ew, b.Pos); ok {
			maxOuter := (float64(max(1, road.LaneCount()/2)) - .5) * f.laneWidth
			if road.LaneCount() <= 2 {
				targetLat = maxOuter + flare
			} else if road.LaneCount() > 4 {
				nextOuter := (math.Floor(curLat/f.laneWidth) + .5) * f.laneWidth
				targetLat = math.Min(maxOuter, math.Max(1.5*f.laneWidth, nextOuter)) + flare
			}
		}
	}
	desiredYaw := sim.Clamp((targetLat-curLat)/28.0, -.45, .45)
	yaw := sim.NormalizeAngle(b.Heading - roadHeading)
	// Keep the initial unpin lock until the heading has crossed past road alignment
	// toward the target corridor; then guide smoothly into the target lane center.
	if yaw*math.Copysign(1, fwdSteer) < .05 {
		return fwdSteer
	}
	return sim.Clamp(3.0*sim.NormalizeAngle((roadHeading+desiredYaw)-b.Heading), -1, 1)
}

func (ad *AutopilotDriver) barrierForwardClearAfterPresteer(f autopilotFrame, base *sim.Bus, fwdSteer float64, maxSteps, minSteps int) (int, bool) {
	roadHeading := ad.currentRoadHeading(f)
	roadForward := sim.UnitFromAngle(roadHeading)
	startYaw := math.Abs(sim.NormalizeAngle(f.bus.Heading - roadHeading))
	checkCenterline := false
	var roadCenter float64
	var roadEW bool
	startClear := math.Inf(1)
	if f.world != nil && f.bus != nil && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) {
		if center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, roadHeading, f.laneWidth); multi || six {
			checkCenterline = true
			roadCenter, roadEW = center, ew
			startClear = busCenterlineClearance(f.bus, roadHeading, center, ew)
		}
	}
	cand := *base
	cand.SteeringAngle = fwdSteer * cand.MaxSteerAngle
	cf := f
	cf.bus = &cand
	cf.speedAbs = 0
	cf.allowBarrierUnwindYaw = true
	savedHold, savedSpeed := ad.roadSafetyHold, ad.roadRecoverySpeed
	checked := ad.roadLaneSafeSteering(cf, fwdSteer)
	hold := ad.roadSafetyHold
	ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
	if hold || math.Abs(checked-fwdSteer) > .01 {
		return 0, false
	}
	if ad.controlSweptSpeed(cf, fwdSteer) < 16 {
		return 0, false
	}
	if cand.IsArticulated() && ad.articulatedSweptSpeed(cf, fwdSteer, true) < 16 {
		return 0, false
	}
	// Simulate the actual forward acceleration and trailer motion so presteer
	// planning agrees with what the vehicle executes after shifting to Drive.
	ahead := cand
	ahead.JunctionCorridor, ahead.RoundaboutEntry, ahead.LaneMergeRequest, ahead.StationMergeRequest = nil, nil, nil, nil
	ahead.TireTracks = nil
	ahead.ShiftTo(sim.GearDrive)
	revDist := f.bus.Pos.Distance(base.Pos)
	longUnpin := revDist > 30 && fwdSteer != 0
	horizonSteps := max(maxSteps, int(math.Ceil((revDist+45)/20*60))+24)
	clearSteps := 0
	lastClearPose := ahead
	hitBarrier := false
	for step := 1; step <= horizonSteps; step++ {
		curSteer := fwdSteer
		if longUnpin {
			curSteer = longBarrierUnpinSteer(f, &ahead, roadHeading, fwdSteer)
			af := f
			af.bus = &ahead
			af.speedAbs = math.Abs(ahead.Speed)
			af.allowBarrierUnwindYaw = true
			curSteer = ad.roadLaneSafeSteering(af, curSteer)
			sHold := ad.roadSafetyHold
			ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
			if sHold {
				break
			}
		}
		th, br := ad.longitudinalControls(ahead.Speed, 20)
		ahead.SetNextAccelerationScale(1)
		ahead.Update(1./60, th, br, curSteer, false, false)
		if ahead.HitsHardBarrier(f.world, f.laneWidth) {
			hitBarrier = true
			break
		}
		if checkCenterline && startClear >= .5 && !f.world.InsideJunction(ahead.Pos, f.laneWidth, 0) {
			if busCenterlineClearance(&ahead, roadHeading, roadCenter, roadEW) < .5 {
				break
			}
		}
		if step%3 == 0 {
			parts := ahead.BodyParts()
			elapsed := float64(step) / 60.0
			hitTraffic := false
			for _, ob := range f.obstacles {
				if ob.Bus != f.bus && sweepBodiesCollision(parts, ob, math.Max(0, ob.Speed)*elapsed, 1, 1) {
					hitTraffic = true
					break
				}
			}
			if hitTraffic {
				break
			}
			af := f
			af.bus = &ahead
			af.speedAbs = math.Abs(ahead.Speed)
			af.allowBarrierUnwindYaw = true
			if !longUnpin {
				sc := ad.roadLaneSafeSteering(af, curSteer)
				sHold := ad.roadSafetyHold
				ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
				if sHold || math.Abs(sc-curSteer) > .01 {
					break
				}
			}
			emptyAf := af
			emptyAf.obstacles = nil
			if ad.controlSweptSpeed(emptyAf, curSteer) < 16 || (ahead.IsArticulated() && ad.articulatedSweptSpeed(emptyAf, curSteer, true) < 16) {
				hitBarrier = true
				break
			}
		}
		clearSteps++
		lastClearPose = ahead
	}
	if clearSteps < minSteps {
		return clearSteps, false
	}
	netForward := lastClearPose.Pos.Sub(f.bus.Pos).Dot(roadForward)
	if hitBarrier && netForward < 32 {
		return clearSteps, false
	}
	endYaw := math.Abs(sim.NormalizeAngle(lastClearPose.Heading - roadHeading))
	if clearSteps < horizonSteps && netForward < 32 {
		settled := lastClearPose
		roadSteer := sim.Clamp(2.5*sim.NormalizeAngle(roadHeading-settled.Heading), -1, 1)
		settled.SteeringAngle = roadSteer * settled.MaxSteerAngle
		emptyEnd := f
		emptyEnd.bus = &settled
		emptyEnd.speedAbs = 0
		emptyEnd.obstacles = nil
		endChecked := ad.roadLaneSafeSteering(emptyEnd, roadSteer)
		endHold := ad.roadSafetyHold
		ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
		unpinned := endYaw <= .22 && !endHold && math.Abs(endChecked-roadSteer) <= .02 &&
			ad.controlSweptSpeed(emptyEnd, roadSteer) >= 16 &&
			(!settled.IsArticulated() || ad.articulatedSweptSpeed(emptyEnd, roadSteer, true) >= 16)
		if !unpinned {
			return clearSteps, false
		}
	}
	if lastClearPose.Pos.Distance(f.bus.Pos) < 10 && netForward < 8 && endYaw >= startYaw-.04 {
		return clearSteps, false
	}
	return clearSteps, true
}

func (ad *AutopilotDriver) barrierReverseControls(f autopilotFrame, desiredSteer, dt float64) (throttle, brake, steer float64, active bool) {
	r := ad.barrierReverse
	if r == nil {
		return 0, 0, 0, false
	}
	if f.world == nil || f.bus.DoorsOpen || f.bus.PassengerServiceRemaining() > 0 || ad.State == AutopilotStateDocked {
		f.bus.ShiftTo(sim.GearDrive)
		ad.barrierReverse = nil
		return 0, 0, 0, false
	}
	if !r.revAligned {
		if f.speedAbs > .5 || math.Abs(f.bus.SteeringAngle-r.revSteer*f.bus.MaxSteerAngle) > .05 {
			return 0, 1, r.revSteer, true
		}
		r.revAligned = true
	}
	if !r.presteer {
		curRev := effectiveBarrierRevSteer(f.bus, r.revSteer)
		probe := *f.bus
		probe.JunctionCorridor, probe.RoundaboutEntry, probe.LaneMergeRequest, probe.StationMergeRequest = nil, nil, nil, nil
		probe.TireTracks = nil
		probe.ShiftTo(sim.GearReverse)
		imminentHazard := false
		for step := 1; step <= 3; step++ {
			pth, pbr := roadReversePedals(probe.Speed)
			probeRev := effectiveBarrierRevSteer(&probe, r.revSteer)
			probe.SetNextAccelerationScale(1)
			probe.Update(1./60, pth, pbr, probeRev, false, false)
			if !ad.barrierReversePoseSafe(f, &probe, step) {
				imminentHazard = true
				break
			}
		}
		maxRevSec := math.Max(1.25, r.minSec)
		if imminentHazard || r.elapsedSec >= r.minSec || r.elapsedSec >= maxRevSec {
			r.presteer = true
		} else {
			f.bus.ShiftTo(sim.GearReverse)
			r.elapsedSec += math.Max(0, dt)
			th, br := roadReversePedals(f.bus.Speed)
			return th, br, curRev, true
		}
	}
	f.bus.ShiftTo(sim.GearDrive)
	if !r.fwdActive {
		if f.speedAbs > .5 || math.Abs(f.bus.SteeringAngle-r.fwdSteer*f.bus.MaxSteerAngle) > .05 {
			return 0, 1, r.fwdSteer, true
		}
		r.fwdActive = true
		r.fwdAnchor = f.bus.Pos
		r.fwdDist = 0
		r.elapsedSec = 0
	}
	r.elapsedSec += math.Max(0, dt)
	r.fwdDist = f.bus.Pos.Distance(r.fwdAnchor)
	curSteer := r.fwdSteer
	savedHold, savedSpeed := ad.roadSafetyHold, ad.roadRecoverySpeed
	var checked float64
	var hold bool
	roadHeading := ad.currentRoadHeading(f)
	uf := f
	uf.allowBarrierUnwindYaw = true
	if r.longUnpin {
		curSteer = longBarrierUnpinSteer(f, f.bus, roadHeading, r.fwdSteer)
		curSteer = ad.roadLaneSafeSteering(uf, curSteer)
		checked = curSteer
		hold = ad.roadSafetyHold
	} else {
		checked = ad.roadLaneSafeSteering(uf, curSteer)
		hold = ad.roadSafetyHold
	}
	ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
	centerlineSafe := true
	if f.world != nil && !f.world.InsideJunction(f.bus.Pos, f.laneWidth, 0) {
		if center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, roadHeading, f.laneWidth); multi || six {
			if busCenterlineClearance(f.bus, roadHeading, center, ew) < .5 && sim.NormalizeAngle(f.bus.Heading-roadHeading) < -.72 {
				centerlineSafe = false
			}
		}
	}
	safeForward := centerlineSafe && !hold && math.Abs(checked-curSteer) <= .01 &&
		ad.controlSweptSpeed(f, curSteer) >= 16 &&
		(!f.bus.IsArticulated() || ad.articulatedSweptSpeed(f, curSteer, true) >= 16) &&
		ad.trafficLightSpeed(f, 30) >= 16 &&
		ad.junctionAdmissionSpeed(f) >= 16 &&
		ad.followingSpeed(f) >= 16
	maxFwdDist := math.Max(64, r.fwdSec*20)
	maxFwdSec := math.Max(3.0, r.fwdSec+.5)
	desiredReady := false
	if !r.longUnpin && r.fwdDist >= 24 && math.Abs(sim.NormalizeAngle(f.bus.Heading-roadHeading)) <= .45 && !hasStoppedTrailerSidecar(f, roadHeading) {
		sHold, sSpd := ad.roadSafetyHold, ad.roadRecoverySpeed
		dChecked := ad.roadLaneSafeSteering(f, desiredSteer)
		dHold := ad.roadSafetyHold
		ad.roadSafetyHold, ad.roadRecoverySpeed = sHold, sSpd
		desiredReady = !dHold && math.Abs(dChecked-desiredSteer) <= .02 &&
			ad.controlSweptSpeed(f, dChecked) >= 16 &&
			(!f.bus.IsArticulated() || ad.articulatedSweptSpeed(f, dChecked, true) >= 16)
	}
	if desiredReady || !safeForward || r.fwdDist >= maxFwdDist || (r.fwdSec > 0 && r.elapsedSec >= r.fwdSec) || r.elapsedSec >= maxFwdSec {
		ad.barrierReverse = nil
		ad.barrierReverseCooldown = 60
		if desiredReady {
			return 0, 0, desiredSteer, false
		}
		return 0, 0, curSteer, false
	}
	th, br := ad.longitudinalControls(f.bus.Speed, 20)
	return th, br, curSteer, true
}

// When a yawed articulated bus is swept-blocked by a stationary car beside its
// trailer and sidecarEscape proved no forward crawl clears that car without
// dragging the trailer into it, allow a checked reverse unpin.
func (ad *AutopilotDriver) trailerSidecarSweepPinned(f autopilotFrame, steer float64) bool {
	if !f.bus.IsArticulated() || ad.sidecarPresteerValid || mergeYieldHold(f) ||
		(sidecarBodyInJunction(f) && !ad.sidecarAtIncomingThroatForDriver(f)) {
		return false
	}
	heading := ad.currentRoadHeading(f)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) < .20 {
		return false
	}
	_, _, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	if !multi && !six {
		return false
	}
	if ad.controlSweptSpeed(f, steer) > .5 && ad.articulatedSweptSpeed(f, steer) > .5 {
		return false
	}
	return hasStoppedTrailerSidecar(f, heading)
}

func hasStoppedTrailerSidecar(f autopilotFrame, heading float64) bool {
	u := sim.UnitFromAngle(heading)
	for _, ob := range f.obstacles {
		if sim.UnitFromAngle(ob.Heading).Dot(u) >= .9 && stoppedSidecarTrailerOffset(f, ob) != 0 {
			return true
		}
	}
	return false
}
