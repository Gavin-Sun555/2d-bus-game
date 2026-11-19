package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// controlSweptSpeed checks the physical pose produced by the steering actuator,
// including the nose and tail during a lane change or turn exit. Lane-center
// exemptions in ordinary following are not enough when the bus is yawed.
func (ad *AutopilotDriver) controlSweptSpeed(f autopilotFrame, steer float64) float64 {
	if f.world == nil || f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return math.Inf(1)
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return math.Inf(1)
		}
	}
	rearCars := ad.departureRearCars(f)
	speed := math.Max(20, f.speedAbs)
	horizon := math.Min(1.2, .20+speed/180)
	limit := math.Inf(1)
	var samples [32]controlSample
	var ready [32]bool
	for _, ob := range f.obstacles {
		if rearCars.matches(ob) {
			continue
		}
		reach := speed*horizon + math.Abs(ob.Speed)*horizon + (f.bus.Length+ob.Length)/2 + 20
		d := ob.Pos.Sub(f.bus.Pos)
		if d.Dot(d) > reach*reach {
			continue
		}
		pos, heading, steering := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
		index := 0
		for elapsed := .04; elapsed <= horizon; elapsed += .04 {
			if ready[index] {
				pos, heading, steering = samples[index].pos, samples[index].heading, samples[index].steering
			} else {
				rate := f.bus.SteerTurnSpeed
				if steer == 0 {
					rate = f.bus.SteerReturnSpd
				}
				steering += sim.Clamp(steer*f.bus.MaxSteerAngle-steering, -rate*.04, rate*.04)
				heading += speed / f.bus.WheelBase * math.Tan(steering) * .04
				pos = pos.Add(sim.UnitFromAngle(heading).Mul(speed * .04))

				body := sim.OBB{Center: pos, Heading: heading, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
				samples[index] = controlSample{sweepSample: sweepSample{front: body}, pos: pos, heading: heading, steering: steering}
				ready[index] = true
			}
			if samples[index].collides(f.bus, ob, math.Max(0, ob.Speed)*elapsed, 1, 1) {
				if controlPaddingRelease(f, ob, steer, speed, horizon) || ad.laneChangeLeaderClearance(f, ob, steer, speed, horizon) {
					limit = math.Min(limit, 35)
					break
				}
				limit = math.Min(limit, math.Sqrt(2*180*math.Max(0, speed*elapsed-18)))
				if limit == 0 {
					return 0
				}
				break
			}
			index++
		}
	}
	return limit
}

type controlSample struct {
	sweepSample
	pos               sim.Vec2
	heading, steering float64
}

// compactControlUnwindSteer softens a sharp pursuit steer on a compact bus when
// rear-overhang swing clips a stationary rear-side vehicle's sweep margin while
// a gentler same-sign or straight crawl stays lane-safe and barrier-clear.
func (ad *AutopilotDriver) compactControlUnwindSteer(f autopilotFrame, desired float64) (float64, bool, bool) {
	if f.world == nil || f.bus == nil || f.bus.IsArticulated() || f.nearRoundabout || math.Abs(desired) < .25 ||
		ad.followingSpeed(f) <= .5 || ad.obstacleSpeed(f, 30) <= .5 {
		return 0, false, false
	}
	if _, _, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth); !multi && !six && desired <= 0 {
		return 0, false, false
	}
	if f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return 0, false, false
	}
	nearInter := false
	for _, inter := range f.world.Intersections {
		j := f.world.JunctionAt(inter, f.laneWidth)
		if math.Abs(f.bus.Pos.X-inter.X) <= j.HalfX+f.laneWidth && math.Abs(f.bus.Pos.Y-inter.Y) <= j.HalfY+f.laneWidth {
			nearInter = true
			break
		}
	}
	if nearInter && ad.approachMerge == nil {
		return 0, false, false
	}
	savedHold, savedSpeed := ad.roadSafetyHold, ad.roadRecoverySpeed
	defer func() {
		ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
	}()
	sweepFrame := frameWithoutClosingRearFollowers(f)
	barrierClear := func(base *sim.Bus, cand float64) bool {
		probe := *base
		pos, heading, wheel := probe.Pos, probe.Heading, probe.SteeringAngle
		for step := 0; step < 10; step++ {
			rate := probe.SteerTurnSpeed
			if cand == 0 {
				rate = probe.SteerReturnSpd
			}
			wheel += sim.Clamp(cand*probe.MaxSteerAngle-wheel, -rate*.04, rate*.04)
			heading += 16 / probe.WheelBase * math.Tan(wheel) * .04
			pos = pos.Add(sim.UnitFromAngle(heading).Mul(16 * .04))
			probe = probe.AtPose(pos, heading)
			if probe.HitsHardBarrier(f.world, f.laneWidth) {
				return false
			}
		}
		return true
	}
	for _, scale := range []float64{.65, .5, .35, .25, .15, 0} {
		cand := desired * scale
		checked := ad.roadLaneSafeSteering(f, cand)
		hold := ad.roadSafetyHold
		ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
		if hold || math.Abs(checked-cand) > .01 {
			continue
		}
		if ad.controlSweptSpeed(sweepFrame, cand) >= 16 && barrierClear(f.bus, cand) {
			return cand, false, true
		}
		if f.speedAbs <= .5 && math.Abs(f.bus.SteeringAngle-cand*f.bus.MaxSteerAngle) > .03 {
			settled := *f.bus
			settled.SteeringAngle = cand * settled.MaxSteerAngle
			settledFrame := sweepFrame
			settledFrame.bus = &settled
			sc := ad.roadLaneSafeSteering(settledFrame, cand)
			sHold := ad.roadSafetyHold
			ad.roadSafetyHold, ad.roadRecoverySpeed = savedHold, savedSpeed
			if !sHold && math.Abs(sc-cand) <= .01 && ad.controlSweptSpeed(settledFrame, cand) >= 16 && barrierClear(&settled, cand) {
				return cand, true, true
			}
		}
	}
	return 0, false, false
}
