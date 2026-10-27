package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// A lane policy can already have frozen its target at a fractional offset,
// so retry from the physical body rather than requiring a new rightward intent.
func (ad *AutopilotDriver) retryStoppedMidLaneReturn(f autopilotFrame) {
	if ad.midLaneRecovery != nil || !ad.DisallowOncomingOvertake || f.world == nil || f.speedAbs > 5 || ad.approachMerge != nil || ad.State != AutopilotStateCruising {
		return
	}
	heading := approachRoadHeading(f.bus.Heading)
	u := sim.UnitFromAngle(heading)
	if ad.pointAt(f.s+80).Sub(ad.pointAt(f.s)).Normalize().Dot(u) < .99 {
		return
	}
	for _, rb := range f.world.Roundabouts {
		if f.bus.Pos.Distance(rb.Center) < rb.Radius+260 {
			return
		}
	}
	center, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	if !multi && !six {
		return
	}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(f.bus.Pos)
		if math.Abs(ob.Speed) < 3 && math.Cos(ob.Heading-heading) > .98 && delta.Dot(u) > 0 && delta.Dot(u) < (f.bus.Length+ob.Length)/2+35 && math.Abs(delta.Dot(r)) < (f.bus.Width+ob.Width)/2+8 {
			ad.beginMidLaneRecovery(f, heading, center, ew)
			return
		}
	}
}

// If a fleet bus's return gap closes after it has crossed the lane marking,
// holding that fractional offset occupies two lanes forever. Retreat into the
// previous lane when its gap is clear; keep the target until physically aligned.
func (ad *AutopilotDriver) beginMidLaneRecovery(f autopilotFrame, heading, center float64, ew bool) {
	if !ad.DisallowOncomingOvertake || f.speedAbs > 5 || f.world.InsideJunction(f.bus.Pos, f.laneWidth, 500) {
		return
	}
	for _, stop := range f.world.Stops {
		if stop.Pos.Distance(f.bus.Pos) < 500 {
			return
		}
	}
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	lateral := f.bus.Pos.Sub(origin).Dot(r)
	lane := math.Floor(lateral/f.laneWidth - .5)
	target := (lane + .5) * f.laneWidth
	if lane < 0 || lateral-target < 8 || lateral-target > f.laneWidth-8 {
		return
	}
	base := ad.pointAt(f.s)
	current := f.bus.Pos.Sub(base).Dot(r)
	desired := origin.Add(r.Mul(target)).Sub(base).Dot(r)
	if !requiredMergeLaneClear(f, heading, current, desired) {
		return
	}
	ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: origin.Add(r.Mul(target)), Heading: heading, Remaining: .3}
}

func (ad *AutopilotDriver) advanceMidLaneRecovery(f autopilotFrame, dt float64) bool {
	request := ad.midLaneRecovery
	if request == nil {
		return false
	}
	r := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	desired := request.Target.Sub(ad.pointAt(f.s)).Dot(r)
	aligned := math.Abs(sim.NormalizeAngle(f.bus.Heading-request.Heading)) < .08 && math.Abs(f.bus.SteeringAngle) < .08
	if math.Abs(f.bus.Pos.Sub(request.Target).Dot(r)) < 3 && aligned {
		ad.holdLaneOffset(desired)
		ad.midLaneRecovery = nil
		if f.bus.LaneMergeRequest == request {
			f.bus.LaneMergeRequest = nil
		}
		return true
	}
	request.Remaining = .3
	f.bus.LaneMergeRequest = request
	ad.TargetLaneOffset = desired
	ad.OvertakeState = OvertakeReturning
	ad.progressLaneOffset(dt, f.speedAbs)
	return true
}

// Predict the real steering actuator at the crawl speed. An exemption from
// conservative following margins is allowed only while the complete body is
// clear of traffic and road barriers over this moving horizon.
func (ad *AutopilotDriver) midLaneSafeSteering(f autopilotFrame, steer float64) float64 {
	ad.midLaneSweepClear = false
	if ad.midLaneRecovery == nil {
		return steer
	}
	var nearby [32]VehicleObstacle
	obstacles := nearby[:0]
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(f.bus.Pos)
		reach := 200 + ob.Length + math.Abs(ob.Speed)*.8
		if delta.Dot(delta) <= reach*reach {
			obstacles = append(obstacles, ob)
		}
	}
	f.obstacles = obstacles
	r := sim.Vec2{X: -math.Sin(ad.midLaneRecovery.Heading), Y: math.Cos(ad.midLaneRecovery.Heading)}
	direction := math.Copysign(1, ad.midLaneRecovery.Target.Sub(f.bus.Pos).Dot(r))
	candidates := [3]float64{sim.Clamp(steer, -1, 1), direction, direction * .8}
	if ad.skippedDeparture != nil && ad.skippedDeparture.served &&
		math.Abs(ad.midLaneRecovery.Target.Sub(f.bus.Pos).Dot(r)) > f.laneWidth*.7 &&
		math.Abs(sim.NormalizeAngle(f.bus.Heading-ad.midLaneRecovery.Heading)) < .45 {
		// Start a whole-lane station exit early; a tiny recentering steer
		// spends the available forward gap before creating lateral clearance.
		candidates[0], candidates[1] = direction, candidates[0]
	}
	for _, candidate := range candidates {
		if ad.midLaneSweepSafe(f, candidate) {
			ad.midLaneSweepClear = true
			return candidate
		}
	}
	// Preposition the wheels while stationary, then retry the moving sweep.
	return direction
}

func (ad *AutopilotDriver) midLaneSweepSafe(f autopilotFrame, steer float64) bool {
	center, ew, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, ad.midLaneRecovery.Heading, f.laneWidth)
	pos, angle, wheel := f.bus.Pos, f.bus.Heading, f.bus.SteeringAngle
	const dt = .02
	for elapsed := 0.; elapsed <= .8; elapsed += dt {
		if roadMedianClearance(f.bus, pos, angle, ad.midLaneRecovery.Heading, center, ew) < 0 {
			return false
		}
		body := sim.OBB{Center: pos, Heading: angle, HalfLen: f.bus.Length/2 + 1, HalfWid: f.bus.Width/2 + 1}
		u := sim.UnitFromAngle(angle)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		for _, a := range []float64{-1, 1} {
			for _, b := range []float64{-1, 1} {
				corner := pos.Add(u.Mul(a * body.HalfLen)).Add(r.Mul(b * body.HalfWid))
				if f.world.CheckBoundaries(corner, 0, f.laneWidth).Zone == sim.ZoneHardBarrier {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			other := sim.OBB{Center: ob.Pos.Add(sim.UnitFromAngle(ob.Heading).Mul(ob.Speed * elapsed)), Heading: ob.Heading, HalfLen: ob.Length/2 + 1, HalfWid: ob.Width/2 + 1}
			if ob.JunctionTurn != nil {
				other.Center, other.Heading = ob.JunctionTurn.Predict(math.Max(0, ob.Speed) * elapsed)
			}
			if hit, _, _ := sim.CheckBusAtBody(f.bus, body, other); hit {
				return false
			}
		}
		rate := f.bus.SteerTurnSpeed
		if steer == 0 {
			rate = f.bus.SteerReturnSpd
		}
		wheel += sim.Clamp(steer*f.bus.MaxSteerAngle-wheel, -rate*dt, rate*dt)
		angle += 20 / f.bus.WheelBase * math.Tan(wheel) * dt
		pos = pos.Add(sim.UnitFromAngle(angle).Mul(20 * dt))
	}
	return true
}

func (ad *AutopilotDriver) clearsQueueWhileRecentering(f autopilotFrame, ob VehicleObstacle) bool {
	request := ad.midLaneRecovery
	if request == nil || !ad.midLaneSweepClear || math.Abs(ob.Speed) > 3 || ob.Length > 65 || math.Cos(ob.Heading-request.Heading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(request.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	delta := ob.Pos.Sub(f.bus.Pos)
	shift := request.Target.Sub(f.bus.Pos).Dot(r)
	if delta.Dot(u) <= 0 || delta.Dot(u) > 220 || delta.Dot(r)*shift >= 0 {
		return false
	}
	return math.Abs(ob.Pos.Sub(request.Target).Dot(r)) > f.bus.Width/2+ob.Width/2+6
}
