package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

const (
	carDeadlockReverseTriggerSec    = 5.0
	carDeadlockReverseMaxDist       = 60.0
	carDeadlockReverseMinDist       = 20.0
	carDeadlockYieldMinDist         = 8.0
	carDeadlockReverseSpeed         = 30.0
	carDeadlockReverseHoldSec       = 2.5
	carDeadlockReverseMinHoldSec    = 0.8
	carDeadlockReverseCooldown      = 8.0
	carDeadlockReverseRetryCooldown = 3.0
	carDeadlockReverseMaxAttempts   = 4
	carDeadlockYieldRequestTTL      = 0.6
	carDeadlockYieldConfirmDelay    = 0.6
	carDeadlockYieldRetryDelay      = 0.15
	carDeadlockReverseDecel         = 120.0
)

// stepCarDeadlockReverse triggers and drives a bounded reverse maneuver when
// a civilian car has deadlocked behind a wedged bus and safe rear space exists,
// propagating retreat requests along rear followers when a single car is boxed in.
func (tm *TrafficManager) stepCarDeadlockReverse(f carFrame, dt float64) (bool, float64) {
	tm.cancelUnstartedCarPass(f)
	finishAlignedCarPass(f)
	if handled, speed := tm.stepJunctionRetreat(f); handled {
		return true, speed
	}
	car := f.car
	tm.finishPassRetreat(car)
	if car.ReverseRemaining > 0 || car.ReverseHold > 0 {
		tm.requestCarRearBusReverse(f, carDeadlockYieldMinDist)
	}
	if car.ReverseCooldown > 0 {
		car.ReverseCooldown = math.Max(0, car.ReverseCooldown-dt)
	}
	if car.YieldReverseRequest > 0 {
		car.YieldReverseRequest = math.Max(0, car.YieldReverseRequest-dt)
		if car.YieldReverseRequest <= 0 {
			car.YieldReverseDelay = 0
		}
	}
	if car.YieldReverseDelay > 0 {
		car.YieldReverseDelay = math.Max(0, car.YieldReverseDelay-dt)
	}
	if car.Speed > 5.0 && car.ReverseRemaining <= 0 && car.ReverseHold <= 0 {
		car.ReverseAttempts = 0
	}

	if car.ReverseHold > 0 {
		if car.InRoundabout || (car.JunctionTurn != nil && !car.PassRetreatActive) {
			car.ReverseHold = 0
			car.ReverseAttempts = 0
			return false, 0
		}
		car.ReverseHold = math.Max(0, car.ReverseHold-dt)
		if car.ReverseHold <= carDeadlockReverseHoldSec-carDeadlockReverseMinHoldSec &&
			(!tm.carBlockedByStoppedBus(f, false) || car.PassRetreatActive) && car.YieldReverseRequest <= 0 {
			car.ReverseHold = 0
			if car.PassRetreatActive {
				return true, 0
			}
			car.ReverseAttempts = 0
			return false, 0
		}
		car.IsBraking = true
		return true, 0
	}

	// A car with no reverse in progress, no earlier attempt, no yield request
	// and not yet stuck long enough cannot start one: every branch below is
	// a no-op for it. Skip the junction, signal and bus scans they need.
	if car.ReverseRemaining <= 0 && car.ReverseAttempts == 0 && car.YieldReverseRequest <= 0 &&
		car.DeadlockTimer < carDeadlockReverseTriggerSec && !car.PassRetreatActive {
		return false, 0
	}
	if car.ReverseRemaining <= 0 {
		tm.tryStartCarDeadlockReverse(f)
	}
	if car.ReverseRemaining <= 0 {
		if tm.canStartCarReverseContext(f) && math.Abs(car.Speed) < 1.0 &&
			(car.PassRetreatActive || (car.ReverseAttempts > 0 && car.ReverseAttempts < carReverseAttemptLimit(car) && tm.carBlockedByStoppedBus(f, true)) ||
				car.YieldReverseRequest > 0) {
			car.IsBraking = true
			return true, 0
		}
		return false, 0
	}

	if !tm.canStartCarReverseContext(f) {
		car.ReverseRemaining = 0
		car.ReverseCooldown = carDeadlockReverseCooldown
		car.ReverseAttempts = 0
		return false, 0
	}

	rearClear, _, _ := tm.availableCarReverseDistanceWithBlocker(f, car.ReverseRemaining+8)
	rem := math.Min(car.ReverseRemaining, rearClear)
	// The rear sweep already reserves 12px beyond this distance, enough
	// to stop from the 30px/s retreat cap. A second 6px cutoff prevented
	// proved short rear-slot yields from ever finishing.
	if rearClear <= 0.5 || rem <= 0.5 {
		car.ReverseRemaining = 0
		car.ReverseHold = carDeadlockReverseHoldSec
		if car.ReverseAttempts < carReverseAttemptLimit(car) {
			car.ReverseCooldown = carDeadlockReverseRetryCooldown
		} else {
			car.ReverseCooldown = carDeadlockReverseCooldown
		}
		car.IsBraking = true
		return true, 0
	}

	limit := math.Min(carDeadlockReverseSpeed, math.Sqrt(2*carDeadlockReverseDecel*rem))
	return true, -limit
}

func (tm *TrafficManager) canStartCarReverseContext(f carFrame) bool {
	car := f.car
	if car.InRoundabout || f.world == nil || (car.JunctionTurn != nil && !tm.canRetreatOptionalPass(f)) {
		return false
	}
	if f.world.InsideJunction(car.Pos, f.laneWidth, 12) {
		return false
	}
	return tm.signalSpeed(f, 100) > 0.5
}

func (tm *TrafficManager) tryStartCarDeadlockReverse(f carFrame) {
	car := f.car
	if !tm.canStartCarReverseContext(f) {
		if car.InRoundabout || car.JunctionTurn != nil || f.world == nil {
			car.ReverseAttempts = 0
		}
		return
	}
	if math.Abs(car.Speed) >= 1.0 {
		return
	}

	isBusDeadlocked := tm.carBlockedByStoppedBus(f, true) || tm.blockedOptionalPass(f)
	rootReady := isBusDeadlocked && (car.PassRetreatActive || car.DeadlockTimer >= carDeadlockReverseTriggerSec ||
		(car.ReverseAttempts > 0 && car.ReverseAttempts < carReverseAttemptLimit(car)))
	yieldReady := car.YieldReverseRequest > 0
	if !rootReady && !yieldReady {
		if !isBusDeadlocked {
			car.ReverseAttempts = 0
		}
		return
	}

	minDist := carDeadlockReverseMinDist
	if car.JunctionTurn != nil && car.JunctionTurn.LaneChange {
		minDist = math.Min(4, carTurnHistory(car.JunctionTurn))
	}
	if !rootReady && yieldReady {
		minDist = carDeadlockYieldMinDist
	}

	roadDistance := tm.safeCarReverseRoadDistance(f, carDeadlockReverseMaxDist)
	if car.JunctionTurn != nil && car.JunctionTurn.LaneChange {
		roadDistance = carTurnHistory(car.JunctionTurn)
	}
	if roadDistance < minDist {
		return
	}

	avail, blockingCar, rearReversing := tm.availableCarReverseDistanceWithBlocker(f, carDeadlockReverseMaxDist)
	canPropagate := blockingCar != nil && avail < carDeadlockReverseMaxDist
	if avail < minDist {
		if canPropagate {
			tm.requestCarYieldReverse(f, blockingCar, rootReady || car.ReverseAttempts > 0)
		} else {
			tm.requestCarRearBusReverse(f, minDist)
		}
		return
	}

	if car.ReverseCooldown > 0 {
		if canPropagate {
			tm.requestCarYieldReverse(f, blockingCar, rootReady || car.ReverseAttempts > 0)
		}
		return
	}
	if !rootReady && car.YieldReverseDelay > 0 {
		if canPropagate {
			tm.requestCarYieldReverse(f, blockingCar, false)
		}
		return
	}

	targetDist := avail
	if rearReversing && targetDist < carDeadlockReverseMaxDist {
		targetDist = carDeadlockReverseMaxDist
	}
	if car.JunctionTurn != nil && car.JunctionTurn.LaneChange {
		car.PassRetreatActive = true
		targetDist = math.Min(targetDist, carTurnHistory(car.JunctionTurn))
	}
	car.ReverseRemaining = targetDist
	car.DeadlockTimer = 0
	car.ReverseAttempts++
	if canPropagate {
		tm.requestCarYieldReverse(f, blockingCar, true)
	}
}

// A car's reverse queue can end at a fleet bus instead of another car. Ask
// that stationary bus to open space only when removing it makes the complete
// rear sweep clear. The car still waits for the real gap before backing.
func (tm *TrafficManager) requestCarRearBusReverse(f carFrame, minDist float64) {
	if f.world == nil || f.car.InRoundabout || f.car.JunctionTurn != nil || math.Abs(f.car.Speed) > carDeadlockReverseSpeed+1 {
		return
	}
	u := sim.UnitFromAngle(f.car.Heading)
	for _, b := range f.buses {
		if b == nil || math.Abs(b.Speed) > 1 || b.DoorsOpen || b.PassengerServiceRemaining() > 0 ||
			math.Cos(b.Heading-f.car.Heading) < .98 || b.Pos.Sub(f.car.Pos).Dot(u) >= -f.car.Length/2 {
			continue
		}
		trial := f
		trial.buses = nil
		for _, other := range f.buses {
			if other != b {
				trial.buses = append(trial.buses, other)
			}
		}
		if available, _, _ := tm.availableCarReverseDistanceWithBlocker(trial, carDeadlockReverseMaxDist); available >= minDist {
			if current, _, _ := tm.availableCarReverseDistanceWithBlocker(f, carDeadlockReverseMaxDist); current < available {
				b.ReverseYieldRequest = .6
			}
		}
	}
}

func (tm *TrafficManager) requestCarYieldReverse(f carFrame, target *sim.TrafficCar, fastRetry bool) {
	if target == nil || target.InRoundabout {
		return
	}
	if target.Speed >= 2.0 || target.Speed <= -carDeadlockReverseSpeed-5 {
		return
	}
	if sim.Cos(f.car.Heading-target.Heading) <= 0.85 {
		return
	}
	fwd := sim.UnitFromAngle(f.car.Heading)
	toOther := target.Pos.Sub(f.car.Pos)
	lat := math.Abs(-toOther.X*fwd.Y + toOther.Y*fwd.X)
	if lat > (f.car.Width+target.Width)*0.5+4.0 {
		return
	}
	if !finishCarTurnExitBeforeReverse(f, target) {
		return
	}

	wasActive := target.YieldReverseRequest > 0
	target.YieldReverseRequest = carDeadlockYieldRequestTTL
	if !wasActive && target.ReverseRemaining <= 0 && target.ReverseHold <= 0 {
		if fastRetry && target.ReverseAttempts > 0 {
			target.YieldReverseDelay = carDeadlockYieldRetryDelay
		} else {
			target.YieldReverseDelay = carDeadlockYieldConfirmDelay
		}
	}
}

// Turn paths include a straight exit tail beyond the junction. A car wholly
// past the last bend can finish that bookkeeping before yielding backward;
// keeping it marked as turning otherwise prevents a boxed queue from relaying
// the request. The existing rear-bumper road/junction sweep bounds the reverse.
func finishCarTurnExitBeforeReverse(f carFrame, car *sim.TrafficCar) bool {
	turn := car.JunctionTurn
	if turn == nil {
		return true
	}
	if turn.Straight && !turn.LaneChange && f.world != nil && len(turn.Points) >= 2 &&
		math.Cos(car.Heading-turn.ExitHeading) > .999 && !f.world.InsideJunction(car.Pos, f.laneWidth, car.Length/2+8) {
		// Straight plans have many collinear tail samples. Once the actual
		// rear is outside the box, an arbitrary last-sample marker must not
		// prevent this follower from relaying a checked rear-slot request.
		forward := sim.UnitFromAngle(turn.ExitHeading)
		straight := true
		for i := turn.Segment; i < len(turn.Points)-1; i++ {
			if turn.Points[i+1].Sub(turn.Points[i]).Normalize().Dot(forward) < .999 {
				straight = false
				break
			}
		}
		if straight {
			car.Heading, car.IsEW, car.RoadCenter, car.LaneIndex = turn.ExitHeading, turn.ExitEW, turn.ExitRoadCenter, turn.ExitLane
			car.JunctionTurn, car.JunctionCorridor = nil, nil
			return true
		}
	}
	if turn.LaneChange && turn.SpeedLimit == 0 && f.world != nil && carTurnHistory(turn) <= 260 && math.Cos(car.Heading-turn.ExitHeading) > .75 && !f.world.InsideJunction(car.Pos, f.laneWidth, car.Length/2+8) {
		return true
	}
	if turn.LaneChange || f.world == nil || len(turn.Points) < 2 || turn.Segment != len(turn.Points)-2 ||
		math.Cos(car.Heading-turn.ExitHeading) < .999 || f.world.InsideJunction(car.Pos, f.laneWidth, car.Length/2+8) {
		return false
	}
	u := sim.UnitFromAngle(turn.ExitHeading)
	tangent := turn.Points[len(turn.Points)-2]
	delta := car.Pos.Sub(tangent)
	if delta.Dot(u) < car.Length/2+carDeadlockYieldMinDist+8 || math.Abs(delta.X*u.Y-delta.Y*u.X) > 1 {
		return false
	}
	car.Heading, car.IsEW, car.RoadCenter, car.LaneIndex = turn.ExitHeading, turn.ExitEW, turn.ExitRoadCenter, turn.ExitLane
	car.JunctionTurn, car.JunctionCorridor = nil, nil
	return true
}

func (tm *TrafficManager) carBlockedByStoppedBus(f carFrame, honorRedQueue bool) bool {
	car := f.car
	sinH, cosH := sim.Sincos(car.Heading)
	redAhead := honorRedQueue && tm.carHasRedSignalAhead(f, 1200)

	for _, b := range f.buses {
		if b == nil || b.DoorsOpen || math.Abs(b.Speed) >= 2.0 {
			continue
		}
		mergingIntoCar := tm.busMergeCrossesCar(f, b)
		sameDir := sim.Cos(car.Heading-b.Heading) > 0.992
		straightTrailer := !b.IsArticulated() || math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) <= 0.12
		roadAlignedTrailer := straightTrailer && (!b.IsArticulated() || sim.Cos(car.Heading-b.TailHeading()) > .98)
		// A straight non-merging bus queuing ahead in the same direction is
		// not wedged with or trying to merge past this car.
		if sameDir && roadAlignedTrailer && !mergingIntoCar {
			continue
		}
		if redAhead && !mergingIntoCar && sim.Cos(car.Heading-b.Heading) > .98 && straightTrailer {
			continue
		}

		matched := false
		for _, part := range b.BodyParts() {
			if part.HalfLen <= 0 {
				continue
			}
			d := part.Center.Sub(car.Pos)
			fwd := d.X*cosH + d.Y*sinH
			lat := math.Abs(-d.X*sinH + d.Y*cosH)
			pSin, pCos := sim.Sincos(part.Heading)
			pFwdExt := math.Abs(cosH*pCos+sinH*pSin)*part.HalfLen + math.Abs(-cosH*pSin+sinH*pCos)*part.HalfWid
			pLatExt := math.Abs(-sinH*pCos+cosH*pSin)*part.HalfLen + math.Abs(sinH*pSin+cosH*pCos)*part.HalfWid
			maxLat := pLatExt + car.Width/2 + 8.0
			if fwd > 0 && fwd < car.Length/2+pFwdExt+70 && lat <= maxLat {
				matched = true
				break
			}
			if fwd <= 0 && fwd+pFwdExt > 0 && lat <= maxLat {
				matched = true
				break
			}
		}
		if !matched && mergingIntoCar && !redAhead && carInBusMergeLongitudinalZone(car, b) {
			matched = true
		}
		if !matched {
			continue
		}
		// While waiting in a red-light queue, only reverse if the stopped bus
		// is actively merging across our lane or wedged crosswise.
		if redAhead && !mergingIntoCar && sim.Cos(car.Heading-b.Heading) > .98 && straightTrailer {
			continue
		}
		return true
	}
	return false
}

func (tm *TrafficManager) busMergeCrossesCar(f carFrame, b *sim.Bus) bool {
	car := f.car
	if req := b.LaneMergeRequest; req != nil && !req.Joined && (req.Remaining > 0 || req.LeftTurn || req.RightTurn) {
		if req.Is2Lane || sim.Cos(car.Heading-req.Heading) > 0.5 {
			bFwd := sim.UnitFromAngle(req.Heading)
			bRight := sim.Vec2{X: -bFwd.Y, Y: bFwd.X}
			busLat := b.Pos.Sub(req.Target).Dot(bRight)
			carLat := car.Pos.Sub(req.Target).Dot(bRight)
			pad := b.Width/2 + car.Width/2 + 6.0
			minLat := math.Min(busLat, 0) - pad
			maxLat := math.Max(busLat, 0) + pad
			if carLat >= minLat && carLat <= maxLat {
				return true
			}
		}
	}
	if req := b.StationMergeRequest; req != nil && req.Remaining > 0 {
		if car.YieldingToBus == b {
			return true
		}
		if sim.Cos(car.Heading-b.Heading) > 0.70 {
			bFwd := sim.UnitFromAngle(b.Heading)
			bRight := sim.Vec2{X: -bFwd.Y, Y: bFwd.X}
			if math.Abs(car.Pos.Sub(b.Pos).Dot(bRight)) <= f.laneWidth*1.2+b.Width/2 {
				return true
			}
		}
	}
	return false
}

func carInBusMergeLongitudinalZone(car *sim.TrafficCar, b *sim.Bus) bool {
	carFwd := sim.UnitFromAngle(car.Heading)
	for _, part := range b.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		fwd := part.Center.Sub(car.Pos).Dot(carFwd)
		pFwdExt := math.Abs(sim.Cos(car.Heading-part.Heading))*part.HalfLen +
			math.Abs(math.Sin(car.Heading-part.Heading))*part.HalfWid
		if fwd > -pFwdExt && fwd < car.Length/2+pFwdExt+70 {
			return true
		}
	}
	return false
}

func (tm *TrafficManager) carHasRedSignalAhead(f carFrame, horizon float64) bool {
	if f.tls == nil {
		return false
	}
	car := f.car
	sinH, cosH := sim.Sincos(car.Heading)
	bumper := car.Pos.Add(sim.Vec2{X: cosH * (car.Length / 2), Y: sinH * (car.Length / 2)})
	bestForward := math.Inf(1)
	bestRed := false
	for _, inter := range f.tls.Intersections {
		stopLine := inter.GetStopLineDistForHeading(car.Heading, f.laneWidth)
		delta := inter.Center.Sub(bumper)
		forward := delta.X*cosH + delta.Y*sinH
		lateral := math.Abs(-delta.X*sinH + delta.Y*cosH)
		if forward <= stopLine || forward >= stopLine+horizon || forward >= bestForward {
			continue
		}
		if lateral >= math.Max(f.laneWidth*3.6, inter.ApproachRoadHalfWidth(car.Heading, f.laneWidth)) {
			continue
		}
		bestForward = forward
		wantLeft, wantRight := carIntersectionTurns(f, inter)
		sig := inter.GetSignalForHeading(car.Heading, wantLeft)
		bestRed = sig != sim.SignalGreen && !wantRight
	}
	return bestRed
}

func (tm *TrafficManager) safeCarReverseRoadDistance(f carFrame, maxDist float64) float64 {
	car := f.car
	if f.world == nil || maxDist <= 0 {
		return 0
	}
	if _, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos); !ok {
		return 0
	}

	fwd := sim.UnitFromAngle(car.Heading)
	back := fwd.Mul(-1)
	frontBumper := car.Pos.Add(fwd.Mul(car.Length / 2))

	const step = 4.0
	safeRoadDist := 0.0
	for d := step; d <= maxDist; d += step {
		rearBumper := car.Pos.Add(back.Mul(d + car.Length/2 + 6))
		if _, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, rearBumper); !ok {
			break
		}
		if f.world.InsideJunction(rearBumper, f.laneWidth, 8) || f.world.IsAtRoundabout(rearBumper) {
			break
		}
		if f.world.VehicleOnLevelCrossing(frontBumper, car.Heading, car.Length+d+8) {
			break
		}
		safeRoadDist = d
	}
	return safeRoadDist
}

func (tm *TrafficManager) availableCarReverseDistance(f carFrame, maxDist float64) float64 {
	avail, _, _ := tm.availableCarReverseDistanceWithBlocker(f, maxDist)
	return avail
}

func (tm *TrafficManager) availableCarReverseDistanceWithBlocker(f carFrame, maxDist float64) (float64, *sim.TrafficCar, bool) {
	car := f.car
	if car.JunctionTurn != nil && car.JunctionTurn.LaneChange {
		return tm.availablePassRetreatDistance(f, maxDist)
	}
	limit := tm.safeCarReverseRoadDistance(f, maxDist)
	if limit <= 0 {
		return 0, nil, false
	}

	fwd := sim.UnitFromAngle(car.Heading)
	back := fwd.Mul(-1)

	// Collect nearby obstacles once (cars + bus body parts) with stopping reserve
	// for approaching rear traffic, then walk d up to limit with exact OBB checks.
	const step = 4.0
	const rearGap = 12.0
	type reverseObstacle struct {
		obb sim.OBB
		car *sim.TrafficCar
	}
	var obs [16]reverseObstacle
	obstacles := obs[:0]

	addObstacle := func(otherCar *sim.TrafficCar, center sim.Vec2, heading, halfLen, halfWid, speed float64, fwdReserveScale, minReserve float64) {
		if center.Sub(car.Pos).Dot(back) <= 0 {
			return
		}
		oFwd := sim.UnitFromAngle(heading)
		reserve := 0.0
		if speed > 2.0 && oFwd.Dot(fwd) > 0.5 {
			reserve = speed*speed/(2*fwdReserveScale) + speed*minReserve
		}
		padCenter := center.Add(oFwd.Mul(reserve / 2))
		obb := sim.OBB{
			Center:  padCenter,
			Heading: heading,
			HalfLen: halfLen + reserve/2,
			HalfWid: halfWid,
		}
		if padCenter.Distance(car.Pos) > limit+car.Length+obb.HalfLen+rearGap {
			return
		}
		obstacles = append(obstacles, reverseObstacle{obb: obb, car: otherCar})
	}

	for _, other := range tm.carsForFrame(f) {
		if other == nil || other == car {
			continue
		}
		addObstacle(other, other.Pos, other.Heading, other.Length/2, other.Width/2, other.Speed, 220, 0.35)
	}
	for _, b := range f.buses {
		if b == nil {
			continue
		}
		for _, part := range b.BodyParts() {
			if part.HalfLen <= 0 {
				continue
			}
			addObstacle(nil, part.Center, part.Heading, part.HalfLen, part.HalfWid, b.Speed, 160, 0.4)
		}
	}
	if len(obstacles) == 0 {
		return limit, nil, false
	}

	safeVehicleDist := 0.0
	var blockingCar *sim.TrafficCar
	halfGap := rearGap / 2
	for d := step; d <= limit; d += step {
		carBox := sim.OBB{
			Center:  car.Pos.Add(back.Mul(d + halfGap)),
			Heading: car.Heading,
			HalfLen: car.Length/2 + halfGap,
			HalfWid: car.Width/2 + 1,
		}
		blocked := false
		for i := range obstacles {
			if hit, _, _ := sim.CheckOBBCollision(carBox, obstacles[i].obb); hit {
				blocked = true
				blockingCar = obstacles[i].car
				break
			}
		}
		if blocked {
			break
		}
		safeVehicleDist = d
	}
	rearReversing := blockingCar != nil && (blockingCar.Speed <= -1.0 || blockingCar.ReverseRemaining > 0)
	return safeVehicleDist, blockingCar, rearReversing
}
