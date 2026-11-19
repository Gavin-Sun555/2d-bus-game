package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// HeadonDeadlockRecovery manages a gentle reverse and yield maneuver when
// two vehicles meet head-to-head (顶牛) at an intersection or narrow road,
// remaining stuck for more than 10 seconds without a rear queue blockage.
type HeadonDeadlockRecovery struct {
	StartPos           sim.Vec2
	TargetDistance     float64
	YieldHoldRemaining float64
	ReversingDone      bool
	JunctionRetreat    bool
	YieldForUTurn      bool
	AlignEntryHeading  bool
	EntryHeading       float64
	ParallelRetreat    bool
	StationRetreat     bool
	RetreatSteer       float64
	FollowerRetreat    bool
	SettleSteerFirst   bool
	SkipPreSteer       bool
}

// HeadonDeadlockTimer returns the current accumulated headon deadlock duration.
func (ad *AutopilotDriver) HeadonDeadlockTimer() float64 {
	if ad == nil {
		return 0
	}
	return ad.headonDeadlockTimer
}

// HeadonRecoveryActive reports whether the headon reverse/yield recovery is currently in progress.
func (ad *AutopilotDriver) HeadonRecoveryActive() bool {
	return ad != nil && ad.headonRecovery != nil
}

func isArticulatedAndBent(b *sim.Bus) bool {
	if b == nil || !b.IsArticulated() {
		return false
	}
	return math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > 0.08
}

// an opposing or crossing vehicle that prevents forward movement.
func (ad *AutopilotDriver) checkHeadonConflict(f autopilotFrame) (hasConflict bool, isEgoYielding bool) {
	if f.bus == nil || f.world == nil || f.bus.DoorsOpen || isArticulatedAndBent(f.bus) || len(f.obstacles) == 0 {
		return false, false
	}
	if f.speedAbs > 1.5 {
		return false, false
	}

	uEgo := sim.UnitFromAngle(f.bus.Heading)
	egoHalfLen := f.bus.Length / 2
	egoHalfWid := f.bus.Width / 2

	for _, ob := range f.obstacles {
		// Only stationary or crawl-speed obstacles
		if math.Abs(ob.Speed) > 2.0 {
			continue
		}

		obPos, obHeading, obHalfLen, obHalfWid := ob.Pos, ob.Heading, ob.Length/2, ob.Width/2
		if ob.Bus != nil && f.bus.Pos.Distance(ob.Bus.Pos) < f.bus.Pos.Distance(obPos) {
			obPos = ob.Bus.Pos
			obHeading = ob.Bus.Heading
			obHalfLen = ob.Bus.Length / 2
			obHalfWid = ob.Bus.Width / 2
		}

		dist := f.bus.Pos.Distance(obPos)
		if dist > egoHalfLen+obHalfLen+70.0 {
			continue
		}

		delta := obPos.Sub(f.bus.Pos)
		fwdEgo := delta.Dot(uEgo)
		// Obstacle must be in front of ego within close blocking range
		if fwdEgo <= 5.0 || fwdEgo > egoHalfLen+obHalfLen+56.0 {
			continue
		}

		uOb := sim.UnitFromAngle(obHeading)
		fwdOb := -delta.Dot(uOb)
		// Ego must be in front of obstacle
		if fwdOb <= 5.0 || fwdOb > egoHalfLen+obHalfLen+56.0 {
			continue
		}

		// Opposing or crossing headings (not same-direction following queue)
		cosHead := uEgo.Dot(uOb)
		if cosHead > 0.80 {
			continue
		}

		// Check lateral offset along ego's heading
		latEgo := math.Abs(-delta.X*uEgo.Y + delta.Y*uEgo.X)
		if latEgo > egoHalfWid+obHalfWid+22.0 {
			continue
		}

		// Conflict found!
		hasConflict = true

		// Determine right-of-way priority:
		// Turning vehicles yield to straight vehicles ("转弯让直行"), unless the
		// opposing vehicle is a bent articulated bus that cannot reverse safely.
		egoIsTurning := ad.isBusTurning(f)
		obIsTurning := isObstacleTurning(ob)

		if ob.Bus != nil && isArticulatedAndBent(ob.Bus) {
			isEgoYielding = true
		} else if egoIsTurning && !obIsTurning {
			isEgoYielding = true
		} else if !egoIsTurning && obIsTurning {
			isEgoYielding = false
		} else {
			// Both turning or both straight: deterministic tie-breaker based on position.
			if f.bus.Pos.X != ob.Pos.X {
				isEgoYielding = f.bus.Pos.X < ob.Pos.X
			} else {
				isEgoYielding = f.bus.Pos.Y < ob.Pos.Y
			}
		}

		return true, isEgoYielding
	}

	return false, false
}

// isBusTurning checks whether the ego bus is in the process of turning or will turn soon.
func (ad *AutopilotDriver) isBusTurning(f autopilotFrame) bool {
	roadH := approachRoadHeading(f.bus.Heading)
	yawDev := math.Abs(sim.NormalizeAngle(f.bus.Heading - roadH))
	if yawDev > 0.15 {
		return true
	}
	// Check upcoming path
	if len(ad.drivePath) > 1 {
		pAhead := ad.pointAt(f.s + 80.0)
		pFar := ad.pointAt(f.s + 120.0)
		diff := pFar.Sub(pAhead)
		if diff.Length() > 5.0 {
			pathH := math.Atan2(diff.Y, diff.X)
			if math.Abs(sim.NormalizeAngle(pathH-roadH)) > 0.35 {
				return true
			}
		}
	}
	return false
}

// isObstacleTurning checks whether the obstacle vehicle is in a turn.
func isObstacleTurning(ob VehicleObstacle) bool {
	roadH := approachRoadHeading(ob.Heading)
	yawDev := math.Abs(sim.NormalizeAngle(ob.Heading - roadH))
	return yawDev > 0.15
}

func obstacleBehindBusParts(egoParts [2]sim.OBB, uRoad sim.Vec2, ob VehicleObstacle) bool {
	obParts := predictedObstacleBodies(ob, 0)
	for _, part := range egoParts {
		if part.HalfLen <= 0 {
			continue
		}
		uPart := sim.UnitFromAngle(part.Heading)
		for _, obPart := range obParts {
			if obPart.HalfLen <= 0 {
				continue
			}
			delta := obPart.Center.Sub(part.Center)
			if delta.Dot(uPart) < -part.HalfLen*0.5 || delta.Dot(uRoad) < -part.HalfLen*0.5 {
				return true
			}
		}
	}
	return false
}

func rearCollisionImminent(f autopilotFrame, rearMargin, widthMargin, behindRatio float64) bool {
	if f.bus == nil || rearMargin <= 0 {
		return false
	}
	egoParts := f.bus.BodyParts()
	for _, ob := range f.obstacles {
		obParts := predictedObstacleBodies(ob, 0)
		for _, part := range egoParts {
			if part.HalfLen <= 0 {
				continue
			}
			uPart := sim.UnitFromAngle(part.Heading)
			rearProbe := part
			rearProbe.Center = part.Center.Sub(uPart.Mul(rearMargin / 2))
			rearProbe.HalfLen += rearMargin / 2
			rearProbe.HalfWid += widthMargin
			for _, obPart := range obParts {
				if obPart.HalfLen <= 0 {
					continue
				}
				if obPart.Center.Sub(part.Center).Dot(uPart) < -part.HalfLen*behindRatio && obbHits(rearProbe, obPart) {
					return true
				}
			}
		}
	}
	return false
}

// headonRearClear verifies that the corridor behind the bus is not blocked by
// traffic or hard barriers ("并非拥堵").
func (ad *AutopilotDriver) headonRearClear(f autopilotFrame, reverseDist float64) bool {
	if f.bus == nil || f.world == nil {
		return false
	}
	if ad.straightLaneRetreatActive() {
		return ad.laneRetreatReverseClear(f, reverseDist, 0)
	}
	uEgo := sim.UnitFromAngle(f.bus.Heading)
	roadH := approachRoadHeading(f.bus.Heading)
	uRoad := sim.UnitFromAngle(roadH)
	rRoad := sim.Vec2{X: -uRoad.Y, Y: uRoad.X}

	// 1. Check all obstacles behind any physical section of the bus (including articulated trailer)
	egoParts := f.bus.BodyParts()
	for _, ob := range f.obstacles {
		obParts := predictedObstacleBodies(ob, 0)
		for _, part := range egoParts {
			if part.HalfLen <= 0 {
				continue
			}
			uPart := sim.UnitFromAngle(part.Heading)
			for _, obPart := range obParts {
				if obPart.HalfLen <= 0 {
					continue
				}
				delta := obPart.Center.Sub(part.Center)
				along := delta.Dot(uPart)
				alongRoad := delta.Dot(uRoad)
				if along < -5.0 || alongRoad < -5.0 {
					rearGap := -math.Min(along, alongRoad) - (part.HalfLen + obPart.HalfLen)
					latRoad := math.Abs(delta.Dot(rRoad))

					// If obstacle is in an adjacent lane along the road, it does not block our lane
					if latRoad > part.HalfWid+obPart.HalfWid+2.0 {
						continue
					}
					if rearGap < reverseDist+4.0 {
						return false // Rear is blocked by an obstacle in our lane
					}
				}
			}
		}
	}

	// 2. Check world boundaries and swept rear obstacle collisions when reversed
	stepStart := math.Min(2.0, math.Max(0.5, reverseDist))
	for d := stepStart; d <= reverseDist+5.0; d += 5.0 {
		revPos := f.bus.Pos.Sub(uEgo.Mul(d))
		testBus := *f.bus
		testBus.Pos = revPos
		if testBus.HitsHardBarrier(f.world, f.laneWidth) {
			return false // Hits a hard barrier when reversing
		}
		revParts := testBus.BodyParts()
		elapsed := d / 16.0
		for _, ob := range f.obstacles {
			if !obstacleBehindBusParts(egoParts, uRoad, ob) {
				continue
			}
			if sweepBodiesCollision(revParts, ob, math.Max(0, ob.Speed)*elapsed, 1.5, 0.5) {
				return false
			}
		}
	}

	return true
}

// checkMergeConflict evaluates whether the ego bus is in a lane-change or merging
// deadlock with a vehicle in the target lane.
func (ad *AutopilotDriver) checkMergeConflict(f autopilotFrame) (hasConflict bool, isEgoYielding bool) {
	if f.bus == nil || f.world == nil || f.bus.DoorsOpen || isArticulatedAndBent(f.bus) || len(f.obstacles) == 0 {
		return false, false
	}
	if f.speedAbs > 1.5 {
		return false, false
	}
	if ad.State == AutopilotStateDocked {
		return false, false
	}

	roadH := approachRoadHeading(f.bus.Heading)
	egoYawDev := math.Abs(sim.NormalizeAngle(f.bus.Heading - roadH))
	uEgo := sim.UnitFromAngle(f.bus.Heading)
	egoHalfLen := f.bus.Length / 2
	egoHalfWid := f.bus.Width / 2

	for _, ob := range f.obstacles {
		if math.Abs(ob.Speed) > 2.0 {
			continue
		}
		dist := f.bus.Pos.Distance(ob.Pos)
		obHalfLen := ob.Length / 2
		obHalfWid := ob.Width / 2
		if dist > egoHalfLen+obHalfLen+35.0 {
			continue
		}

		uOb := sim.UnitFromAngle(ob.Heading)
		cosHead := uEgo.Dot(uOb)
		// Same general travel direction along the road (not opposing traffic)
		if cosHead < 0.60 {
			continue
		}

		delta := ob.Pos.Sub(f.bus.Pos)
		fwdEgo := delta.Dot(uEgo)
		latEgo := math.Abs(-delta.X*uEgo.Y + delta.Y*uEgo.X)

		// Overlapping longitudinally or in front-corner
		if fwdEgo < -egoHalfLen-5.0 || fwdEgo > egoHalfLen+obHalfLen+20.0 {
			continue
		}
		// Must have physical lateral overlap or pinch
		if latEgo > egoHalfWid+obHalfWid+8.0 {
			continue
		}

		obYawDev := math.Abs(sim.NormalizeAngle(ob.Heading - roadH))
		isEgoMerging := egoYawDev > 0.08 || f.bus.LaneMergeRequest != nil || (f.bus.StationMergeRequest != nil && !f.bus.StationMergeRequest.Departing) || math.Abs(ad.LaneOffset) > 8.0
		isObMerging := obYawDev > 0.08 || (ob.Bus != nil && ob.Bus.LaneMergeRequest != nil)

		if !isEgoMerging && !isObMerging && latEgo > egoHalfWid+obHalfWid+6.0 {
			// Both are straight in separated lanes without lateral pinch
			continue
		}

		// Conflict confirmed!
		hasConflict = true

		// Right of way:
		// 1. Merging/turning vehicle yields to straight vehicle in target lane ("变道让直行")
		if isEgoMerging && !isObMerging {
			isEgoYielding = true
		} else if !isEgoMerging && isObMerging {
			isEgoYielding = false
		} else {
			// Both merging or both straight:
			// Tie breaker: The vehicle with more rear clearance or obstacle further ahead yields
			if fwdEgo > 5.0 {
				isEgoYielding = true
			} else if fwdEgo < -5.0 {
				isEgoYielding = false
			} else if f.bus.Pos.X != ob.Pos.X {
				isEgoYielding = f.bus.Pos.X < ob.Pos.X
			} else {
				isEgoYielding = f.bus.Pos.Y < ob.Pos.Y
			}
		}
		return true, isEgoYielding
	}

	return false, false
}

// checkStationaryBlockConflict detects a stationary or wedged vehicle directly blocking
// the ego bus's forward path for an extended period where ordinary traffic flow is blocked
// (e.g. at an intersection entrance, roundabout entry, or angled turn).
func (ad *AutopilotDriver) checkStationaryBlockConflict(f autopilotFrame) (hasConflict bool, isEgoYielding bool) {
	if f.bus == nil || f.world == nil || f.bus.DoorsOpen || isArticulatedAndBent(f.bus) || len(f.obstacles) == 0 {
		return false, false
	}
	if f.speedAbs > 1.5 {
		return false, false
	}
	if ad.State == AutopilotStateDocked {
		return false, false
	}
	// Never reverse if waiting for a red traffic light
	if f.tls != nil && ad.trafficLightSpeed(f, 150) <= 0.5 {
		return false, false
	}

	uEgo := sim.UnitFromAngle(f.bus.Heading)
	egoHalfLen := f.bus.Length / 2
	egoHalfWid := f.bus.Width / 2
	roadH := approachRoadHeading(f.bus.Heading)
	egoYawDev := math.Abs(sim.NormalizeAngle(f.bus.Heading - roadH))

	for _, ob := range f.obstacles {
		// Only stationary or crawling vehicles
		if math.Abs(ob.Speed) > 2.0 {
			continue
		}
		dist := f.bus.Pos.Distance(ob.Pos)
		obHalfLen := ob.Length / 2
		obHalfWid := ob.Width / 2
		if dist > egoHalfLen+obHalfLen+35.0 {
			continue
		}

		delta := ob.Pos.Sub(f.bus.Pos)
		fwdEgo := delta.Dot(uEgo)
		latEgo := math.Abs(-delta.X*uEgo.Y + delta.Y*uEgo.X)

		// Must be overlapping or in close forward blocking zone
		if fwdEgo < -egoHalfLen+10.0 || fwdEgo > egoHalfLen+obHalfLen+25.0 {
			continue
		}
		if latEgo > egoHalfWid+obHalfWid+8.0 {
			continue
		}

		// Distinguish from ordinary straight-lane following queue at stop lines:
		// If both vehicles are heading in the same general direction and ego is behind obstacle,
		// it is an ordinary queue behind a leader, not a deadlock.
		cosHead := uEgo.Dot(sim.UnitFromAngle(ob.Heading))
		if cosHead > 0.70 && fwdEgo > 0 {
			continue
		}

		// Determine right-of-way priority:
		egoIsTurning := egoYawDev > 0.12 || ad.isBusTurning(f) || f.nearRoundabout
		obIsTurning := isObstacleTurning(ob)

		if egoIsTurning && !obIsTurning {
			isEgoYielding = true
		} else if !egoIsTurning && obIsTurning {
			isEgoYielding = false
		} else {
			if fwdEgo > 5.0 {
				isEgoYielding = true
			} else if fwdEgo < -5.0 {
				isEgoYielding = false
			} else if f.bus.Pos.X != ob.Pos.X {
				isEgoYielding = f.bus.Pos.X < ob.Pos.X
			} else {
				isEgoYielding = f.bus.Pos.Y < ob.Pos.Y
			}
		}
		return true, isEgoYielding
	}
	return false, false
}

// checkDeadlockConflict evaluates head-on crossing conflict, merge conflict, or stationary block conflict.
func (ad *AutopilotDriver) checkDeadlockConflict(f autopilotFrame) (hasConflict bool, isEgoYielding bool, reverseDist float64) {
	if conflict, yielding := ad.checkHeadonConflict(f); conflict {
		for _, dist := range []float64{70.0, 58.0, 46.0, 36.0} {
			if ad.headonRearClear(f, dist) {
				return true, yielding, dist
			}
		}
		return true, yielding, 70.0
	}
	if conflict, yielding := ad.checkMergeConflict(f); conflict {
		if ad.straightLaneRetreatActive() {
			// Clear the adjacent car with the powered section before retrying
			// a lane retreat. A 22px reverse leaves this same pinch alongside.
			return true, yielding, f.bus.Length + 40
		}
		return true, yielding, 22.0
	}
	if conflict, yielding := ad.checkStationaryBlockConflict(f); conflict {
		return true, yielding, 22.0
	}
	return false, false, 0
}

// roundaboutEntryWedgeRetreat detects an unheld bus wedged side-by-side with
// another stopped bus at a roundabout entrance where mutual entry-corridor
// checks block forward motion.
func (ad *AutopilotDriver) roundaboutEntryWedgeRetreat(f autopilotFrame) (float64, bool) {
	if !ad.FleetBus || f.bus == nil || f.world == nil || f.speedAbs > 1 || f.bus.DoorsOpen ||
		f.bus.RoundaboutEntry == nil || f.bus.RoundaboutEntry.Remaining <= 0 || f.bus.RoundaboutAdmissionHold != nil {
		return 0, false
	}
	entry := f.bus.RoundaboutEntry
	var rb *sim.Roundabout
	for i := range f.world.Roundabouts {
		if f.world.Roundabouts[i].Center == entry.Center {
			rb = &f.world.Roundabouts[i]
			break
		}
	}
	if rb == nil || ad.roundaboutEntryDriveSpeed(f) > 0.5 {
		return 0, false
	}
	uFwd := entry.Forward
	rFwd := sim.Vec2{X: -uFwd.Y, Y: uFwd.X}
	entryHeading := math.Atan2(uFwd.Y, uFwd.X)
	egoYaw := math.Abs(sim.NormalizeAngle(f.bus.Heading - entryHeading))
	for _, ob := range f.obstacles {
		if ob.Bus == nil || ob.Bus == f.bus || ob.Bus.DoorsOpen || math.Abs(ob.Speed) > 2.0 {
			continue
		}
		if ob.Pos.Distance(rb.Center) <= rb.Radius+math.Max(ob.Length, ob.Width)/2+12 {
			continue
		}
		if uFwd.Dot(sim.UnitFromAngle(ob.Heading)) < 0.80 {
			continue
		}
		delta := ob.Pos.Sub(f.bus.Pos)
		fwd := delta.Dot(uFwd)
		lat := math.Abs(delta.Dot(rFwd))
		if lat < (f.bus.Width+ob.Width)/2-4 || lat > 2.0*f.laneWidth {
			continue
		}
		if fwd < -(f.bus.Length+ob.Length)/2-10 || fwd > (f.bus.Length+ob.Length)/2+30 {
			continue
		}
		obYaw := math.Abs(sim.NormalizeAngle(ob.Heading - entryHeading))
		yielding := false
		if egoYaw > 0.10 && obYaw <= 0.10 {
			yielding = true
		} else if egoYaw <= 0.10 && obYaw > 0.10 {
			yielding = false
		} else {
			yielding = delta.Dot(rFwd) < 0
		}
		if ad.laneRetreatReverseClear(f, 34, 0) {
			return 34, yielding
		}
		if ad.laneRetreatReverseClear(f, 22, 0) {
			return 22, yielding
		}
		if ad.laneRetreatReverseClear(f, 14, 0) {
			return 14, yielding
		}
	}
	return 0, false
}

// tryHeadonDeadlockRecovery evaluates deadlock conditions and initiates reverse recovery
// when stuck head-to-head or in a merging wedge for over 8-10 seconds with clear space behind.
func (ad *AutopilotDriver) tryHeadonDeadlockRecovery(f autopilotFrame, dt float64) bool {
	yieldUTurn := junctionRetreatNeeded(f)
	displacedInJunction := f.bus != nil && f.bus.JunctionCorridor != nil && f.bus.JunctionCorridor.Entered &&
		f.bus.Pos.Distance(ad.pointAt(ad.LastPathS)) > f.laneWidth*1.2
	curWheelSteer := 0.0
	if f.bus != nil && f.bus.MaxSteerAngle > 0 {
		curWheelSteer = sim.Clamp(f.bus.SteeringAngle/f.bus.MaxSteerAngle, -1, 1)
	}
	junctionBlocked := ad.junctionSweptSpeed(f) <= .5 ||
		(displacedInJunction && (ad.controlSweptSpeed(f, curWheelSteer) <= .5 || (f.bus.IsArticulated() && ad.articulatedSweptSpeed(f, curWheelSteer) <= .5)))
	junctionRetreat := f.speedAbs < 1 && (yieldUTurn || ad.committedCarTurnBlocksBus(f) || displacedInJunction) && junctionBlocked
	parallelDistance := ad.parallelMergeRetreatDistance(f)
	followerSteer := 0.
	if parallelDistance == 0 {
		parallelDistance = ad.turnFollowerRetreatDistance(f)
	}
	if parallelDistance == 0 {
		parallelDistance, followerSteer = ad.laneChangeFollowerRetreatPlan(f)
	}
	if parallelDistance == 0 {
		parallelDistance, followerSteer = ad.requestedReverseYieldPlan(f)
	}
	wedgeDist, wedgeYielding := 0.0, false
	if parallelDistance == 0 {
		wedgeDist, wedgeYielding = ad.roundaboutEntryWedgeRetreat(f)
		if wedgeDist > 0 {
			parallelDistance = wedgeDist
		}
	}
	hasConflict, isEgoYielding, targetDist := ad.checkDeadlockConflict(f)
	if parallelDistance > 0 {
		hasConflict, isEgoYielding, targetDist = true, true, parallelDistance
	}
	if junctionRetreat {
		hasConflict, isEgoYielding, targetDist = true, true, 185
		if yieldUTurn {
			targetDist = 155
		}
	}
	if !hasConflict {
		ad.headonDeadlockTimer = math.Max(0, ad.headonDeadlockTimer-dt*2)
		return false
	}

	ad.headonDeadlockTimer += dt

	threshold := 10.0
	if wedgeDist > 0 && !wedgeYielding {
		threshold = 2.0
	} else if junctionRetreat || parallelDistance > 0 {
		threshold = 2
	}
	if !isEgoYielding {
		threshold = 13.5
	}

	if ad.headonDeadlockTimer < threshold-1e-3 {
		return false
	}

	// Check rear clearance ("并非拥堵")
	alignEntry := junctionRetreat && !yieldUTurn && f.bus.JunctionCorridor != nil && f.bus.JunctionCorridor.Entered
	entryHdg := 0.0
	if alignEntry {
		entryHdg = f.bus.JunctionCorridor.EntryHeading
	}
	retreatFrame := f
	presteered := *f.bus
	if followerSteer != 0 {
		presteered.SteeringAngle = followerSteer * presteered.MaxSteerAngle
		retreatFrame.bus = &presteered
	}
	if (junctionRetreat && !junctionReverseClear(f, 60, alignEntry, entryHdg)) || (!junctionRetreat && parallelDistance == 0 && !ad.headonRearClear(f, targetDist)) ||
		(!junctionRetreat && parallelDistance > 0 && !ad.laneRetreatReverseClear(retreatFrame, targetDist, followerSteer)) {
		return false
	}

	if ad.headonRecovery == nil {
		ad.headonRecovery = &HeadonDeadlockRecovery{
			StartPos:           f.bus.Pos,
			TargetDistance:     targetDist,
			YieldHoldRemaining: 4.0,
			ReversingDone:      false,
			JunctionRetreat:    junctionRetreat,
			YieldForUTurn:      yieldUTurn,
			AlignEntryHeading:  alignEntry,
			EntryHeading:       entryHdg,
			ParallelRetreat:    parallelDistance > 0 && !junctionRetreat && followerSteer == 0,
			FollowerRetreat:    followerSteer != 0 && !junctionRetreat,
			RetreatSteer:       followerSteer,
		}
	}
	return true
}

// headonRecoveryControls outputs driving controls for the reverse escape and subsequent yield hold.
func (ad *AutopilotDriver) headonRecoveryControls(f autopilotFrame, dt float64) (throttle, brake, steer float64, done bool) {
	r := ad.headonRecovery
	if r == nil || f.bus == nil {
		return 0, 1, 0, true
	}
	if (r.ParallelRetreat || r.FollowerRetreat) && f.speedAbs < 1 {
		left := math.Max(0, r.TargetDistance-f.bus.Pos.Distance(r.StartPos))
		if left > 1 {
			trial, requests := rearReverseYieldFrame(f)
			if !requests.empty() && ad.laneRetreatReverseClear(trial, math.Max(10, left), r.RetreatSteer) {
				requests.send()
			}
		}
	}

	if f.bus.DoorsOpen {
		return 0, 1, 0, false
	}

	f.bus.SetNextAccelerationScale(1)

	// Phase 1: Reversing backward
	if !r.ReversingDone {
		distMoved := f.bus.Pos.Distance(r.StartPos)
		remaining := r.TargetDistance - distMoved

		if r.JunctionRetreat {
			if remaining <= 1 || !junctionReverseClear(f, 30, r.AlignEntryHeading, r.EntryHeading) {
				r.ReversingDone = true
				f.bus.ShiftTo(sim.GearDrive)
				st := 0.0
				if !r.YieldForUTurn {
					st, _, _, _ = ad.pursuitSteering(f)
				}
				return 0, 1, st, false
			}
			f.bus.ShiftTo(sim.GearReverse)
			ad.LastTargetSpeed = 16
			th, br, st := junctionReverseControls(f.bus, r.AlignEntryHeading, r.EntryHeading)
			return th, br, st, false
		}

		// Safety check: is rear still clear?
		steering := 0.
		if r.StationRetreat || r.FollowerRetreat {
			steering = r.RetreatSteer
		}
		if (r.FollowerRetreat || r.SettleSteerFirst) && f.speedAbs < .5 && math.Abs(f.bus.SteeringAngle-steering*f.bus.MaxSteerAngle) > .02 {
			// The plan was proved from stationary, settled wheels. Reposition
			// them under the brake before checking and driving that reverse.
			f.bus.ShiftTo(sim.GearReverse)
			return 0, 1, steering, false
		}
		if (r.ParallelRetreat || (r.StationRetreat && steering == 0)) && math.Abs(f.bus.Speed) < .5 && math.Abs(f.bus.SteeringAngle) > .02 {
			f.bus.ShiftTo(sim.GearReverse)
			return 0, 1, 0, false
		}
		sweptRetreat := r.ParallelRetreat || r.StationRetreat || r.FollowerRetreat
		checkDistance := math.Max(10, remaining)
		if sweptRetreat {
			// The trajectory checker already includes 5px of stopping room.
			// Requiring another minimum 10px after a short retreat has begun
			// aborts a proved 8px maneuver before it can open the gap.
			checkDistance = math.Max(0, remaining)
		}
		if (sweptRetreat && !ad.laneRetreatReverseClear(f, checkDistance, steering)) ||
			(!sweptRetreat && !ad.headonRearClear(f, math.Max(10, remaining))) {
			// Rear suddenly blocked; abort reverse and transition to yield hold
			r.ReversingDone = true
			r.YieldHoldRemaining = 4.0
			f.bus.ShiftTo(sim.GearDrive)
			return 0, 1, 0, false
		}

		// Immediate physical collision guard with rear obstacles across all body parts:
		rearGuardMargin, rearWidthMargin, behindRatio := 8.0, 0.5, 0.5
		if ad.straightLaneRetreatActive() || sweptRetreat {
			rearGuardMargin, rearWidthMargin, behindRatio = 2.0, 0.1, 1.0
		}
		if rearCollisionImminent(f, rearGuardMargin, rearWidthMargin, behindRatio) {
			r.ReversingDone = true
			r.YieldHoldRemaining = 4.0
			f.bus.ShiftTo(sim.GearDrive)
			return 0, 1, 0, false
		}

		if remaining <= 1.0 {
			r.ReversingDone = true
			r.YieldHoldRemaining = 4.0
			f.bus.ShiftTo(sim.GearDrive)
			return 0, 1, 0, false
		}

		// Shift to reverse
		f.bus.ShiftTo(sim.GearReverse)
		speed := math.Abs(f.bus.Speed)
		targetRevSpeed := math.Min(16.0, math.Sqrt(2*80*math.Max(0, remaining-1.0)))
		ad.LastTargetSpeed = targetRevSpeed

		if speed > targetRevSpeed {
			return 0, 0.3, steering, false
		}
		th := sim.Clamp((targetRevSpeed-speed)/15.0, 0.05, 0.35)
		return th, 0, steering, false
	}

	// Phase 2: Yield hold in GearDrive to let the opposing/crossing traffic pass
	f.bus.ShiftTo(sim.GearDrive)
	r.YieldHoldRemaining -= dt
	if r.JunctionRetreat && !r.YieldForUTurn {
		preSteer, _, _, _ := ad.pursuitSteering(f)
		ad.LastTargetSpeed = 0
		if math.Abs(f.bus.Speed) < 1 {
			ad.headonRecovery = nil
			ad.headonDeadlockTimer = 0
			ad.pathSynced = false
			return 0, 1, preSteer, true
		}
		return 0, 1, preSteer, false
	}
	if r.StationRetreat {
		if math.Abs(f.bus.Speed) >= 1 || (!r.SkipPreSteer && f.bus.SteeringAngle > -f.bus.MaxSteerAngle+.03) {
			preSteer := 0.
			if !r.SkipPreSteer {
				preSteer = -1
			}
			return 0, 1, preSteer, false
		}
		ad.headonRecovery = nil
		ad.headonDeadlockTimer = 0
		ad.WaypointIdx = 0
		ad.pathSynced = false
		return 0, 1, 0, true
	}
	ad.LastTargetSpeed = 0

	hasConflict, _, _ := ad.checkDeadlockConflict(f)
	if r.YieldHoldRemaining <= 0 {
		if !hasConflict || r.YieldHoldRemaining <= -3.0 {
			ad.headonRecovery = nil
			ad.headonDeadlockTimer = 0
			return 0, 1, 0, true
		}
	}

	return 0, 1, 0, false
}
