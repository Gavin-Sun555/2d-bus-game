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

		dist := f.bus.Pos.Distance(ob.Pos)
		obHalfLen := ob.Length / 2
		obHalfWid := ob.Width / 2
		if dist > egoHalfLen+obHalfLen+55.0 {
			continue
		}

		delta := ob.Pos.Sub(f.bus.Pos)
		fwdEgo := delta.Dot(uEgo)
		// Obstacle must be in front of ego within close blocking range
		if fwdEgo <= 5.0 || fwdEgo > egoHalfLen+obHalfLen+40.0 {
			continue
		}

		uOb := sim.UnitFromAngle(ob.Heading)
		fwdOb := -delta.Dot(uOb)
		// Ego must be in front of obstacle
		if fwdOb <= 5.0 || fwdOb > egoHalfLen+obHalfLen+40.0 {
			continue
		}

		// Opposing or crossing headings (not same-direction following queue)
		cosHead := uEgo.Dot(uOb)
		if cosHead > 0.80 {
			continue
		}

		// Check lateral offset along ego's heading
		latEgo := math.Abs(-delta.X*uEgo.Y + delta.Y*uEgo.X)
		if latEgo > egoHalfWid+obHalfWid+16.0 {
			continue
		}

		// Conflict found!
		hasConflict = true

		// Determine right-of-way priority:
		// Turning vehicles yield to straight vehicles ("转弯让直行").
		egoIsTurning := ad.isBusTurning(f)
		obIsTurning := isObstacleTurning(ob)

		if egoIsTurning && !obIsTurning {
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

// headonRearClear verifies that the corridor behind the bus is not blocked by
// traffic or hard barriers ("并非拥堵").
func (ad *AutopilotDriver) headonRearClear(f autopilotFrame, reverseDist float64) bool {
	if f.bus == nil || f.world == nil {
		return false
	}
	uEgo := sim.UnitFromAngle(f.bus.Heading)
	roadH := approachRoadHeading(f.bus.Heading)
	uRoad := sim.UnitFromAngle(roadH)
	rRoad := sim.Vec2{X: -uRoad.Y, Y: uRoad.X}

	egoHalfLen := f.bus.Length / 2
	egoHalfWid := f.bus.Width / 2

	// 1. Check all obstacles behind the bus
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(f.bus.Pos)
		along := delta.Dot(uEgo)
		alongRoad := delta.Dot(uRoad)
		if along < -5.0 || alongRoad < -5.0 {
			obHalfLen := ob.Length / 2
			obHalfWid := ob.Width / 2
			rearGap := -math.Min(along, alongRoad) - (egoHalfLen + obHalfLen)

			// Check lateral offset across the road and vehicle heading
			latRoad := math.Abs(delta.Dot(rRoad))

			// If obstacle is in an adjacent lane along the road, it does not block our lane
			if latRoad > egoHalfWid+obHalfWid+2.0 {
				continue
			}
			if rearGap < reverseDist+4.0 {
				return false // Rear is blocked by an obstacle in our lane
			}
		}
	}

	// 2. Check world boundaries when reversed
	for d := 5.0; d <= reverseDist+5.0; d += 5.0 {
		revPos := f.bus.Pos.Sub(uEgo.Mul(d))
		testBus := *f.bus
		testBus.Pos = revPos
		bRes := testBus.CheckBoundaries(f.world, f.laneWidth)
		if bRes.Zone == sim.ZoneHardBarrier {
			return false // Hits a hard barrier when reversing
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
		return true, yielding, 65.0
	}
	if conflict, yielding := ad.checkMergeConflict(f); conflict {
		return true, yielding, 22.0
	}
	if conflict, yielding := ad.checkStationaryBlockConflict(f); conflict {
		return true, yielding, 22.0
	}
	return false, false, 0
}

// tryHeadonDeadlockRecovery evaluates deadlock conditions and initiates reverse recovery
// when stuck head-to-head or in a merging wedge for over 8-10 seconds with clear space behind.
func (ad *AutopilotDriver) tryHeadonDeadlockRecovery(f autopilotFrame, dt float64) bool {
	hasConflict, isEgoYielding, targetDist := ad.checkDeadlockConflict(f)
	if !hasConflict {
		ad.headonDeadlockTimer = math.Max(0, ad.headonDeadlockTimer-dt*2)
		return false
	}

	ad.headonDeadlockTimer += dt

	threshold := 10.0
	if !isEgoYielding {
		threshold = 13.5
	}

	if ad.headonDeadlockTimer < threshold-1e-3 {
		return false
	}

	// Check rear clearance ("并非拥堵")
	if !ad.headonRearClear(f, targetDist) {
		return false
	}

	if ad.headonRecovery == nil {
		ad.headonRecovery = &HeadonDeadlockRecovery{
			StartPos:           f.bus.Pos,
			TargetDistance:     targetDist,
			YieldHoldRemaining: 4.0,
			ReversingDone:      false,
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

	if f.bus.DoorsOpen {
		return 0, 1, 0, false
	}

	f.bus.SetNextAccelerationScale(1)

	// Phase 1: Reversing backward
	if !r.ReversingDone {
		distMoved := f.bus.Pos.Distance(r.StartPos)
		remaining := r.TargetDistance - distMoved

		// Safety check: is rear still clear?
		if !ad.headonRearClear(f, math.Max(10.0, remaining)) {
			// Rear suddenly blocked; abort reverse and transition to yield hold
			r.ReversingDone = true
			r.YieldHoldRemaining = 4.0
			f.bus.ShiftTo(sim.GearDrive)
			return 0, 1, 0, false
		}

		// Immediate physical collision guard with rear obstacles:
		uEgo := sim.UnitFromAngle(f.bus.Heading)
		for _, ob := range f.obstacles {
			delta := ob.Pos.Sub(f.bus.Pos)
			along := delta.Dot(uEgo)
			if along < -5.0 {
				lat := math.Abs(-delta.X*uEgo.Y + delta.Y*uEgo.X)
				if lat < (f.bus.Width+ob.Width)/2+2.0 && -along < (f.bus.Length+ob.Length)/2+8.0 {
					r.ReversingDone = true
					r.YieldHoldRemaining = 4.0
					f.bus.ShiftTo(sim.GearDrive)
					return 0, 1, 0, false
				}
			}
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
			return 0, 0.3, 0, false
		}
		th := sim.Clamp((targetRevSpeed-speed)/15.0, 0.05, 0.35)
		return th, 0, 0, false
	}

	// Phase 2: Yield hold in GearDrive to let the opposing/crossing traffic pass
	f.bus.ShiftTo(sim.GearDrive)
	r.YieldHoldRemaining -= dt
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
