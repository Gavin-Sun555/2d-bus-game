package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// targetSpeed applies corner, recovery, station, signal and obstacle constraints
// in that order. Keep this ordering: passing floors must not raise an earlier cap.
func (ad *AutopilotDriver) targetSpeed(f autopilotFrame) float64 {
	targetSpd := apCruiseSpeed
	if ad.CruiseSpeedLimit > 0 {
		targetSpd = ad.CruiseSpeedLimit
	}
	isApproachingSharpTurn := false
	for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
		d := ad.cumLen[k] - f.s
		if d < -10.0 {
			continue
		}
		if d > 280.0 {
			break
		}
		segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
		segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
		turnAngle := math.Abs(sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
		if turnAngle > 0.6 {
			isApproachingSharpTurn = true
			break
		}
	}

	if ad.BerserkMode {
		if !isApproachingSharpTurn {
			// On straight roadways & mid-block lane changes in Berserk mode, maintain full cruise speed
			targetSpd = ad.GetBerserkCruiseSpeed()
		} else {
			targetSpd = math.Max(apCruiseSpeed, ad.cornerSpeedLimit(f.s, f.speedAbs))
		}
	}

	// A. Anticipate corners / roundabout arcs ahead, and slow while circulating
	targetSpd = math.Min(targetSpd, ad.cornerSpeedLimit(f.s, f.speedAbs))
	if f.nearRoundabout {
		rbSpeed := apRoundaboutSpeed
		if ad.BerserkMode {
			rbSpeed = 120.0
		}
		targetSpd = math.Min(targetSpd, rbSpeed)
	}

	if ad.uTurn != nil && ad.uTurn.entered {
		targetSpd = math.Min(targetSpd, 70)
	}
	targetSpd = ad.recoverySpeed(f, targetSpd, isApproachingSharpTurn)
	targetSpd = ad.stationSpeed(f, targetSpd)
	targetSpd = ad.trafficLightSpeed(f, targetSpd)
	if f.bus.YieldingToBus != nil {
		if limit, ok := sim.BusMergeYieldLimit(f.bus, f.bus.YieldingToBus, f.laneWidth, true); ok {
			targetSpd = math.Min(targetSpd, limit)
		}
	}
	targetSpd = math.Min(targetSpd, ad.junctionAdmissionSpeed(f))
	if ad.approachMerge != nil {
		mergeSpeed := 80.0
		if ad.approachMergeJoined {
			mergeSpeed = apCruiseSpeed
			if ad.BerserkMode {
				mergeSpeed = ad.GetBerserkCruiseSpeed()
			}
		}
		targetSpd = math.Min(targetSpd, mergeSpeed)
		targetSpd = math.Min(targetSpd, ad.requiredMergeTrafficSpeed(f))
	}

	// Fail-safe: A vehicle borrowing or occupying the opposing lane on a 2-lane road
	// must NEVER enter an intersection from the opposing lane!
	// If approaching the stop line and not yet merged back, force stop before the line.
	if f.world != nil && len(ad.drivePath) >= 2 {
		_, _, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
		if !multi && !six && (ad.LaneOffset < -0.5*f.laneWidth || ad.TargetLaneOffset < -0.5*f.laneWidth) {
			approach := ad.approachingIntersection(f, multi, six)
			if approach.approachingInter != nil && approach.distToStopLine > -f.bus.Length/2 && approach.distToStopLine < 10.0 {
				targetSpd = 0.0
			}
		}
	}
	targetSpd = ad.obstacleSpeed(f, targetSpd)
	targetSpd = math.Min(targetSpd, ad.junctionSweptSpeed(f))

	if ad.CruiseSpeedLimit > 0 {
		targetSpd = math.Min(targetSpd, ad.CruiseSpeedLimit)
	}
	if ad.roadRecoverySpeed > 0 {
		targetSpd = math.Min(targetSpd, ad.roadRecoverySpeed)
	}
	if ad.roadSafetyHold {
		return 0
	}
	if ad.midLaneRecovery != nil {
		targetSpd = math.Min(targetSpd, 20)
		if !ad.midLaneSweepClear {
			targetSpd = 0
		}
	}

	return targetSpd
}

// longitudinalControls converts the chosen speed to pedals, with a coasting hold band.
func (ad *AutopilotDriver) longitudinalControls(speed, targetSpd float64) (throttle, brake float64) {
	speedAbs := math.Abs(speed)
	// F. Acceleration & Foot Braking
	switch {
	case targetSpd <= 0.5:
		throttle = 0.0
		if speedAbs < 10.0 {
			brake = 1.0
		} else {
			brake = sim.Clamp(speedAbs/50.0, 0.5, 1.0)
		}
	case speed < targetSpd-2.5:
		if ad.BerserkMode {
			throttle = sim.Clamp((targetSpd-speed)/22.0, 0.45, 1.0)
		} else {
			throttle = sim.Clamp((targetSpd-speed)/35.0, 0.3, 1.0)
		}
		brake = 0.0
	case speed > targetSpd+3.5:
		throttle = 0.0
		if ad.BerserkMode {
			brake = sim.Clamp((speed-targetSpd)/25.0, 0.2, 1.0)
		} else {
			brake = sim.Clamp((speed-targetSpd)/40.0, 0.08, 1.0)
		}
	case speed < targetSpd:
		throttle = 0.15
		brake = 0.0
	default:
		throttle = 0.0 // coast (rolling resistance) inside the hold band
		brake = 0.0
	}

	return throttle, brake
}

// recoverySpeed limits speed only for unintended tracking error; an active lane
// change is intentional and only a severe heading error should slow it down.
func (ad *AutopilotDriver) recoverySpeed(f autopilotFrame, targetSpd float64, sharpTurn bool) float64 {
	if ad.OvertakeState != OvertakeNone {
		if math.Abs(f.alpha) > 1.1 {
			return math.Min(targetSpd, 70)
		}
		return targetSpd
	}

	tangent := ad.pointAt(f.s + 25).Sub(ad.pointAt(f.s))
	headingErr := 0.0
	if tangent.Length() > 0.5 {
		headingErr = math.Abs(sim.NormalizeAngle(math.Atan2(tangent.Y, tangent.X) - f.bus.Heading))
	}

	severeAngle, mildAngle, lateralLimit := 0.5, 0.25, 40.0
	severeSpeed, mildSpeed := 38.0, 48.0
	if ad.BerserkMode {
		severeAngle, mildAngle, lateralLimit = 0.75, 0.35, 55
		severeSpeed, mildSpeed = 100, 130
		if sharpTurn {
			severeAngle, lateralLimit = 0.70, 50
			severeSpeed, mildSpeed = 75, 85
		}
	}

	switch {
	case math.Abs(f.alpha) > severeAngle || headingErr > severeAngle || f.lateralErr > lateralLimit:
		return math.Min(targetSpd, severeSpeed)
	case math.Abs(f.alpha) > mildAngle || headingErr > mildAngle:
		return math.Min(targetSpd, mildSpeed)
	default:
		return targetSpd
	}
}
