package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// updateRoundaboutOvertaking controls entry, passing and return to the outer ring.
func (ad *AutopilotDriver) updateRoundaboutOvertaking(bus *sim.Bus, currentRb *sim.Roundabout, obstacles []VehicleObstacle, laneWidth, s, speedAbs, dt float64) {
	rbCenter := currentRb.Center
	rApron := currentRb.InnerRadius + 16.0
	rOuter := currentRb.Radius
	midR := (rApron + rOuter) * 0.5
	outerLaneR := (midR + rOuter) * 0.5
	innerLaneR := (rApron + midR) * 0.5
	passOffset := -(outerLaneR - innerLaneR)
	if math.Abs(passOffset) < laneWidth*0.5 {
		passOffset = -laneWidth
	}

	// 1. Check distance to route's exit from roundabout
	isApproachingExit := false
	seenRing := bus.Pos.Distance(rbCenter) <= currentRb.Radius+15.0
	for k := ad.WaypointIdx + 1; k < len(ad.drivePath); k++ {
		if ad.drivePath[k].Distance(rbCenter) <= currentRb.Radius+15.0 {
			seenRing = true
			continue
		}
		if seenRing {
			dToExit := ad.cumLen[k] - s
			if dToExit > 0 && dToExit < math.Max(180.0, speedAbs*1.5) {
				isApproachingExit = true
			}
			break
		}
	}

	// An empty outer lane is not a completed pass while joining the inner ring.
	if ad.joiningInnerRoundabout(currentRb, s, laneWidth) {
		if ad.TargetLaneOffset > -laneWidth*0.3 {
			ad.TargetLaneOffset = ad.LaneOffset
			ad.OvertakeState = OvertakePassing
			ad.OvertakeTimer = 0
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	if isApproachingExit {
		// Approaching exit: must return to base outer lane
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// 2. Roundabout overtaking state machine
	thetaBus := math.Atan2(bus.Pos.Y-rbCenter.Y, bus.Pos.X-rbCenter.X)

	switch ad.OvertakeState {
	case OvertakeNone:
		foundBlocker := false
		for _, obs := range obstacles {
			dObs := obs.Pos.Distance(rbCenter)
			if !(dObs >= midR-12.0 && dObs <= rOuter+25.0) {
				continue
			}
			thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
			dAngle := sim.NormalizeAngle(thetaBus - thetaObs)
			if !(dAngle > 0.05 && dAngle < math.Pi*0.65) {
				continue
			}
			arcDist := dAngle * outerLaneR
			if arcDist > 15.0 && arcDist < 260.0 && obs.Speed < apRoundaboutSpeed*0.85 {
				foundBlocker = true
				break
			}
		}

		if foundBlocker {
			// Verify inner passing lane is clear
			innerLaneClear := true
			for i := range obstacles {
				obs := &obstacles[i]
				dObs := obs.Pos.Distance(rbCenter)
				if dObs < midR+10.0 && dObs >= currentRb.InnerRadius-10.0 {
					thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
					dAngle := sim.NormalizeAngle(thetaBus - thetaObs)
					arcDist := dAngle * innerLaneR
					if arcDist > -60.0 && arcDist < 160.0 {
						innerLaneClear = false
						break
					}
				}
			}

			if innerLaneClear {
				ad.OvertakeState = OvertakeChangingLane
				ad.TargetLaneOffset = passOffset
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeChangingLane:
		ad.OvertakeTimer += dt
		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.5 {
			ad.LaneOffset = ad.TargetLaneOffset
			ad.OvertakeState = OvertakePassing
			ad.OvertakeTimer = 0.0
		}

	case OvertakePassing:
		ad.OvertakeTimer += dt
		hasOuterBlockerAhead := false
		for i := range obstacles {
			obs := &obstacles[i]
			dObs := obs.Pos.Distance(rbCenter)
			if dObs >= midR-12.0 && dObs <= rOuter+25.0 {
				thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
				dAngle := sim.NormalizeAngle(thetaBus - thetaObs)
				arcDist := dAngle * outerLaneR
				if arcDist > -20.0 && arcDist < 140.0 {
					hasOuterBlockerAhead = true
					break
				}
			}
		}

		if !hasOuterBlockerAhead && ad.OvertakeTimer > 0.5 {
			outerClear := true
			for i := range obstacles {
				obs := &obstacles[i]
				dObs := obs.Pos.Distance(rbCenter)
				if dObs >= midR-12.0 && dObs <= rOuter+25.0 {
					thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
					dAngle := sim.NormalizeAngle(thetaBus - thetaObs)
					arcDist := dAngle * outerLaneR
					if arcDist > -50.0 && arcDist < 80.0 {
						outerClear = false
						break
					}
				}
			}
			if outerClear {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeReturning:
		ad.OvertakeTimer += dt
		ad.TargetLaneOffset = 0.0
		if math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	}

	ad.progressLaneOffset(dt, speedAbs)
	return
}
