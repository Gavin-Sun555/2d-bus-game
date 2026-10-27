package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// dockedControls advances the door/dwell cycle. Route advancement happens only
// after dwell has finished and the doors have fully closed.
func (ad *AutopilotDriver) dockedControls(bus *sim.Bus, route *sim.Route, speedAbs, dt float64) (throttle, brake, steer float64, handbrake, toggleDoors, isDocked bool) {
	if bus.TerminalReplacementHold {
		return 0, 1, 0, false, !bus.DoorsOpen, true
	}
	if !ad.DwellFinished {
		switch {
		case !bus.DoorsOpen && bus.DoorProgress <= 0.05:
			// Open only below crawl speed; otherwise continue braking.
			return 0, 1, 0, false, speedAbs < 15.0, true
		case bus.DoorsOpen && ad.DwellTimer > 0:
			// Finish this dwell tick before considering door closure next tick.
			ad.DwellTimer -= dt
			return 0, 1, 0, false, false, true
		case bus.DoorsOpen && ad.DwellTimer <= 0 && bus.DoorProgress >= 0.80:
			ad.DwellFinished = true
			return 0, 1, 0, false, true, true
		}
	}

	if !(ad.DwellFinished && !bus.DoorsOpen && bus.DoorProgress <= 0.05) {
		return 0, 1, 0, false, false, true
	}
	if route.GetCurrentTargetStopID() == ad.DockedStopID {
		route.AdvanceToNextStop()
	}
	roadDeparture := false
	if ad.stationDock != nil && ad.stationDock.rank > 0 && ad.pathWorld != nil && ad.DockedStopID >= 0 && ad.DockedStopID < len(ad.pathWorld.Stops) {
		stop, laneWidth := ad.pathWorld.Stops[ad.DockedStopID], ad.pathLane
		ad.beginRoadDeparture(bus, stop, laneWidth)
		ad.skippedDeparture.served = true
		roadDeparture = true
	}
	ad.LastTargetID = route.GetCurrentTargetStopID()
	ad.departureFromDock = !roadDeparture
	ad.stationDock = nil
	ad.State = AutopilotStateDeparting
	if roadDeparture {
		// Already in a road lane: allow the normal safe passing policy after
		// closing the doors rather than holding behind an occupied front berth.
		ad.State = AutopilotStateCruising
	}
	ad.DockedStopID = -1
	ad.WaypointIdx = 0
	ad.pathSynced = false
	ad.DwellFinished = false
	return 1, 0, 0, false, false, false
}

func (ad *AutopilotDriver) stationSpeed(f autopilotFrame, targetSpd float64) float64 {
	// C. Station approach: brake along a constant-deceleration profile to stop at the bay center
	if ad.CanSkipTargetStop {
		return targetSpd
	}
	stopDecel := apStopDecel
	if ad.BerserkMode {
		stopDecel = 195.0 // Strong crisp deceleration in Berserk mode
	}
	stopSpd := math.Sqrt(2 * stopDecel * math.Max(0, f.remaining-2.0))
	isBerserkOuterDock := ad.BerserkMode && f.targetStop.ContainsBusInOuterLane(f.bus, f.laneWidth)
	isStuckInOuterLane := !isBerserkOuterDock && f.remaining < 60.0 && f.distToStop < 250.0 && ((f.distToStop > 45.0 && !f.targetStop.ContainsBus(f.bus)) || math.Abs(ad.LaneOffset) > 15.0 || math.Abs(ad.TargetLaneOffset) > 15.0)
	if f.remaining > 2.0 || isStuckInOuterLane {
		creepFloor := math.Min(16.0, math.Max(f.remaining, f.distToStop)*1.5)
		if ad.BerserkMode {
			creepFloor = math.Min(22.0, math.Max(f.remaining, f.distToStop)*1.8)
		}
		creepFloor = math.Max(creepFloor, 14.0)
		if isStuckInOuterLane && f.remaining <= 20.0 {
			// Bus reached station in outer lane (distToStop > 65px): do not stall at 0 km/h; maintain floor cruise so it flies past cleanly
			creepFloor = math.Max(creepFloor, 24.0)
		}
		stopSpd = math.Max(stopSpd, creepFloor)
	} else {
		stopSpd = 0.0
	}
	targetSpd = math.Min(targetSpd, stopSpd)

	// Point 3: If approaching destination stop while offset in outer lane, brake early to wait and merge into station lane
	if f.remaining < 500.0 && (math.Abs(ad.LaneOffset) > 5.0 || math.Abs(ad.TargetLaneOffset) > 5.0) {
		entrySpd := math.Max(50.0, math.Sqrt(2*120.0*math.Max(0, f.remaining-30.0)))
		targetSpd = math.Min(targetSpd, entrySpd)
	}

	// Station approach speed control when waiting alongside in adjacent lane:
	if f.remaining < 350.0 && (ad.LaneOffset < -f.laneWidth*0.3 || ad.TargetLaneOffset < -f.laneWidth*0.3) {
		sinH, cosH := math.Sincos(f.bus.Heading)
		hasAhead, hasSide, _, leadObs := checkStationApproachLeadVehicle(
			f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, f.targetStop,
		)
		if (hasAhead || hasSide) && leadObs != nil && f.targetStop != nil && math.Cos(leadObs.Heading-f.targetStop.Heading) > 0.4 {
			dBay := leadObs.Pos.Distance(f.targetStop.Pos)
			if dBay < (f.targetStop.Length*0.6 + leadObs.Length*0.5 + 35.0) {
				dObs := leadObs.Pos.Sub(f.bus.Pos)
				fwdObs := dObs.X*cosH + dObs.Y*sinH
				leadSpd := math.Max(0.0, leadObs.Speed)
				if fwdObs < 35.0 {
					targetSpd = math.Min(targetSpd, leadSpd)
				}
				if leadSpd < 5.0 && fwdObs > -25.0 {
					targetSpd = 0.0
				}
			}
		}
	}

	return targetSpd
}

// updateStationApproachLane decides whether to merge into the bay or wait beside it.
func (ad *AutopilotDriver) updateStationApproachLane(bus *sim.Bus, targetStop *sim.BusStop, obstacles []VehicleObstacle, laneWidth, speedAbs, remaining, dt float64, isMultiLane, is6Lane bool) {
	sinH, cosH := math.Sincos(bus.Heading)
	hasLeadAhead, hasLeadAlongside, hasPassedLead, leadObs := checkStationApproachLeadVehicle(
		bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, targetStop,
	)

	// Case A: Bus is in the base curb lane (OvertakeNone)
	if ad.OvertakeState == OvertakeNone && math.Abs(ad.LaneOffset) < 1.0 && (hasLeadAhead || hasLeadAlongside) {
		minOvertakeRunway := math.Max(460.0, speedAbs*2.2+160.0)
		leadDistToStop := 9999.0
		if targetStop != nil && leadObs != nil {
			leadDistToStop = leadObs.Pos.Distance(targetStop.Pos)
		}
		passClear := isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth)
		isLeadLargeBus := leadObs != nil && (leadObs.Length > 65.0 || leadDistToStop < 350.0)
		hasSpaceToOvertake := remaining >= minOvertakeRunway && leadDistToStop >= 320.0 && passClear && (isMultiLane || is6Lane) && !isLeadLargeBus

		if hasSpaceToOvertake {
			targetOffset := -laneWidth
			if is6Lane && ad.LaneOffset <= -laneWidth*0.5 {
				targetOffset = -2.0 * laneWidth
			}
			ad.TargetLaneOffset = targetOffset
			ad.OvertakeState = OvertakeChangingLane
			ad.OvertakeTimer = 0.0
		} else {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Case B: Bus is in the adjacent/passing lane
	if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
		baseClear := isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, targetStop)
		isBayOccupied := false
		if targetStop != nil {
			for i := range obstacles {
				d := obstacles[i].Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				dBay := obstacles[i].Pos.Distance(targetStop.Pos)
				if fwd > -20.0 && math.Cos(obstacles[i].Heading-targetStop.Heading) > 0.4 &&
					dBay < (targetStop.Length*0.6+obstacles[i].Length*0.5+35.0) {
					isBayOccupied = true
					break
				}
			}
		}

		canMerge := remaining >= 70.0 || (speedAbs < 25.0 && remaining >= 25.0)
		if isBayOccupied {
			canMerge = hasPassedLead && remaining >= 90.0 && !hasLeadAhead
		}
		if canMerge && !hasLeadAlongside && baseClear {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		} else {
			ad.TargetLaneOffset = -laneWidth
			if is6Lane && ad.LaneOffset <= -laneWidth*1.5 {
				ad.TargetLaneOffset = -2.0 * laneWidth
			}
			ad.OvertakeState = OvertakePassing
		}
	}

	if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
		ad.LaneOffset = 0.0
		ad.TargetLaneOffset = 0.0
		ad.OvertakeState = OvertakeNone
		ad.OvertakeTimer = 0.0
	}
	ad.progressLaneOffset(dt, speedAbs)
}

func (ad *AutopilotDriver) updateDrivingState(bus *sim.Bus, prevStop *sim.BusStop, remaining, speedAbs float64) {
	if ad.CanSkipTargetStop {
		if ad.State != AutopilotStateDocked {
			ad.State = AutopilotStateCruising
		}
		return
	}
	approachThreshold := apApproachDist
	if ad.BerserkMode {
		approachThreshold = math.Max(apApproachDist*0.65, speedAbs*speedAbs/(2*195.0)+35.0)
	}
	if remaining < approachThreshold {
		ad.State = AutopilotStateApproaching
		return
	}
	if ad.State == AutopilotStateDeparting && !(bus.Pos.Distance(prevStop.Pos) > 120.0) {
		return
	}
	ad.State = AutopilotStateCruising
}
