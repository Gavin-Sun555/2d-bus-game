package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) progressLaneOffset(dt, speedAbs float64) {
	targetDiff := math.Abs(ad.TargetLaneOffset - ad.lastTargetLaneOffset)
	if targetDiff > 0.0 {
		// Only restart the transition curve if the target changed significantly (> 2.0px)
		// or if we were already done with the previous transition (LaneOffsetProgress >= 1.0).
		// Continuous dynamic adjustments (such as tracking a tapering lane into an intersection)
		// must preserve transition progress so the vehicle does not stall in its lane!
		if targetDiff > 2.0 || ad.LaneOffsetProgress >= 1.0 {
			ad.lastTargetLaneOffset = ad.TargetLaneOffset
			ad.LaneOffsetStart = ad.LaneOffset
			ad.LaneOffsetProgress = 0.0
			totalDist := math.Abs(ad.TargetLaneOffset - ad.LaneOffsetStart)
			if totalDist < 0.1 {
				ad.LaneOffset = ad.TargetLaneOffset
				ad.LaneOffsetProgress = 1.0
				return
			}
			latSpeed := sim.Clamp(speedAbs*0.32, 65.0, 95.0)
			minDuration := 0.65
			maxDuration := 1.0
			if ad.BerserkMode {
				latSpeed = sim.Clamp(speedAbs*0.50, 95.0, 160.0)
				minDuration = 0.35
				maxDuration = 0.55
			}
			if !ad.BerserkMode && ad.TargetLaneOffset < ad.LaneOffsetStart && ad.TargetLaneOffset <= -80.0 {
				// Shifting left towards innermost left-turn lane on 6-lane road in standard mode (right next to double yellow line):
				// Must transition smoothly with generous duration so yaw angle stays small
				latSpeed = sim.Clamp(speedAbs*0.22, 35.0, 60.0)
				minDuration = 0.95
				maxDuration = 1.45
			}
			ad.LaneOffsetDuration = sim.Clamp(totalDist/latSpeed, minDuration, maxDuration)
		} else {
			ad.lastTargetLaneOffset = ad.TargetLaneOffset
		}
	}

	if ad.LaneOffsetProgress < 1.0 {
		if ad.LaneOffsetDuration <= 1e-4 {
			ad.LaneOffsetDuration = 0.7
		}
		ad.LaneOffsetProgress += dt / ad.LaneOffsetDuration
		if ad.LaneOffsetProgress >= 1.0 {
			ad.LaneOffsetProgress = 1.0
			ad.LaneOffset = ad.TargetLaneOffset
		} else {
			u := ad.LaneOffsetProgress
			// Ken Perlin's quintic smootherstep: 6u^5 - 15u^4 + 10u^3 (zero 1st & 2nd derivative at ends)
			w := u * u * u * (u*(u*6.0-15.0) + 10.0)
			ad.LaneOffset = ad.LaneOffsetStart + (ad.TargetLaneOffset-ad.LaneOffsetStart)*w
		}
	} else if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) > 0.05 {
		// Fallback for direct manual/test manipulation of LaneOffset
		ad.LaneOffsetStart = ad.LaneOffset
		ad.LaneOffsetProgress = 0.0
		totalDist := math.Abs(ad.TargetLaneOffset - ad.LaneOffsetStart)
		latSpeed := sim.Clamp(speedAbs*0.32, 65.0, 95.0)
		ad.LaneOffsetDuration = sim.Clamp(totalDist/latSpeed, 0.65, 1.0)
	}
}

// evaluate6LaneTraffic evaluates traffic across all 3 lanes of a 6-lane road in the driving direction:
// 1. Leftmost lane (offset -2*lw): passing / fast lane
// 2. Middle lane (offset -lw): center through lane
// 3. Rightmost lane (offset 0.0): outer through / slow lane
func evaluate6LaneTraffic(bus *sim.Bus, obstacles []VehicleObstacle, cosH, sinH, laneWidth, currentLaneOffset float64, berserkCruiseSpeedOptional ...float64) (leftScore, midScore, rightScore float64, leftBlocked, midBlocked, rightBlocked bool) {
	berserkCruise := apBerserkCruiseSpeed
	if len(berserkCruiseSpeedOptional) > 0 && berserkCruiseSpeedOptional[0] > 50.0 {
		berserkCruise = berserkCruiseSpeedOptional[0]
	}
	corridor := bus.Width/2 + 10.0

	// Dynamic lookahead horizon based on cruise speed
	scanHorizon := math.Max(600.0, berserkCruise*1.6+100.0)

	for _, obs := range obstacles {
		delta := obs.Pos.Sub(bus.Pos)
		fwd := delta.X*cosH + delta.Y*sinH
		lat := -delta.X*sinH + delta.Y*cosH
		latPath := lat + currentLaneOffset
		corridorWithObs := corridor + obs.Width/2

		closingSpeed := math.Max(0.0, berserkCruise-obs.Speed)
		dynAdvance := closingSpeed*2.2 + (bus.Length/2 + obs.Length/2 + 50.0)
		blockedLookahead := math.Max(380.0, dynAdvance)

		// 1. Right lane (offset 0.0)
		distRight := math.Abs(latPath - 0.0)
		if distRight < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				rightBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0 // Stopped at red light or in traffic queue
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5 // Significantly slower than cruise
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				rightScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					rightBlocked = true
				}
			}
		}

		// 2. Middle lane (offset -laneWidth)
		distMid := math.Abs(latPath - (-laneWidth))
		if distMid < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				midBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				midScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					midBlocked = true
				}
			}
		}

		// 3. Leftmost lane (offset -2.0*laneWidth)
		distLeft := math.Abs(latPath - (-2.0 * laneWidth))
		if distLeft < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				leftBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				leftScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					leftBlocked = true
				}
			}
		}
	}
	return
}

// evaluate6LaneThroughTraffic analyzes vehicles ahead in the two through lanes of a 6-lane road
// for backward compatibility with existing 2-lane intersection approach callers.
func evaluate6LaneThroughTraffic(bus *sim.Bus, obstacles []VehicleObstacle, cosH, sinH, laneWidth, currentLaneOffset float64, berserkCruiseSpeedOptional ...float64) (innerScore, outerScore float64, innerBlocked, outerBlocked bool) {
	_, midScore, rightScore, _, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, currentLaneOffset, berserkCruiseSpeedOptional...)
	return midScore, rightScore, midBlocked, rightBlocked
}

// isLaneClearToChange checks whether a target lane offset is clear of vehicles alongside, ahead within safety buffer,
// or approaching fast from behind.
func isLaneClearToChange(bus *sim.Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset, targetOffset, laneWidth float64) bool {
	corridorBase := bus.Width/2 + 10.0
	body := bus.BodyEnvelope()
	forward := sim.Vec2{X: cosH, Y: sinH}
	extent := body.HalfLen*math.Abs(math.Cos(body.Heading-math.Atan2(sinH, cosH))) + body.HalfWid*math.Abs(math.Sin(body.Heading-math.Atan2(sinH, cosH)))
	shift := body.Center.Sub(bus.Pos).Dot(forward)
	ownFront, ownRear := extent+shift, extent-shift
	for _, other := range obstacles {
		if yieldedVehicleClearsRear(bus, other, cosH, sinH) {
			continue
		}
		dOther := other.Pos.Sub(bus.Pos)
		fwdOther := dOther.X*cosH + dOther.Y*sinH
		latOther := -dOther.X*sinH + dOther.Y*cosH
		latPathOther := latOther + currentOffset
		distToTarget := math.Abs(latPathOther - targetOffset)
		corridor := corridorBase + other.Width/2
		if distToTarget < corridor {
			minSafeBehind := -(ownRear + other.Length/2 + 70.0)
			minSafeAhead := (ownFront + other.Length/2 + 75.0)
			closingAhead := math.Max(minSafeAhead, (bus.Speed-other.Speed)*1.6+65.0)
			if other.Speed < 20.0 {
				closingAhead = math.Max(closingAhead, ownFront+other.Length/2+200.0)
			}
			if fwdOther > minSafeBehind && fwdOther < closingAhead {
				return false
			}
			if fwdOther <= minSafeBehind && fwdOther > -280.0 && other.Speed > bus.Speed+12.0 {
				return false
			}
		}

		// Also check intermediate lane if this is a multi-lane change
		if math.Abs(targetOffset-currentOffset) > laneWidth*1.3 {
			midOffset := (currentOffset + targetOffset) * 0.5
			distToMid := math.Abs(latPathOther - midOffset)
			if distToMid < corridor {
				alongsideBehind := -(ownRear + other.Length/2 + 25.0)
				alongsideAhead := (ownFront + other.Length/2 + 35.0)
				if fwdOther > alongsideBehind && fwdOther < alongsideAhead {
					return false
				}
			}
		}
	}
	return true
}

// isOncomingLaneClear checks whether the oncoming lane on a 2-way 2-lane road
// (typically at targetOffset = -laneWidth) is completely free of oncoming traffic
// and any obstacles within safe sight distance before borrowing the lane to overtake.
func isOncomingLaneClear(bus *sim.Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset, targetOffset, laneWidth, speedAbs, cruiseSpeed float64) bool {
	passingSpeed := math.Max(speedAbs, cruiseSpeed)
	corridorBase := bus.Width/2 + 12.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToTarget := math.Abs(latPath - targetOffset)
		corridor := corridorBase + obs.Width/2
		if distToTarget < corridor {
			cosDiff := math.Cos(obs.Heading - bus.Heading)
			// 1. Vehicle is alongside the bus in the target oncoming lane
			sideDist := bus.Length/2 + obs.Length/2 + 40.0
			if fwd > -sideDist && fwd < sideDist {
				return false
			}

			// 2. Oncoming vehicle heading towards the bus (closing speed = bus.Speed + obs.Speed)
			if cosDiff < -0.2 {
				// Must have ample clear sight distance ahead
				// Reserve time for acceleration, the pass and a safe return; use
				// actual opposing speed rather than treating it as a fixed 200.
				reqClearDist := math.Max(700.0, (passingSpeed+math.Abs(obs.Speed))*4.5)
				if fwd >= -40.0 && fwd < reqClearDist {
					return false
				}
			} else {
				// 3. Same-direction or stopped vehicle occupying the target lane ahead
				sameDirClearDist := math.Max(420.0, speedAbs*1.8+90.0)
				if fwd >= -40.0 && fwd < sameDirClearDist {
					return false
				}
				// 4. Same-direction vehicle approaching fast from behind
				if fwd < -sideDist && fwd > -260.0 && obs.Speed > speedAbs+12.0 {
					return false
				}
			}
		}
	}
	return true
}

// countQueuedCars counts the number of stopped or queued vehicles in a specific lane corridor
// between the bus and the upcoming stop line.
func countQueuedCars(obstacles []VehicleObstacle, bus *sim.Bus, cosH, sinH, currentOffset, checkOffset, distToStopLine, laneWidth float64) int {
	count := 0
	corridor := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		corridorWithObs := corridor + obs.Width/2
		if math.Abs(latPath-checkOffset) < corridorWithObs {
			if fwd > -20.0 && fwd < distToStopLine+40.0 {
				if obs.Speed < 25.0 {
					count++
				}
			}
		}
	}
	return count
}

// checkStationApproachLeadVehicle checks whether another vehicle (e.g. lead AI bus) is currently
// ahead in the curb lane, alongside in the adjacent lane, or occupying the destination stop bay.
func checkStationApproachLeadVehicle(
	bus *sim.Bus,
	obstacles []VehicleObstacle,
	cosH, sinH float64,
	currentOffset float64,
	drivePath []sim.Vec2,
	waypointIdx int,
	toStop *sim.BusStop,
) (hasLeadAhead bool, hasLeadAlongside bool, hasPassedLead bool, leadObs *VehicleObstacle) {
	for i := range obstacles {
		obs := &obstacles[i]
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToBaseLane := math.Abs(latPath - 0.0)

		// Check distance to base drive path or destination bay
		if len(drivePath) >= 2 {
			dPath := minDistanceToPath(obs.Pos, drivePath, waypointIdx)
			if dPath < distToBaseLane {
				distToBaseLane = dPath
			}
			if len(drivePath) > 5 {
				endWP := len(drivePath) - 3
				dEnd := minDistanceToPath(obs.Pos, drivePath, endWP)
				if dEnd < distToBaseLane {
					distToBaseLane = dEnd
				}
			}
		}
		if toStop != nil && math.Cos(obs.Heading-toStop.Heading) > 0.4 && fwd > -20.0 {
			dBay := obs.Pos.Distance(toStop.Pos)
			if dBay < (toStop.Length*0.6 + obs.Length*0.5 + 20.0) {
				// Obstacle is inside or at the entrance of the destination stop bay ahead of us
				distToBaseLane = math.Min(distToBaseLane, 5.0)
			}
		}

		corridor := bus.Width/2 + obs.Width/2 + 16.0
		if distToBaseLane < corridor {
			safeClearBehind := -(bus.Length/2 + obs.Length/2 + 25.0) // ~ -73px
			safeClearAhead := (bus.Length/2 + obs.Length/2 + 25.0)   // ~ +73px

			if fwd > safeClearAhead {
				hasLeadAhead = true
				leadObs = obs
			} else if fwd >= safeClearBehind && fwd <= safeClearAhead {
				hasLeadAlongside = true
				leadObs = obs
			} else if fwd < safeClearBehind && fwd > -250.0 {
				hasPassedLead = true
			}
		}
	}
	return
}

// isBaseLaneClearToReturn verifies whether the base lane (offset 0.0) is completely clear of any
// vehicles alongside, closely behind, or approaching fast from behind before initiating a return maneuver.
// If toStop is provided, it also verifies that no vehicle is occupying the destination stop bay or
// blocking the approach in the base lane ahead of our vehicle.
func isBaseLaneClearToReturn(bus *sim.Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset float64, drivePath []sim.Vec2, waypointIdx int, toStop *sim.BusStop) bool {
	corridorBase := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH

		if toStop != nil && math.Cos(obs.Heading-toStop.Heading) > 0.4 && fwd > -20.0 {
			dBay := obs.Pos.Distance(toStop.Pos)
			if dBay < (toStop.Length*0.6 + obs.Length*0.5 + 35.0) {
				return false
			}
		}
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToBaseLane := math.Abs(latPath - 0.0)
		if len(drivePath) >= 2 {
			dPath := minDistanceToPath(obs.Pos, drivePath, waypointIdx)
			if dPath < distToBaseLane {
				distToBaseLane = dPath
			}
			if len(drivePath) > 5 {
				endWP := len(drivePath) - 3
				dEnd := minDistanceToPath(obs.Pos, drivePath, endWP)
				if dEnd < distToBaseLane {
					distToBaseLane = dEnd
				}
			}
		}
		corridor := corridorBase + obs.Width/2
		if distToBaseLane < corridor {
			if yieldedVehicleClearsRear(bus, obs, cosH, sinH) {
				continue
			}
			minSafeBehind := -(bus.Length/2 + obs.Length/2 + 30.0)
			if obs.Speed > bus.Speed+10.0 {
				minSafeBehind = -(bus.Length/2 + obs.Length/2 + 70.0)
			}
			minSafeAhead := (bus.Length/2 + obs.Length/2 + 80.0) // ~ +152px
			if fwd > minSafeBehind && fwd < minSafeAhead {
				return false
			}
			if fwd <= minSafeBehind && fwd > -240.0 && obs.Speed > bus.Speed+15.0 {
				return false
			}
		}
	}
	return true
}

// leftTurnLaneOffset follows the path's taper while staying within the available
// lanes on the current side of the road.
func leftTurnLaneOffset(distBaseToCenter, laneWidth float64, is6Lane bool) float64 {
	maxShift := laneWidth
	if is6Lane {
		maxShift = 2 * laneWidth
	}
	return sim.Clamp(-(distBaseToCenter - laneWidth*0.5), -maxShift, 0)
}
