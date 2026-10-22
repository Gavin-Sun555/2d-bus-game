package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// updateOvertaking resolves lane policies in priority order: disabled/invalid,
// circulating roundabout, station approach, corner, intersection, roundabout
// entry, then mid-block passing. A terminal policy advances lane interpolation
// once and returns; later policies must not overwrite its lane decision.
func (ad *AutopilotDriver) updateOvertaking(
	bus *sim.Bus,
	world *sim.World,
	tls *sim.TrafficLightSystem,
	obstacles []VehicleObstacle,
	laneWidth float64,
	s, speedAbs, remaining float64,
	dt float64,
	targetStop *sim.BusStop,
) {
	ad.retryStoppedMidLaneReturn(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs})
	if ad.advanceMidLaneRecovery(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs}, dt) {
		return
	}
	// All lane policies, including an already-running return, must recheck
	// rightward clearance before their interpolated target reaches steering.
	defer func() {
		ad.holdBlockedRightMerge(autopilotFrame{bus: bus, world: world, obstacles: obstacles, laneWidth: laneWidth, s: ad.LastPathS, speedAbs: speedAbs})
	}()
	// A skip is not a departure from a bay: retain the current lane until
	// clear of the skipped stop, including in standard autopilot mode.
	if ad.holdSkippedDepartureLane(bus, world, laneWidth, dt, speedAbs) {
		return
	}

	ad.busTurnHoldDistance = 0
	if ad.uTurn != nil && ad.uTurn.entered {
		ad.approachMerge = nil
		ad.approachMergeJoined = false
		ad.rightTurnJunction = nil
		ad.rightTurnWaiting = false
		ad.busLaneJunction = nil
		ad.busJunctionEntered = false
		ad.holdLaneOffset(0)
		ad.OvertakeState = OvertakeNone
		return
	}

	if !ad.BerserkMode {
		ad.approachMerge = nil
		ad.approachMergeJoined = false
	}
	if ad.BerserkMode && world != nil && len(ad.drivePath) >= 2 && (ad.State == AutopilotStateCruising || ad.State == AutopilotStateApproaching) {
		if ad.updateRequiredApproachMerge(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs}, dt) {
			return
		}
	}
	if ad.updateRightTurnOffset(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs}) {
		return
	}
	if ad.trackJunctionTurnLane(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs}, dt) {
		return
	}
	if ad.updateBusLane(autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs, remaining: remaining, targetStop: targetStop}, dt) {
		return
	}
	// If Berserk mode is disabled, not in driving state, or path invalid: smoothly return to base lane
	isDrivingState := ad.State == AutopilotStateCruising || ad.State == AutopilotStateApproaching
	if !ad.BerserkMode || !isDrivingState || world == nil || len(ad.drivePath) < 2 {
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// =========================================================================
	// ROUNDABOUT DUAL-LANE OVERTAKING (BERSERK MODE)
	// =========================================================================
	var currentRb *sim.Roundabout
	for i := range world.Roundabouts {
		if bus.Pos.Distance(world.Roundabouts[i].Center) <= world.Roundabouts[i].Radius+85.0 {
			currentRb = &world.Roundabouts[i]
			break
		}
	}

	if currentRb != nil {
		ad.updateRoundaboutLane(bus, currentRb, obstacles, laneWidth, s, speedAbs, dt)
		return
	}

	roadCenter, isEW, isMultiLane, is6Lane := sim.CurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
	is2Lane := !isMultiLane && !is6Lane
	if !isMultiLane && (!ad.BerserkMode || !is2Lane || ad.DisallowOncomingOvertake) {
		// Not a multi-lane road in current travel direction (e.g. 2-lane road with 1 lane each way)
		// Only the player in Berserk mode may borrow the opposing lane.
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Check if upcoming corner prevents overtaking, and whether the route turns left along this road
	isUpcomingCorner := false
	isUpcomingCornerLeft := false
	hasNextLeftTurn := false

	maxCornerDist := math.Max(450.0, speedAbs*1.2)
	if is6Lane {
		maxCornerDist = math.Max(650.0, speedAbs*1.6)
	} else if isMultiLane {
		maxCornerDist = math.Max(500.0, speedAbs*1.3)
	}

	for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
		d := ad.cumLen[k] - s
		if d < -5.0 {
			continue
		}
		if d > 1800.0 {
			break
		}
		a := ad.drivePath[k].Sub(ad.drivePath[k-1])
		b := ad.drivePath[k+1].Sub(ad.drivePath[k])
		// Waypoints entering or navigating a roundabout are part of circular rotary flow, not a sharp street corner
		if world != nil {
			isRbWpt := false
			for _, rb := range world.Roundabouts {
				if ad.drivePath[k].Distance(rb.Center) <= rb.Radius+80.0 || ad.drivePath[k+1].Distance(rb.Center) <= rb.Radius+80.0 {
					isRbWpt = true
					break
				}
			}
			if isRbWpt {
				continue
			}
		}
		// A segment b continuing straight along the current road heading is not a corner turn
		// (e.g. straightening out from a bus stop bay exit or lane taper)
		angB := sim.NormalizeAngle(math.Atan2(b.Y, b.X) - bus.Heading)
		if math.Abs(angB) < 0.25 {
			continue
		}
		turn := sim.NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
		if (turn < -0.35 && turn > -2.80) || math.Abs(turn) >= 2.80 {
			if !hasNextLeftTurn {
				hasNextLeftTurn = true
			}
			if d <= maxCornerDist && !isUpcomingCorner {
				isUpcomingCorner = true
				isUpcomingCornerLeft = true
			}
			break
		} else if turn > 0.35 && turn < 2.80 {
			if d <= maxCornerDist && !isUpcomingCorner {
				isUpcomingCorner = true
			}
			break
		}
	}

	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	// =========================================================================
	// STATION APPROACH & BAY ENTRY CONTROLLER
	// =========================================================================
	// When approaching destination stop bay:
	// 1. If there is enough runway and we have cleanly passed the lead vehicle (or bay is clear),
	//    merge ahead in front of it into the bay.
	// 2. If there is NOT enough runway to overtake, wait behind in the curb lane,
	//    or wait alongside in the adjacent passing lane without colliding.
	stationApproachThreshold := math.Max(420.0, speedAbs*2.2+150.0)
	if !ad.CanSkipTargetStop && remaining < stationApproachThreshold {
		ad.updateStationApproachLane(bus, targetStop, obstacles, laneWidth, speedAbs, remaining, dt, isMultiLane, is6Lane)
		return
	}

	// Determine base lane: outer lane (~1.5*lw from center on 4-lane, ~2.5*lw on 6-lane) or inner lane (~0.5*lw from center)
	basePt := ad.pointAt(s)
	distBaseToCenter := math.Abs(basePt.Y - roadCenter)
	if !isEW {
		distBaseToCenter = math.Abs(basePt.X - roadCenter)
	}

	// Approaching an actual road corner/turn (left or right):
	if isUpcomingCorner {
		if isUpcomingCornerLeft && (isMultiLane || is6Lane) {
			// Upcoming corner is a LEFT turn on a multi-lane road:
			if ad.LaneOffset < -laneWidth*0.3 || ad.OvertakeState != OvertakeNone {
				// Bus was overtaking on the left.
				// Do not return to the outer right lane; stay in the left lane which leads directly into the left-turn lane.
				targetLeftOffset := leftTurnLaneOffset(distBaseToCenter, laneWidth, is6Lane)
				ad.TargetLaneOffset = targetLeftOffset
				if math.Abs(ad.LaneOffset-targetLeftOffset) < 1.5 || distBaseToCenter <= laneWidth*1.15 {
					ad.LaneOffset = targetLeftOffset
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
			} else {
				// Bus was not overtaking: keep offset 0.0, drivePath will naturally merge into left-turn lane
				ad.TargetLaneOffset = 0.0
				ad.LaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
		} else {
			// Right turn or 2-lane road: return to base outer lane ONLY if safe and clear
			if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
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

	// =========================================================================
	// INTERSECTION APPROACH LANE SELECTION & RED LIGHT QUEUE BALANCING
	// =========================================================================
	frame := autopilotFrame{bus: bus, world: world, tls: tls, obstacles: obstacles, laneWidth: laneWidth, s: s, speedAbs: speedAbs}
	approach := ad.approachingIntersection(frame, isMultiLane, is6Lane)
	isApproachingIntersection, isUpcomingLeft := approach.isApproachingIntersection, approach.isUpcomingLeft
	if isApproachingIntersection && approach.approachingInter != nil {
		ad.updateIntersectionLane(frame, approach, is2Lane, is6Lane, distBaseToCenter, dt)
		return
	}

	// =========================================================================
	// ROUNDABOUT APPROACH MULTI-LANE OVERTAKING & LANE KEEPING
	// =========================================================================
	var nearRbApproach *sim.Roundabout
	if world != nil {
		for i := range world.Roundabouts {
			if bus.Pos.Distance(world.Roundabouts[i].Center) <= world.Roundabouts[i].Radius+220.0 {
				nearRbApproach = &world.Roundabouts[i]
				break
			}
		}
	}

	if nearRbApproach != nil && (isMultiLane || is6Lane) {
		// Approaching a multi-lane roundabout:
		// Inner lane connects directly to the inner circulatory ring.
		// If bus is in inner lane, or if outer lane is queued/blocked, stay in or switch to inner lane!
		queuedOuter := countQueuedCars(obstacles, bus, cosH, sinH, ad.LaneOffset, 0.0, 350.0, laneWidth)
		hasStoppedCarsInOuter := false
		for _, obs := range obstacles {
			d := obs.Pos.Sub(bus.Pos)
			fwd := d.X*cosH + d.Y*sinH
			lat := -d.X*sinH + d.Y*cosH
			latPath := lat + ad.LaneOffset
			if math.Abs(latPath-0.0) < (bus.Width/2+obs.Width/2+10.0) && fwd > -20.0 && fwd < 320.0 && obs.Speed < 20.0 {
				hasStoppedCarsInOuter = true
				break
			}
		}

		if ad.LaneOffset <= -laneWidth*0.4 || ad.TargetLaneOffset <= -laneWidth*0.4 {
			// Already in inner passing lane: stay in inner lane and enter roundabout smoothly!
			ad.TargetLaneOffset = -laneWidth
			if is6Lane && ad.LaneOffset <= -laneWidth*1.5 {
				ad.TargetLaneOffset = -2.0 * laneWidth
			}
			if ad.OvertakeState == OvertakeReturning {
				ad.OvertakeState = OvertakePassing
			}
			ad.progressLaneOffset(dt, speedAbs)
			return
		} else if (hasStoppedCarsInOuter || queuedOuter > 0) && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
			// Move inward only for an actual queue. Berserk mode alone is
			// not a reason to leave a clear outer approach and then merge back.
			ad.TargetLaneOffset = -laneWidth
			ad.OvertakeState = OvertakeChangingLane
			ad.OvertakeTimer = 0.0
			ad.progressLaneOffset(dt, speedAbs)
			return
		}
	}

	// Mid-block overtaking
	var passOffset float64
	if is2Lane {
		passOffset = -laneWidth // On 2-lane road: borrow oncoming lane to the left in RHT
	} else if distBaseToCenter > laneWidth*0.95 {
		passOffset = -laneWidth // Inner passing lane (left side in RHT)
	} else {
		passOffset = laneWidth // Outer lane (right side)
	}

	// =========================================================================
	// 6-LANE ROAD STRAIGHT-THROUGH MID-BLOCK TRAFFIC SELECTION
	// =========================================================================
	minOvertakeRunway := math.Max(450.0, speedAbs*2.4+160.0)
	is6LaneStraight := is6Lane && !isUpcomingCorner && !isUpcomingLeft && !isUpcomingCornerLeft && !hasNextLeftTurn && !isApproachingIntersection && remaining >= minOvertakeRunway
	if is6LaneStraight {
		leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())

		var desiredOffset float64
		if ad.BerserkMode {
			// In Berserk Mode on a 3-lane-per-direction road (Right: 0.0, Mid: -lw, Left: -2*lw):
			if rightBlocked && midBlocked {
				// Both right two lanes have cars/blockers:
				if !leftBlocked {
					desiredOffset = -2.0 * laneWidth // Overtake via the clear leftmost lane!
				} else {
					// All 3 lanes blocked, stay in current lane
					if ad.LaneOffset <= -laneWidth*1.5 {
						desiredOffset = -2.0 * laneWidth
					} else if ad.LaneOffset <= -laneWidth*0.5 {
						desiredOffset = -laneWidth
					} else {
						desiredOffset = 0.0
					}
				}
			} else if rightBlocked && !midBlocked {
				// Right lane blocked, middle lane open
				if !leftBlocked && leftScore < midScore-0.5 {
					desiredOffset = -2.0 * laneWidth
				} else {
					desiredOffset = -laneWidth
				}
			} else if midBlocked && !rightBlocked {
				// Middle lane blocked
				if !leftBlocked {
					desiredOffset = -2.0 * laneWidth // Pass on the left
				} else {
					desiredOffset = 0.0 // Pass on the right
				}
			} else if leftBlocked && midBlocked && !rightBlocked {
				desiredOffset = 0.0
			} else if !rightBlocked && !midBlocked {
				// Both right lanes clear: if in leftmost lane, return to middle lane once safe
				if ad.LaneOffset <= -laneWidth*1.5 {
					desiredOffset = -laneWidth
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
			} else {
				if ad.LaneOffset <= -laneWidth*1.5 {
					desiredOffset = -2.0 * laneWidth
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
			}
		} else {
			// Standard mode: middle through lane (-lw) or outer lane (0.0)
			if midBlocked && !rightBlocked {
				desiredOffset = 0.0
			} else if rightBlocked && !midBlocked {
				desiredOffset = -laneWidth
			} else if rightScore > midScore+0.6 {
				desiredOffset = -laneWidth
			} else if midScore > rightScore+0.6 {
				desiredOffset = 0.0
			} else {
				if ad.LaneOffset < -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
			}
		}

		if ad.OvertakeState == OvertakeNone {
			if desiredOffset != ad.TargetLaneOffset {
				if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, laneWidth) {
					ad.OvertakeState = OvertakeChangingLane
					ad.TargetLaneOffset = desiredOffset
					ad.OvertakeTimer = 0.0
				}
			}
		}
	}

	switch ad.OvertakeState {
	case OvertakeNone:
		// Don't initiate overtaking if approaching destination stop without sufficient runway
		// to complete the pass and cleanly pull into the bay in front, or when approaching an intersection/corner
		minOvertakeRunway := math.Max(450.0, speedAbs*2.4+160.0)
		if remaining < minOvertakeRunway || isUpcomingCorner {
			break
		}
		if is2Lane {
			if approach.approachingInter != nil && approach.distToStopLine < math.Max(350.0, speedAbs*1.5) {
				break
			}
		} else if isApproachingIntersection {
			break
		}

		// On 2-lane roads (borrowing oncoming lane):
		// Must ONLY initiate borrowing after the turn has fully completed, vehicle has settled
		// in the travel lane, and heading/steering are straight. Never borrow lane while turning
		// or right at the corner exit, which would swing into the opposite shoulder!
		if is2Lane {
			roadHeading := 0.0
			if isEW {
				if math.Cos(bus.Heading) < 0 {
					roadHeading = math.Pi
				}
			} else {
				if math.Sin(bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
				} else {
					roadHeading = -math.Pi / 2.0
				}
			}
			headingErr := math.Abs(sim.NormalizeAngle(bus.Heading - roadHeading))
			if headingErr > 0.10 { // > ~5.7 deg: still turning or settling!
				break
			}

			if math.Abs(bus.SteeringAngle) > 0.06 {
				break
			}

			if math.Abs(ad.LastLateralErr) > 6.0 {
				break
			}

			hasCurvatureNearby := false
			for k := 1; k < len(ad.drivePath)-1; k++ {
				d := ad.cumLen[k] - s
				if d < -180.0 {
					continue
				}
				if d > 180.0 {
					break
				}
				segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
				segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
				turnAngle := math.Abs(sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if turnAngle > 0.18 {
					hasCurvatureNearby = true
					break
				}
			}
			if hasCurvatureNearby {
				break
			}

			if world != nil {
				interTurnClearance := math.Max(260.0, laneWidth*4.2+55.0)
				inIntersection := false
				for _, ipt := range world.Intersections {
					if bus.Pos.Distance(ipt) < interTurnClearance {
						inIntersection = true
						break
					}
				}
				if inIntersection {
					break
				}
			}
		}

		// Dynamic lookahead (提前量) for slow/stopped blocker detection:
		// Ensures slow lead vehicles are detected with ample margin so the lane change
		// initiates well before the bus enters the braking distance, preventing collisions
		// before completing the lateral transition.
		fwdMax := math.Max(480.0, speedAbs*2.4+160.0)

		var blockerDist float64 = 9999.0
		foundBlocker := false

		for i := range obstacles {
			obs := &obstacles[i]
			if targetStop != nil && obs.Pos.Distance(targetStop.Pos) < 260.0 {
				continue // Vehicle already at/entering destination stop bay
			}
			delta := obs.Pos.Sub(bus.Pos)
			fwd := delta.X*cosH + delta.Y*sinH
			lat := math.Abs(-delta.X*sinH + delta.Y*cosH)
			if fwd > (bus.Length/2+obs.Length/2+12.0) && fwd < fwdMax && lat < (bus.Width/2+obs.Width/2+10.0) {
				if math.Cos(obs.Heading-bus.Heading) > 0.4 {
					cruise := ad.GetBerserkCruiseSpeed()
					if !ad.BerserkMode {
						cruise = apCruiseSpeed
					}
					if obs.Speed < cruise*0.85 || obs.Speed < 20.0 {
						closingSpeed := math.Max(0.0, speedAbs-obs.Speed)
						// Dynamic lookahead: lateral transition time (~0.85-1.2s) * closing speed + cushion
						reqAdvance := closingSpeed*2.2 + (bus.Length/2 + obs.Length/2 + 80.0)
						if fwd < math.Max(reqAdvance, 400.0) {
							if fwd < blockerDist {
								blockerDist = fwd
								foundBlocker = true
							}
						}
					}
				}
			}
		}

		if foundBlocker {
			targetPassOffset := passOffset
			if (is6LaneStraight || (is6Lane && hasNextLeftTurn)) && ad.BerserkMode {
				leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				_ = rightScore
				_ = midScore
				_ = leftScore

				if ad.LaneOffset > -laneWidth*0.5 {
					// Currently in outer lane (0.0):
					// Right lane is blocked (foundBlocker).
					if (midBlocked || midScore > 0.5) && !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth // Both right lanes have cars, left lane is clear!
					} else if !midBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
						targetPassOffset = -laneWidth
					} else if !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth
					} else {
						targetPassOffset = -laneWidth
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 && ad.LaneOffset > -laneWidth*1.5 {
					// Currently in middle lane (-laneWidth):
					// Middle lane is blocked (foundBlocker).
					if !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth // Pass on clear left lane!
					} else if !rightBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
						targetPassOffset = 0.0
					} else if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth
					} else {
						targetPassOffset = 0.0
					}
				} else {
					// Currently in leftmost lane (-2.0*laneWidth):
					if !midBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
						targetPassOffset = -laneWidth
					} else if !rightBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
						targetPassOffset = 0.0
					} else {
						targetPassOffset = -laneWidth
					}
				}
			} else if is6LaneStraight {
				if ad.LaneOffset < -laneWidth*0.5 {
					targetPassOffset = 0.0
				} else {
					targetPassOffset = -laneWidth
				}
			}

			// Verify passing lane is safe and clear to change into
			clearToChange := false
			if is2Lane {
				clearToChange = isOncomingLaneClear(bus, obstacles, cosH, sinH, ad.LaneOffset, targetPassOffset, laneWidth, speedAbs, ad.GetBerserkCruiseSpeed())
			} else {
				clearToChange = isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, targetPassOffset, laneWidth)
			}

			if clearToChange {
				ad.OvertakeState = OvertakeChangingLane
				ad.TargetLaneOffset = targetPassOffset
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeChangingLane:
		ad.OvertakeTimer += dt

		// Active oncoming threat check for 2-lane roads:
		// If an oncoming car approaches while changing into oncoming lane, abort immediately!
		if is2Lane {
			if (approach.approachingInter != nil && approach.distToStopLine < math.Max(350.0, speedAbs*1.5)) || isUpcomingCorner {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
				ad.OvertakeTimer = 0.0
				break
			}
			if !isOncomingLaneClear(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.TargetLaneOffset, laneWidth, speedAbs, ad.GetBerserkCruiseSpeed()) {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
					ad.OvertakeTimer = 0.0
					break
				}
			}
		}

		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
			if is6LaneStraight {
				// Arrived in selected straight through / passing lane on 6-lane road
				_, midScore, rightScore, _, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				if ad.TargetLaneOffset <= -laneWidth*1.5 {
					// Arrived in leftmost lane (-2.0*laneWidth)
					if midBlocked || rightBlocked || midScore > 0 || rightScore > 0 {
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					}
				} else if ad.TargetLaneOffset <= -laneWidth*0.5 {
					// Arrived in middle lane (-laneWidth)
					if rightBlocked || rightScore >= midScore {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					}
				} else {
					// Arrived in outer lane (0.0)
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
			} else {
				ad.OvertakeState = OvertakePassing
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakePassing:
		ad.OvertakeTimer += dt

		// Active oncoming threat check for 2-lane roads
		if is2Lane {
			if (approach.approachingInter != nil && approach.distToStopLine < math.Max(350.0, speedAbs*1.5)) || isUpcomingCorner {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
				ad.OvertakeTimer = 0.0
				break
			}
			hasImminentOncoming := false
			for _, obs := range obstacles {
				d := obs.Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				lat := -d.X*sinH + d.Y*cosH
				latPath := lat + ad.LaneOffset
				distToOncoming := math.Abs(latPath - (-laneWidth))
				corridor := bus.Width/2 + obs.Width/2 + 14.0
				if distToOncoming < corridor {
					cosDiff := math.Cos(obs.Heading - bus.Heading)
					if cosDiff < -0.2 && fwd > 0 && fwd < math.Max(600.0, speedAbs*2.0+120.0) {
						hasImminentOncoming = true
						break
					}
				}
			}
			if hasImminentOncoming {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
					ad.OvertakeTimer = 0.0
					break
				}
			}
		}

		// Check if we have passed all obstacles in the lanes being overtaken
		hasObstacleNotCleared := false
		for _, obs := range obstacles {
			d := obs.Pos.Sub(bus.Pos)
			fwd := d.X*cosH + d.Y*sinH
			lat := -d.X*sinH + d.Y*cosH
			latPath := lat + ad.LaneOffset

			isOvertakingObs := false
			corridor := bus.Width/2 + obs.Width/2 + 10.0

			if is6LaneStraight {
				if ad.LaneOffset <= -laneWidth*1.5 {
					// In leftmost lane: overtaking middle lane (-lw) and outer lane (0.0)
					if math.Abs(latPath-(-laneWidth)) < corridor || math.Abs(latPath-0.0) < corridor {
						isOvertakingObs = true
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					// In middle lane: overtaking outer lane (0.0)
					if math.Abs(latPath-0.0) < corridor {
						isOvertakingObs = true
					}
				} else {
					// In outer lane: overtaking middle lane (-lw)
					if math.Abs(latPath-(-laneWidth)) < corridor {
						isOvertakingObs = true
					}
				}
			} else {
				distToBaseLane := math.Abs(latPath - 0.0)
				if len(ad.drivePath) >= 2 {
					dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
					if dPath < distToBaseLane {
						distToBaseLane = dPath
					}
				}
				if distToBaseLane < corridor {
					isOvertakingObs = true
				}
			}

			if isOvertakingObs {
				minSafeBehind := -(bus.Length/2 + obs.Length/2 + 75.0) // ~ -141px
				cruise := ad.GetBerserkCruiseSpeed()
				if !ad.BerserkMode {
					cruise = apCruiseSpeed
				}
				aheadBuffer := math.Max(480.0, speedAbs*2.0+100.0)
				// Only shrink aheadBuffer for station approach if station is on THIS same road segment and NOT past a roundabout or in Berserk mode
				if targetStop != nil && !ad.CanSkipTargetStop && !ad.BerserkMode && remaining < 650.0 {
					distBusToStop := bus.Pos.Distance(targetStop.Pos)
					nearRb := false
					if world != nil {
						for _, rb := range world.Roundabouts {
							if bus.Pos.Distance(rb.Center) <= rb.Radius+150.0 || targetStop.Pos.Distance(rb.Center) <= rb.Radius+150.0 {
								nearRb = true
								break
							}
						}
					}
					if distBusToStop < 500.0 && !nearRb {
						aheadBuffer = math.Max(80.0, remaining-100.0)
					}
				}
				if fwd > minSafeBehind && fwd < aheadBuffer && (fwd < 150.0 || obs.Speed < cruise*0.88 || obs.Speed < 25.0) {
					hasObstacleNotCleared = true
					break
				}
			}
		}

		// If approaching a roundabout on a multi-lane road, stay in inner passing lane!
		nearRb := false
		if world != nil {
			for i := range world.Roundabouts {
				if bus.Pos.Distance(world.Roundabouts[i].Center) <= world.Roundabouts[i].Radius+220.0 {
					nearRb = true
					break
				}
			}
		}
		if nearRb && (isMultiLane || is6Lane) {
			ad.TargetLaneOffset = -laneWidth
			if is6Lane && ad.LaneOffset <= -laneWidth*1.5 {
				ad.TargetLaneOffset = -2.0 * laneWidth
			}
			break
		}

		shouldReturn := false
		if !hasObstacleNotCleared && ad.OvertakeTimer > 0.6 {
			if isUpcomingLeft || isUpcomingCornerLeft || hasNextLeftTurn {
				targetLeftOffset := leftTurnLaneOffset(distBaseToCenter, laneWidth, is6Lane)
				ad.TargetLaneOffset = targetLeftOffset
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
				break
			}
			if is6LaneStraight {
				leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				_ = leftScore
				_ = leftBlocked
				if ad.LaneOffset <= -laneWidth*1.5 {
					// Currently in leftmost lane (-2*lw)
					if midBlocked {
						if rightBlocked {
							// Both right lanes still blocked; stay in leftmost lane!
							ad.TargetLaneOffset = -2.0 * laneWidth
							ad.OvertakeState = OvertakeNone
							ad.OvertakeTimer = 0.0
							break
						} else {
							// Middle lane blocked, but outer lane clear: return to outer lane (0.0)
							if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
								ad.TargetLaneOffset = 0.0
								ad.OvertakeState = OvertakeReturning
								ad.OvertakeTimer = 0.0
								break
							}
						}
					} else {
						// Middle lane is clear! Return to middle lane (-laneWidth)
						if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
							ad.TargetLaneOffset = -laneWidth
							ad.OvertakeState = OvertakeReturning
							ad.OvertakeTimer = 0.0
							break
						}
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					// Currently in middle lane (-laneWidth)
					if rightBlocked || rightScore >= midScore {
						// Stay in middle through lane!
						ad.TargetLaneOffset = -laneWidth
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
						break
					}
				}
			}
			shouldReturn = true
		}

		if shouldReturn {
			targetRetOffset := 0.0
			if is6LaneStraight && ad.LaneOffset <= -laneWidth*1.5 {
				targetRetOffset = -laneWidth
			}
			if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, targetRetOffset, laneWidth) {
				ad.OvertakeState = OvertakeReturning
				ad.TargetLaneOffset = targetRetOffset
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeReturning:
		ad.OvertakeTimer += dt

		if isUpcomingLeft || isUpcomingCornerLeft {
			targetLeftOffset := leftTurnLaneOffset(distBaseToCenter, laneWidth, is6Lane)
			ad.TargetLaneOffset = targetLeftOffset
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
			break
		}

		// Active collision avoidance during return maneuver:
		// If return lane suddenly becomes obstructed while we are still transitioning, abort return!
		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) > laneWidth*0.15 {
			for _, obs := range obstacles {
				d := obs.Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				lat := -d.X*sinH + d.Y*cosH
				latPath := lat + ad.LaneOffset
				distToRetLane := math.Abs(latPath - ad.TargetLaneOffset)
				if ad.TargetLaneOffset == 0.0 && len(ad.drivePath) >= 2 {
					dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
					if dPath < distToRetLane {
						distToRetLane = dPath
					}
				}
				if distToRetLane < (bus.Width/2 + obs.Width/2 + 10.0) {
					dangerBehind := -(bus.Length/2 + obs.Length/2 + 45.0)
					dangerAhead := (bus.Length/2 + obs.Length/2 + 75.0)
					if obs.Speed < 20.0 {
						dangerAhead = (bus.Length/2 + obs.Length/2 + 200.0)
					}
					if fwd > dangerBehind && fwd < dangerAhead {
						// Cancel lateral motion at the actual corridor instead
						// of swinging back across another lane.
						heading := approachRoadHeading(bus.Heading)
						right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
						ad.holdLaneOffset(bus.Pos.Sub(ad.pointAt(s)).Dot(right))
						break
					}
				}
			}
		}

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	}

	ad.progressLaneOffset(dt, speedAbs)
}
