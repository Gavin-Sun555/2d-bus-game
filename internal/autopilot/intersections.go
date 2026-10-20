package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// intersectionApproach records lane-planning observations. Signal braking uses a
// different lookahead and fallback policy, but shares route-exit classification.
type intersectionApproach struct {
	approachingInter                *sim.TrafficIntersection
	distToStopLine                  float64
	isApproachingIntersection       bool
	isUpcomingLeft, isUpcomingRight bool
}

type routeTurn int

const (
	routeTurnStraight routeTurn = iota
	routeTurnLeft
	routeTurnRight
)

// intersectionExitTurn ignores waypoints before the intersection mouth. An exit
// can be found without a classified turn; callers must preserve that distinction
// when deciding whether to use their fallback policy.
func (ad *AutopilotDriver) intersectionExitTurn(center sim.Vec2, cosH, sinH, stopLineDist, exitLatThreshold float64) (routeTurn, bool) {
	foundExit := false
	for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
		toPt := ad.drivePath[idx].Sub(center)
		forward := toPt.X*cosH + toPt.Y*sinH
		lateral := -toPt.X*sinH + toPt.Y*cosH
		if forward < -stopLineDist {
			continue
		}
		if !(forward > 30.0 || math.Abs(lateral) > exitLatThreshold) {
			continue
		}
		foundExit = true
		angle := sim.NormalizeAngle(math.Atan2(lateral, forward))
		switch {
		case angle < -0.35 && angle > -2.80:
			return routeTurnLeft, true
		case angle > 0.35 && angle < 2.80:
			return routeTurnRight, true
		case math.Abs(angle) <= 0.35:
			return routeTurnStraight, true
		}
	}
	return routeTurnStraight, foundExit
}

func (ad *AutopilotDriver) approachingIntersection(f autopilotFrame, isMultiLane, is6Lane bool) intersectionApproach {
	var approachingInter *sim.TrafficIntersection
	distToStopLine := 999.0
	isApproachingIntersection, isUpcomingLeft, isUpcomingRight := false, false, false
	// Lane-change yaw must not rotate the approach corridor or its exit.
	heading := approachRoadHeading(f.bus.Heading)
	cosH, sinH := math.Cos(heading), math.Sin(heading)

	if f.tls != nil {
		for _, inter := range f.tls.Intersections {
			stopLineOffset := inter.GetStopLineDistForHeading(heading, f.laneWidth)
			toInter := inter.Center.Sub(f.bus.Pos)
			forwardDist := toInter.X*cosH + toInter.Y*sinH
			lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

			maxInterDist := 450.0
			if is6Lane {
				maxInterDist = 900.0
			} else if isMultiLane {
				maxInterDist = 550.0
			}
			maxForwardDist := stopLineOffset + maxInterDist + 80.0
			maxLatDist := f.laneWidth * 3.6
			if is6Lane {
				maxLatDist = f.laneWidth * 4.8
			}

			if forwardDist > -stopLineOffset && forwardDist < maxForwardDist && lateralDist < maxLatDist {
				approachingInter = inter
				distToStopLine = forwardDist - stopLineOffset
				isApproachingIntersection = (distToStopLine > 0 && distToStopLine < maxInterDist)

				hasRouteExit := false
				exitLatThreshold := f.laneWidth * 1.6
				if is6Lane {
					exitLatThreshold = f.laneWidth * 2.8
				}
				if inter.ApproachLanes[inter.GetApproachIndexForHeading(heading)] >= 8 {
					exitLatThreshold = f.laneWidth * 4.2
				}
				turn, foundExit := ad.intersectionExitTurn(inter.Center, cosH, sinH, stopLineOffset, exitLatThreshold)
				hasRouteExit = foundExit
				isUpcomingLeft = turn == routeTurnLeft
				isUpcomingRight = turn == routeTurnRight

				if !hasRouteExit {
					for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
						d := ad.cumLen[k] - f.s
						if d > maxInterDist {
							break
						}
						segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
						segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
						angPrev := sim.NormalizeAngle(math.Atan2(segPrev.Y, segPrev.X) - f.bus.Heading)
						angNext := sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - f.bus.Heading)
						if math.Abs(angPrev) < 0.45 && math.Abs(angNext) < 0.45 {
							continue
						}
						turnAngle := sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X))
						if turnAngle < -0.35 || math.Abs(turnAngle) >= 2.8 {
							isUpcomingLeft = true
							break
						} else if turnAngle > 0.35 && math.Abs(turnAngle) < 2.8 {
							isUpcomingRight = true
							break
						}
					}
				}
				break
			}
		}
	}

	return intersectionApproach{approachingInter, distToStopLine, isApproachingIntersection, isUpcomingLeft, isUpcomingRight}
}

func (ad *AutopilotDriver) updateIntersectionLane(f autopilotFrame, approach intersectionApproach, is2Lane, is6Lane bool, distBaseToCenter, dt float64) {
	cosH, sinH := math.Cos(f.bus.Heading), math.Sin(f.bus.Heading)
	if is2Lane {
		// On 2-lane roads, borrowing oncoming lane through or near intersections is strictly prohibited!
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			if isBaseLaneClearToReturn(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
		}
		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
		ad.progressLaneOffset(dt, f.speedAbs)
		return
	}

	isStraight := !approach.isUpcomingLeft && !approach.isUpcomingRight
	hasDedicatedLeft := approach.approachingInter.ApproachHasDedicatedLanesForHeading(f.bus.Heading)

	if approach.isUpcomingRight {
		// Right turn must be from outer lane (0.0)
		if ad.TargetLaneOffset != 0.0 {
			if isBaseLaneClearToReturn(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
		}
		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	} else if approach.isUpcomingLeft {
		// Left turn: check whether we are currently overtaking on the left or have LaneOffset
		if ad.LaneOffset < -f.laneWidth*0.3 || ad.OvertakeState != OvertakeNone {
			// Bus was overtaking on the left.
			// The left passing lane aligns directly with the dedicated left-turn lane!
			// DO NOT return to the outer right lane; seamlessly transition into the left-turn lane.
			targetLeftOffset := leftTurnLaneOffset(distBaseToCenter, f.laneWidth, is6Lane)
			if math.Abs(ad.LaneOffset-targetLeftOffset) < 1.5 || distBaseToCenter <= f.laneWidth*1.15 {
				// In left-turn corridor: synchronize LaneOffset with targetLeftOffset
				// as drivePath tapers into left-turn lane, so physical lateral position stays rock-solid
				ad.TargetLaneOffset = targetLeftOffset
				ad.LaneOffset = targetLeftOffset
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			} else {
				ad.TargetLaneOffset = targetLeftOffset
				if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
					ad.LaneOffset = ad.TargetLaneOffset
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
			}
		} else {
			// Not overtaking: drivePath naturally leads into the dedicated left-turn lane
			ad.TargetLaneOffset = 0.0
			ad.LaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	} else if isStraight {
		// Going straight through intersection:
		if is6Lane {
			// 6-Lane Road: Two valid straight through lanes:
			// Middle through lane (-laneWidth) and Outer through lane (0.0).
			// (Innermost lane -2*lw is dedicated left-turn, strictly prohibited for straight)
			desiredOffset := ad.preferredThroughLane(f, approach.distToStopLine, true)

			if desiredOffset != ad.TargetLaneOffset {
				if isLaneClearToChange(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, f.laneWidth) {
					ad.TargetLaneOffset = desiredOffset
					ad.OvertakeState = OvertakeChangingLane
					ad.OvertakeTimer = 0.0
				} else if ad.LaneOffset < -f.laneWidth*1.5 {
					// In innermost left-turn lane, must exit to through lanes (middle or outer) before stop line
					altOffset := 0.0
					if desiredOffset == 0.0 {
						altOffset = -f.laneWidth
					}
					if isLaneClearToChange(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, altOffset, f.laneWidth) {
						ad.TargetLaneOffset = altOffset
						ad.OvertakeState = OvertakeChangingLane
						ad.OvertakeTimer = 0.0
					} else {
						// A lane arrow is not permission to cut through an
						// occupied lane. Continue in the current corridor and retry.
						ad.TargetLaneOffset = ad.LaneOffset
						ad.LaneOffsetProgress = 1
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					}
				}
			} else if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
				ad.LaneOffset = ad.TargetLaneOffset
				ad.OvertakeState = OvertakeNone
			}
		} else if hasDedicatedLeft {
			// 4-Lane Road with dedicated left-turn lane:
			// Left lane is STRICTLY dedicated to left turns.
			// STRAIGHT TRAFFIC MUST NEVER ENTER LEFT-TURN LANE!
			// When cars ahead are waiting for red light, queue in outer through lane!
			if ad.LaneOffset < -f.laneWidth*0.5 || ad.TargetLaneOffset < -f.laneWidth*0.5 {
				// Currently in left lane: must return to through lane as soon as safe
				if isBaseLaneClearToReturn(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			} else {
				ad.TargetLaneOffset = 0.0
				if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset) < 1.0 {
					ad.LaneOffset = 0.0
					ad.OvertakeState = OvertakeNone
				}
			}
		} else {
			// Multi-lane road without dedicated left-turn lane: both lanes can go straight
			desiredOffset := ad.preferredThroughLane(f, approach.distToStopLine, false)
			if desiredOffset != ad.TargetLaneOffset {
				if isLaneClearToChange(f.bus, f.obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, f.laneWidth) {
					ad.TargetLaneOffset = desiredOffset
					ad.OvertakeState = OvertakeChangingLane
					ad.OvertakeTimer = 0.0
				}
			} else if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
				ad.LaneOffset = ad.TargetLaneOffset
				ad.OvertakeState = OvertakeNone
			}
		}
	}

	if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
		ad.LaneOffset = ad.TargetLaneOffset
		ad.OvertakeState = OvertakeNone
	}
	ad.progressLaneOffset(dt, f.speedAbs)
	return
}

// preferredThroughLane chooses only between the inner through lane and the outer
// lane. Safety checks before committing the change remain with the caller.
func (ad *AutopilotDriver) preferredThroughLane(f autopilotFrame, distToStopLine float64, is6Lane bool) float64 {
	cosH, sinH := math.Cos(f.bus.Heading), math.Sin(f.bus.Heading)
	queuedInner := countQueuedCars(f.obstacles, f.bus, cosH, sinH, ad.LaneOffset, -f.laneWidth, distToStopLine, f.laneWidth)
	queuedOuter := countQueuedCars(f.obstacles, f.bus, cosH, sinH, ad.LaneOffset, 0, distToStopLine, f.laneWidth)
	if queuedInner < queuedOuter {
		return -f.laneWidth
	}
	if queuedOuter < queuedInner {
		return 0
	}
	if is6Lane && queuedInner == 0 {
		innerScore, outerScore, innerBlocked, outerBlocked := evaluate6LaneThroughTraffic(f.bus, f.obstacles, cosH, sinH, f.laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
		switch {
		case innerBlocked && !outerBlocked:
			return 0
		case outerBlocked && !innerBlocked:
			return -f.laneWidth
		case outerScore > innerScore+0.6:
			return -f.laneWidth
		case innerScore > outerScore+0.6:
			return 0
		}
	}
	// Equal queues and equivalent traffic: keep the current through lane.
	if ad.LaneOffset < -f.laneWidth*0.5 {
		return -f.laneWidth
	}
	return 0
}

// The generated road network is orthogonal. Classify its approach by road
// direction, retaining temporary lane-change yaw only for physical steering.
func approachRoadHeading(heading float64) float64 {
	return sim.NormalizeAngle(math.Round(heading/(math.Pi/2)) * (math.Pi / 2))
}
