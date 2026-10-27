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
	entered := false
	for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
		// Classify the first road exit along a segment, not just its endpoint.
		// A sparse segment can cross this junction straight before bending at
		// the next one; the later endpoint must not select this signal's arrow.
		if idx > 0 {
			a := ad.drivePath[idx-1].Sub(center)
			b := ad.drivePath[idx].Sub(center)
			fa, fb := a.X*cosH+a.Y*sinH, b.X*cosH+b.Y*sinH
			if fa < stopLineDist && fb >= stopLineDist && fb > fa {
				p := a.Add(b.Sub(a).Mul((stopLineDist - fa) / (fb - fa)))
				if math.Abs(-p.X*sinH+p.Y*cosH) <= exitLatThreshold {
					return routeTurnStraight, true
				}
			}
		}
		toPt := ad.drivePath[idx].Sub(center)
		forward := toPt.X*cosH + toPt.Y*sinH
		lateral := -toPt.X*sinH + toPt.Y*cosH
		if forward >= -stopLineDist && forward <= stopLineDist && math.Abs(lateral) <= exitLatThreshold {
			entered = true
		}
		// A U-turn returns through the incoming mouth rather than crossing a
		// far exit. Its outgoing tangent, not its position angle, identifies
		// the left-arrow movement even on a wide multi-lane road.
		if entered && idx > 0 && forward <= stopLineDist && lateral < 0 {
			tangent := ad.drivePath[idx].Sub(ad.drivePath[idx-1]).Normalize()
			if tangent.Dot(sim.Vec2{X: cosH, Y: sinH}) < -.9 {
				return routeTurnLeft, true
			}
		}
		if forward < -stopLineDist {
			continue
		}
		// A wide turning apron is still inside the junction, even where it
		// extends beyond the narrow road corridor.
		if forward >= stopLineDist || math.Abs(lateral) > exitLatThreshold {
			foundExit = true
		}
		exitBoundary := math.Max(exitLatThreshold, stopLineDist-sim.JunctionStopInset)
		if !(forward >= stopLineDist || math.Abs(lateral) > exitBoundary) {
			continue
		}
		// A straight exit may remain far from the center in a wide curb lane.
		// Its position angle is not a turn angle. Wait for an actual exit from
		// the junction, then recognize the full outgoing road corridor.
		if forward >= stopLineDist && math.Abs(lateral) <= exitLatThreshold {
			return routeTurnStraight, true
		}
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
	return ad.intersectionApproachWithRunway(f, isMultiLane, is6Lane, false)
}

func (ad *AutopilotDriver) intersectionApproachWithRunway(f autopilotFrame, isMultiLane, is6Lane, earlyRight bool) intersectionApproach {
	var approachingInter *sim.TrafficIntersection
	distToStopLine := math.Inf(1)
	isApproachingIntersection, isUpcomingLeft, isUpcomingRight := false, false, false
	// Lane-change yaw must not rotate the approach corridor or its exit.
	heading := approachRoadHeading(f.bus.Heading)
	sinH, cosH := math.Sincos(heading)

	if f.tls != nil {
		for _, inter := range f.tls.Intersections {
			stopLineOffset := inter.GetStopLineDistForHeading(heading, f.laneWidth)
			toInter := inter.Center.Sub(f.bus.Pos)
			forwardDist := toInter.X*cosH + toInter.Y*sinH
			lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

			lanes := inter.ApproachLanes[inter.GetApproachIndexForHeading(heading)]
			maxInterDist := requiredApproachDistance(f, lanes)
			rightMergeDist := maxInterDist
			if earlyRight {
				rightMergeDist = rightTurnApproachDistance(f, lanes)
			}
			maxForwardDist := stopLineOffset + math.Max(maxInterDist, rightMergeDist) + 80.0
			maxLatDist := f.laneWidth * 3.6
			if is6Lane {
				maxLatDist = f.laneWidth * 4.8
			}

			if forwardDist > -stopLineOffset && forwardDist < maxForwardDist && lateralDist < maxLatDist && forwardDist-stopLineOffset < distToStopLine {
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
				if isUpcomingRight {
					isApproachingIntersection = distToStopLine > 0 && distToStopLine < rightMergeDist
				}

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
			}
		}
	}

	return intersectionApproach{approachingInter, distToStopLine, isApproachingIntersection, isUpcomingLeft, isUpcomingRight}
}

func (ad *AutopilotDriver) updateIntersectionLane(f autopilotFrame, approach intersectionApproach, is2Lane, is6Lane bool, distBaseToCenter, dt float64) {
	sinH, cosH := math.Sincos(f.bus.Heading)
	if is2Lane {
		// On 2-lane roads, borrowing oncoming lane through or near intersections is strictly prohibited!
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
	sinH, cosH := math.Sincos(f.bus.Heading)
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

// Reserve braking, a full lane transition per crossed lane, and a settled
// approach before the entry line. Wide roads need more room than narrow ones.
func requiredApproachDistance(f autopilotFrame, lanes int) float64 {
	crossings := math.Max(1, float64(lanes/2-1))
	settle := f.bus.Length + 2*f.laneWidth
	return math.Max(650+180*(crossings-1), f.speedAbs*f.speedAbs/(2*130)+f.speedAbs*(1.4+.7*crossings)+settle)
}

// A right turn may require crossing every passing lane before a curb queue.
// Reserve extra low-speed runway without changing straight-road queue policy.
func rightTurnApproachDistance(f autopilotFrame, lanes int) float64 {
	crossings := math.Max(1, float64(lanes/2-1))
	settle := f.bus.Length + 2*f.laneWidth
	return math.Max(requiredApproachDistance(f, lanes), 900+240*(crossings-1)+settle)
}
