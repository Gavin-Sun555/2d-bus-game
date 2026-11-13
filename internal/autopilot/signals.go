package autopilot

import (
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) trafficLightSpeed(f autopilotFrame, targetSpd float64) float64 {
	// D. Red / Yellow Light Stopping
	if f.tls == nil {
		return targetSpd
	}
	bumperPos := f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(f.bus.Length * .5))
	targetSpd = math.Min(targetSpd, f.world.RailSpeedLimit(f.tls, bumperPos, f.bus.Heading, f.speedAbs, 135))
	targetSpd = math.Min(targetSpd, f.world.StationCrossingSpeedLimit(f.tls, bumperPos, f.bus.Heading, f.speedAbs, f.laneWidth, 135))
	// Once lawfully across the entry line, the same junction's other
	// approaches must not stop the bus in the middle of its U-turn. Signals
	// at subsequent junctions and every obstacle check remain active.
	if ad.uTurn != nil && ad.uTurn.entered {
		filtered := *f.tls
		filtered.Intersections = make([]*sim.TrafficIntersection, 0, len(f.tls.Intersections))
		for _, inter := range f.tls.Intersections {
			if inter.Center.Distance(ad.uTurn.center) > 1 {
				filtered.Intersections = append(filtered.Intersections, inter)
			}
		}
		f.tls = &filtered
	}
	lookaheadTL := math.Max(140.0, f.speedAbs*f.speedAbs/(2*130.0)+60.0)

	isUpcomingLeft := false
	isUpcomingRight := false
	hasIntersectionExit := false

	defaultStopLineDist := f.laneWidth*1.6 + 18.0
	heading := approachRoadHeading(f.bus.Heading)
	sinH, cosH := math.Sincos(heading)

	// 1. Direct intersection route analysis:
	// Classify the movement at the nearest junction whose stop line is ahead.
	// CheckStopForVehicle applies turn intent to that junction only, so a turn
	// planned at a later junction must not be attributed to it (that granted
	// right-on-red to straight movements through closely spaced minor streets).
	type tlCandidate struct {
		inter        *sim.TrafficIntersection
		forwardDist  float64
		stopLineDist float64
		railPullback float64
	}
	var candidates []tlCandidate
	for _, inter := range f.tls.Intersections {
		stopLineDist := inter.GetStopLineDistForHeading(heading, f.laneWidth)
		railPullback := f.world.RailIntersectionPullback(inter, bumperPos, heading, f.laneWidth, f.bus.OverallLength())
		toInter := inter.Center.Sub(f.bus.Pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)
		if forwardDist > -stopLineDist && forwardDist < stopLineDist+railPullback+lookaheadTL+120.0 && lateralDist < f.laneWidth*4.8 {
			candidates = append(candidates, tlCandidate{inter, forwardDist, stopLineDist, railPullback})
		}
	}
	sort.Slice(candidates, func(i, j int) bool { return candidates[i].forwardDist < candidates[j].forwardDist })

	exitLatBase := f.laneWidth * 1.6
	if len(candidates) > 0 && f.world != nil {
		_, _, _, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
		if is6 {
			exitLatBase = f.laneWidth * 2.8
		}
	}
	fallbackLimit := -1.0
	allowYellow := ad.BerserkMode
	upcomingUTurn := false
	bumperAhead := f.bus.Length * 0.5 * math.Cos(f.bus.Heading-heading)
	var selectedInter *sim.TrafficIntersection
	railPullback := 0.0
	for _, c := range candidates {
		inter, stopLineDist := c.inter, c.stopLineDist
		exitLatThreshold := exitLatBase
		if inter.ApproachLanes[inter.GetApproachIndexForHeading(heading)] >= 8 {
			exitLatThreshold = f.laneWidth * 4.2
		}
		// Check ad.drivePath points through and beyond this intersection
		turn, hasRouteExit := ad.intersectionExitTurn(inter.Center, cosH, sinH, stopLineDist, exitLatThreshold)

		if c.forwardDist-bumperAhead-stopLineDist <= 0 {
			// Front bumper already past this stop line (CheckStopForVehicle skips
			// it). A turn here leaves the corridor, so signals farther along the
			// current heading do not apply.
			if hasRouteExit && turn != routeTurnStraight {
				return targetSpd
			}
			continue
		}

		selectedInter = inter
		railPullback = c.railPullback
		hasIntersectionExit = hasRouteExit
		isUpcomingLeft = turn == routeTurnLeft
		isUpcomingRight = turn == routeTurnRight
		fallbackLimit = c.forwardDist + stopLineDist
		if ad.uTurn != nil && !ad.uTurn.entered && inter.Center.Distance(ad.uTurn.center) < 1 {
			hasIntersectionExit, isUpcomingLeft, isUpcomingRight = true, true, false
			upcomingUTurn = true
			allowYellow = false
			// Use the controller's active window, including priority extension.
			if inter.GetSignalForHeading(heading, true) == sim.SignalGreen &&
				inter.GreenRemaining(f.tls.BusPriorityEnabled) < 1.2 {
				gap := c.forwardDist - bumperAhead - stopLineDist - c.railPullback
				return math.Min(targetSpd, math.Sqrt(2*135*math.Max(0, gap-14)))
			}
		}

		// If no exit waypoint was found, check if vehicle is physically positioned in dedicated left-turn lane
		if !hasRouteExit && inter.ApproachHasDedicatedLanesForHeading(f.bus.Heading) {
			distFromCenter := 0.0
			if math.Abs(cosH) > math.Abs(sinH) {
				distFromCenter = math.Abs(f.bus.Pos.Y - inter.Center.Y)
			} else {
				distFromCenter = math.Abs(f.bus.Pos.X - inter.Center.X)
			}
			if distFromCenter > 5.0 && distFromCenter < f.laneWidth*0.95 {
				isUpcomingLeft = true
			}
		}
		break
	}

	// 2. Fallback waypoint turn angle check (for test fixtures without full intersection objects)
	if !hasIntersectionExit && !isUpcomingLeft && !isUpcomingRight {
		lookaheadTurn := math.Max(450.0, lookaheadTL+defaultStopLineDist+150.0)
		if fallbackLimit > 0 {
			// Do not attribute a turn at a later junction to the nearest one.
			lookaheadTurn = math.Min(lookaheadTurn, fallbackLimit)
		}
		for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
			d := ad.cumLen[k] - f.s
			if d < -10.0 {
				continue
			}
			if d > lookaheadTurn {
				break
			}
			segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
			segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
			// Ignore pure lateral lane-merge jogs or tapers that stay along current road heading
			angPrev := sim.NormalizeAngle(math.Atan2(segPrev.Y, segPrev.X) - f.bus.Heading)
			angNext := sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - f.bus.Heading)
			if math.Abs(angPrev) < 0.45 && math.Abs(angNext) < 0.45 {
				continue
			}
			turnAngle := sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X))
			if turnAngle > 0.35 && math.Abs(turnAngle) < 2.8 {
				isUpcomingRight = true
			} else if turnAngle < -0.35 || math.Abs(turnAngle) >= 2.8 {
				isUpcomingLeft = true
			}
		}
	}

	// Mutual exclusion: left turn cannot be right turn
	if isUpcomingLeft {
		isUpcomingRight = false
	}

	// While returning before the line, consult the intended through signal.
	// The merge controller still stops for an occupied gap, and red through
	// signals retain their normal longitudinal stopping profile.
	if ad.approachMerge != nil && !upcomingUTurn {
		right := sim.Vec2{X: -sinH, Y: cosH}
		bumperPos = bumperPos.Add(right.Mul(ad.approachMerge.Target.Sub(bumperPos).Dot(right)))
		isUpcomingLeft, isUpcomingRight = ad.approachMerge.LeftTurn, ad.approachMerge.RightTurn
		if selectedInter != nil {
			railPullback = f.world.RailIntersectionPullback(selectedInter, bumperPos, heading, f.laneWidth, f.bus.OverallLength())
		}
	}
	onRailCrossing := f.world.VehicleOnLevelCrossing(bumperPos, heading, f.bus.OverallLength())
	if railPullback > 0 && selectedInter != nil {
		allowYellow = false
		idx := selectedInter.GetApproachIndexForHeading(heading)
		lateral := math.Abs(bumperPos.Sub(selectedInter.Center).Dot(sim.Vec2{X: -sinH, Y: cosH}))
		medianRight := isUpcomingRight && !isUpcomingLeft && selectedInter.ApproachMedianBusLane[idx] && lateral < f.laneWidth
		rightOnRed := isUpcomingRight && !isUpcomingLeft && !medianRight
		if !rightOnRed {
			sig := selectedInter.GetSignalForHeading(heading, isUpcomingLeft)
			greenDur := selectedInter.GreenDuration
			if isUpcomingLeft && selectedInter.ApproachHasDedicatedLanesForHeading(heading) {
				greenDur = selectedInter.LeftGreenDuration
			}
			if medianRight {
				sig = selectedInter.GetMedianBusRightSignal(heading)
				greenDur = selectedInter.LeftGreenDuration
			}
			distToStopLine := selectedInter.Center.Sub(bumperPos).Dot(sim.Vec2{X: cosH, Y: sinH}) - selectedInter.GetStopLineDistForHeading(heading, f.laneWidth)
			distToHold := distToStopLine - railPullback
			if distToHold > 0 {
				distToClearRail := distToHold + 210.0 + f.bus.OverallLength()
				timeNeeded := distToClearRail / math.Max(f.speedAbs, 90.0)
				if f.speedAbs < 40.0 {
					timeNeeded = math.Sqrt(2*distToClearRail/70.0) + 0.8
				}
				greenRem := greenDur - selectedInter.Timer
				if sig != sim.SignalGreen || (greenDur > 0 && greenRem < timeNeeded) {
					if distToHold < lookaheadTL {
						if distToHold <= 14.0 {
							return 0
						}
						return math.Min(targetSpd, math.Sqrt(2*135.0*math.Max(0.0, distToHold-14.0)))
					}
				}
			}
		}
	} else if onRailCrossing {
		allowYellow = true
	}
	checkSpeed := f.speedAbs
	if onRailCrossing {
		checkSpeed = math.Max(checkSpeed, 140.0)
	}
	// Berserk mode may proceed on yellow only when it reaches the line before
	// red; otherwise it would enter on red, so it brakes like normal mode.
	shouldStop, distToLine := f.tls.CheckStopForVehicleAtSpeed(bumperPos, heading, lookaheadTL+railPullback, f.laneWidth, isUpcomingLeft, isUpcomingRight, allowYellow, checkSpeed, hasIntersectionExit || ad.approachMerge != nil)
	if !shouldStop {
		return targetSpd
	}
	distToLine, _ = f.world.ClampStopBeforeRailCrossing(bumperPos, heading, distToLine, f.bus.OverallLength(), 14.0)
	if distToLine <= 14.0 {
		return 0
	}
	stopProfile := math.Sqrt(2 * 135.0 * math.Max(0.0, distToLine-14.0))
	return math.Min(targetSpd, stopProfile)
}
