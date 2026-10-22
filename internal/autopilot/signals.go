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
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	// 1. Direct intersection route analysis:
	// Classify the movement at the nearest junction whose stop line is ahead.
	// CheckStopForVehicle applies turn intent to that junction only, so a turn
	// planned at a later junction must not be attributed to it (that granted
	// right-on-red to straight movements through closely spaced minor streets).
	type tlCandidate struct {
		inter        *sim.TrafficIntersection
		forwardDist  float64
		stopLineDist float64
	}
	var candidates []tlCandidate
	for _, inter := range f.tls.Intersections {
		stopLineDist := inter.GetStopLineDistForHeading(heading, f.laneWidth)
		toInter := inter.Center.Sub(f.bus.Pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)
		if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookaheadTL+120.0 && lateralDist < f.laneWidth*4.8 {
			candidates = append(candidates, tlCandidate{inter, forwardDist, stopLineDist})
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
	bumperAhead := f.bus.Length * 0.5 * math.Cos(f.bus.Heading-heading)
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

		hasIntersectionExit = hasRouteExit
		isUpcomingLeft = turn == routeTurnLeft
		isUpcomingRight = turn == routeTurnRight
		fallbackLimit = c.forwardDist + stopLineDist

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

	bumperPos := f.bus.Pos.Add(sim.Vec2{X: math.Cos(f.bus.Heading) * (f.bus.Length * 0.5), Y: math.Sin(f.bus.Heading) * (f.bus.Length * 0.5)})

	// While returning before the line, consult the intended through signal.
	// The merge controller still stops for an occupied gap, and red through
	// signals retain their normal longitudinal stopping profile.
	if ad.approachMerge != nil {
		right := sim.Vec2{X: -sinH, Y: cosH}
		bumperPos = bumperPos.Add(right.Mul(ad.approachMerge.Target.Sub(bumperPos).Dot(right)))
	}
	if ad.approachMerge != nil {
		isUpcomingLeft, isUpcomingRight = ad.approachMerge.LeftTurn, ad.approachMerge.RightTurn
	}
	// Berserk mode may proceed on yellow only when it reaches the line before
	// red; otherwise it would enter on red, so it brakes like normal mode.
	shouldStop, distToLine := f.tls.CheckStopForVehicleAtSpeed(bumperPos, heading, lookaheadTL, f.laneWidth, isUpcomingLeft, isUpcomingRight, ad.BerserkMode, f.speedAbs, hasIntersectionExit || ad.approachMerge != nil)
	if !shouldStop {
		return targetSpd
	}
	if distToLine <= 14.0 {
		return 0
	}
	stopProfile := math.Sqrt(2 * 135.0 * math.Max(0.0, distToLine-14.0))
	return math.Min(targetSpd, stopProfile)
}
