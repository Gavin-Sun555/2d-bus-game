package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) trafficLightSpeed(f autopilotFrame, targetSpd float64) float64 {
	// D. Red / Yellow Light Stopping
	if f.tls == nil {
		return targetSpd
	}
	lookaheadTL := math.Max(140.0, f.speedAbs*f.speedAbs/(2*130.0)+60.0)

	isUpcomingLeft := false
	isUpcomingRight := false

	defaultStopLineDist := f.laneWidth*1.6 + 18.0
	cosH := math.Cos(f.bus.Heading)
	sinH := math.Sin(f.bus.Heading)

	// 1. Direct intersection route analysis:
	// Check the intersection that the bus is approaching within lookahead distance
	for _, inter := range f.tls.Intersections {
		stopLineDist := inter.GetStopLineDistForHeading(f.bus.Heading, f.laneWidth)
		toInter := inter.Center.Sub(f.bus.Pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

		if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookaheadTL+120.0 && lateralDist < f.laneWidth*4.8 {
			exitLatThreshold := f.laneWidth * 1.6
			if f.world != nil {
				_, _, _, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
				if is6 {
					exitLatThreshold = f.laneWidth * 2.8
				}
			}

			// Check ad.drivePath points through and beyond this intersection
			turn, hasRouteExit := ad.intersectionExitTurn(inter.Center, cosH, sinH, stopLineDist, exitLatThreshold)
			isUpcomingLeft = turn == routeTurnLeft
			isUpcomingRight = turn == routeTurnRight

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
	}

	// 2. Fallback waypoint turn angle check (for test fixtures without full intersection objects)
	if !isUpcomingLeft && !isUpcomingRight {
		lookaheadTurn := math.Max(450.0, lookaheadTL+defaultStopLineDist+150.0)
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

	bumperPos := f.bus.Pos.Add(sim.Vec2{X: cosH * (f.bus.Length * 0.5), Y: sinH * (f.bus.Length * 0.5)})

	shouldStop, distToLine := f.tls.CheckStopForVehicle(bumperPos, f.bus.Heading, lookaheadTL, f.laneWidth, isUpcomingLeft, isUpcomingRight, ad.BerserkMode)
	if !shouldStop {
		return targetSpd
	}
	if distToLine <= 14.0 {
		return 0
	}
	stopProfile := math.Sqrt(2 * 135.0 * math.Max(0.0, distToLine-14.0))
	return math.Min(targetSpd, stopProfile)
}
