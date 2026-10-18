package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) roundaboutSpeed(f carFrame, targetSpd float64) float64 {
	// E. Check Roundabout Navigation & Yielding
	if f.world == nil || len(f.world.Roundabouts) == 0 {
		return targetSpd
	}
	if f.car.InRoundabout {
		// Speed limit inside roundabout for safe, smooth rotary circulation
		targetSpd = math.Min(targetSpd, 115.0)

		// Check if bus is ahead inside the roundabout or in direct proximity
		if f.bus != nil && (f.world.IsAtRoundabout(f.bus.Pos) || f.bus.Pos.Distance(f.car.RoundaboutCenter) <= f.car.RoundaboutOuterRadius+25.0) {
			directDist := f.car.Pos.Distance(f.bus.Pos)
			if directDist < 85.0 {
				f.car.IsBraking = true
				targetSpd = 0
			} else {
				busDist := f.bus.Pos.Distance(f.car.RoundaboutCenter)
				if math.Abs(f.car.RoundaboutTargetR-busDist) <= 45.0 {
					busAngle := math.Atan2(f.bus.Pos.Y-f.car.RoundaboutCenter.Y, f.bus.Pos.X-f.car.RoundaboutCenter.X)
					angDiff := sim.NormalizeAngle(f.car.RoundaboutAngle - busAngle)
					if angDiff > 0.0 && angDiff < 0.70 {
						f.car.IsBraking = true
						targetSpd = 0
					} else if angDiff >= 0.70 && angDiff < 1.20 {
						f.car.IsBraking = true
						targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.65)*160.0))
					}
				}
			}
		}

		// Check if AI route bus is ahead inside roundabout
		if tm.AIBusMgr != nil {
			for _, ab := range tm.AIBusMgr.Buses {
				if f.world.IsAtRoundabout(ab.Bus.Pos) || ab.Bus.Pos.Distance(f.car.RoundaboutCenter) <= f.car.RoundaboutOuterRadius+25.0 {
					directDist := f.car.Pos.Distance(ab.Bus.Pos)
					if directDist < 85.0 {
						f.car.IsBraking = true
						targetSpd = 0
						break
					} else {
						abDist := ab.Bus.Pos.Distance(f.car.RoundaboutCenter)
						if math.Abs(f.car.RoundaboutTargetR-abDist) <= 45.0 {
							abAngle := math.Atan2(ab.Bus.Pos.Y-f.car.RoundaboutCenter.Y, ab.Bus.Pos.X-f.car.RoundaboutCenter.X)
							angDiff := sim.NormalizeAngle(f.car.RoundaboutAngle - abAngle)
							if angDiff > 0.0 && angDiff < 0.70 {
								f.car.IsBraking = true
								targetSpd = 0
								break
							} else if angDiff >= 0.70 && angDiff < 1.20 {
								f.car.IsBraking = true
								targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.65)*160.0))
								break
							}
						}
					}
				}
			}
		}

		// Check if another AI car is ahead inside the roundabout in our lane
		for j := 0; j < f.activeCount; j++ {
			if j == f.index {
				continue
			}
			other := tm.Cars[j]
			if other.InRoundabout && other.RoundaboutCenter == f.car.RoundaboutCenter {
				directDist := f.car.Pos.Distance(other.Pos)
				if directDist < 60.0 {
					f.car.IsBraking = true
					targetSpd = 0
					break
				}
				if math.Abs(f.car.RoundaboutTargetR-other.RoundaboutTargetR) > 35.0 {
					continue // In a different concentric circular lane
				}
				angDiff := sim.NormalizeAngle(f.car.RoundaboutAngle - other.RoundaboutAngle)
				if angDiff > 0.0 && angDiff < 0.52 {
					f.car.IsBraking = true
					targetSpd = 0
					break
				} else if angDiff >= 0.52 && angDiff < 0.95 {
					f.car.IsBraking = true
					targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.5)*180.0))
					break
				}
			}
		}
	} else {
		// Approaching roundabout check
		for _, rb := range f.world.Roundabouts {
			distToCenter := f.car.Pos.Distance(rb.Center)
			if distToCenter <= rb.Radius+95.0 && f.car.RoundaboutCooldown <= 0 {
				toCenter := rb.Center.Sub(f.car.Pos)
				fwdProj := toCenter.X*math.Cos(f.car.Heading) + toCenter.Y*math.Sin(f.car.Heading)
				if fwdProj > 0.0 {
					// 1. Yield check at entry line:
					// Circulating vehicles coming from the left or traversing across entry mouth have right of way
					myAngle := math.Atan2(f.car.Pos.Y-rb.Center.Y, f.car.Pos.X-rb.Center.X)
					needYield := false

					// Check if player bus is circulating approaching this entry or crossing the entry mouth
					if f.bus != nil && (f.world.IsAtRoundabout(f.bus.Pos) || f.bus.Pos.Distance(rb.Center) <= rb.Radius+25.0) {
						busAng := math.Atan2(f.bus.Pos.Y-rb.Center.Y, f.bus.Pos.X-rb.Center.X)
						diff := sim.NormalizeAngle(busAng - myAngle)
						if (diff > -0.40 && diff < 1.15) || f.car.Pos.Distance(f.bus.Pos) < 140.0 {
							needYield = true
						}
					}

					// Check if AI route bus is circulating approaching this entry or crossing the entry mouth
					if !needYield && tm.AIBusMgr != nil {
						for _, ab := range tm.AIBusMgr.Buses {
							if f.world.IsAtRoundabout(ab.Bus.Pos) || ab.Bus.Pos.Distance(rb.Center) <= rb.Radius+25.0 {
								abAng := math.Atan2(ab.Bus.Pos.Y-rb.Center.Y, ab.Bus.Pos.X-rb.Center.X)
								diff := sim.NormalizeAngle(abAng - myAngle)
								if (diff > -0.40 && diff < 1.15) || f.car.Pos.Distance(ab.Bus.Pos) < 140.0 {
									needYield = true
									break
								}
							}
						}
					}

					// Check if another AI car is circulating approaching this entry
					if !needYield {
						for j := 0; j < f.activeCount; j++ {
							if j == f.index {
								continue
							}
							other := tm.Cars[j]
							if other.InRoundabout && other.RoundaboutCenter == rb.Center {
								diff := sim.NormalizeAngle(other.RoundaboutAngle - myAngle)
								if (diff > -0.30 && diff < 1.05) || f.car.Pos.Distance(other.Pos) < 110.0 {
									needYield = true
									break
								}
							}
						}
					}

					stopLineDist := rb.Radius + f.car.Length*0.5 + 16.0
					if needYield {
						if distToCenter > stopLineDist+4.0 {
							targetSpd = math.Min(targetSpd, math.Max(0.0, (distToCenter-stopLineDist)*2.0))
						} else {
							f.car.IsBraking = true
							targetSpd = 0.0
							if distToCenter <= stopLineDist+1.0 {
								f.car.Speed = 0.0
							}
						}
					}

					// 2. Enter roundabout when at/inside the circular roadway and NOT yielding
					canEnter := !needYield && distToCenter <= rb.Radius+12.0
					if canEnter {
						// Strict safety clearance: cannot enter if any bus/car is within 85px of entrance mouth
						if f.bus != nil && f.car.Pos.Distance(f.bus.Pos) < 85.0 {
							canEnter = false
						} else if tm.AIBusMgr != nil {
							for _, ab := range tm.AIBusMgr.Buses {
								if f.car.Pos.Distance(ab.Bus.Pos) < 85.0 {
									canEnter = false
									break
								}
							}
						}
					}

					if canEnter {
						incomingHeading := f.car.Heading
						f.car.InRoundabout = true
						f.car.RoundaboutCenter = rb.Center
						f.car.RoundaboutTraveled = 0

						rApron := rb.InnerRadius + 16.0
						rOuter := rb.Radius
						rMid := (rApron + rOuter) * 0.5
						circulatingR := (rMid + rOuter) * 0.5 // Outer lane center
						if f.car.LaneIndex == 0 {
							circulatingR = (rApron + rMid) * 0.5 // Inner lane center
						}
						f.car.RoundaboutTargetR = circulatingR
						f.car.RoundaboutOuterRadius = rb.Radius

						// Smooth continuous entry: initialize radius at current distance to center to eliminate any position snapping
						f.car.RoundaboutRadius = distToCenter
						f.car.RoundaboutAngle = math.Atan2(f.car.Pos.Y-rb.Center.Y, f.car.Pos.X-rb.Center.X)

						isOutgoing4Lane := false
						if math.Abs(math.Cos(incomingHeading)) > math.Abs(math.Sin(incomingHeading)) {
							// East-West road
							if rb.Center.Y == -1200 || rb.Center.Y == 0 || rb.Center.Y == 1200 {
								isOutgoing4Lane = true
							}
						} else {
							// North-South road
							if rb.Center.X == 0 {
								isOutgoing4Lane = true
							}
						}
						laneOff := f.laneWidth * 0.75
						if isOutgoing4Lane {
							if f.car.LaneIndex == 0 {
								laneOff = f.laneWidth * 0.52
							} else {
								laneOff = f.laneWidth * 1.48
							}
						}
						offsetAngle := math.Asin(math.Min(0.7, laneOff/circulatingR))

						// Determine destination exit based on incoming road approach
						if math.Abs(math.Cos(incomingHeading)) > math.Abs(math.Sin(incomingHeading)) {
							// East-West road
							if math.Cos(incomingHeading) > 0 { // Eastbound
								f.car.RoundaboutExitHeading = 0
								f.car.RoundaboutExitEW = true
								f.car.RoundaboutExitRC = rb.Center.Y
								f.car.RoundaboutExitAngle = sim.NormalizeAngle(0 + offsetAngle)
							} else { // Westbound
								f.car.RoundaboutExitHeading = math.Pi
								f.car.RoundaboutExitEW = true
								f.car.RoundaboutExitRC = rb.Center.Y
								f.car.RoundaboutExitAngle = sim.NormalizeAngle(-math.Pi + offsetAngle)
							}
						} else {
							// North-South road
							if math.Sin(incomingHeading) > 0 { // Southbound
								f.car.RoundaboutExitHeading = math.Pi / 2
								f.car.RoundaboutExitEW = false
								f.car.RoundaboutExitRC = rb.Center.X
								f.car.RoundaboutExitAngle = sim.NormalizeAngle(math.Pi/2 + offsetAngle)
							} else { // Northbound
								f.car.RoundaboutExitHeading = -math.Pi / 2
								f.car.RoundaboutExitEW = false
								f.car.RoundaboutExitRC = rb.Center.X
								f.car.RoundaboutExitAngle = sim.NormalizeAngle(-math.Pi/2 + offsetAngle)
							}
						}
						break
					}
				}
			}
		}
	}

	return targetSpd
}
