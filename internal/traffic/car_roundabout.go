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
		if f.bus != nil && !canPassWaitingRoundaboutBus(f, f.bus) && (f.world.IsAtRoundabout(f.bus.Pos) || f.bus.Pos.Distance(f.car.RoundaboutCenter) <= f.car.RoundaboutOuterRadius+25.0) {
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
				if !canPassWaitingRoundaboutBus(f, ab.Bus) && (f.world.IsAtRoundabout(ab.Bus.Pos) || ab.Bus.Pos.Distance(f.car.RoundaboutCenter) <= f.car.RoundaboutOuterRadius+25.0) {
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
			forward := roundaboutApproachDirection(f.car)
			fwdProj := rb.Center.Sub(f.car.Pos).Dot(forward)
			coord := rb.Center.X
			if f.car.IsEW {
				coord = rb.Center.Y
			}
			stopGap := roundaboutYieldGap(f.car, rb)
			lookahead := math.Max(100, f.car.Speed*f.car.Speed/(2*220)+math.Abs(f.car.Speed)*.2+40)
			if stopGap <= lookahead && math.Abs(f.car.RoadCenter-coord) < 25 && f.car.RoundaboutCooldown <= 0 {
				if fwdProj > 0 {

					// 1. Yield check at entry line:
					// Circulating vehicles coming from the left or traversing across entry mouth have right of way
					myAngle := math.Atan2(f.car.Pos.Y-rb.Center.Y, f.car.Pos.X-rb.Center.X)
					needYield := false

					// Account for long buses whose noses are already at the mouth
					// while their centers are still outside the circular roadway.
					for _, bus := range f.buses {
						if bus == nil || canPassWaitingRoundaboutBus(f, bus) || bus.Pos.Distance(rb.Center) > rb.Radius+bus.Length/2+40 {
							continue
						}
						angle := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
						diff := sim.NormalizeAngle(angle - myAngle)
						if (diff > -.4 && diff < 1.15) || f.car.Pos.Distance(bus.Pos) < 140 {
							needYield = true
							break
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

					if needYield {
						f.car.IsBraking = true
						targetSpd = math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, stopGap-6)))
					}

					// 2. Enter roundabout when at/inside the circular roadway and NOT yielding
					canEnter := !needYield && distToCenter <= rb.Radius+12.0
					if canEnter {
						// Strict safety clearance: cannot enter if any bus/car is within 85px of entrance mouth
						if f.bus != nil && f.car.Pos.Distance(f.bus.Pos) < 85.0 && !canPassWaitingRoundaboutBus(f, f.bus) {
							canEnter = false
						} else if tm.AIBusMgr != nil {
							for _, ab := range tm.AIBusMgr.Buses {
								if f.car.Pos.Distance(ab.Bus.Pos) < 85.0 && !canPassWaitingRoundaboutBus(f, ab.Bus) {
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

						configureRoundaboutExit(f.car, f.world, &rb, f.laneWidth, incomingHeading)
						beginRoundaboutEntry(f.car)

						break
					}
				}
			}
		}
	}

	return targetSpd
}
