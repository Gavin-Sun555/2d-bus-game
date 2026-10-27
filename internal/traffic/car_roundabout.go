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
		rb := sim.Roundabout{}
		for _, candidate := range f.world.Roundabouts {
			if candidate.Center == f.car.RoundaboutCenter {
				rb = candidate
				break
			}
		}
		targetSpd = math.Min(targetSpd, roundaboutExitBusSpeed(f, rb))
		// Recheck the right of way until the full nose reaches the yield line.
		// InRoundabout also covers the long approach curve outside the ring.
		targetSpd = math.Min(targetSpd, tm.roundaboutEntryPrioritySpeed(f, rb))
		// Speed limit inside roundabout for safe, smooth rotary circulation
		targetSpd = math.Min(targetSpd, sim.RoundaboutSpeedLimit)

		if f.car.RoundaboutCurve != nil && f.car.RoundaboutCurve.Exiting {
			// Exiting the roundabout: only maintain headway behind any vehicle
			// directly ahead on the same exit ramp. Do not yield to circulating
			// traffic behind or alongside on the ring whose paths diverge, or
			// to oncoming vehicles in the adjacent entrance lane.
			exitDir := sim.UnitFromAngle(f.car.RoundaboutExitHeading)
			exitRight := sim.Vec2{X: -exitDir.Y, Y: exitDir.X}
			checkObstacle := func(pos sim.Vec2, heading, length, width float64) {
				// Must be moving in generally same direction along exit road
				if math.Cos(heading-f.car.RoundaboutExitHeading) < 0.7 {
					return
				}
				// Must be in or merging into our exit corridor (not oncoming entrance lane)
				otherOffset := pos.Sub(f.car.RoundaboutCenter).Dot(exitRight)
				if math.Abs(otherOffset-f.car.RoundaboutExitOffset) > f.laneWidth*0.75 {
					return
				}
				u := sim.UnitFromAngle(f.car.Heading)
				delta := pos.Sub(f.car.Pos)
				fwd := delta.Dot(u)
				lat := math.Abs(-delta.X*u.Y + delta.Y*u.X)
				if fwd > 0 && fwd < 180 && lat < f.car.Width/2+width/2+6 {
					gap := fwd - f.car.Length/2 - length/2
					if gap <= 6 {
						f.car.IsBraking = true
						targetSpd = 0
						return
					}
					targetSpd = math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, gap-6)))
				}
			}
			for _, bus := range f.buses {
				if bus != nil {
					body := bus.BodyEnvelope()
					checkObstacle(body.Center, body.Heading, body.HalfLen*2, body.HalfWid*2)
					if targetSpd <= 0 {
						return 0
					}
				}
			}
			for j := 0; j < f.activeCount; j++ {
				if j != f.index {
					other := tm.Cars[j]
					checkObstacle(other.Pos, other.Heading, other.Length, other.Width)
					if targetSpd <= 0 {
						return 0
					}
				}
			}
			return targetSpd
		}

		for _, bus := range f.buses {
			if bus == nil || bus.Pos.Distance(f.car.RoundaboutCenter) > f.car.RoundaboutOuterRadius+bus.Length/2+25 {
				continue
			}
			body := bus.BodyEnvelope()
			if (f.car.RoundaboutCurve == nil && math.Abs(bus.Speed) < 1 && roundaboutBusPathClear(f.car, bus)) || separateCirculatingBodies(f.car, body.Center, body.Heading, body.HalfLen*2, body.HalfWid*2, false) || parallelRoundaboutBus(f, rb, bus) || roundaboutBusFollowing(f.car, bus) || canPassWaitingRoundaboutBus(f, bus) {
				continue
			}
			limit := roundaboutBusHeadway(f.car, bus)
			if limit < targetSpd {
				f.car.IsBraking = true
				targetSpd = limit
			}
		}

		// Check if another AI car is ahead inside the roundabout in our lane
		for j := 0; j < f.activeCount; j++ {
			if j == f.index {
				continue
			}
			other := tm.Cars[j]
			if other.InRoundabout && other.RoundaboutCenter == f.car.RoundaboutCenter {
				if roundaboutCarPathClear(f.car, other.Pos, other.Heading, other.Length, other.Width) || separateCirculatingBodies(f.car, other.Pos, other.Heading, other.Length, other.Width, (other.RoundaboutCurve != nil && other.RoundaboutCurve.Exiting)) || parallelRoundaboutEntry(f, rb, other.Pos, other.Heading, other.Length, other.Width, false) || roundaboutVehicleFollowing(f.car, other.Pos, other.Heading, other.Length, other.Width) {
					continue
				}
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
			if fwdProj > 0 && math.Abs(f.car.RoadCenter-coord) < 25 && f.car.RoundaboutCooldown <= 0 {
				if tm.roundaboutReserved(rb.Center) {
					outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
					join := sim.RoundaboutEntryJoin(rb, forward, (float64(f.car.LaneIndex)+.5)*f.laneWidth, outer, f.laneWidth)
					start, _ := sim.RoundaboutEntryPoint(join, 0)
					gap := start.Sub(f.car.Pos).Dot(forward) - f.car.Length/2 - 8
					targetSpd = math.Min(targetSpd, math.Sqrt(2*220*math.Max(0, gap)))
				}
				outerR := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
				join := sim.RoundaboutEntryJoin(rb, forward, (float64(f.car.LaneIndex)+.5)*f.laneWidth, outerR, f.laneWidth)
				start, _ := sim.RoundaboutEntryPoint(join, 0)
				gap := math.Max(0, start.Sub(f.car.Pos).Dot(forward)-20)
				targetSpd = math.Min(targetSpd, math.Sqrt(sim.RoundaboutSpeedLimit*sim.RoundaboutSpeedLimit+2*220*gap))
			}
			lookahead := math.Max(100, f.car.Speed*f.car.Speed/(2*220)+math.Abs(f.car.Speed)*.2+40)
			if stopGap <= lookahead && math.Abs(f.car.RoadCenter-coord) < 25 && f.car.RoundaboutCooldown <= 0 {
				if fwdProj > 0 {

					// 1. Yield check at entry line:
					// Circulating vehicles coming from the left or traversing across entry mouth have right of way
					myAngle := math.Atan2(f.car.Pos.Y-rb.Center.Y, f.car.Pos.X-rb.Center.X)
					needYield := tm.roundaboutReserved(rb.Center)

					// Account for long buses whose noses are already at the mouth
					// while their centers are still outside the circular roadway.
					for _, bus := range f.buses {
						if bus == nil || parallelRoundaboutBus(f, rb, bus) || roundaboutBusFollowing(f.car, bus) || canPassWaitingRoundaboutBus(f, bus) || bus.Pos.Distance(rb.Center) > rb.Radius+bus.Length/2+40 {
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
							if other.InRoundabout && other.RoundaboutCenter == rb.Center && !parallelRoundaboutEntry(f, rb, other.Pos, other.Heading, other.Length, other.Width, false) {
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
					outerR := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
					join := sim.RoundaboutEntryJoin(rb, forward, (float64(f.car.LaneIndex)+.5)*f.laneWidth, outerR, f.laneWidth)
					start, _ := sim.RoundaboutEntryPoint(join, 0)
					canEnter := !needYield && fwdProj <= rb.Center.Sub(start).Dot(forward)
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
						legacy := false
						for _, bus := range f.buses {
							if bus != nil && bus.RoundaboutEntry == nil && bus.Pos.Distance(f.car.Pos) < 180 && math.Cos(bus.Heading-incomingHeading) > .6 {
								legacy = true
								break
							}
						}
						if legacy {
							beginLegacyRoundaboutEntry(f.car)
						} else {
							beginRoundaboutEntry(f.car, rb, f.laneWidth)
						}

						break
					}
				}
			}
		}
	}

	return targetSpd
}
