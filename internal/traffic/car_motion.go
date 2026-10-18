package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func moveCar(car *sim.TrafficCar, dt float64) {
	// 3. Move along Heading or Circulate in Roundabout
	if car.InRoundabout {
		omega := car.Speed / car.RoundaboutRadius
		dTheta := omega * dt
		car.RoundaboutAngle -= dTheta
		car.RoundaboutTraveled += dTheta
		car.RoundaboutAngle = sim.NormalizeAngle(car.RoundaboutAngle)

		// Angle remaining to destination exit (decreasing counter-clockwise towards exit)
		diff := sim.NormalizeAngle(car.RoundaboutAngle - car.RoundaboutExitAngle)
		for diff < 0 {
			diff += 2 * math.Pi
		}

		// Keep car strictly tracking circular lane arc at circulating radius
		targetR := car.RoundaboutTargetR
		tangentH := sim.NormalizeAngle(car.RoundaboutAngle - math.Pi/2)

		maxTurnRate := 5.0 // rad/s (generates natural steering rate without snaps)
		maxTurnStep := maxTurnRate * dt

		// Smoothly ease heading into outgoing road heading as the car approaches the exit junction mouth,
		// without expanding radius into corners (targetR remains strictly on circular arc!).
		if car.RoundaboutTraveled >= 1.0 && diff <= 0.45 {
			tExit := 1.0 - math.Max(0, math.Min(1, diff/0.45))
			targetH := sim.LerpAngle(tangentH, car.RoundaboutExitHeading, tExit)
			car.Heading = sim.RotateTowards(car.Heading, targetH, maxTurnStep)
		} else {
			car.Heading = sim.RotateTowards(car.Heading, tangentH, maxTurnStep)
		}

		// Smooth radial approach to circular lane radius capped to smooth physical lateral rate
		radialDiff := targetR - car.RoundaboutRadius
		maxRadialStep := 150.0 * dt
		if radialDiff > maxRadialStep {
			car.RoundaboutRadius += maxRadialStep
		} else if radialDiff < -maxRadialStep {
			car.RoundaboutRadius -= maxRadialStep
		} else {
			car.RoundaboutRadius = targetR
		}

		// Continuous position along circular trajectory
		car.Pos = car.RoundaboutCenter.Add(sim.Vec2{
			X: math.Cos(car.RoundaboutAngle) * car.RoundaboutRadius,
			Y: math.Sin(car.RoundaboutAngle) * car.RoundaboutRadius,
		})

		// Seamless exit onto outgoing lane when aligned with exit junction
		if car.RoundaboutTraveled >= 1.0 && (diff <= 0.06 || diff >= 2*math.Pi-0.06) {
			car.InRoundabout = false
			car.Heading = car.RoundaboutExitHeading
			car.IsEW = car.RoundaboutExitEW
			car.RoadCenter = car.RoundaboutExitRC
			car.RoundaboutCooldown = 6.0
			// Do NOT snap car.Pos! car.Pos is already at the exit perimeter on the lane!
		}
	} else if car.Speed > 0 {
		car.Pos.X += car.Speed * math.Cos(car.Heading) * dt
		car.Pos.Y += car.Speed * math.Sin(car.Heading) * dt
	}

}

func (tm *TrafficManager) turnCarAtIntersection(f carFrame) {
	// 3.5 AI Intersection Turning:
	// If the road does not continue straight (T-junction, corner, or boundary), the vehicle MUST turn!
	// At standard 4-way intersections, cars in Lane 0 (left-turn lane) turn left.
	if f.tls != nil && !f.car.InRoundabout {
		for _, inter := range f.tls.Intersections {
			if f.world != nil && f.world.IsAtRoundabout(inter.Center) {
				continue
			}
			if f.car.Pos.Distance(inter.Center) < 32.0 {
				hasStraight := hasRoadInDirection(f.world, inter.Center, f.car.Heading)
				leftHeading := sim.NormalizeAngle(f.car.Heading - math.Pi/2)
				hasLeft := hasRoadInDirection(f.world, inter.Center, leftHeading)
				rightHeading := sim.NormalizeAngle(f.car.Heading + math.Pi/2)
				hasRight := hasRoadInDirection(f.world, inter.Center, rightHeading)

				shouldTurn := false
				var targetHeading float64
				turnIsLeft := false

				if !hasStraight {
					// Forced turn: dead end / T-junction / map corner
					if hasLeft && hasRight {
						if f.car.LaneIndex == 0 {
							targetHeading = leftHeading
							turnIsLeft = true
							shouldTurn = true
						} else {
							targetHeading = rightHeading
							turnIsLeft = false
							shouldTurn = true
						}
					} else if hasLeft {
						targetHeading = leftHeading
						turnIsLeft = true
						shouldTurn = true
					} else if hasRight {
						targetHeading = rightHeading
						turnIsLeft = false
						shouldTurn = true
					}
				} else if f.car.LaneIndex == 0 && hasLeft && inter.ApproachHasDedicatedLanes(f.car.IsEW) {
					// Left turn from dedicated turning bay
					targetHeading = leftHeading
					turnIsLeft = true
					shouldTurn = true
				}

				if shouldTurn {
					// Yield to stopped f.bus inside the intersection
					busInside := false
					for _, b := range f.buses {
						if inter.Center.Distance(b.Pos) < 110.0 && math.Abs(b.Speed) < 8.0 && !sim.IsBusFacingWrongWay(b) {
							busInside = true
							break
						}
					}
					if busInside {
						f.car.Speed = 0
						f.car.IsBraking = true
						break
					}

					newIsEW := math.Abs(math.Cos(targetHeading)) > math.Abs(math.Sin(targetHeading))
					newRC := inter.Center.Y
					if !newIsEW {
						newRC = inter.Center.X
					}
					newLanes := 2
					if f.world != nil {
						newLanes = f.world.GetRoadLanes(newRC, newIsEW, inter.Center)
					}
					newOffset := f.laneWidth * 0.5
					if newLanes >= 6 && !turnIsLeft {
						newOffset = f.laneWidth * 1.5
					} else if newLanes >= 4 && !turnIsLeft {
						newOffset = f.laneWidth * 1.5
					}

					f.car.Heading = targetHeading
					f.car.IsEW = newIsEW
					f.car.RoadCenter = newRC

					if newIsEW {
						if math.Cos(targetHeading) > 0 {
							f.car.Pos.Y = newRC + newOffset
						} else {
							f.car.Pos.Y = newRC - newOffset
						}
					} else {
						if math.Sin(targetHeading) > 0 {
							f.car.Pos.X = newRC - newOffset
						} else {
							f.car.Pos.X = newRC + newOffset
						}
					}
					f.car.LaneIndex = 1
					break
				}
			}
		}
	}

}
