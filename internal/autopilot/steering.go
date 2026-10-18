package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// pursuitSteering tracks the current lane and applies road-edge recovery.
func (ad *AutopilotDriver) pursuitSteering(f autopilotFrame) (steer, alpha float64, targetPt sim.Vec2, nearRoundabout bool) {
	nearRoundabout = false
	var nearRb *sim.Roundabout
	if f.world != nil {
		for i := range f.world.Roundabouts {
			if f.bus.Pos.Distance(f.world.Roundabouts[i].Center) <= f.world.Roundabouts[i].Radius+80.0 {
				nearRoundabout = true
				nearRb = &f.world.Roundabouts[i]
				break
			}
		}
	}

	isTransitioningToLeftTurn := false
	is2LaneRoad := false
	if f.world != nil {
		_, _, isMulti, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
		if (isMulti || is6) && ad.TargetLaneOffset <= -f.laneWidth*0.8 && !nearRoundabout {
			isTransitioningToLeftTurn = true
		}
		if !isMulti && !is6 && !nearRoundabout {
			is2LaneRoad = true
		}
	}

	lookahead := sim.Clamp(f.speedAbs*0.55+45.0, 55.0, 140.0)
	if nearRoundabout {
		lookahead = sim.Clamp(f.speedAbs*0.35+30.0, 35.0, 52.0)
	} else if is2LaneRoad && (ad.OvertakeState == OvertakeChangingLane || (ad.LaneOffset != 0 && math.Abs(f.lateralErr) > 6.0)) {
		// On 2-lane roads borrowing oncoming lane: smooth, longer lookahead prevents steep yaw into opposite shoulder
		lookahead = sim.Clamp(f.speedAbs*0.42+50.0, 65.0, 105.0)
	} else if !isTransitioningToLeftTurn && (ad.OvertakeState == OvertakeChangingLane || (ad.LaneOffset != 0 && math.Abs(f.lateralErr) > 6.0)) {
		// Responsive lookahead during mid-block lane transition so bus steers decisively into new lane
		lookahead = sim.Clamp(f.speedAbs*0.28+36.0, 48.0, 92.0)
	} else if ad.BerserkMode {
		lookahead = sim.Clamp(f.speedAbs*0.55+45.0, 55.0, 210.0)
	}

	// Clamp lookahead when approaching or executing sharp corners so Pure Pursuit does not cut across the corner
	// into oncoming lanes before or during the intersection turn (not for circular roundabout tracking):
	if !nearRoundabout {
		startClampK := ad.WaypointIdx
		if startClampK < 1 {
			startClampK = 1
		}
		for k := startClampK; k < len(ad.drivePath)-1; k++ {
			d := ad.cumLen[k] - f.s
			if d < -20.0 {
				continue
			}
			if d > lookahead+60.0 {
				break
			}
			segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
			segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
			turn := math.Abs(sim.NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
			if k+2 < len(ad.drivePath) {
				seg2 := ad.drivePath[k+2].Sub(ad.drivePath[k+1])
				t2 := math.Abs(sim.NormalizeAngle(math.Atan2(seg2.Y, seg2.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t2 > turn {
					turn = t2
				}
			}
			if k+4 < len(ad.drivePath) {
				seg4 := ad.drivePath[k+4].Sub(ad.drivePath[k+3])
				t4 := math.Abs(sim.NormalizeAngle(math.Atan2(seg4.Y, seg4.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t4 > turn {
					turn = t4
				}
			}
			if k+6 < len(ad.drivePath) {
				seg6 := ad.drivePath[k+6].Sub(ad.drivePath[k+5])
				t6 := math.Abs(sim.NormalizeAngle(math.Atan2(seg6.Y, seg6.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t6 > turn {
					turn = t6
				}
			}

			if turn > 0.15 {
				maxLookahead := 65.0
				if turn > 0.35 {
					maxLookahead = 52.0
				}
				if turn > 0.8 {
					maxLookahead = 42.0
				}
				if lookahead > maxLookahead {
					lookahead = maxLookahead
				}
				break
			}
		}
	}
	baseTargetPt := ad.pointAt(f.s + lookahead)
	targetPt = baseTargetPt
	if ad.LaneOffset != 0 && len(ad.drivePath) >= 2 {
		nextPt := ad.pointAt(f.s + lookahead + 2.0)
		toNext := nextPt.Sub(baseTargetPt)
		var uNext sim.Vec2
		if toNext.Length() > 1e-3 {
			uNext = toNext.Normalize()
		} else {
			// At the very end of drivePath, extrapolate from the last path segment:
			lastSeg := ad.drivePath[len(ad.drivePath)-1].Sub(ad.drivePath[len(ad.drivePath)-2])
			if lastSeg.Length() > 1e-3 {
				uNext = lastSeg.Normalize()
			}
		}
		if uNext.Length() > 1e-3 {
			trgRight := sim.Vec2{X: -uNext.Y, Y: uNext.X}
			effOffset := ad.LaneOffset
			if nearRb != nil {
				effOffset = roundaboutLaneOffset(nearRb, f.bus, effOffset)
			} else if f.world != nil {
				roadCenter, isEW, isMultiLane, is6Lane := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
				if !isMultiLane && !is6Lane {
					roadCenter, isEW, isMultiLane, is6Lane = sim.CurrentRoadInfo(f.world, baseTargetPt, f.bus.Heading, f.laneWidth)
				}
				if isMultiLane || is6Lane {
					dBaseCenter := math.Abs(baseTargetPt.Y - roadCenter)
					if !isEW {
						dBaseCenter = math.Abs(baseTargetPt.X - roadCenter)
					}
					// Innermost lane center is at 0.5*laneWidth from road center.
					// Vehicle must never cross the double-yellow line into oncoming traffic on multi-lane roads.
					maxLeftShift := math.Max(0.0, dBaseCenter-f.laneWidth*0.5)
					if effOffset < -maxLeftShift {
						effOffset = -maxLeftShift
					}
				} else {
					// On 2-lane roads: clamp effOffset so targetPt stays at or inside oncoming lane center
					maxBorrowShift := f.laneWidth * 0.88
					if effOffset < -maxBorrowShift {
						effOffset = -maxBorrowShift
					}
				}
			}

			targetPt = baseTargetPt.Add(trgRight.Mul(effOffset))

			if nearRb != nil {
				rApron := nearRb.InnerRadius + 16.0
				midR := (rApron + nearRb.Radius) * 0.5

				if entryTarget, joining := ad.innerRoundaboutEntryTarget(f, nearRb, lookahead, effOffset); joining {
					targetPt = entryTarget
				} else if baseTargetPt.Distance(nearRb.Center) <= nearRb.Radius+10.0 {
					toTrg := targetPt.Sub(nearRb.Center)
					distTrg := toTrg.Length()
					if distTrg > 1e-3 {
						uTrg := toTrg.Mul(1.0 / distTrg)
						minAllowedR := roundaboutOuterLaneRadius(nearRb) + roundaboutLaneOffset(nearRb, f.bus, math.Inf(-1))
						maxAllowedR := nearRb.Radius - 10.0
						if effOffset < -f.laneWidth*0.3 {
							maxAllowedR = midR - 2.0
						}
						clampedR := sim.Clamp(distTrg, minAllowedR, math.Max(minAllowedR, maxAllowedR))
						targetPt = nearRb.Center.Add(uTrg.Mul(clampedR))
					}
				}
			} else if f.world != nil {
				roadCenter, isEW, isMultiLane, is6Lane := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
				if isMultiLane || is6Lane {
					// Guard targetPt so it never crosses the double yellow line (0.5*laneWidth from roadCenter)
					if isEW {
						if math.Cos(f.bus.Heading) > 0 {
							if targetPt.Y < roadCenter+f.laneWidth*0.5 {
								targetPt.Y = roadCenter + f.laneWidth*0.5
							}
						} else {
							if targetPt.Y > roadCenter-f.laneWidth*0.5 {
								targetPt.Y = roadCenter - f.laneWidth*0.5
							}
						}
					} else {
						if math.Sin(f.bus.Heading) > 0 {
							if targetPt.X > roadCenter-f.laneWidth*0.5 {
								targetPt.X = roadCenter - f.laneWidth*0.5
							}
						} else {
							if targetPt.X < roadCenter+f.laneWidth*0.5 {
								targetPt.X = roadCenter + f.laneWidth*0.5
							}
						}
					}
				} else {
					// 2-lane road opposite shoulder guard:
					// Opposite shoulder is at roadCenter ± laneWidth.
					// Guard targetPt so it never penetrates closer than 0.52*laneWidth to the opposite shoulder!
					marginFromShoulder := f.laneWidth * 0.52
					if isEW {
						if math.Cos(f.bus.Heading) > 0 {
							minAllowedY := roadCenter - f.laneWidth + marginFromShoulder
							if targetPt.Y < minAllowedY {
								targetPt.Y = minAllowedY
							}
						} else {
							maxAllowedY := roadCenter + f.laneWidth - marginFromShoulder
							if targetPt.Y > maxAllowedY {
								targetPt.Y = maxAllowedY
							}
						}
					} else {
						if math.Sin(f.bus.Heading) > 0 {
							// Traveling South: oncoming lane is East (X > roadCenter), opposite shoulder is East (max X)
							maxAllowedX := roadCenter + f.laneWidth - marginFromShoulder
							if targetPt.X > maxAllowedX {
								targetPt.X = maxAllowedX
							}
						} else {
							// Traveling North: oncoming lane is West (X < roadCenter), opposite shoulder is West (min X)
							minAllowedX := roadCenter - f.laneWidth + marginFromShoulder
							if targetPt.X < minAllowedX {
								targetPt.X = minAllowedX
							}
						}
					}
				}
			}
		}
	}

	// Station approach lateral protection:
	// 1. When bus is waiting alongside or passing in adjacent lane approaching destination stop (TargetLaneOffset < -laneWidth*0.3 || LaneOffset < -laneWidth*0.3), OR
	// 2. When bus has reached the stop area (remaining < 80.0) but is laterally separated from the bay (distToStop > 25.0 && !targetStop.ContainsBus(bus)), OR
	// 3. When destination stop can be skipped (ad.CanSkipTargetStop):
	// Keep targetPt centered on the main roadway travel lane, preventing it from steering into the curb bay!
	isOuterLanePassing := ad.TargetLaneOffset < -f.laneWidth*0.3 && f.distToStop < 250.0
	if nearRb == nil && f.world != nil && f.distToStop < 250.0 && f.remaining < 350.0 && (isOuterLanePassing || ad.CanSkipTargetStop) {
		roadCenter, isEW, isMultiLane, is6Lane := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
		if isMultiLane || is6Lane {
			distAdj := f.laneWidth * 0.5 // Inner lane on 4-lane
			if isOuterLanePassing {
				if is6Lane {
					if ad.TargetLaneOffset <= -f.laneWidth*1.5 || ad.LaneOffset <= -f.laneWidth*1.5 {
						distAdj = f.laneWidth * 0.5 // Leftmost lane
					} else {
						distAdj = f.laneWidth * 1.5 // Middle lane
					}
				}
			} else {
				// Base through travel lane when skipping the stop
				distAdj = f.laneWidth * 1.5
				if is6Lane {
					distAdj = f.laneWidth * 2.5
				}
			}
			if isEW {
				if math.Cos(f.bus.Heading) > 0 {
					targetPt.Y = roadCenter + distAdj
				} else {
					targetPt.Y = roadCenter - distAdj
				}
			} else {
				if math.Sin(f.bus.Heading) > 0 {
					targetPt.X = roadCenter - distAdj
				} else {
					targetPt.X = roadCenter + distAdj
				}
			}
		} else {
			// 2-lane road
			if ad.TargetLaneOffset < -f.laneWidth*0.3 {
				// Borrowed oncoming lane
				distAdj := f.laneWidth * 0.5
				if isEW {
					if math.Cos(f.bus.Heading) > 0 {
						targetPt.Y = roadCenter - distAdj
					} else {
						targetPt.Y = roadCenter + distAdj
					}
				} else {
					if math.Sin(f.bus.Heading) > 0 {
						targetPt.X = roadCenter + distAdj
					} else {
						targetPt.X = roadCenter - distAdj
					}
				}
			} else {
				// Normal travel lane
				distAdj := f.laneWidth * 0.5
				if isEW {
					if math.Cos(f.bus.Heading) > 0 {
						targetPt.Y = roadCenter + distAdj
					} else {
						targetPt.Y = roadCenter - distAdj
					}
				} else {
					if math.Sin(f.bus.Heading) > 0 {
						targetPt.X = roadCenter - distAdj
					} else {
						targetPt.X = roadCenter + distAdj
					}
				}
			}
		}

		// Longitudinal forward guard:
		// When bypassing, passing in outer lane, or approaching past the station center without docking,
		// targetPt must be projected ahead of the bus along the travel heading so the bus drives forward,
		// never pulling targetPt behind the vehicle!
		cosH := math.Cos(f.targetStop.Heading)
		sinH := math.Sin(f.targetStop.Heading)
		fwdDist := (targetPt.X-f.bus.Pos.X)*cosH + (targetPt.Y-f.bus.Pos.Y)*sinH
		minFwd := math.Max(lookahead, 35.0)
		if fwdDist < minFwd {
			targetPt = targetPt.Add(sim.Vec2{X: cosH * (minFwd - fwdDist), Y: sinH * (minFwd - fwdDist)})
		}
	}
	toTarget := targetPt.Sub(f.bus.Pos)
	alpha = sim.NormalizeAngle(math.Atan2(toTarget.Y, toTarget.X) - f.bus.Heading)
	if math.Abs(alpha) > math.Pi*0.5 {
		// Target is behind the vehicle.
		// On roads, a vehicle must NEVER lock steering at ±1.0 (±39°) to make sharp U-turns into oncoming traffic.
		// Instead, determine the forward road travel direction and steer smoothly forward.
		roadHeading := f.bus.Heading
		if f.targetStop != nil {
			roadHeading = f.targetStop.Heading
		} else if f.world != nil {
			_, isEW, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
			if isEW {
				if math.Cos(f.bus.Heading) >= 0 {
					roadHeading = 0
				} else {
					roadHeading = math.Pi
				}
			} else {
				if math.Sin(f.bus.Heading) >= 0 {
					roadHeading = math.Pi / 2
				} else {
					roadHeading = -math.Pi / 2
				}
			}
		}
		headingDiff := sim.NormalizeAngle(roadHeading - f.bus.Heading)
		if math.Abs(headingDiff) < math.Pi*0.65 {
			alpha = sim.Clamp(headingDiff, -math.Pi*0.35, math.Pi*0.35)
		} else {
			// Vehicle is misaligned: smooth recovery steer without full-lock spinout
			alpha = sim.Clamp(alpha, -math.Pi*0.35, math.Pi*0.35)
		}
	} else {
		alpha = sim.Clamp(alpha, -math.Pi/2, math.Pi/2)
	}
	ld := math.Max(toTarget.Length(), 25.0)
	delta := math.Atan2(2.0*f.bus.WheelBase*math.Sin(alpha), ld)
	steer = sim.Clamp(delta/f.bus.MaxSteerAngle, -1.0, 1.0)

	// Double-yellow line collision protection on multi-lane and 6-lane roads:
	// When running parallel to the road centerline on an intersection approach or road segment,
	// enforce that vehicle's front corner clearance to the double-yellow line >= margin.
	if f.world != nil {
		rCenter, rEW, isMulti, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
		if isTransitioningToLeftTurn && (isMulti || is6) && !nearRoundabout {
			roadHeading := 0.0
			dCenter := f.bus.Pos.Y - rCenter
			if rEW {
				if math.Cos(f.bus.Heading) < 0 {
					roadHeading = math.Pi
					dCenter = rCenter - f.bus.Pos.Y
				}
			} else {
				if math.Sin(f.bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
					dCenter = rCenter - f.bus.Pos.X
				} else {
					roadHeading = -math.Pi / 2.0
					dCenter = f.bus.Pos.X - rCenter
				}
			}

			// Enforce when bus is on our side of the road and in or near inner/middle lane corridor (<= 1.9*lw)
			if dCenter > 0 && dCenter <= f.laneWidth*1.9 {
				isBeforeInterMouth := true
				if f.tls != nil {
					for _, inter := range f.tls.Intersections {
						cosH := math.Cos(f.bus.Heading)
						sinH := math.Sin(f.bus.Heading)
						toInter := inter.Center.Sub(f.bus.Pos)
						fwdDist := toInter.X*cosH + toInter.Y*sinH
						stopLineOffset := inter.GetStopLineDistForHeading(f.bus.Heading, f.laneWidth)
						if fwdDist > -30.0 && fwdDist < stopLineOffset*0.35 {
							isBeforeInterMouth = false
							break
						}
					}
				}

				if isBeforeInterMouth {
					psi := sim.NormalizeAngle(roadHeading - f.bus.Heading)
					halfL := f.bus.Length * 0.5
					halfW := f.bus.Width * 0.5
					margin := 1.5

					// Compute maximum safe yaw angle towards oncoming traffic:
					// While transitioning towards the innermost left-turn lane, yaw should not exceed ~18.3 deg (0.32 rad)
					// and must dynamically tighten to ensure front-left corner never violates the double-yellow line.
					avail := (dCenter - halfW - margin) / halfL
					maxPsi := 0.32
					if avail <= 0.0 {
						maxPsi = 0.0
					} else if avail < 1.0 {
						geomPsi := math.Asin(avail)
						if geomPsi < maxPsi {
							maxPsi = geomPsi
						}
					}

					if psi > maxPsi {
						// Heading is too steep towards double-yellow line!
						excess := psi - maxPsi
						correctiveSteer := sim.Clamp(excess*6.0+0.25, 0.30, 1.0)
						if steer < correctiveSteer {
							steer = correctiveSteer
						}
					} else if psi > maxPsi*0.75 {
						if steer < 0 {
							steer = 0.0
						}
					}
					if dCenter < halfW+0.5 && psi > 0 {
						if steer < 0.20 {
							steer = 0.20
						}
					}
				}
			}
		}

		// Opposite shoulder collision protection on 2-lane roads when borrowing oncoming lane:
		// When borrowing oncoming lane (LaneOffset < 0) or vehicle position is near oncoming half, enforce clearance to opposite shoulder.
		if !isMulti && !is6 && !nearRoundabout {
			roadHeading := 0.0
			dOppositeShoulder := 0.0

			if rEW {
				if math.Cos(f.bus.Heading) > 0 {
					roadHeading = 0.0
					dOppositeShoulder = f.bus.Pos.Y - (rCenter - f.laneWidth)
				} else {
					roadHeading = math.Pi
					dOppositeShoulder = (rCenter + f.laneWidth) - f.bus.Pos.Y
				}
			} else {
				if math.Sin(f.bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
					dOppositeShoulder = (rCenter + f.laneWidth) - f.bus.Pos.X
				} else {
					roadHeading = -math.Pi / 2.0
					dOppositeShoulder = f.bus.Pos.X - (rCenter - f.laneWidth)
				}
			}

			if ad.LaneOffset < -f.laneWidth*0.25 || ad.TargetLaneOffset < -f.laneWidth*0.25 {
				psiTowardsShoulder := -sim.NormalizeAngle(f.bus.Heading - roadHeading)
				halfL := f.bus.Length * 0.5
				halfW := f.bus.Width * 0.5
				margin := 6.0 // Preserve at least 6px daylight from opposite shoulder

				avail := (dOppositeShoulder - halfW - margin) / halfL
				maxPsi := 0.16 // ~9 degrees max safe yaw towards opposite shoulder
				if avail <= 0.0 {
					maxPsi = 0.0
				} else if avail < 1.0 {
					geomPsi := math.Asin(avail)
					if geomPsi < maxPsi {
						maxPsi = geomPsi
					}
				}

				if psiTowardsShoulder > maxPsi {
					excess := psiTowardsShoulder - maxPsi
					correctiveSteer := sim.Clamp(excess*4.5+0.15, 0.25, 1.0)
					// Steer right away from opposite shoulder
					if steer < correctiveSteer {
						steer = correctiveSteer
					}
				} else if psiTowardsShoulder > maxPsi*0.70 {
					if steer < 0 {
						steer = 0.0
					}
				}
				if dOppositeShoulder <= halfW+margin {
					if steer < 0.35 {
						steer = 0.35 // Firm corrective steer back towards center
					}
				}
			}
		}
	}

	return steer, alpha, targetPt, nearRoundabout
}
