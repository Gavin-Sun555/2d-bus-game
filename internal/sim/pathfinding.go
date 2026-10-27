package sim

import (
	"math"
)

// RoadPathOptions selects the legal approach corridor for bus-lane left turns.
type RoadPathOptions struct {
	OrdinaryBusLaneLeft bool
	LongVehicle         bool
}

// ComputeRoadPathBetweenStops builds an uncached driving-lane path in the supplied world.
func ComputeRoadPathBetweenStops(w *World, s1, s2 *BusStop, lw float64, options ...RoadPathOptions) []Vec2 {
	if s1 == nil || s2 == nil {
		return nil
	}
	if s1.ID == s2.ID {
		return []Vec2{s1.Pos}
	}

	xs, ys := getGridCoordinates(w)
	numX := len(xs)
	numY := len(ys)

	info1 := getStopGridInfo(s1, xs, ys)
	info2 := getStopGridInfo(s2, xs, ys)

	isS1EW := math.Abs(math.Cos(s1.Heading)) > math.Abs(math.Sin(s1.Heading))
	s1Coord := s1.RoadCenter.Y
	if !isS1EW {
		s1Coord = s1.RoadCenter.X
	}
	s1Off := calcSegOffset(s1Coord, isS1EW, info1.dir, lw, w, s1.RoadCenter)

	isS2EW := math.Abs(math.Cos(s2.Heading)) > math.Abs(math.Sin(s2.Heading))
	s2Coord := s2.RoadCenter.Y
	if !isS2EW {
		s2Coord = s2.RoadCenter.X
	}
	s2Off := calcSegOffset(s2Coord, isS2EW, info2.dir, lw, w, s2.RoadCenter)

	// Check if s1 and s2 are on the exact same road segment in the exact same direction
	if info1.entryI == info2.entryI && info1.entryJ == info2.entryJ && info1.dir == info2.dir {
		sinH, cosH := math.Sincos(s1.Heading)
		isAhead := false
		if math.Abs(cosH) > 0.5 {
			isAhead = (s2.RoadCenter.X-s1.RoadCenter.X)*cosH > 0
		} else {
			isAhead = (s2.RoadCenter.Y-s1.RoadCenter.Y)*sinH > 0
		}
		if isAhead {
			var lanePt1, lanePt2 Vec2
			if isS1EW {
				lanePt1 = Vec2{X: s1.Pos.X, Y: s1Coord + s1Off}
				lanePt2 = Vec2{X: s2.Pos.X, Y: s1Coord + s1Off}
			} else {
				lanePt1 = Vec2{X: s1Coord + s1Off, Y: s1.Pos.Y}
				lanePt2 = Vec2{X: s1Coord + s1Off, Y: s2.Pos.Y}
			}
			raw := []Vec2{s1.Pos, lanePt1, lanePt2, s2.Pos}
			filtered := make([]Vec2, 0, len(raw))
			for _, pt := range raw {
				if len(filtered) == 0 || filtered[len(filtered)-1].Distance(pt) > 5 {
					filtered = append(filtered, pt)
				}
			}
			return insertRoundaboutArcs(filtered, w)
		}
	}

	startI, startJ, startDir := info1.exitI, info1.exitJ, info1.dir
	targetI, targetJ, targetDir := info2.entryI, info2.entryJ, info2.dir

	if startI >= numX {
		startI = numX - 1
	}
	if startJ >= numY {
		startJ = numY - 1
	}
	if targetI >= numX {
		targetI = numX - 1
	}
	if targetJ >= numY {
		targetJ = numY - 1
	}

	states, found := shortestRoadStates(w, xs, ys, roadState{startI, startJ, startDir}, roadState{targetI, targetJ, targetDir})
	if !found {
		return []Vec2{s1.Pos, s2.Pos}
	}

	// Assemble orthogonal driving-lane waypoints
	pts := make([]Vec2, 0, len(states)*3+6)
	pts = append(pts, s1.Pos)

	// 1. Project s1.Pos onto s1 driving lane
	if isS1EW {
		pts = append(pts, Vec2{X: s1.Pos.X, Y: s1Coord + s1Off})
	} else {
		pts = append(pts, Vec2{X: s1Coord + s1Off, Y: s1.Pos.Y})
	}

	// 2. Waypoints for intersection traversals and corners
	for k := 0; k < len(states); k++ {
		st := states[k]
		interPt := Vec2{X: xs[st.i], Y: ys[st.j]}
		var nearRb *Roundabout
		for idx := range w.Roundabouts {
			if interPt.Distance(w.Roundabouts[idx].Center) < 40.0 {
				nearRb = &w.Roundabouts[idx]
				break
			}
		}

		lastK := k
		for lastK+1 < len(states) && states[lastK+1].i == st.i && states[lastK+1].j == st.j {
			lastK++
		}
		isTurn := (lastK > k && states[lastK].dir != st.dir)

		if isTurn {
			prevDir := st.dir
			currDir := states[lastK].dir
			uPrev := getDirUnit(prevDir)
			uCurr := getDirUnit(currDir)
			pPrev := interPt.Sub(uPrev.Mul(100.0))
			pCurr := interPt.Add(uCurr.Mul(100.0))

			if nearRb != nil {
				circR := getRoundaboutOuterLaneRadius(*nearRb)
				var pEnter, pExit Vec2
				yOffPrev := calcSegOffset(ys[st.j], true, prevDir, lw, w, pPrev)
				xOffPrev := calcSegOffset(xs[st.i], false, prevDir, lw, w, pPrev)
				yOffCurr := calcSegOffset(ys[st.j], true, currDir, lw, w, pCurr)
				xOffCurr := calcSegOffset(xs[st.i], false, currDir, lw, w, pCurr)

				switch prevDir {
				case 0: // Eastbound
					pEnter = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOffPrev}
				case 2: // Westbound
					pEnter = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOffPrev}
				case 1: // Southbound
					pEnter = Vec2{X: nearRb.Center.X + xOffPrev, Y: nearRb.Center.Y - circR}
				case 3: // Northbound
					pEnter = Vec2{X: nearRb.Center.X + xOffPrev, Y: nearRb.Center.Y + circR}
				}

				switch currDir {
				case 0: // Eastbound
					pExit = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOffCurr}
				case 2: // Westbound
					pExit = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOffCurr}
				case 1: // Southbound
					pExit = Vec2{X: nearRb.Center.X + xOffCurr, Y: nearRb.Center.Y + circR}
				case 3: // Northbound
					pExit = Vec2{X: nearRb.Center.X + xOffCurr, Y: nearRb.Center.Y - circR}
				}
				turn := (currDir - prevDir + 4) % 4
				if turn == 2 {
					// Roundabout U-turn: circumnavigate counter-clockwise along outer lane (~320-360 deg)
					// instead of taking a wrong-way shortcut across the junction mouth
					pts = append(pts, pEnter)
					enterAngle := math.Atan2(pEnter.Y-nearRb.Center.Y, pEnter.X-nearRb.Center.X)
					exitAngle := math.Atan2(pExit.Y-nearRb.Center.Y, pExit.X-nearRb.Center.X)
					dTheta := enterAngle - exitAngle
					for dTheta <= 0 {
						dTheta += 2 * math.Pi
					}
					for dTheta > 2*math.Pi {
						dTheta -= 2 * math.Pi
					}
					if dTheta < math.Pi {
						dTheta += 2 * math.Pi
					}

					numSteps := int(math.Ceil(dTheta / (math.Pi / 24.0)))
					if numSteps < 24 {
						numSteps = 24
					}
					for step := 1; step <= numSteps; step++ {
						ang := enterAngle - dTheta*float64(step)/float64(numSteps)
						arcPt := nearRb.Center.Add(Vec2{
							X: math.Cos(ang) * circR,
							Y: math.Sin(ang) * circR,
						})
						if len(pts) == 0 || pts[len(pts)-1].Distance(arcPt) > 4.0 {
							pts = append(pts, arcPt)
						}
					}

					if len(pts) == 0 || pts[len(pts)-1].Distance(pExit) > 4.0 {
						pts = append(pts, pExit)
					}
				} else {
					pts = append(pts, pEnter, pExit)
				}
			} else {
				turn := (currDir - prevDir + 4) % 4
				rPrev := getDirPerp(prevDir)
				rCurr := getDirPerp(currDir)

				isPrevEW := (prevDir == 0 || prevDir == 2)
				prevCoord := ys[st.j]
				if !isPrevEW {
					prevCoord = xs[st.i]
				}

				isCurrEW := (currDir == 0 || currDir == 2)
				currCoord := ys[st.j]
				if !isCurrEW {
					currCoord = xs[st.i]
				}

				segLen := 600.0
				if len(pts) > 0 {
					segLen = interPt.Distance(pts[len(pts)-1])
				}

				if turn == 1 && w.GetRoadLanes(prevCoord, isPrevEW, pPrev) == 8 {
					pts = appendEightLaneRightTurn(pts, w, interPt, uPrev, uCurr, prevCoord, currCoord, isPrevEW, isCurrEW, pPrev, pCurr, lw)
				} else if turn == 1 {
					// 90-degree right turn from outer lane into outer lane
					var corner Vec2
					if isPrevEW {
						yOff := calcSegOffset(ys[st.j], true, prevDir, lw, w, pPrev)
						xOff := calcSegOffset(xs[st.i], false, currDir, lw, w, pCurr)
						corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
					} else {
						xOff := calcSegOffset(xs[st.i], false, prevDir, lw, w, pPrev)
						yOff := calcSegOffset(ys[st.j], true, currDir, lw, w, pCurr)
						corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
					}

					outOff := RoadLaneOffset(currCoord, isCurrEW, lw, w, pCurr)
					rightGeometry := w.JunctionAt(interPt, lw)
					inOff := RoadLaneOffset(prevCoord, isPrevEW, lw, w, pPrev)
					if outOff <= lw*0.6 {
						// Destination road is a 2-lane road (1 lane per direction, 52px wide).
						// With enlarged intersections, start the cubic Bezier transition earlier
						// and sweep smoothly with generous turning radius:
						dIn := math.Max(130, rightGeometry.TurnRadius-outOff+70)
						dOut := math.Max(115, rightGeometry.TurnRadius-inOff+70)
						dCurbShift := 8.5 // Shift towards right curb for oncoming clearance

						b0 := corner.Sub(uPrev.Mul(dIn)).Add(rPrev.Mul(dCurbShift * 0.35))
						b1 := corner.Sub(uPrev.Mul(dIn * 0.40)).Add(rPrev.Mul(dCurbShift * 0.90)).Add(rCurr.Mul(dCurbShift * 0.45))
						b2 := corner.Add(uCurr.Mul(dOut * 0.40)).Add(rCurr.Mul(dCurbShift * 0.90)).Add(rPrev.Mul(dCurbShift * 0.45))
						b3 := corner.Add(uCurr.Mul(dOut)).Add(rCurr.Mul(dCurbShift))

						const nSteps = 8
						for m := 0; m <= nSteps; m++ {
							u := float64(m) / float64(nSteps)
							u1 := 1.0 - u
							// Cubic Bezier: B(u) = (1-u)^3*b0 + 3(1-u)^2*u*b1 + 3(1-u)*u^2*b2 + u^3*b3
							pt := b0.Mul(u1 * u1 * u1).
								Add(b1.Mul(3.0 * u1 * u1 * u)).
								Add(b2.Mul(3.0 * u1 * u * u)).
								Add(b3.Mul(u * u * u))
							if len(pts) == 0 || pts[len(pts)-1].Distance(pt) > 4.0 {
								pts = append(pts, pt)
							}
						}
						// Smoothly return to standard lane center downstream
						pMergeBack := corner.Add(uCurr.Mul(dOut + 80.0))
						pts = append(pts, pMergeBack)
					} else {
						// Destination is a 4-lane or 6-lane road:
						// Smooth cubic Bezier right turn curve into target outer lane instead of harsh 90-degree corner
						dIn := math.Max(105, rightGeometry.TurnRadius-outOff+70)
						dOut := math.Max(105, rightGeometry.TurnRadius-inOff+70)
						b0 := corner.Sub(uPrev.Mul(dIn))
						b1 := corner.Sub(uPrev.Mul(dIn * 0.38))
						b2 := corner.Add(uCurr.Mul(dOut * 0.38))
						b3 := corner.Add(uCurr.Mul(dOut))

						const nSteps = 7
						for m := 0; m <= nSteps; m++ {
							u := float64(m) / float64(nSteps)
							u1 := 1.0 - u
							pt := b0.Mul(u1 * u1 * u1).
								Add(b1.Mul(3.0 * u1 * u1 * u)).
								Add(b2.Mul(3.0 * u1 * u * u)).
								Add(b3.Mul(u * u * u))
							if len(pts) == 0 || pts[len(pts)-1].Distance(pt) > 4.0 {
								pts = append(pts, pt)
							}
						}
					}
				} else if turn == 3 || turn == 2 {
					// Left turn (turn == 3) or U-turn (turn == 2)
					geometry := w.JunctionAt(interPt, lw)
					inStopDist := geometry.HalfY + JunctionStopInset
					crossStopDist := geometry.HalfX + JunctionStopInset
					if !isPrevEW {
						inStopDist, crossStopDist = crossStopDist, inStopDist
					}

					// Intelligently scale merge distance based on block length (segLen, distance between intersections):
					// Finish merge before the intersection mouth so vehicle is stabilized in dedicated left-turn lane
					// Finish far enough before the stop line for the whole bus and
					// steering actuator to settle, especially across two through lanes.
					dFinish := math.Max(crossStopDist+120, geometry.TurnRadius-lw/2+70)

					var dMerge float64
					if segLen <= 320.0 {
						dMerge = math.Min(175.0, segLen*0.52)
					} else if segLen <= 600.0 {
						dMerge = 165.0 + (segLen-320.0)*0.70
					} else {
						dMerge = math.Min(800.0, 361.0+(segLen-600.0)*0.55)
					}
					if dMerge < lw*2.5 {
						dMerge = lw * 2.5
					}
					inOff := RoadLaneOffset(prevCoord, isPrevEW, lw, w, pPrev)
					minimumTaper := lw * 5
					if inOff > lw*1.6 {
						minimumTaper = math.Max(minimumTaper, 450)
					}
					if inOff > lw*3 {
						// Three crossed lanes need a width-scaled runway. The old
						// fixed taper yawed a wide-road bus into its curb queue.
						minimumTaper = math.Max(minimumTaper, (inOff-lw*.5)*4.5)
					}
					dMerge = math.Min(segLen*.92, math.Max(dMerge, dFinish+minimumTaper))
					dFinish = math.Max(geometry.TurnRadius-lw/2, math.Min(dFinish, dMerge-60))

					approachRoad, roadFound := w.RoadAt(prevCoord, isPrevEW, pPrev)
					longBusUTurn := turn == 2 && len(options) > 0 && options[0].LongVehicle && !options[0].OrdinaryBusLaneLeft && roadFound && approachRoad.HasBusLane()
					busLaneLeft := (turn == 3 || longBusUTurn) && roadFound && approachRoad.HasBusLane() && (len(options) == 0 || !options[0].OrdinaryBusLaneLeft)
					if inOff > lw*0.6 && !busLaneLeft {
						// Multi-lane avenue (4-lane or 6-lane): merge smoothly via cubic Hermite S-curve
						// from travel lane (inOff) into dedicated left-turn lane (targetOff = 0.5*lw)
						targetOff := lw * 0.5
						deltaOff := math.Abs(inOff - targetOff)
						availTaper := math.Max(0.0, dMerge-dFinish)

						var lMerge float64
						if deltaOff > lw*1.2 {
							// 6-lane road (104px shift across 2 lanes): long, silky smooth taper
							maximumTaper := 520.0
							if inOff > lw*3 {
								maximumTaper = math.Max(maximumTaper, deltaOff*3.5)
							}
							lMerge = math.Min(availTaper*0.90, maximumTaper)
						} else {
							// 4-lane road (52px shift across 1 lane): graceful taper
							lMerge = math.Min(availTaper*0.80, 240.0)
						}
						lMerge = math.Max(lMerge, math.Min(100.0, availTaper))

						if lMerge < 20.0 {
							pMergeStart := interPt.Sub(uPrev.Mul(dMerge)).Add(rPrev.Mul(inOff))
							pMergeEnd := interPt.Sub(uPrev.Mul(dFinish)).Add(rPrev.Mul(targetOff))
							pts = append(pts, pMergeStart, pMergeEnd)
						} else {
							dMergeStart := dMerge
							const nMergeSteps = 6
							for m := 0; m <= nMergeSteps; m++ {
								u := float64(m) / float64(nMergeSteps)
								// Cubic Hermite smoothstep easing: w(u) = 3u^2 - 2u^3
								wStep := u * u * (3.0 - 2.0*u)
								d := dMergeStart - u*lMerge
								off := inOff + (targetOff-inOff)*wStep
								pt := interPt.Sub(uPrev.Mul(d)).Add(rPrev.Mul(off))
								pts = append(pts, pt)
							}
							if dMergeStart-lMerge > dFinish+15.0 {
								pStopApproach := interPt.Sub(uPrev.Mul(dFinish)).Add(rPrev.Mul(targetOff))
								pts = append(pts, pStopApproach)
							}
						}
					}

					if turn == 3 {
						wideExit := w.GetRoadLanes(currCoord, isCurrEW, pCurr) == 8
						if wideExit {
							entryOffset := lw / 2
							if busLaneLeft {
								entryOffset = 3.5 * lw
							}
							outOffset := RoadLaneOffset(currCoord, isCurrEW, lw, w, pCurr)
							parallelLeft := roadFound && approachRoad.HasBusLane() && len(options) > 0 && options[0].OrdinaryBusLaneLeft
							travelOffset := outOffset
							if parallelLeft {
								// The median left-turn lane must not converge with the
								// curb bus turn. Use the inner through lane at the exit.
								outOffset = 1.5 * lw
							}
							for _, point := range WideLeftTurnPath(interPt, uPrev, uCurr, entryOffset, outOffset, geometry) {
								if len(pts) == 0 || point.Sub(pts[len(pts)-1]).Dot(uPrev) >= -.01 {
									pts = append(pts, point)
								}
							}
							if parallelLeft {
								available := s2.Pos.Sub(interPt).Dot(uCurr)
								if k+2 < len(states) {
									available = Vec2{X: xs[states[k+2].i], Y: ys[states[k+2].j]}.Sub(interPt).Dot(uCurr)
								}
								pts = appendWideLeftLaneReturn(pts, interPt, uCurr, outOffset, travelOffset, available, geometry)
							}
						} else if busLaneLeft {
							pts = append(pts, QuarterTurnPath(interPt, uPrev, uCurr, 3.5*lw, lw/2, geometry.TurnRadius)...)
						} else {
							// Finish the lane taper before entering the enlarged junction.
							approachDist := math.Min(dFinish, geometry.TurnRadius-lw/2+35)
							approachPoint := interPt.Sub(uPrev.Mul(approachDist)).Add(rPrev.Mul(lw / 2))
							if len(pts) == 0 || approachPoint.Sub(pts[len(pts)-1]).Dot(uPrev) >= 0 {
								pts = append(pts, approachPoint)
							}
							for _, point := range LeftTurnPath(interPt, math.Atan2(uPrev.Y, uPrev.X), lw, geometry.TurnRadius) {
								if len(pts) == 0 || point.Sub(pts[len(pts)-1]).Dot(uPrev) >= -.01 {
									pts = append(pts, point)
								}
							}
						}
						if !wideExit {
							dExitMouth := math.Max(inStopDist+15, geometry.TurnRadius-lw/2+35)
							pts = append(pts, interPt.Add(uCurr.Mul(dExitMouth)).Add(rCurr.Mul(lw/2)))

							outOff := RoadLaneOffset(currCoord, isCurrEW, lw, w, pCurr)
							if outOff > lw*0.6 {
								availDist := 400.0
								if k+2 < len(states) {
									nextPt := Vec2{X: xs[states[k+2].i], Y: ys[states[k+2].j]}
									dNext := nextPt.Sub(interPt).Dot(uCurr)
									if dNext > 50.0 {
										availDist = dNext
									}
								} else {
									dS2 := s2.Pos.Sub(interPt).Dot(uCurr)
									if dS2 > 50.0 {
										availDist = dS2
									}
								}

								dExitStart := dExitMouth + 35
								deltaOut := math.Abs(outOff - lw*0.5)
								availExit := math.Max(0.0, availDist*0.88-dExitStart)
								if availExit > 50.0 {
									var lExit float64
									if deltaOut > lw*1.2 {
										lExit = math.Min(availExit*0.85, 420.0)
									} else {
										lExit = math.Min(availExit*0.80, 240.0)
									}
									lExit = math.Max(lExit, math.Min(80.0, availExit))

									const nExitSteps = 6
									for m := 0; m <= nExitSteps; m++ {
										u := float64(m) / float64(nExitSteps)
										wStep := u * u * (3.0 - 2.0*u)
										d := dExitStart + u*lExit
										off := (lw * 0.5) + (outOff-(lw*0.5))*wStep
										pt := interPt.Add(uCurr.Mul(d)).Add(rCurr.Mul(off))
										pts = append(pts, pt)
									}
								} else {
									dMergeOut := math.Min(180.0, availDist*0.45)
									if dMergeOut > lw*2.0 {
										pTargetInner := interPt.Add(uCurr.Mul(dMergeOut)).Add(rCurr.Mul(lw * 0.5))
										pTargetOuter := interPt.Add(uCurr.Mul(dMergeOut + 60.0)).Add(rCurr.Mul(outOff))
										pts = append(pts, pTargetInner, pTargetOuter)
									}
								}
							}
						}
					} else {
						// Preserve tangency and a drivable radius across both halves of the turn.
						outOff := RoadLaneOffset(currCoord, isCurrEW, lw, w, pCurr)
						entryOffset := lw / 2
						if longBusUTurn {
							entryOffset = inOff
						}
						pts = append(pts, UTurnPath(interPt, math.Atan2(uPrev.Y, uPrev.X), entryOffset, outOff, geometry)...)
					}
				}
			}
			// Skip all intermediate states at this intersection
			k = lastK
		} else if k > 0 || nearRb != nil {
			if nearRb != nil {
				circR := getRoundaboutOuterLaneRadius(*nearRb)
				dir := st.dir
				uDir := getDirUnit(dir)
				pEnterProbe := interPt.Sub(uDir.Mul(100.0))
				yOff := calcSegOffset(ys[st.j], true, dir, lw, w, pEnterProbe)
				xOff := calcSegOffset(xs[st.i], false, dir, lw, w, pEnterProbe)
				var pEnter, pExit Vec2
				switch dir {
				case 0:
					pEnter = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOff}
					pExit = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOff}
				case 2:
					pEnter = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOff}
					pExit = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOff}
				case 1:
					pEnter = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y - circR}
					pExit = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y + circR}
				case 3:
					pEnter = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y + circR}
					pExit = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y - circR}
				}
				pts = append(pts, pEnter, pExit)
			} else {
				// Straight traversal across intersection
				pt := getLanePt(st.i, st.j, st.dir, xs, ys, lw, w)
				pts = append(pts, pt)
			}
		}
	}

	// 3. Project s2.Pos onto s2 driving lane
	if isS2EW {
		pts = append(pts, Vec2{X: s2.Pos.X, Y: s2Coord + s2Off})
	} else {
		pts = append(pts, Vec2{X: s2Coord + s2Off, Y: s2.Pos.Y})
	}

	// 4. Target stop bay
	pts = append(pts, s2.Pos)

	// Filter out duplicate or extremely close points
	filtered := make([]Vec2, 0, len(pts))
	for _, pt := range pts {
		if len(filtered) == 0 || filtered[len(filtered)-1].Distance(pt) > 5 {
			filtered = append(filtered, pt)
		}
	}

	// Collinear simplification along grid axes
	if len(filtered) >= 3 {
		simplified := make([]Vec2, 0, len(filtered))
		simplified = append(simplified, filtered[0])
		for i := 1; i < len(filtered)-1; i++ {
			pPrev := simplified[len(simplified)-1]
			pCurr := filtered[i]
			pNext := filtered[i+1]
			if math.Abs(pPrev.Y-pCurr.Y) < 0.05 && math.Abs(pCurr.Y-pNext.Y) < 0.05 {
				continue
			}
			if math.Abs(pPrev.X-pCurr.X) < 0.05 && math.Abs(pCurr.X-pNext.X) < 0.05 {
				continue
			}
			simplified = append(simplified, pCurr)
		}
		simplified = append(simplified, filtered[len(filtered)-1])
		filtered = simplified
	}

	return insertRoundaboutArcs(filtered, w)
}
