package sim

import (
	"math"
)

// getRoundaboutOuterLaneRadius returns the center radius of the outer travel lane of a roundabout
func getRoundaboutOuterLaneRadius(rb Roundabout) float64 {
	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	rMid := (rApron + rOuter) * 0.5
	return (rMid + rOuter) * 0.5
}

// insertRoundaboutArcs curves route line segments that would otherwise cut across a roundabout
// smoothly along the circular carriageway in a counter-clockwise arc (following right-hand traffic rules).
func insertRoundaboutArcs(pts []Vec2, w *World) []Vec2 {
	if w == nil || len(w.Roundabouts) == 0 || len(pts) < 2 {
		return pts
	}

	res := make([]Vec2, 0, len(pts)+16)
	res = append(res, pts[0])

	for k := 0; k < len(pts)-1; k++ {
		p1 := pts[k]
		p2 := pts[k+1]

		segHandled := false
		for _, rb := range w.Roundabouts {
			circR := getRoundaboutOuterLaneRadius(rb)
			v := p2.Sub(p1)
			lenV2 := v.X*v.X + v.Y*v.Y
			if lenV2 < 1e-4 {
				continue
			}

			// Distance from roundabout center to segment
			tClosest := ((rb.Center.X-p1.X)*v.X + (rb.Center.Y-p1.Y)*v.Y) / lenV2
			if tClosest < 0 {
				tClosest = 0
			} else if tClosest > 1 {
				tClosest = 1
			}
			closestPt := p1.Add(v.Mul(tClosest))
			distClosest := closestPt.Distance(rb.Center)

			// If the straight segment passes through the roundabout inner core or cuts backwards across an entrance
			isWrongWayChord := false
			if p1.Distance(rb.Center) <= rb.Radius+40.0 && p2.Distance(rb.Center) <= rb.Radius+40.0 && distClosest <= rb.Radius+25.0 {
				rx := closestPt.X - rb.Center.X
				ry := closestPt.Y - rb.Center.Y
				// In screen coordinates (Y down), counter-clockwise tangent is (ry, -rx)
				// Dot product v · T = v.X*ry - v.Y*rx
				// If negative, it moves clockwise (wrong-way / 逆行 in a counter-clockwise roundabout)
				if v.X*ry-v.Y*rx < -10.0 {
					isWrongWayChord = true
				}
			}

			if distClosest < circR-5.0 || isWrongWayChord {
				var enterPt, exitPt Vec2
				if isWrongWayChord {
					enterPt = p1
					exitPt = p2
				} else {
					f := p1.Sub(rb.Center)
					a := lenV2
					b := 2.0 * (f.X*v.X + f.Y*v.Y)
					c := (f.X*f.X + f.Y*f.Y) - circR*circR
					disc := b*b - 4.0*a*c

					if disc >= 0 {
						sqrtDisc := math.Sqrt(disc)
						t1 := (-b - sqrtDisc) / (2.0 * a)
						t2 := (-b + sqrtDisc) / (2.0 * a)

						if t1 >= -0.15 && t1 <= 1.15 {
							enterPt = p1.Add(v.Mul(math.Max(0, math.Min(1, t1))))
						} else {
							enterPt = p1
						}
						if t2 >= -0.15 && t2 <= 1.15 {
							exitPt = p1.Add(v.Mul(math.Max(0, math.Min(1, t2))))
						} else {
							exitPt = p2
						}
					} else {
						enterPt = p1
						exitPt = p2
					}
				}

				enterAngle := math.Atan2(enterPt.Y-rb.Center.Y, enterPt.X-rb.Center.X)
				exitAngle := math.Atan2(exitPt.Y-rb.Center.Y, exitPt.X-rb.Center.X)

				// Counter-clockwise rotary circulation: angle decreases from enterAngle to exitAngle
				dTheta := enterAngle - exitAngle
				for dTheta <= 0 {
					dTheta += 2 * math.Pi
				}
				for dTheta > 2*math.Pi {
					dTheta -= 2 * math.Pi
				}
				if isWrongWayChord && dTheta < math.Pi {
					dTheta += 2 * math.Pi
				}

				// Generate smooth arc waypoints along circR
				numSteps := int(math.Ceil(dTheta / (math.Pi / 24.0)))
				if numSteps < 12 {
					numSteps = 12
				}
				if res[len(res)-1].Distance(enterPt) > 4.0 {
					res = append(res, enterPt)
				}
				for step := 1; step <= numSteps; step++ {
					ang := enterAngle - dTheta*float64(step)/float64(numSteps)
					arcPt := rb.Center.Add(Vec2{
						X: math.Cos(ang) * circR,
						Y: math.Sin(ang) * circR,
					})
					if res[len(res)-1].Distance(arcPt) > 4.0 {
						res = append(res, arcPt)
					}
				}

				segHandled = true
				break
			}
		}

		if !segHandled {
			if res[len(res)-1].Distance(p2) > 4.0 {
				res = append(res, p2)
			}
		} else {
			inInnerIsland := false
			for _, rb := range w.Roundabouts {
				if p2.Distance(rb.Center) < rb.InnerRadius-2.0 {
					inInnerIsland = true
					break
				}
			}
			if !inInnerIsland && res[len(res)-1].Distance(p2) > 4.0 {
				res = append(res, p2)
			}
		}
	}

	// Filter out any stray waypoint that falls inside any roundabout inner island
	clean := make([]Vec2, 0, len(res))
	for _, pt := range res {
		inIsland := false
		for _, rb := range w.Roundabouts {
			if pt.Distance(rb.Center) < rb.InnerRadius-2.0 {
				inIsland = true
				break
			}
		}
		if !inIsland {
			if len(clean) == 0 || clean[len(clean)-1].Distance(pt) > 3.0 {
				clean = append(clean, pt)
			}
		}
	}

	return clean
}
