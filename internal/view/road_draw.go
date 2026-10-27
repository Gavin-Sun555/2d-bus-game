package view

import (
	"image/color"
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func drawIntersectionPads(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		geometry := sim.NewJunctionGeometry(ewHalf, nsHalf, laneWidth)
		shoulderHalfX := geometry.HalfX + sim.ShoulderWidth
		shoulderHalfY := geometry.HalfY + sim.ShoulderWidth
		if !cam.IsPointVisible(c, math.Max(shoulderHalfX, shoulderHalfY)*1.5) {
			continue
		}
		// In plan mode, skip shoulder backing apron at intersections
		if !planMode {
			drawRotatedBox(screen, cam, c, 1, 0, shoulderHalfX*2, shoulderHalfY*2, sceneColors(w.Theme).paving)
		}
	}
}

func drawIntersectionAsphalt(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		geometry := sim.NewJunctionGeometry(ewHalf, nsHalf, laneWidth)
		padHalfX := geometry.HalfX
		padHalfY := geometry.HalfY
		if !cam.IsPointVisible(c, math.Max(padHalfX, padHalfY)*1.5) {
			continue
		}
		cornerRadius := math.Min(36, math.Min(padHalfX, padHalfY)*.2)
		drawRotatedBox(screen, cam, c, 1, 0, padHalfX*2-2*cornerRadius, padHalfY*2, sceneColors(w.Theme).asphalt)
		drawRotatedBox(screen, cam, c, 1, 0, padHalfX*2, padHalfY*2-2*cornerRadius, sceneColors(w.Theme).asphalt)

		// Draw smooth curved curb return fillets at all 4 corners of the intersection
		// so turning vehicles have broad, sweeping asphalt connecting the perpendicular roads
		curbR := cornerRadius * cam.Zoom
		if curbR >= 2.0 {
			quads := [4][2]float64{
				{-padHalfX + cornerRadius, -padHalfY + cornerRadius},
				{padHalfX - cornerRadius, -padHalfY + cornerRadius},
				{-padHalfX + cornerRadius, padHalfY - cornerRadius},
				{padHalfX - cornerRadius, padHalfY - cornerRadius},
			}
			for _, q := range quads {
				cornerPt := cam.WorldToScreen(c.Add(sim.Vec2{X: q[0], Y: q[1]}))
				filledCircle(screen, float32(cornerPt.X), float32(cornerPt.Y), float32(curbR), sceneColors(w.Theme).asphalt, true)
			}
		}
	}
}

func drawRoad(screen *ebiten.Image, cam *sim.Camera, w *sim.World, r sim.RoadSegment, laneWidth float64, planMode bool) {
	dir := r.P2.Sub(r.P1)
	len := dir.Length()
	if len == 0 {
		return
	}
	center := r.P1.Add(r.P2).Mul(0.5)
	roadW := r.GetWidth(laneWidth)

	// Frustum culling check for road segment
	margin := roadW/2 + sim.ShoulderWidth + 20
	if !cam.IsBoxVisible(math.Min(r.P1.X, r.P2.X)-margin, math.Min(r.P1.Y, r.P2.Y)-margin, math.Max(r.P1.X, r.P2.X)+margin, math.Max(r.P1.Y, r.P2.Y)+margin) {
		return
	}

	heading := math.Atan2(dir.Y, dir.X)
	sinH, cosH := math.Sincos(heading)

	// 1. Soft Shoulder (drivable buffer with deceleration, ShoulderWidth = 40px)
	if !planMode && cam.Zoom >= 0.35 {
		shoulderTotalW := roadW + sim.ShoulderWidth*2
		drawRotatedBox(screen, cam, center, cosH, sinH, len, shoulderTotalW, sceneColors(w.Theme).paving) // Slate sidewalk / shoulder
	}

	// 2. Asphalt Road Surface (#1e293b)
	drawRotatedBox(screen, cam, center, cosH, sinH, len, roadW, sceneColors(w.Theme).asphalt)

	if r.HasBusLane() {
		drawBusLaneSurface(screen, cam, r, center, cosH, sinH, len, laneWidth)
	}
	if !planMode && cam.Zoom >= 0.5 {
		drawPavingJoints(screen, cam, center, dir.Mul(1/len), len, roadW, sceneColors(w.Theme))
	}

	// 3. Road Curb dividing lines & Hard Crash Barrier along outer edge of shoulder (skip in plan mode / low zoom)
	if !planMode && cam.Zoom >= 0.35 {
		curbOffset := roadW / 2
		drawParallelLine(screen, cam, center, cosH, sinH, len, curbOffset, sceneColors(w.Theme).curb, 2)
		drawParallelLine(screen, cam, center, cosH, sinH, len, -curbOffset, sceneColors(w.Theme).curb, 2)

		var splits []float64
		isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
		minX, maxX := math.Min(r.P1.X, r.P2.X), math.Max(r.P1.X, r.P2.X)
		minY, maxY := math.Min(r.P1.Y, r.P2.Y), math.Max(r.P1.Y, r.P2.Y)
		for _, pt := range w.Intersections {
			if isEW {
				if math.Abs(pt.Y-r.P1.Y) < 5.0 && pt.X > minX+5.0 && pt.X < maxX-5.0 {
					splits = append(splits, pt.X)
				}
			} else {
				if math.Abs(pt.X-r.P1.X) < 5.0 && pt.Y > minY+5.0 && pt.Y < maxY-5.0 {
					splits = append(splits, pt.Y)
				}
			}
		}
		sort.Float64s(splits)

		barrierOffset := roadW/2 + sim.ShoulderWidth
		drawHazardBarrier(screen, cam, w, r, center, cosH, sinH, len, barrierOffset, laneWidth, splits)
		drawHazardBarrier(screen, cam, w, r, center, cosH, sinH, len, -barrierOffset, laneWidth, splits)
	}

	// 5. Center Dividers & Lane Markings
	if r.HasLines {
		if r.LaneCount() >= 4 {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6, sim.RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6, sim.RGBA(250, 204, 21, 240))
			for lane := 1; lane < r.LaneCount()/2; lane++ {
				for _, sign := range []float64{-1, 1} {
					offset := sign * float64(lane) * laneWidth
					if r.HasBusLane() && lane == 3 {
						drawParallelLine(screen, cam, center, cosH, sinH, len, offset, sim.RGBA(250, 204, 21, 230), 2.5)
					} else {
						drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, offset, planMode)
					}
				}
			}

		} else {
			drawDashedYellowLine(screen, cam, center, cosH, sinH, len, planMode)
		}
	}
}

func drawParallelLine(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, offset float64, c color.RGBA, stroke float32) {
	p1 := center.Add(sim.Vec2{X: -cosH*len/2 - sinH*offset, Y: -sinH*len/2 + cosH*offset})
	p2 := center.Add(sim.Vec2{X: cosH*len/2 - sinH*offset, Y: sinH*len/2 + cosH*offset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), stroke*float32(cam.Zoom), c, true)
}

func drawHazardBarrier(screen *ebiten.Image, cam *sim.Camera, w *sim.World, r sim.RoadSegment, center sim.Vec2, cosH, sinH, len, offset, laneWidth float64, splits []float64) {
	isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
	gapAt := func(along float64) float64 {
		at := sim.Vec2{X: along, Y: r.P1.Y}
		if !isEW {
			at = sim.Vec2{X: r.P1.X, Y: along}
		}
		geometry := w.JunctionAt(at, laneWidth)
		if isEW {
			return geometry.HalfX + sim.ShoulderWidth
		}
		return geometry.HalfY + sim.ShoulderWidth
	}

	if isEW {
		minX := math.Min(r.P1.X, r.P2.X)
		maxX := math.Max(r.P1.X, r.P2.X)
		prevX := minX
		for _, ix := range splits {
			segStart := prevX + gapAt(prevX)
			if prevX == minX {
				segStart = minX
			}
			segEnd := ix - gapAt(ix)
			if segEnd > segStart {
				p1 := sim.Vec2{X: segStart, Y: r.P1.Y + offset}
				p2 := sim.Vec2{X: segEnd, Y: r.P1.Y + offset}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
				strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
			}
			prevX = ix
		}
		// Final segment to road end
		segStart := prevX + gapAt(prevX)
		if prevX == minX {
			segStart = minX
		}
		segEnd := maxX
		if segEnd > segStart {
			p1 := sim.Vec2{X: segStart, Y: r.P1.Y + offset}
			p2 := sim.Vec2{X: segEnd, Y: r.P1.Y + offset}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
			strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
		}
	} else {
		minY := math.Min(r.P1.Y, r.P2.Y)
		maxY := math.Max(r.P1.Y, r.P2.Y)
		prevY := minY
		for _, iy := range splits {
			segStart := prevY + gapAt(prevY)
			if prevY == minY {
				segStart = minY
			}
			segEnd := iy - gapAt(iy)
			if segEnd > segStart {
				p1 := sim.Vec2{X: r.P1.X - offset, Y: segStart}
				p2 := sim.Vec2{X: r.P1.X - offset, Y: segEnd}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
				strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
			}
			prevY = iy
		}
		// Final segment to road end
		segStart := prevY + gapAt(prevY)
		if prevY == minY {
			segStart = minY
		}
		segEnd := maxY
		if segEnd > segStart {
			p1 := sim.Vec2{X: r.P1.X - offset, Y: segStart}
			p2 := sim.Vec2{X: r.P1.X - offset, Y: segEnd}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
			strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
		}
	}
}

func drawCenterDividers(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, perpOffset float64, c color.RGBA) {
	p1 := center.Add(sim.Vec2{X: -cosH*len/2 - sinH*perpOffset, Y: -sinH*len/2 + cosH*perpOffset})
	p2 := center.Add(sim.Vec2{X: cosH*len/2 - sinH*perpOffset, Y: sinH*len/2 + cosH*perpOffset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), c, true)
}

func drawDashedYellowLine(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len float64, planMode bool) {
	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Draw single continuous line to eliminate ~110 vector calls per road
		drawCenterDividers(screen, cam, center, cosH, sinH, len, 0, sim.RGBA(250, 204, 21, 200))
		return
	}
	dashLen := 26.0
	gapLen := 20.0
	step := dashLen + gapLen
	first, last := visibleDashRange(cam, center, sim.Vec2{X: cosH, Y: sinH}, len, dashLen, step)
	for i := first; i < last; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(sim.Vec2{X: cosH * dStart, Y: sinH * dStart})
		db := center.Add(sim.Vec2{X: cosH * dEnd, Y: sinH * dEnd})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		strokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(3*cam.Zoom), sim.RGBA(250, 204, 21, 230), true)
	}
}

func drawDashedWhiteLine(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, perpOffset float64, planMode bool) {
	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Draw single continuous line to eliminate ~220 vector calls per road
		drawCenterDividers(screen, cam, center, cosH, sinH, len, perpOffset, sim.RGBA(241, 245, 249, 130))
		return
	}
	dashLen := 24.0
	gapLen := 22.0
	step := dashLen + gapLen
	lineCenter := center.Add(sim.Vec2{X: -sinH * perpOffset, Y: cosH * perpOffset})
	first, last := visibleDashRange(cam, lineCenter, sim.Vec2{X: cosH, Y: sinH}, len, dashLen, step)
	for i := first; i < last; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(sim.Vec2{X: cosH*dStart - sinH*perpOffset, Y: sinH*dStart + cosH*perpOffset})
		db := center.Add(sim.Vec2{X: cosH*dEnd - sinH*perpOffset, Y: sinH*dEnd + cosH*perpOffset})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		strokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(2*cam.Zoom), sim.RGBA(241, 245, 249, 180), true)
	}
}

func drawCrosswalks(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, pt := range w.Intersections {
		if w.IsAtRoundabout(pt) {
			continue // Roundabouts feature self-regulating circular rotary flow, not straight 4-way crosswalks or lane arrows
		}

		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(pt, laneWidth)
		geometry := sim.NewJunctionGeometry(ewHalf, nsHalf, laneWidth)
		ewStopLineDist := geometry.HalfX + sim.JunctionStopInset
		ewCrosswalkDist := ewStopLineDist - 25.0
		nsStopLineDist := geometry.HalfY + sim.JunctionStopInset
		nsCrosswalkDist := nsStopLineDist - 25.0

		maxBound := math.Max(ewStopLineDist, nsStopLineDist) + 60.0
		if !cam.IsPointVisible(pt, maxBound) {
			continue
		}

		hasWest, westLanes := false, 0
		hasEast, eastLanes := false, 0
		hasNorth, northLanes := false, 0
		hasSouth, southLanes := false, 0

		for _, r := range w.Roads {
			if math.Abs(r.P1.Y-r.P2.Y) < 1.0 && math.Abs(r.P1.Y-pt.Y) < 10.0 {
				minX := math.Min(r.P1.X, r.P2.X)
				maxX := math.Max(r.P1.X, r.P2.X)
				if minX < pt.X-15.0 && maxX >= pt.X-15.0 {
					hasWest = true
					if r.Lanes > westLanes {
						westLanes = r.Lanes
					}
				}
				if minX <= pt.X+15.0 && maxX > pt.X+15.0 {
					hasEast = true
					if r.Lanes > eastLanes {
						eastLanes = r.Lanes
					}
				}
			}
			if math.Abs(r.P1.X-r.P2.X) < 1.0 && math.Abs(r.P1.X-pt.X) < 10.0 {
				minY := math.Min(r.P1.Y, r.P2.Y)
				maxY := math.Max(r.P1.Y, r.P2.Y)
				if minY < pt.Y-15.0 && maxY >= pt.Y-15.0 {
					hasNorth = true
					if r.Lanes > northLanes {
						northLanes = r.Lanes
					}
				}
				if minY <= pt.Y+15.0 && maxY > pt.Y+15.0 {
					hasSouth = true
					if r.Lanes > southLanes {
						southLanes = r.Lanes
					}
				}
			}
		}

		// 1. Draw Zebra Crossings around intersection for connected branches only
		if hasNorth {
			drawZebraBranch(screen, cam, pt.Add(sim.Vec2{X: 0, Y: -nsCrosswalkDist}), true, northLanes, laneWidth)
		}
		if hasSouth {
			drawZebraBranch(screen, cam, pt.Add(sim.Vec2{X: 0, Y: nsCrosswalkDist}), true, southLanes, laneWidth)
		}
		if hasWest {
			drawZebraBranch(screen, cam, pt.Add(sim.Vec2{X: -ewCrosswalkDist, Y: 0}), false, westLanes, laneWidth)
		}
		if hasEast {
			drawZebraBranch(screen, cam, pt.Add(sim.Vec2{X: ewCrosswalkDist, Y: 0}), false, eastLanes, laneWidth)
		}

		// All approaches share the same per-lane arrow policy.
		for approach, lanes := range []int{westLanes, eastLanes, northLanes, southLanes} {
			if lanes < 4 {
				continue
			}
			heading := []float64{0, math.Pi, math.Pi / 2, -math.Pi / 2}[approach]
			fwd := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -fwd.Y, Y: fwd.X}
			stop := ewStopLineDist
			if approach >= 2 {
				stop = nsStopLineDist
			}
			for lane := 0; lane < lanes/2; lane++ {
				for _, back := range []float64{45, 110} {
					pos := pt.Sub(fwd.Mul(stop + back)).Add(right.Mul((float64(lane) + .5) * laneWidth))
					if lanes == 8 && lane == 3 {
						drawGroundTurnArrow(screen, cam, pos, heading, true)
						road, ok := w.RoadAt(func() float64 {
							if approach < 2 {
								return pt.Y
							}
							return pt.X
						}(), approach < 2, pt.Sub(fwd.Mul(stop+50)))
						if ok && road.HasBusLane() {
							drawGroundArrow(screen, cam, pos, heading, true)
							drawGroundArrow(screen, cam, pos, heading, false)
						}
					} else {
						drawGroundArrow(screen, cam, pos, heading, lane == 0)
					}
				}
				if lane > 0 {
					drawSolidWhiteDivider(screen, cam, pt.Sub(fwd.Mul(stop+60)).Add(right.Mul(float64(lane)*laneWidth)), fwd.X, fwd.Y, 120)
				}
			}
		}

	}
}

func drawZebraBranch(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, horizontal bool, lanes int, laneWidth float64) {
	numStripes := 6
	step := 15.0
	if lanes >= 8 {
		numStripes = int(float64(lanes)*laneWidth/step) - 1
	} else if lanes >= 6 {
		numStripes = lanes * 3
	} else if lanes >= 4 {
		numStripes = 12
	}
	for i := -numStripes / 2; i <= numStripes/2; i++ {
		var p1, p2 sim.Vec2
		if horizontal {
			offset := float64(i) * step
			p1 = center.Add(sim.Vec2{X: offset, Y: -10})
			p2 = center.Add(sim.Vec2{X: offset, Y: 10})
		} else {
			offset := float64(i) * step
			p1 = center.Add(sim.Vec2{X: -10, Y: offset})
			p2 = center.Add(sim.Vec2{X: 10, Y: offset})
		}
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(7.5*cam.Zoom), sim.RGBA(241, 245, 249, 215), true)
	}
}

func drawSolidWhiteDivider(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, length float64) {
	halfLen := length * 0.5
	p1 := center.Add(sim.Vec2{X: -cosH * halfLen, Y: -sinH * halfLen})
	p2 := center.Add(sim.Vec2{X: cosH * halfLen, Y: sinH * halfLen})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), sim.RGBA(248, 250, 252, 220), true)
}

func drawGroundArrow(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading float64, isLeftTurn bool) {
	drawGroundLaneArrow(screen, cam, pos, heading, isLeftTurn, false)
}
func drawGroundTurnArrow(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading float64, right bool) {
	drawGroundLaneArrow(screen, cam, pos, heading, true, right)
}
func drawGroundLaneArrow(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading float64, isLeftTurn, rightTurn bool) {
	scale := float32(cam.Zoom)
	if scale < 0.28 {
		return
	}
	sinH, cosH := math.Sincos(heading)
	fwd := sim.Vec2{X: cosH, Y: sinH}
	left := sim.Vec2{X: sinH, Y: -cosH} // In screen coords (+Y is down), driver's left is (sinH, -cosH)

	if rightTurn {
		left = left.Mul(-1)
	}
	col := sim.RGBA(248, 250, 252, 225)
	lw := float32(2.8 * scale)

	if !isLeftTurn {
		// Straight Arrow (↑)
		p1 := pos.Sub(fwd.Mul(14))
		p2 := pos.Add(fwd.Mul(14))
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lw, col, true)

		pl := pos.Add(fwd.Mul(6)).Add(left.Mul(6))
		pr := pos.Add(fwd.Mul(6)).Sub(left.Mul(6))
		sl := cam.WorldToScreen(pl)
		sr := cam.WorldToScreen(pr)
		strokeLine(screen, float32(s2.X), float32(s2.Y), float32(sl.X), float32(sl.Y), lw, col, true)
		strokeLine(screen, float32(s2.X), float32(s2.Y), float32(sr.X), float32(sr.Y), lw, col, true)
	} else {
		// Left-Turn Arrow (↰)
		p1 := pos.Sub(fwd.Mul(14))
		pCurve := pos.Add(fwd.Mul(4))
		s1 := cam.WorldToScreen(p1)
		sc := cam.WorldToScreen(pCurve)
		strokeLine(screen, float32(s1.X), float32(s1.Y), float32(sc.X), float32(sc.Y), lw, col, true)

		pTip := pCurve.Add(left.Mul(15))
		stip := cam.WorldToScreen(pTip)
		strokeLine(screen, float32(sc.X), float32(sc.Y), float32(stip.X), float32(stip.Y), lw, col, true)

		pBarbT := pTip.Sub(left.Mul(5)).Add(fwd.Mul(4))
		pBarbB := pTip.Sub(left.Mul(5)).Sub(fwd.Mul(4))
		sbt := cam.WorldToScreen(pBarbT)
		sbb := cam.WorldToScreen(pBarbB)
		strokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbt.X), float32(sbt.Y), lw, col, true)
		strokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbb.X), float32(sbb.Y), lw, col, true)
	}
}
