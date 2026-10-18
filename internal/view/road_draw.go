package view

import (
	"image/color"
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func drawIntersectionPads(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64, planMode bool) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		shoulderHalfX := nsHalf + sim.ShoulderWidth + 14.0
		shoulderHalfY := ewHalf + sim.ShoulderWidth + 14.0
		if !cam.IsPointVisible(c, math.Max(shoulderHalfX, shoulderHalfY)*1.5) {
			continue
		}
		// In plan mode, skip shoulder backing apron at intersections
		if !planMode {
			drawRotatedBox(screen, cam, c, 1, 0, shoulderHalfX*2, shoulderHalfY*2, sim.RGBA(51, 65, 85, 255))
		}
	}
}

func drawIntersectionAsphalt(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		padHalfX := nsHalf + 18.0
		padHalfY := ewHalf + 18.0
		if !cam.IsPointVisible(c, math.Max(padHalfX, padHalfY)*1.5) {
			continue
		}
		drawRotatedBox(screen, cam, c, 1, 0, padHalfX*2, padHalfY*2, sim.RGBA(30, 41, 59, 255))

		// Draw smooth curved curb return fillets at all 4 corners of the intersection
		// so turning vehicles have broad, sweeping asphalt connecting the perpendicular roads
		curbR := 36.0 * cam.Zoom
		if curbR >= 2.0 {
			quads := [4][2]float64{
				{-padHalfX + 16.0, -padHalfY + 16.0},
				{padHalfX - 16.0, -padHalfY + 16.0},
				{-padHalfX + 16.0, padHalfY - 16.0},
				{padHalfX - 16.0, padHalfY - 16.0},
			}
			for _, q := range quads {
				cornerPt := cam.WorldToScreen(c.Add(sim.Vec2{X: q[0], Y: q[1]}))
				vector.DrawFilledCircle(screen, float32(cornerPt.X), float32(cornerPt.Y), float32(curbR), sim.RGBA(30, 41, 59, 255), true)
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
	boundRadius := len*0.5 + roadW*1.5 + sim.ShoulderWidth*2
	if !cam.IsPointVisible(center, boundRadius) {
		return
	}

	heading := math.Atan2(dir.Y, dir.X)
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	// 1. Soft Shoulder (drivable buffer with deceleration, ShoulderWidth = 40px)
	if !planMode && cam.Zoom >= 0.35 {
		shoulderTotalW := roadW + sim.ShoulderWidth*2
		drawRotatedBox(screen, cam, center, cosH, sinH, len, shoulderTotalW, sim.RGBA(51, 65, 85, 255)) // Slate sidewalk / shoulder
	}

	// 2. Asphalt Road Surface (#1e293b)
	drawRotatedBox(screen, cam, center, cosH, sinH, len, roadW, sim.RGBA(30, 41, 59, 255))

	// 3. Road Curb dividing lines & Hard Crash Barrier along outer edge of shoulder (skip in plan mode / low zoom)
	if !planMode && cam.Zoom >= 0.35 {
		curbOffset := roadW / 2
		drawParallelLine(screen, cam, center, cosH, sinH, len, curbOffset, sim.RGBA(148, 163, 184, 180), 2)
		drawParallelLine(screen, cam, center, cosH, sinH, len, -curbOffset, sim.RGBA(148, 163, 184, 180), 2)

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
		drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, barrierOffset, laneWidth, splits)
		drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, -barrierOffset, laneWidth, splits)
	}

	// 5. Center Dividers & Lane Markings
	if r.HasLines {
		if r.Lanes >= 6 || r.Type == sim.RoadExpressway6Lane {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6.0, sim.RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6.0, sim.RGBA(250, 204, 21, 240))

			laneOffset1 := laneWidth * 1.0
			laneOffset2 := laneWidth * 2.0
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset2, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset2, planMode)
		} else if r.Lanes == 4 || r.Type == sim.RoadAvenue4Lane {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6.0, sim.RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6.0, sim.RGBA(250, 204, 21, 240))

			laneOffset1 := laneWidth
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset1, planMode)
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
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), stroke*float32(cam.Zoom), c, true)
}

func drawHazardBarrier(screen *ebiten.Image, cam *sim.Camera, r sim.RoadSegment, center sim.Vec2, cosH, sinH, len, offset, laneWidth float64, splits []float64) {
	isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
	gapRadius := math.Max(245.0, laneWidth*4.2+50.0) // generous opening at all road intersections for comfortable turning

	if isEW {
		minX := math.Min(r.P1.X, r.P2.X)
		maxX := math.Max(r.P1.X, r.P2.X)
		prevX := minX
		for _, ix := range splits {
			segStart := prevX + gapRadius
			if prevX == minX {
				segStart = minX
			}
			segEnd := ix - gapRadius
			if segEnd > segStart {
				p1 := sim.Vec2{X: segStart, Y: r.P1.Y + offset}
				p2 := sim.Vec2{X: segEnd, Y: r.P1.Y + offset}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
			}
			prevX = ix
		}
		// Final segment to road end
		segStart := prevX + gapRadius
		if prevX == minX {
			segStart = minX
		}
		segEnd := maxX
		if segEnd > segStart {
			p1 := sim.Vec2{X: segStart, Y: r.P1.Y + offset}
			p2 := sim.Vec2{X: segEnd, Y: r.P1.Y + offset}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
		}
	} else {
		minY := math.Min(r.P1.Y, r.P2.Y)
		maxY := math.Max(r.P1.Y, r.P2.Y)
		prevY := minY
		for _, iy := range splits {
			segStart := prevY + gapRadius
			if prevY == minY {
				segStart = minY
			}
			segEnd := iy - gapRadius
			if segEnd > segStart {
				p1 := sim.Vec2{X: r.P1.X - offset, Y: segStart}
				p2 := sim.Vec2{X: r.P1.X - offset, Y: segEnd}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
			}
			prevY = iy
		}
		// Final segment to road end
		segStart := prevY + gapRadius
		if prevY == minY {
			segStart = minY
		}
		segEnd := maxY
		if segEnd > segStart {
			p1 := sim.Vec2{X: r.P1.X - offset, Y: segStart}
			p2 := sim.Vec2{X: r.P1.X - offset, Y: segEnd}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), sim.RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), sim.RGBA(234, 179, 8, 255), true)
		}
	}
}

func drawCenterDividers(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, len, perpOffset float64, c color.RGBA) {
	p1 := center.Add(sim.Vec2{X: -cosH*len/2 - sinH*perpOffset, Y: -sinH*len/2 + cosH*perpOffset})
	p2 := center.Add(sim.Vec2{X: cosH*len/2 - sinH*perpOffset, Y: sinH*len/2 + cosH*perpOffset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
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
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(sim.Vec2{X: cosH * dStart, Y: sinH * dStart})
		db := center.Add(sim.Vec2{X: cosH * dEnd, Y: sinH * dEnd})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
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
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(sim.Vec2{X: cosH*dStart - sinH*perpOffset, Y: sinH*dStart + cosH*perpOffset})
		db := center.Add(sim.Vec2{X: cosH*dEnd - sinH*perpOffset, Y: sinH*dEnd + cosH*perpOffset})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(2*cam.Zoom), sim.RGBA(241, 245, 249, 180), true)
	}
}

func drawCrosswalks(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, pt := range w.Intersections {
		if w.IsAtRoundabout(pt) {
			continue // Roundabouts feature self-regulating circular rotary flow, not straight 4-way crosswalks or lane arrows
		}

		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(pt, laneWidth)
		ewStopLineDist := nsHalf + 43.0
		ewCrosswalkDist := ewStopLineDist - 25.0
		nsStopLineDist := ewHalf + 43.0
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

		// 2. Draw dedicated turning lane markings & pavement arrows for multi-lane avenues
		if westLanes >= 6 {
			// West approach: 6-lane express avenue (Lane 0 left-turn, Lane 1 through, Lane 2 through/turn)
			wCenter := pt.X - ewStopLineDist
			wLane0Y := pt.Y + 4 + laneWidth*0.5
			wLane1Y := pt.Y + 4 + laneWidth*1.5
			wLane2Y := pt.Y + 4 + laneWidth*2.5
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 45, Y: wLane0Y}, 0, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 110, Y: wLane0Y}, 0, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 45, Y: wLane1Y}, 0, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 110, Y: wLane1Y}, 0, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 45, Y: wLane2Y}, 0, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 110, Y: wLane2Y}, 0, false)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: wCenter - 60, Y: pt.Y + 4 + laneWidth*1.0}, 1, 0, 120)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: wCenter - 60, Y: pt.Y + 4 + laneWidth*2.0}, 1, 0, 120)
		} else if westLanes >= 4 {
			wCenter := pt.X - ewStopLineDist
			wInnerY := pt.Y + 4 + laneWidth*0.5
			wOuterY := pt.Y + 4 + laneWidth*1.5
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 45, Y: wInnerY}, 0, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 110, Y: wInnerY}, 0, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 45, Y: wOuterY}, 0, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: wCenter - 110, Y: wOuterY}, 0, false)
			wLineY := pt.Y + 4 + laneWidth
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: wCenter - 60, Y: wLineY}, 1, 0, 120)
		}

		if eastLanes >= 6 {
			// East approach: 6-lane express avenue
			eCenter := pt.X + ewStopLineDist
			eLane0Y := pt.Y - 4 - laneWidth*0.5
			eLane1Y := pt.Y - 4 - laneWidth*1.5
			eLane2Y := pt.Y - 4 - laneWidth*2.5
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 45, Y: eLane0Y}, math.Pi, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 110, Y: eLane0Y}, math.Pi, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 45, Y: eLane1Y}, math.Pi, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 110, Y: eLane1Y}, math.Pi, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 45, Y: eLane2Y}, math.Pi, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 110, Y: eLane2Y}, math.Pi, false)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: eCenter + 60, Y: pt.Y - 4 - laneWidth*1.0}, -1, 0, 120)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: eCenter + 60, Y: pt.Y - 4 - laneWidth*2.0}, -1, 0, 120)
		} else if eastLanes >= 4 {
			eCenter := pt.X + ewStopLineDist
			eInnerY := pt.Y - 4 - laneWidth*0.5
			eOuterY := pt.Y - 4 - laneWidth*1.5
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 45, Y: eInnerY}, math.Pi, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 110, Y: eInnerY}, math.Pi, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 45, Y: eOuterY}, math.Pi, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: eCenter + 110, Y: eOuterY}, math.Pi, false)
			eLineY := pt.Y - 4 - laneWidth
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: eCenter + 60, Y: eLineY}, -1, 0, 120)
		}

		if northLanes >= 6 {
			// North approach: 6-lane express avenue
			nCenter := pt.Y - nsStopLineDist
			nLane0X := pt.X - 4 - laneWidth*0.5
			nLane1X := pt.X - 4 - laneWidth*1.5
			nLane2X := pt.X - 4 - laneWidth*2.5
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane0X, Y: nCenter - 45}, math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane0X, Y: nCenter - 110}, math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane1X, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane1X, Y: nCenter - 110}, math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane2X, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: nLane2X, Y: nCenter - 110}, math.Pi/2, false)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: pt.X - 4 - laneWidth*1.0, Y: nCenter - 60}, 0, 1, 120)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: pt.X - 4 - laneWidth*2.0, Y: nCenter - 60}, 0, 1, 120)
		} else if northLanes >= 4 {
			nCenter := pt.Y - nsStopLineDist
			nInnerX := pt.X - 4 - laneWidth*0.5
			nOuterX := pt.X - 4 - laneWidth*1.5
			drawGroundArrow(screen, cam, sim.Vec2{X: nInnerX, Y: nCenter - 45}, math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: nInnerX, Y: nCenter - 110}, math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: nOuterX, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: nOuterX, Y: nCenter - 110}, math.Pi/2, false)
			nLineX := pt.X - 4 - laneWidth
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: nLineX, Y: nCenter - 60}, 0, 1, 120)
		}

		if southLanes >= 6 {
			// South approach: 6-lane express avenue
			sCenter := pt.Y + nsStopLineDist
			sLane0X := pt.X + 4 + laneWidth*0.5
			sLane1X := pt.X + 4 + laneWidth*1.5
			sLane2X := pt.X + 4 + laneWidth*2.5
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane0X, Y: sCenter + 45}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane0X, Y: sCenter + 110}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane1X, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane1X, Y: sCenter + 110}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane2X, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: sLane2X, Y: sCenter + 110}, -math.Pi/2, false)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: pt.X + 4 + laneWidth*1.0, Y: sCenter + 60}, 0, -1, 120)
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: pt.X + 4 + laneWidth*2.0, Y: sCenter + 60}, 0, -1, 120)
		} else if southLanes >= 4 {
			sCenter := pt.Y + nsStopLineDist
			sInnerX := pt.X + 4 + laneWidth*0.5
			sOuterX := pt.X + 4 + laneWidth*1.5
			drawGroundArrow(screen, cam, sim.Vec2{X: sInnerX, Y: sCenter + 45}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: sInnerX, Y: sCenter + 110}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, sim.Vec2{X: sOuterX, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, sim.Vec2{X: sOuterX, Y: sCenter + 110}, -math.Pi/2, false)
			sLineX := pt.X + 4 + laneWidth
			drawSolidWhiteDivider(screen, cam, sim.Vec2{X: sLineX, Y: sCenter + 60}, 0, -1, 120)
		}
	}
}

func drawZebraBranch(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, horizontal bool, lanes int, laneWidth float64) {
	numStripes := 6
	step := 15.0
	if lanes >= 6 {
		numStripes = 18
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
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(7.5*cam.Zoom), sim.RGBA(241, 245, 249, 215), true)
	}
}

func drawSolidWhiteDivider(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, cosH, sinH, length float64) {
	halfLen := length * 0.5
	p1 := center.Add(sim.Vec2{X: -cosH * halfLen, Y: -sinH * halfLen})
	p2 := center.Add(sim.Vec2{X: cosH * halfLen, Y: sinH * halfLen})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), sim.RGBA(248, 250, 252, 220), true)
}

func drawGroundArrow(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, heading float64, isLeftTurn bool) {
	scale := float32(cam.Zoom)
	if scale < 0.28 {
		return
	}
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	fwd := sim.Vec2{X: cosH, Y: sinH}
	left := sim.Vec2{X: sinH, Y: -cosH} // In screen coords (+Y is down), driver's left is (sinH, -cosH)

	col := sim.RGBA(248, 250, 252, 225)
	lw := float32(2.8 * scale)

	if !isLeftTurn {
		// Straight Arrow (↑)
		p1 := pos.Sub(fwd.Mul(14))
		p2 := pos.Add(fwd.Mul(14))
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lw, col, true)

		pl := pos.Add(fwd.Mul(6)).Add(left.Mul(6))
		pr := pos.Add(fwd.Mul(6)).Sub(left.Mul(6))
		sl := cam.WorldToScreen(pl)
		sr := cam.WorldToScreen(pr)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sl.X), float32(sl.Y), lw, col, true)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sr.X), float32(sr.Y), lw, col, true)
	} else {
		// Left-Turn Arrow (↰)
		p1 := pos.Sub(fwd.Mul(14))
		pCurve := pos.Add(fwd.Mul(4))
		s1 := cam.WorldToScreen(p1)
		sc := cam.WorldToScreen(pCurve)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(sc.X), float32(sc.Y), lw, col, true)

		pTip := pCurve.Add(left.Mul(15))
		stip := cam.WorldToScreen(pTip)
		vector.StrokeLine(screen, float32(sc.X), float32(sc.Y), float32(stip.X), float32(stip.Y), lw, col, true)

		pBarbT := pTip.Sub(left.Mul(5)).Add(fwd.Mul(4))
		pBarbB := pTip.Sub(left.Mul(5)).Sub(fwd.Mul(4))
		sbt := cam.WorldToScreen(pBarbT)
		sbb := cam.WorldToScreen(pBarbB)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbt.X), float32(sbt.Y), lw, col, true)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbb.X), float32(sbb.Y), lw, col, true)
	}
}
