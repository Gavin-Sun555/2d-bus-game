package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func DrawWorld(w *sim.World, screen *ebiten.Image, cam *sim.Camera, targetStopID int, laneWidth float64, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	// 1. Draw base ground with theme-tailored palette
	switch w.Theme {
	case sim.MapThemeCoastalBay:
		screen.Fill(sim.RGBA(24, 32, 47, 255)) // Warm maritime navy-slate
	case sim.MapThemeEmeraldLake:
		screen.Fill(sim.RGBA(17, 29, 24, 255)) // Verdant parkland slate-green
	default:
		screen.Fill(sim.RGBA(15, 23, 42, 255)) // Dark metropolitan slate
	}

	// 2. Draw Scenic Waterbodies, Oceans, Lakes & Canals
	drawWaterBodies(screen, cam, w)
	for _, riv := range w.Rivers {
		drawRiver(screen, cam, riv)
	}

	// 3. Draw Road Network, Soft Shoulders & Hard Crash Barriers
	drawIntersectionPads(screen, cam, w, laneWidth, planMode)
	for _, r := range w.Roads {
		drawRoad(screen, cam, w, r, laneWidth, planMode)
	}
	drawIntersectionAsphalt(screen, cam, w, laneWidth)

	// 4. Draw Roundabouts (Rotary plazas and central monuments)
	drawRoundabouts(screen, cam, w)

	// 5. Draw Bridges (Railings, cable stays, stone arches, overpass viaducts)
	drawBridges(screen, cam, w, laneWidth)

	// 6. Draw Intersections & Crosswalks (skip in plan mode or when zoomed out)
	if !planMode && cam.Zoom >= 0.35 {
		drawCrosswalks(screen, cam, w, laneWidth)
	}

	// 7. Draw Bus Stops
	for _, s := range w.Stops {
		DrawStop(s, screen, cam, s.ID == targetStopID, false, planMode)
	}

	// 8. Draw Buildings & Shadows
	for _, b := range w.Buildings {
		drawBuilding(screen, cam, b, planMode)
	}

	// 9. Draw Trees
	for _, t := range w.Trees {
		drawTree(screen, cam, t, planMode)
	}
}

func drawWaterBodies(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	for _, wb := range w.Waters {
		halfW := wb.Width / 2
		halfH := wb.Height / 2
		if !cam.IsBoxVisible(wb.Center.X-halfW, wb.Center.Y-halfH, wb.Center.X+halfW, wb.Center.Y+halfH) {
			continue
		}

		if wb.IsOcean {
			// Coastal Golden Beach Strip along west shore of ocean
			beachW := 90.0
			beachCenter := sim.Vec2{X: wb.Center.X - halfW + beachW/2, Y: wb.Center.Y}
			drawRotatedBox(screen, cam, beachCenter, 1, 0, beachW, wb.Height, sim.RGBA(217, 119, 6, 190))
			beachEdge := beachCenter.Add(sim.Vec2{X: beachW / 2, Y: 0})
			drawRotatedBox(screen, cam, beachEdge, 1, 0, 16, wb.Height, sim.RGBA(254, 240, 138, 160))
		}

		// Main water surface
		drawRotatedBox(screen, cam, wb.Center, 1, 0, wb.Width, wb.Height, wb.Color)

		// Water shimmer / wave lines
		if cam.Zoom >= 0.25 {
			s := cam.WorldToScreen(wb.Center)
			shimmerLen := float32(120 * cam.Zoom)
			vector.StrokeLine(screen, float32(s.X-60), float32(s.Y-40), float32(s.X-60)+shimmerLen, float32(s.Y-40), float32(1.8*cam.Zoom), wb.Shimmer, true)
			vector.StrokeLine(screen, float32(s.X+80), float32(s.Y+50), float32(s.X+80)+shimmerLen, float32(s.Y+50), float32(1.8*cam.Zoom), wb.Shimmer, true)
		}
	}
}

func drawBridges(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	for _, br := range w.Bridges {
		rad := math.Max(br.Length, br.Width) * 0.65
		if !cam.IsPointVisible(br.Pos, rad) {
			continue
		}

		cosH := math.Cos(br.Heading)
		sinH := math.Sin(br.Heading)

		// 1. Heavy reinforced bridge deck borders
		offset := br.Width / 2
		drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, sim.RGBA(15, 23, 42, 255), 4)
		drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, sim.RGBA(15, 23, 42, 255), 4)

		if br.IsOverpass {
			// Grade-separated Highway Overpass Viaduct (立体跨线立交桥)
			// 1. Sleek drop shadow along outer bottom edge outside the bridge deck
			drawParallelLine(screen, cam, br.Pos.Add(sim.Vec2{X: 6, Y: 10}), cosH, sinH, br.Length, offset+8, sim.RGBA(15, 23, 42, 150), 10)
			// 2. Sturdy reinforced support pier pillars on outer sides at canal banks
			pierOffset := br.Width/2 + 8
			pierW := 22.0
			pierL := 40.0
			drawRotatedBox(screen, cam, br.Pos.Add(sim.Vec2{X: -br.Length * 0.35, Y: pierOffset}), cosH, sinH, pierL, pierW, sim.RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(sim.Vec2{X: -br.Length * 0.35, Y: -pierOffset}), cosH, sinH, pierL, pierW, sim.RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(sim.Vec2{X: br.Length * 0.35, Y: pierOffset}), cosH, sinH, pierL, pierW, sim.RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(sim.Vec2{X: br.Length * 0.35, Y: -pierOffset}), cosH, sinH, pierL, pierW, sim.RGBA(100, 116, 139, 255))
			// 3. Heavy reinforced concrete crash barriers with warning stripes
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, sim.RGBA(226, 232, 240, 255), 4.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, sim.RGBA(226, 232, 240, 255), 4.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset-4, sim.RGBA(245, 158, 11, 230), 2.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset+4, sim.RGBA(245, 158, 11, 230), 2.5)
		} else if br.IsCable {
			// Grand Bay Suspension Cable Bridge
			p1 := cam.WorldToScreen(br.Pos.Add(sim.Vec2{X: -br.Length * 0.25, Y: 0}))
			p2 := cam.WorldToScreen(br.Pos.Add(sim.Vec2{X: br.Length * 0.25, Y: 0}))
			towerH := float32(65 * cam.Zoom)
			towerW := float32(7 * cam.Zoom)

			// Pylon 1
			vector.StrokeLine(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(p1.X), float32(p1.Y+float64(towerH*0.3)), towerW, sim.RGBA(148, 163, 184, 255), true)
			vector.DrawFilledCircle(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(4*cam.Zoom), sim.RGBA(239, 68, 68, 255), true)

			// Pylon 2
			vector.StrokeLine(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(p2.X), float32(p2.Y+float64(towerH*0.3)), towerW, sim.RGBA(148, 163, 184, 255), true)
			vector.DrawFilledCircle(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(4*cam.Zoom), sim.RGBA(239, 68, 68, 255), true)

			// Suspension stay cables
			cableLw := float32(math.Max(1.0, 1.6*cam.Zoom))
			cableCol := sim.RGBA(226, 232, 240, 170)
			for d := -br.Length * 0.45; d <= br.Length*0.45; d += br.Length * 0.15 {
				ptDeck := cam.WorldToScreen(br.Pos.Add(sim.Vec2{X: d, Y: 0}))
				vector.StrokeLine(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(ptDeck.X), float32(ptDeck.Y), cableLw, cableCol, true)
				vector.StrokeLine(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(ptDeck.X), float32(ptDeck.Y), cableLw, cableCol, true)
			}
		} else if br.IsArch {
			// Classical Jiangnan Stone Arch Bridge Balustrades
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, sim.RGBA(241, 245, 249, 230), 3)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, sim.RGBA(241, 245, 249, 230), 3)
		} else {
			// Modern Twin Canal Bridges with yellow hazard warning markers
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, sim.RGBA(234, 179, 8, 230), 2.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, sim.RGBA(234, 179, 8, 230), 2.5)
		}
	}
}

func drawRiver(screen *ebiten.Image, cam *sim.Camera, riv sim.River) {
	if !cam.IsBoxVisible(-16000, riv.Y-riv.Height/2, 16000, riv.Y+riv.Height/2) {
		return
	}
	drawRotatedBox(screen, cam, sim.Vec2{X: 0, Y: riv.Y}, 1, 0, 32000, riv.Height, sim.RGBA(14, 116, 144, 210)) // Cyan-blue canal water
}

func drawBuilding(screen *ebiten.Image, cam *sim.Camera, b sim.Building, planMode bool) {
	if !cam.IsBoxVisible(b.Pos.X-b.W/2-20, b.Pos.Y-b.H/2-20, b.Pos.X+b.W/2+20, b.Pos.Y+b.H/2+20) {
		return
	}

	if !planMode {
		shadowOffset := sim.Vec2{X: 12, Y: 16}
		drawRotatedBox(screen, cam, b.Pos.Add(shadowOffset), 1, 0, b.W, b.H, sim.RGBA(2, 6, 23, 110))
	}
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, b.Color)

	roofPadding := 10.0
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-roofPadding, b.H-roofPadding, b.Roof)

	// Window lights: skip in plan mode or when zoomed out to eliminate ~400 vector circles
	if !planMode && cam.Zoom >= 0.45 {
		for wx := -b.W/2 + 25; wx < b.W/2-25; wx += 32 {
			for wy := -b.H/2 + 25; wy < b.H/2-25; wy += 32 {
				wp := b.Pos.Add(sim.Vec2{X: wx, Y: wy})
				ws := cam.WorldToScreen(wp)
				vector.DrawFilledCircle(screen, float32(ws.X), float32(ws.Y), float32(3*cam.Zoom),
					sim.RGBA(254, 240, 138, 160), true)
			}
		}
	}
}

func drawTree(screen *ebiten.Image, cam *sim.Camera, t sim.Tree, planMode bool) {
	if !cam.IsPointVisible(t.Pos, t.Radius+15) {
		return
	}

	ts := cam.WorldToScreen(t.Pos)
	r := float32(t.Radius * cam.Zoom)
	if r < 1.5 {
		r = 1.5
	}

	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Single canopy circle eliminates 500+ vector calls
		vector.DrawFilledCircle(screen, float32(ts.X), float32(ts.Y), r, t.Color, true)
		return
	}

	vector.DrawFilledCircle(screen, float32(ts.X+float64(r*0.4)), float32(ts.Y+float64(r*0.4)), r, sim.RGBA(2, 6, 23, 70), true)
	vector.DrawFilledCircle(screen, float32(ts.X), float32(ts.Y), r, t.Color, true)
	vector.DrawFilledCircle(screen, float32(ts.X-float64(r*0.2)), float32(ts.Y-float64(r*0.2)), r*0.5,
		sim.RGBA(74, 222, 128, 140), true)
}
