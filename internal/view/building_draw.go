package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawBuilding(screen *ebiten.Image, cam *sim.Camera, b sim.Building, plan bool, theme sim.MapTheme) {
	if !cam.IsBoxVisible(b.Pos.X-b.W/2-45, b.Pos.Y-b.H/2-45, b.Pos.X+b.W/2+45, b.Pos.Y+b.H/2+45) {
		return
	}
	p := sceneColors(theme)
	// The shapes retain their real footprint; depth comes from roof structure and
	// shadows, rather than shifting the physical building toward a drivable road.
	shadow := sim.Vec2{X: 18, Y: 22}
	if theme == sim.MapThemeMetropolis {
		shadow = sim.Vec2{X: 30, Y: 38}
	}
	drawRotatedBox(screen, cam, b.Pos.Add(shadow), 1, 0, b.W, b.H, sim.RGBA(9, 17, 23, 95))
	switch theme {
	case sim.MapThemeCoastalBay:
		if b.Pos.X > 1000*sim.MapGridScale {
			drawIndustrialBuilding(screen, cam, b, plan, p)
			return
		}
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, rgb(233, 221, 187))
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-14, b.H-14, tint(b.Roof, rgb(181, 117, 79), 0.65))
		// Sun terrace, turquoise pool and ivory pergola distinguish resort roofs.
		pool := b.Pos.Add(sim.Vec2{X: b.W * 0.19})
		drawRotatedBox(screen, cam, pool, 1, 0, b.W*0.27, b.H*0.42, rgb(236, 220, 178))
		drawRotatedBox(screen, cam, pool, 1, 0, b.W*0.23, b.H*0.35, rgb(59, 152, 164))
		if !plan && cam.Zoom > 0.23 {
			for x := -b.W * 0.4; x < -b.W*0.05; x += 24 {
				drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{X: x}), 1, 0, 9, b.H*0.55, rgb(236, 225, 195))
			}
		}
	case sim.MapThemeRiverValley:
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, rgb(189, 180, 150))
		roof := rgb(118, 91, 70)
		if b.Pos.Y > 0 {
			roof = rgb(82, 105, 111)
		}
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-14, b.H-14, roof)
		// Split gables with a lit ridge and regular roof seams.
		drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{Y: -b.H * 0.22}), 1, 0, b.W-14, b.H*0.4, tint(roof, rgb(218, 190, 135), 0.18))
		worldLine(screen, cam, b.Pos.Add(sim.Vec2{X: -b.W/2 + 8}), b.Pos.Add(sim.Vec2{X: b.W/2 - 8}), 5, tint(roof, p.paving, 0.6))
		if !plan && cam.Zoom > 0.24 {
			for x := -b.W/2 + 22; x < b.W/2-10; x += 26 {
				worldLine(screen, cam, b.Pos.Add(sim.Vec2{X: x, Y: -b.H/2 + 9}), b.Pos.Add(sim.Vec2{X: x, Y: b.H/2 - 9}), 2, tint(roof, p.ground, 0.25))
			}
		}
	case sim.MapThemeEmeraldLake:
		drawWaterTownBuilding(screen, cam, b, plan, p)
	case sim.MapThemeTwinCities:
		if b.Pos.X < 0 {
			drawIndustrialBuilding(screen, cam, b, plan, p)
		} else {
			drawGlassBuilding(screen, cam, b, plan, p, true)
		}
	case sim.MapThemeCustom:
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, b.Color)
		drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-10, b.H-10, b.Roof)
	default:
		drawGlassBuilding(screen, cam, b, plan, p, false)
	}
}

func drawGlassBuilding(screen *ebiten.Image, cam *sim.Camera, b sim.Building, plan bool, p scenePalette, solar bool) {
	wall := rgb(74, 95, 116)
	roof := rgb(37, 58, 78)
	if solar {
		wall = rgb(144, 156, 181)
		roof = rgb(65, 76, 111)
		p.accent = rgb(165, 177, 219)
	}
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, wall)
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-14, b.H-14, roof)
	drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{Y: -b.H/2 + 11}), 1, 0, b.W-16, 6, p.accent)
	// A shaded south facade and reflected window bays read as building volume.
	facade := b.Pos.Add(sim.Vec2{Y: b.H/2 - 10})
	drawRotatedBox(screen, cam, facade, 1, 0, b.W-8, 12, tint(wall, rgb(12, 20, 36), .38))
	if cam.Zoom > .23 && !plan {
		for x := -b.W/2 + 16; x < b.W/2-10; x += 18 {
			window := b.Pos.Add(sim.Vec2{X: x, Y: b.H/2 - 10})
			drawRotatedBox(screen, cam, window, 1, 0, 8, 6, tint(p.accent, rgb(230, 212, 153), .45))
		}
	}
	// Large roof volumes remain legible even when looking at the whole city.
	core := b.Pos.Add(sim.Vec2{X: -b.W * 0.2})
	drawRotatedBox(screen, cam, core, 1, 0, b.W*0.30, b.H*0.52, tint(roof, wall, 0.45))
	drawRotatedBox(screen, cam, core.Add(sim.Vec2{X: -4, Y: -5}), 1, 0, b.W*0.30-8, b.H*0.52-10, roof)
	if plan || cam.Zoom < 0.2 {
		return
	}
	for x := -b.W/2 + 24; x < b.W/2-14; x += 34 {
		worldLine(screen, cam, b.Pos.Add(sim.Vec2{X: x, Y: -b.H/2 + 15}), b.Pos.Add(sim.Vec2{X: x, Y: b.H/2 - 15}), 2, tint(roof, p.accent, 0.28))
	}
	for y := -b.H/2 + 30; y < b.H/2-10; y += 30 {
		worldLine(screen, cam, b.Pos.Add(sim.Vec2{X: -b.W/2 + 12, Y: y}), b.Pos.Add(sim.Vec2{X: b.W/2 - 12, Y: y}), 1, tint(roof, wall, 0.5))
	}
	if solar {
		plaza := b.Pos.Add(sim.Vec2{X: -b.W * .20})
		worldCircle(screen, cam, plaza, math.Min(b.W, b.H)*.17, rgb(112, 134, 167))
		worldCircle(screen, cam, plaza, math.Min(b.W, b.H)*.13, rgb(59, 80, 115))
	}
	for i := 0; i < 3; i++ {
		pt := b.Pos.Add(sim.Vec2{X: b.W * (.07 + float64(i)*.13), Y: b.H * 0.14})
		c := rgb(156, 177, 184)
		if solar {
			c = rgb(36, 50, 88)
		}
		drawRotatedBox(screen, cam, pt, 1, 0, math.Min(24, b.W*.12), b.H*0.32, c)
		worldLine(screen, cam, pt.Add(sim.Vec2{Y: -b.H * .14}), pt.Add(sim.Vec2{Y: b.H * .14}), 1, p.accent)
	}
}

func drawIndustrialBuilding(screen *ebiten.Image, cam *sim.Camera, b sim.Building, plan bool, p scenePalette) {
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, rgb(139, 153, 158))
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-14, b.H-14, rgb(79, 100, 111))
	for i := 0; i < 4; i++ {
		x := -b.W*.37 + float64(i)*b.W*.245
		pt := b.Pos.Add(sim.Vec2{X: x})
		drawRotatedBox(screen, cam, pt, 1, 0, b.W*.18, b.H-24, rgb(105, 129, 137))
		drawRotatedBox(screen, cam, pt.Add(sim.Vec2{X: b.W * .07}), 1, 0, 6, b.H-24, rgb(65, 86, 100))
	}
	if plan || cam.Zoom < 0.2 {
		return
	}
	for x := -b.W * .43; x < b.W*.43; x += 30 {
		drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{X: x, Y: b.H * .38}), 1, 0, 14, 6, rgb(223, 184, 102))
	}
	// Vent housings and a rust-colored service strip.
	drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{Y: -b.H * .28}), 1, 0, b.W*.55, 20, tint(p.accent, rgb(169, 117, 82), 0.6))
}

func drawWaterTownBuilding(screen *ebiten.Image, cam *sim.Camera, b sim.Building, plan bool, p scenePalette) {
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, rgb(229, 226, 205))
	roof := rgb(56, 65, 60)
	// Four tiled wings around an open courtyard: white walls and charcoal eaves.
	for _, side := range []float64{-1, 1} {
		drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{Y: side * b.H * .32}), 1, 0, b.W-14, b.H*.30, roof)
		drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{X: side * b.W * .37}), 1, 0, b.W*.22, b.H-14, roof)
	}
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W*.45, b.H*.30, tint(p.ground, p.paving, 0.55))
	worldCircle(screen, cam, b.Pos, math.Min(b.W, b.H)*0.10, p.foliage)
	drawRotatedBox(screen, cam, b.Pos.Add(sim.Vec2{Y: b.H * .45}), 1, 0, 24, 10, p.accent)
	if plan || cam.Zoom < 0.25 {
		return
	}
	for x := -b.W/2 + 16; x < b.W/2-10; x += 16 {
		for _, side := range []float64{-1, 1} {
			a := b.Pos.Add(sim.Vec2{X: x, Y: side * b.H * .18})
			worldLine(screen, cam, a, a.Add(sim.Vec2{Y: side * b.H * .27}), 1.4, rgb(96, 102, 88))
		}
	}
}
