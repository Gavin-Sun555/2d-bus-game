package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawWaterBodies(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	for _, wb := range w.Waters {
		drawThemedWater(screen, cam, wb, w.Theme)
	}
}

func drawThemedWater(screen *ebiten.Image, cam *sim.Camera, wb sim.WaterBody, theme sim.MapTheme) {
	left, right := wb.Center.X-wb.Width/2, wb.Center.X+wb.Width/2
	top, bottom := wb.Center.Y-wb.Height/2, wb.Center.Y+wb.Height/2
	if !cam.IsBoxVisible(left-180, top-100, right+100, bottom+100) {
		return
	}
	p := sceneColors(theme)
	if theme == sim.MapThemeCustom {
		p.water = wb.Color
		p.foam = wb.Shimmer
	}
	naturalLake := theme == sim.MapThemeEmeraldLake && wb.IsLake
	if naturalLake {
		for layer := 0; layer < 3; layer++ {
			size := 1.035 - float64(layer)*.025
			points := make([]sim.Vec2, 0, 64)
			for i := 0; i < 64; i++ {
				angle := float64(i) * math.Pi / 32
				ripple := 1 + .025*math.Sin(angle*5) + .018*math.Cos(angle*9)
				points = append(points, wb.Center.Add(sim.Vec2{X: math.Cos(angle) * wb.Width / 2 * size * ripple, Y: math.Sin(angle) * wb.Height / 2 * size * ripple}))
			}
			color := tint(p.ground, p.paving, .55)
			if layer == 1 {
				color = p.shallows
			}
			if layer == 2 {
				color = p.water
			}
			worldPolygon(screen, cam, points, color)
		}
	} else {
		drawRotatedBox(screen, cam, wb.Center, 1, 0, wb.Width, wb.Height, p.water)
	}

	// A thin bank and shallow shelves give a readable shoreline at overview zoom.
	shore := 45.0
	if !naturalLake {
		for _, edge := range []struct {
			center        sim.Vec2
			width, height float64
		}{
			{sim.Vec2{X: left + shore/2, Y: wb.Center.Y}, shore, wb.Height},
			{sim.Vec2{X: right - shore/2, Y: wb.Center.Y}, shore, wb.Height},
			{sim.Vec2{X: wb.Center.X, Y: top + shore/2}, wb.Width, shore},
			{sim.Vec2{X: wb.Center.X, Y: bottom - shore/2}, wb.Width, shore},
		} {
			drawRotatedBox(screen, cam, edge.center, 1, 0, edge.width, edge.height, p.shallows)
		}
	}
	if wb.IsOcean && wb.HasBeach {
		// Draw the beach AFTER the water. Previously the water covered its sand strip.
		for band := 0; band < 6; band++ {
			x := left + float64(band)*55
			c := tint(p.shallows, p.water, float64(band)/6)
			drawRotatedBox(screen, cam, sim.Vec2{X: x + 27.5, Y: wb.Center.Y}, 1, 0, 55, wb.Height, c)
		}
		drawRotatedBox(screen, cam, sim.Vec2{X: left - 65, Y: wb.Center.Y}, 1, 0, 190, wb.Height, rgb(211, 194, 145))
		drawRotatedBox(screen, cam, sim.Vec2{X: left + 28, Y: wb.Center.Y}, 1, 0, 14, wb.Height, p.foam)
		if cam.Zoom > 0.18 {
			_, minY, _, maxY := visibleWorldBounds(cam)
			for y := math.Max(top, math.Floor(minY/180)*180); y < math.Min(bottom, maxY+180); y += 180 {
				a := sim.Vec2{X: left + 65, Y: y}
				worldLine(screen, cam, a, a.Add(sim.Vec2{X: 12, Y: 60}), 3, tint(p.foam, p.shallows, 0.4))
			}
		}
	}
	if cam.Zoom < 0.18 {
		return
	}
	minX, minY, maxX, maxY := visibleWorldBounds(cam)
	minX = math.Max(left+70, minX)
	maxX = math.Min(right-70, maxX)
	minY = math.Max(top+60, minY)
	maxY = math.Min(bottom-60, maxY)
	// World-space rows of gently curved glints, not two screen-fixed dashes.
	for y := math.Floor(minY/135) * 135; y < maxY; y += 135 {
		for x := math.Floor(minX/240) * 240; x < maxX; x += 240 {
			if x < left+60 || y < top+45 {
				continue
			}
			if naturalLake {
				dx := (x - wb.Center.X) / (wb.Width / 2)
				dy := (y - wb.Center.Y) / (wb.Height / 2)
				if dx*dx+dy*dy > .88 {
					continue
				}
			}
			shift := 22 * math.Sin(x*0.012+y*0.009)
			a := sim.Vec2{X: x + shift, Y: y}
			c := tint(p.water, p.foam, 0.28)
			worldLine(screen, cam, a, a.Add(sim.Vec2{X: 27, Y: 3}), 1.3, c)
			worldLine(screen, cam, a.Add(sim.Vec2{X: 27, Y: 3}), a.Add(sim.Vec2{X: 62}), 1.3, c)
			if wb.IsLake && int(math.Abs(x+y))%7 == 0 {
				worldCircle(screen, cam, a.Add(sim.Vec2{Y: 12}), 6, p.foliage)
				worldCircle(screen, cam, a.Add(sim.Vec2{X: 4, Y: 10}), 2, rgb(231, 187, 167))
			}
		}
	}
}

func drawRiver(screen *ebiten.Image, cam *sim.Camera, riv sim.River, theme sim.MapTheme) {
	drawThemedWater(screen, cam, sim.WaterBody{Center: sim.Vec2{Y: riv.Y}, Width: 32000, Height: riv.Height}, theme)
}
