package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawLandscape(screen *ebiten.Image, cam *sim.Camera, w *sim.World) {
	p := sceneColors(w.Theme)
	switch w.Theme {
	case sim.MapThemeRiverValley:
		// Layered, irregular contour terraces on the northern mountain bank.
		for i := 0; i < 9; i++ {
			center := sim.Vec2{X: -8300 + float64(i)*2100, Y: -3300 + math.Sin(float64(i)*1.7)*620}
			if !cam.IsPointVisible(center, 1600) {
				continue
			}
			for layer := 0; layer < 5; layer++ {
				scale := 1 - float64(layer)*.145
				points := make([]sim.Vec2, 0, 24)
				for j := 0; j < 24; j++ {
					angle := float64(j) * 2 * math.Pi / 24
					r := 1 + .10*math.Sin(angle*3+float64(i)) + .04*math.Cos(angle*7)
					points = append(points, center.Add(sim.Vec2{X: math.Cos(angle) * 1250 * scale * r, Y: math.Sin(angle) * 720 * scale * r}))
				}
				color := tint(p.ground, rgb(125, 139, 91), float64(layer)*.13+.08)
				worldPolygon(screen, cam, points, color)
				if cam.Zoom > .18 {
					for j := range points {
						worldLine(screen, cam, points[j], points[(j+1)%len(points)], 2, tint(color, rgb(190, 180, 130), .17))
					}
				}
			}
		}
	case sim.MapThemeTwinCities:
		// Broad west/east campus bands make the two districts distinct at map scale.
		drawRotatedBox(screen, cam, sim.Vec2{X: -5400}, 1, 0, 6200, 8400, sim.RGBA(111, 105, 94, 22))
		drawRotatedBox(screen, cam, sim.Vec2{X: 5400}, 1, 0, 6200, 8400, sim.RGBA(60, 112, 143, 22))
	}
}

func drawLotGarden(screen *ebiten.Image, cam *sim.Camera, b sim.Building, theme sim.MapTheme, plan bool) {
	if !cam.IsPointVisible(b.Pos, math.Hypot(b.W, b.H)/2+40) {
		return
	}
	p := sceneColors(theme)
	foundation := tint(p.paving, p.ground, .55)
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W+28, b.H+28, foundation)
	if plan || cam.Zoom < .23 {
		return
	}
	for _, side := range []float64{-1, 1} {
		position := b.Pos.Add(sim.Vec2{X: side * (b.W/2 + 11)})
		drawRotatedBox(screen, cam, position, 1, 0, 6, b.H*.65, p.foliage)
	}
	entrance := b.Pos.Add(sim.Vec2{Y: b.H/2 + 10})
	drawRotatedBox(screen, cam, entrance, 1, 0, 36, 9, p.paving)
}
