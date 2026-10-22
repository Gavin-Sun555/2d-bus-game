package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func strokeWorldLine(screen *ebiten.Image, a, b sim.Vec2, width float64, c color.RGBA) {
	strokeLine(screen, float32(a.X), float32(a.Y), float32(b.X), float32(b.Y), float32(width), c, true)
}
func worldCircle(screen *ebiten.Image, cam *sim.Camera, center sim.Vec2, radius float64, c color.RGBA) {
	if !cam.IsPointVisible(center, radius) {
		return
	}
	s := cam.WorldToScreen(center)
	filledCircle(screen, float32(s.X), float32(s.Y), float32(radius*cam.Zoom), c, true)
}
func worldPolygon(screen *ebiten.Image, cam *sim.Camera, points []sim.Vec2, c color.RGBA) {
	if len(points) < 3 {
		return
	}
	minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
	for _, p := range points {
		minX = math.Min(minX, p.X)
		minY = math.Min(minY, p.Y)
		maxX = math.Max(maxX, p.X)
		maxY = math.Max(maxY, p.Y)
	}
	if !cam.IsBoxVisible(minX, minY, maxX, maxY) {
		return
	}
	var path vector.Path
	for i, p := range points {
		s := cam.WorldToScreen(p)
		if i == 0 {
			path.MoveTo(float32(s.X), float32(s.Y))
		} else {
			path.LineTo(float32(s.X), float32(s.Y))
		}
	}
	path.Close()
	op := vector.DrawPathOptions{AntiAlias: true}
	op.ColorScale.ScaleWithColor(c)
	vector.FillPath(screen, &path, nil, &op)
}
