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

func worldTriangle(screen *ebiten.Image, cam *sim.Camera, p0, p1, p2 sim.Vec2, c color.RGBA) {
	minX := min(p0.X, p1.X, p2.X)
	maxX := max(p0.X, p1.X, p2.X)
	minY := min(p0.Y, p1.Y, p2.Y)
	maxY := max(p0.Y, p1.Y, p2.Y)
	if !cam.IsBoxVisible(minX, minY, maxX, maxY) {
		return
	}
	r, g, b, a := c.RGBA()
	if a == 0 {
		return
	}
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	cr /= ca
	cg /= ca
	cb /= ca
	s0 := cam.WorldToScreen(p0)
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vertices := [3]ebiten.Vertex{
		{DstX: float32(s0.X), DstY: float32(s0.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
		{DstX: float32(s1.X), DstY: float32(s1.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
		{DstX: float32(s2.X), DstY: float32(s2.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
	}
	indices := [3]uint16{0, 1, 2}
	screen.DrawTriangles(vertices[:], indices[:], getWhiteSubImage(), nil)
}

func worldQuad(screen *ebiten.Image, cam *sim.Camera, p0, p1, p2, p3 sim.Vec2, c color.RGBA) {
	minX := min(p0.X, p1.X, p2.X, p3.X)
	maxX := max(p0.X, p1.X, p2.X, p3.X)
	minY := min(p0.Y, p1.Y, p2.Y, p3.Y)
	maxY := max(p0.Y, p1.Y, p2.Y, p3.Y)
	if !cam.IsBoxVisible(minX, minY, maxX, maxY) {
		return
	}
	r, g, b, a := c.RGBA()
	if a == 0 {
		return
	}
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	cr /= ca
	cg /= ca
	cb /= ca
	s0 := cam.WorldToScreen(p0)
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	s3 := cam.WorldToScreen(p3)
	vertices := [4]ebiten.Vertex{
		{DstX: float32(s0.X), DstY: float32(s0.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
		{DstX: float32(s1.X), DstY: float32(s1.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
		{DstX: float32(s2.X), DstY: float32(s2.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
		{DstX: float32(s3.X), DstY: float32(s3.Y), SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca},
	}
	indices := [6]uint16{0, 1, 2, 0, 2, 3}
	screen.DrawTriangles(vertices[:], indices[:], getWhiteSubImage(), nil)
}

func worldPolygon(screen *ebiten.Image, cam *sim.Camera, points []sim.Vec2, c color.RGBA) {
	if len(points) < 3 {
		return
	}
	if len(points) == 3 {
		worldTriangle(screen, cam, points[0], points[1], points[2], c)
		return
	}
	if len(points) == 4 {
		worldQuad(screen, cam, points[0], points[1], points[2], points[3], c)
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
	r, g, b, a := c.RGBA()
	if a == 0 {
		return
	}
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	cr /= ca
	cg /= ca
	cb /= ca

	if len(points) <= 8 {
		var vertices [8]ebiten.Vertex
		for i, p := range points {
			s := cam.WorldToScreen(p)
			vertices[i] = ebiten.Vertex{
				DstX:   float32(s.X),
				DstY:   float32(s.Y),
				SrcX:   1,
				SrcY:   1,
				ColorR: cr,
				ColorG: cg,
				ColorB: cb,
				ColorA: ca,
			}
		}
		var indices [18]uint16
		nIdx := 0
		for i := 1; i < len(points)-1; i++ {
			indices[nIdx] = 0
			indices[nIdx+1] = uint16(i)
			indices[nIdx+2] = uint16(i + 1)
			nIdx += 3
		}
		screen.DrawTriangles(vertices[:len(points)], indices[:nIdx], getWhiteSubImage(), nil)
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
