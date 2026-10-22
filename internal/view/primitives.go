package view

import (
	"github.com/hajimehoshi/ebiten/v2"
	"image/color"
	"math"
)

// Smooth straight lines need only a feathered quad, not a stencil atlas for
// arbitrary overlapping paths. Coverage fades over one screen pixel at edges.
func strokeLine(dst *ebiten.Image, x0, y0, x1, y1, width float32, clr color.Color, antialias bool) {
	dx, dy := x1-x0, y1-y0
	length := float32(math.Hypot(float64(dx), float64(dy)))
	if length <= 0 || width <= 0 {
		return
	}
	ux, uy := dx/length, dy/length
	r, g, b, a := clr.RGBA()
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	if ca > 0 {
		cr /= ca
		cg /= ca
		cb /= ca
	}
	feather := float32(.5)
	if !antialias {
		feather = 0
	}
	inner := float32(math.Max(0, float64(width/2-feather)))
	outer := width/2 + feather
	coverage := float32(math.Min(1, float64(width/outer)))
	endFeather := float32(math.Min(float64(feather), float64(length/2)))
	coverage *= float32(math.Min(1, float64(length/(length/2+feather))))
	xs := [4]float32{-feather, endFeather, length - endFeather, length + feather}
	ys := [4]float32{-outer, -inner, inner, outer}
	var vertices [16]ebiten.Vertex
	for y := 0; y < 4; y++ {
		for x := 0; x < 4; x++ {
			alpha := ca * coverage
			if antialias && (x == 0 || x == 3 || y == 0 || y == 3) {
				alpha = 0
			}
			vertices[y*4+x] = ebiten.Vertex{DstX: x0 + ux*xs[x] - uy*ys[y], DstY: y0 + uy*xs[x] + ux*ys[y], SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: alpha}
		}
	}
	var indices [54]uint16
	n := 0
	for y := 0; y < 3; y++ {
		for x := 0; x < 3; x++ {
			i := uint16(y*4 + x)
			for _, v := range [6]uint16{i, i + 1, i + 4, i + 1, i + 5, i + 4} {
				indices[n] = v
				n++
			}
		}
	}
	dst.DrawTriangles(vertices[:], indices[:], getWhiteSubImage(), nil)
}

// A tessellated circle with a one-pixel coverage ring batches with straight
// geometry, avoiding repeated vector stencil renders for hundreds of lamps.
func filledCircle(dst *ebiten.Image, x, y, radius float32, clr color.Color, antialias bool) {
	if radius <= 0 {
		return
	}
	r, g, b, a := clr.RGBA()
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	if ca > 0 {
		cr /= ca
		cg /= ca
		cb /= ca
	}
	steps := int(math.Ceil(float64(radius) * 2))
	if steps < 12 {
		steps = 12
	}
	if steps > 128 {
		steps = 128
	}
	feather := float32(.5)
	if !antialias {
		feather = 0
	}
	inner := float32(math.Max(0, float64(radius-feather)))
	outer := radius + feather
	coverage := float32(math.Min(1, float64(radius*2)))
	vertices := make([]ebiten.Vertex, 1+2*steps)
	indices := make([]uint16, 0, 9*steps)
	vertices[0] = ebiten.Vertex{DstX: x, DstY: y, SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca * coverage}
	for i := 0; i < steps; i++ {
		h := 2 * math.Pi * float64(i) / float64(steps)
		u, v := float32(math.Cos(h)), float32(math.Sin(h))
		vertices[1+i] = ebiten.Vertex{DstX: x + u*inner, DstY: y + v*inner, SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca * coverage}
		alpha := float32(0)
		if !antialias {
			alpha = ca
		}
		vertices[1+steps+i] = ebiten.Vertex{DstX: x + u*outer, DstY: y + v*outer, SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: alpha}
		j := (i + 1) % steps
		ii, jj := uint16(1+i), uint16(1+j)
		oi, oj := uint16(1+steps+i), uint16(1+steps+j)
		indices = append(indices, 0, ii, jj, ii, oi, jj, oi, oj, jj)
	}
	dst.DrawTriangles(vertices, indices, getWhiteSubImage(), nil)
}

func filledRect(dst *ebiten.Image, x, y, width, height float32, clr color.Color, antialias bool) {
	if width <= 0 || height <= 0 {
		return
	}
	strokeLine(dst, x, y+height/2, x+width, y+height/2, height, clr, antialias)
}
func strokeRect(dst *ebiten.Image, x, y, width, height, stroke float32, clr color.Color, antialias bool) {
	strokeLine(dst, x, y, x+width, y, stroke, clr, antialias)
	strokeLine(dst, x+width, y, x+width, y+height, stroke, clr, antialias)
	strokeLine(dst, x+width, y+height, x, y+height, stroke, clr, antialias)
	strokeLine(dst, x, y+height, x, y, stroke, clr, antialias)
}
func strokeCircle(dst *ebiten.Image, x, y, radius, width float32, clr color.Color, antialias bool) {
	if radius <= 0 || width <= 0 {
		return
	}
	r, g, b, a := clr.RGBA()
	cr, cg, cb, ca := float32(r)/65535, float32(g)/65535, float32(b)/65535, float32(a)/65535
	if ca > 0 {
		cr /= ca
		cg /= ca
		cb /= ca
	}
	steps := int(math.Ceil(float64(radius) * 2))
	if steps < 12 {
		steps = 12
	}
	if steps > 256 {
		steps = 256
	}
	feather := float32(.5)
	if !antialias {
		feather = 0
	}
	inner := float32(math.Max(0, float64(width/2-feather)))
	outer := width/2 + feather
	radii := [4]float32{float32(math.Max(0, float64(radius-outer))), radius - inner, radius + inner, radius + outer}
	vertices := make([]ebiten.Vertex, 4*steps)
	indices := make([]uint16, 0, 18*steps)
	for j, rr := range radii {
		for i := 0; i < steps; i++ {
			h := 2 * math.Pi * float64(i) / float64(steps)
			alpha := ca * float32(math.Min(1, float64(width)))
			if antialias && (j == 0 || j == 3) {
				alpha = 0
			}
			vertices[j*steps+i] = ebiten.Vertex{DstX: x + float32(math.Cos(h))*rr, DstY: y + float32(math.Sin(h))*rr, SrcX: 1, SrcY: 1, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: alpha}
			if j < 3 {
				ii, jj := uint16(j*steps+i), uint16(j*steps+(i+1)%steps)
				oi, oj := ii+uint16(steps), jj+uint16(steps)
				indices = append(indices, ii, oi, jj, oi, oj, jj)
			}
		}
	}
	dst.DrawTriangles(vertices, indices, getWhiteSubImage(), nil)
}
