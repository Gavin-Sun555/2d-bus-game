package view

import (
	"github.com/hajimehoshi/ebiten/v2"
	"image/color"
	"math"
)

// Frozen pre-buffer-reuse geometry, kept independent of the optimized builders.
func referenceFilledCircleMesh(x, y, radius float32, clr color.Color, antialias bool) ([]ebiten.Vertex, []uint16) {
	if radius <= 0 {
		return nil, nil
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
	return vertices, indices
}

func referenceStrokeCircleMesh(x, y, radius, width float32, clr color.Color, antialias bool) ([]ebiten.Vertex, []uint16) {
	if radius <= 0 || width <= 0 {
		return nil, nil
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
	return vertices, indices
}
