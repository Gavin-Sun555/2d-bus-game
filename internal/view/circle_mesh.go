package view

import (
	"github.com/hajimehoshi/ebiten/v2"
	"math"
)

const maxCircleSteps = 256

type circleDirection struct{ x, y float32 }
type circleTemplate struct {
	directions   []circleDirection
	fill, stroke []uint16
}

// These buffers follow the same render-thread-only contract as sharedQuadVertices.
// DrawTriangles copies the submitted vertices and indices before returning;
// callers must submit a mesh before building the next one. Templates are keyed
// only by the bounded tessellation count, never by radius or window size.
var circleVertices [4 * maxCircleSteps]ebiten.Vertex
var circleTemplates [maxCircleSteps + 1]circleTemplate

func circleTemplateFor(steps int) *circleTemplate {
	t := &circleTemplates[steps]
	if t.directions == nil {
		t.directions = make([]circleDirection, steps)
		for i := range t.directions {
			h := 2 * math.Pi * float64(i) / float64(steps)
			// Preserve the original float64 angle and float32 conversion exactly.
			t.directions[i] = circleDirection{float32(math.Cos(h)), float32(math.Sin(h))}
		}
	}
	return t
}
func (t *circleTemplate) filledIndices(steps int) []uint16 {
	if t.fill == nil {
		t.fill = make([]uint16, 0, 9*steps)
		for i := 0; i < steps; i++ {
			j := (i + 1) % steps
			ii, jj := uint16(1+i), uint16(1+j)
			oi, oj := uint16(1+steps+i), uint16(1+steps+j)
			t.fill = append(t.fill, 0, ii, jj, ii, oi, jj, oi, oj, jj)
		}
	}
	return t.fill
}
func (t *circleTemplate) strokeIndices(steps int) []uint16 {
	if t.stroke == nil {
		t.stroke = make([]uint16, 0, 18*steps)
		for j := 0; j < 3; j++ {
			for i := 0; i < steps; i++ {
				ii, jj := uint16(j*steps+i), uint16(j*steps+(i+1)%steps)
				oi, oj := ii+uint16(steps), jj+uint16(steps)
				t.stroke = append(t.stroke, ii, oi, jj, oi, oj, jj)
			}
		}
	}
	return t.stroke
}
