package view

import (
	"github.com/hajimehoshi/ebiten/v2"
	"image/color"
	"reflect"
	"testing"
)

func TestCircleMeshMatchesOriginalAtEveryTessellation(t *testing.T) {
	colors := []color.Color{color.NRGBA{R: 190, G: 40, B: 220, A: 128}, color.RGBA{R: 255, G: 255, B: 255, A: 255}, color.NRGBA{A: 0}}
	compare := func(gv, wv []ebiten.Vertex, gi, wi []uint16) {
		t.Helper()
		if !reflect.DeepEqual(gv, wv) || !reflect.DeepEqual(gi, wi) {
			t.Fatalf("mesh changed: vertices %d/%d indices %d/%d", len(gv), len(wv), len(gi), len(wi))
		}
	}
	for steps := 12; steps <= 256; steps++ {
		for _, epsilon := range []float32{-.001, 0, .001} {
			radius := float32(steps)/2 + epsilon
			for _, aa := range []bool{false, true} {
				for _, clr := range colors {
					gv, gi := filledCircleMesh(-3.125, 19.75, radius, clr, aa)
					wv, wi := referenceFilledCircleMesh(-3.125, 19.75, radius, clr, aa)
					compare(gv, wv, gi, wi)
					for _, width := range []float32{.1, 1, 3.25, radius * 3} {
						gv, gi := strokeCircleMesh(7.25, -11.125, radius, width, clr, aa)
						wv, wi := referenceStrokeCircleMesh(7.25, -11.125, radius, width, clr, aa)
						compare(gv, wv, gi, wi)
					}
				}
			}
		}
	}
	for _, radius := range []float32{-1, 0, .01, .25, 1, 512} {
		gv, gi := filledCircleMesh(0, 0, radius, colors[0], true)
		wv, wi := referenceFilledCircleMesh(0, 0, radius, colors[0], true)
		compare(gv, wv, gi, wi)
		for _, width := range []float32{-1, 0, .1, 2000} {
			gv, gi := strokeCircleMesh(0, 0, radius, width, colors[0], true)
			wv, wi := referenceStrokeCircleMesh(0, 0, radius, width, colors[0], true)
			compare(gv, wv, gi, wi)
		}
	}
}
func TestCircleMeshSteadyStateDoesNotAllocate(t *testing.T) {
	clr := color.Color(color.NRGBA{R: 100, G: 200, B: 50, A: 128})
	filledCircleMesh(1, 2, 13, clr, true)
	strokeCircleMesh(1, 2, 29, 2, clr, true)
	if got := testing.AllocsPerRun(1000, func() { filledCircleMesh(1, 2, 13, clr, true); strokeCircleMesh(1, 2, 29, 2, clr, true) }); got != 0 {
		t.Fatalf("steady mesh allocations: %.0f", got)
	}
}

var circleBenchVertices []ebiten.Vertex
var circleBenchIndices []uint16

func BenchmarkCircleMesh(b *testing.B) {
	clr := color.Color(color.NRGBA{R: 100, G: 200, B: 50, A: 128})
	for _, old := range []bool{true, false} {
		name := "reused"
		if old {
			name = "original"
		}
		b.Run(name, func(b *testing.B) {
			filledCircleMesh(1, 2, 13, clr, true)
			strokeCircleMesh(1, 2, 29, 2, clr, true)
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if old {
					circleBenchVertices, circleBenchIndices = referenceFilledCircleMesh(1, 2, 13, clr, true)
					circleBenchVertices, circleBenchIndices = referenceStrokeCircleMesh(1, 2, 29, 2, clr, true)
				} else {
					circleBenchVertices, circleBenchIndices = filledCircleMesh(1, 2, 13, clr, true)
					circleBenchVertices, circleBenchIndices = strokeCircleMesh(1, 2, 29, 2, clr, true)
				}
			}
		})
	}
}
