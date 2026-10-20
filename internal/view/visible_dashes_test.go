package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestVisibleDashesPreserveViewportAtEveryRotation(t *testing.T) {
	for _, zoom := range []float64{.4, .6, 1, 2} {
		for _, rotation := range []float64{0, .4, math.Pi / 2, math.Pi} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, 1.2} {
				cam := sim.NewCamera(1280, 720)
				cam.Pos = sim.Vec2{X: 900, Y: 160}
				cam.Zoom = zoom
				cam.Rotation = rotation
				center := sim.Vec2{}
				forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
				first, last := visibleDashRange(cam, center, forward, 16000, 24, 46)
				for i := 0; i < int(16000/46); i++ {
					for _, along := range []float64{-8000 + float64(i)*46, -8000 + float64(i)*46 + 24} {
						p := cam.WorldToScreen(center.Add(forward.Mul(along)))
						if p.X >= -10 && p.X <= 1290 && p.Y >= -10 && p.Y <= 730 && (i < first || i >= last) {
							t.Fatalf("Visible dash omitted: zoom=%v rotation=%v heading=%v i=%d range=%d..%d", zoom, rotation, heading, i, first, last)
						}
					}
				}
				if last-first > int(math.Hypot(1280, 720)/(zoom*46))+10 {
					t.Fatal("Range includes excessive offscreen dashes")
				}
			}
		}
	}
}
