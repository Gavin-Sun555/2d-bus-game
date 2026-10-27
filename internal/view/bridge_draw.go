package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawBridges(screen *ebiten.Image, cam *sim.Camera, w *sim.World, laneWidth float64) {
	p := sceneColors(w.Theme)
	for _, original := range w.Bridges {
		if original.Length <= 0 || original.Width <= 0 || !cam.IsPointVisible(original.Pos, original.Length*.6+200) {
			continue
		}
		br := original // Rendering width adapts to the road without changing collision geometry.
		forward := sim.UnitFromAngle(br.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		for _, road := range w.Roads {
			dir := road.P2.Sub(road.P1).Normalize()
			along := br.Pos.Sub(road.P1).Dot(dir)
			lateral := math.Abs(br.Pos.Sub(road.P1).X*dir.Y - br.Pos.Sub(road.P1).Y*dir.X)
			if math.Abs(dir.Dot(forward)) > .95 && along >= 0 && along <= road.P2.Distance(road.P1) && lateral < 20 {
				br.Width = math.Max(br.Width, road.GetWidth(laneWidth)+12)
			}
		}
		half := br.Width / 2
		for _, side := range []float64{-1, 1} {
			edge := br.Pos.Add(right.Mul(side * (half + 4)))
			drawParallelLine(screen, cam, edge.Add(sim.Vec2{X: 7, Y: 10}), forward.X, forward.Y, br.Length, 0, sim.RGBA(8, 20, 30, 100), 12)
			drawParallelLine(screen, cam, edge, forward.X, forward.Y, br.Length, 0, p.paving, 12)
			drawParallelLine(screen, cam, edge, forward.X, forward.Y, br.Length, 0, tint(p.curb, rgb(245, 241, 219), .55), 3)
			switch {
			case br.IsCable:
				drawBridgeCables(screen, cam, br, forward, right, side)
			case br.IsArch:
				for d := -br.Length/2 + 20; d < br.Length/2; d += 70 {
					post := edge.Add(forward.Mul(d))
					if !cam.IsPointVisible(post, 12) {
						continue
					}
					worldCircle(screen, cam, post, 6, rgb(219, 218, 191))
					worldCircle(screen, cam, post.Add(sim.Vec2{X: -1, Y: -1}), 3, rgb(244, 240, 214))
				}
			case br.IsOverpass:
				for _, d := range []float64{-br.Length * .35, br.Length * .35} {
					pier := edge.Add(forward.Mul(d)).Add(right.Mul(side * 16))
					drawRotatedBox(screen, cam, pier, forward.X, forward.Y, 44, 22, rgb(112, 132, 144))
				}
				drawParallelLine(screen, cam, edge, forward.X, forward.Y, br.Length, 0, rgb(208, 176, 98), 1.5)
			}
		}
	}
}

// Pylons and stay cables follow the bridge heading. The old horizontal-only
// anchors drew a fan across vertical bridge lanes instead of along their edges.
func drawBridgeCables(screen *ebiten.Image, cam *sim.Camera, br sim.Bridge, forward, right sim.Vec2, side float64) {
	base := br.Pos.Add(right.Mul(side * (br.Width/2 + 10)))
	for _, d := range []float64{-br.Length * .25, br.Length * .25} {
		tower := base.Add(forward.Mul(d))
		crown := tower.Add(right.Mul(side * 52))
		drawRotatedBox(screen, cam, tower, forward.X, forward.Y, 26, 22, rgb(162, 176, 180))
		worldLine(screen, cam, tower, crown, 8, rgb(208, 216, 208))
		worldCircle(screen, cam, crown, 4, rgb(225, 141, 109))
		for offset := -br.Length * .18; offset <= br.Length*.18; offset += br.Length * .06 {
			anchor := tower.Add(forward.Mul(offset))
			worldLine(screen, cam, crown, anchor, 1.3, sim.RGBA(212, 225, 218, 155))
		}
	}
}
