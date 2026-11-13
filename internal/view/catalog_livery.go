package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
	"math"
)

func drawCatalogLivery(b *sim.Bus, screen *ebiten.Image, cam *sim.Camera) {
	s, ok := sim.FindVehicleSpec(b.CatalogID)
	if !ok {
		return
	}
	forward := sim.UnitFromAngle(b.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	// Each manufacturer has a different roof layout and side stripe pattern.
	switch s.Style {
	case 0:
		p := b.Pos.Add(forward.Mul(-b.Length * .28))
		drawRotatedBox(screen, cam, p, forward.X, forward.Y, 12, b.Width-8, sim.RGBA(235, 245, 240, 255))
	case 1:
		for _, side := range []float64{-1, 1} {
			p := b.Pos.Add(right.Mul(side * (b.Width/2 - 3)))
			drawRotatedBox(screen, cam, p, forward.X, forward.Y, b.Length*.65, 3, sim.RGBA(240, 245, 255, 255))
		}
	case 2:
		for _, d := range []float64{-.27, -.12} {
			p := b.Pos.Add(forward.Mul(b.Length * d))
			drawRotatedBox(screen, cam, p, math.Cos(b.Heading+.3), math.Sin(b.Heading+.3), 7, b.Width-7, sim.RGBA(255, 245, 220, 255))
		}
	}
}
