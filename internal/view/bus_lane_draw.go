package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawBusLaneSurface(screen *ebiten.Image, cam *sim.Camera, road sim.RoadSegment, center sim.Vec2, cosH, sinH, length, lw float64) {
	forward := sim.Vec2{X: cosH, Y: sinH}
	right := sim.Vec2{X: -sinH, Y: cosH}
	for _, sign := range []float64{-1, 1} {
		laneCenter := center.Add(right.Mul(sign * 3.5 * lw))
		drawRotatedBox(screen, cam, laneCenter, cosH, sinH, length, lw-3, sim.RGBA(112, 53, 49, 255))
		if cam.Zoom < .35 {
			continue
		}
		for along := -length/2 + 220; along < length/2-100; along += 380 {
			pos := laneCenter.Add(forward.Mul(along))
			if !cam.IsPointVisible(pos, 60) {
				continue
			}
			// A compact readable label also identifies the lane in rotated views.
			point := cam.WorldToScreen(pos)
			textui.DrawText(screen, "BUS", point.X-13*cam.Zoom, point.Y-8*cam.Zoom, 14*cam.Zoom, sim.RGBA(255, 233, 196, 255))
		}
	}
}
