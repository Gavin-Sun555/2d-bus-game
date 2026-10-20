package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Preserve the original dash phase while sampling only the visible part of a
// long road. A circumscribed viewport circle also covers rotated cameras.
func visibleDashRange(cam *sim.Camera, center, forward sim.Vec2, length, dash, step float64) (int, int) {
	delta := cam.Pos.Sub(center)
	along := delta.Dot(forward)
	span := (math.Hypot(float64(cam.ScreenWidth), float64(cam.ScreenHeight))/2 + 55) / cam.Zoom
	if math.Abs(delta.X*forward.Y-delta.Y*forward.X) > span {
		return 0, 0
	}
	count := int(length / step)
	first := max(0, min(count, int(math.Floor((along-span+length/2-dash)/step))))
	last := max(first, min(count, int(math.Ceil((along+span+length/2)/step))+1))
	return first, last
}
