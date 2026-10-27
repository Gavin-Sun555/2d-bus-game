package ui

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (s *Settings) drawAISpeedRanges(screen *ebiten.Image, x, y float64) {
	values := []int{s.AICarMinSpeed, s.AICarMaxSpeed}
	for row := 0; row < 1; row++ {
		ry := y + float64(row)*48
		title := locale.T("AI 小车限速 (km/h)", "AI car limits (km/h)")
		textui.DrawText(screen, title, x+26, ry+8, 13, sim.RGBA(226, 232, 240, 255))
		for column := 0; column < 2; column++ {
			field := row*2 + column
			bx := x + 230 + float64(column)*156
			caption := locale.T("最低", "Min")
			if column == 1 {
				caption = locale.T("最高", "Max")
			}
			textui.DrawText(screen, caption, bx, ry+8, 12, sim.RGBA(148, 163, 184, 255))
			textui.DrawText(screen, fmt.Sprint(values[field]), bx+65, ry+8, 13, sim.RGBA(56, 189, 248, 255))
			for direction := 0; direction < 2; direction++ {
				px := bx + 36
				if direction == 1 {
					px = bx + 108
				}
				rect := sim.Rect{X: px, Y: ry, W: 26, H: 30}
				s.speedRangeBounds[field][direction] = rect
				vector.DrawFilledRect(screen, float32(px), float32(ry), 26, 30, sim.RGBA(30, 41, 59, 255), true)
				vector.StrokeRect(screen, float32(px), float32(ry), 26, 30, 1, sim.RGBA(71, 85, 105, 255), true)
				label := "-"
				if direction == 1 {
					label = "+"
				}
				textui.DrawText(screen, label, px+8, ry+7, 14, sim.RGBA(226, 232, 240, 255))
			}
		}
	}
}
