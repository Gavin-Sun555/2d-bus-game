package ui

import (
	"image/color"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
)

const hudTopHeight = 96

type hudTopLayout struct {
	mode, cash, pax, maps, camera, settings sim.Rect
	stop, metrics, bell                     sim.Rect
}

// The game maintains a minimum logical viewport of 1280 pixels wide.
func newHUDTopLayout(w float64) hudTopLayout {
	return hudTopLayout{
		mode:     sim.Rect{X: 16, Y: 12, W: 220, H: 32},
		cash:     sim.Rect{X: 248, Y: 12, W: 180, H: 32},
		pax:      sim.Rect{X: 440, Y: 12, W: w - 982, H: 32},
		maps:     sim.Rect{X: w - 530, Y: 12, W: 165, H: 32},
		camera:   sim.Rect{X: w - 355, Y: 12, W: 165, H: 32},
		settings: sim.Rect{X: w - 180, Y: 12, W: 165, H: 32},
		stop:     sim.Rect{X: 16, Y: 56, W: w - 486, H: 28},
		metrics:  sim.Rect{X: w - 458, Y: 56, W: 245, H: 28},
		bell:     sim.Rect{X: w - 201, Y: 56, W: 186, H: 28},
	}
}

func fitHUDText(s string, size, width float64) string {
	s = strings.Join(strings.Fields(s), " ")
	if w, _ := textui.MeasureText(s, size); w <= width {
		return s
	}
	suffix := "…"
	if w, _ := textui.MeasureText(suffix, size); w > width {
		return ""
	}
	runes := []rune(s)
	lo, hi := 0, len(runes)
	for lo < hi {
		mid := (lo + hi + 1) / 2
		if w, _ := textui.MeasureText(string(runes[:mid])+suffix, size); w <= width {
			lo = mid
		} else {
			hi = mid - 1
		}
	}
	return string(runes[:lo]) + suffix
}

func drawHUDText(screen *ebiten.Image, s string, bounds sim.Rect, size float64, clr color.Color) {
	textui.DrawText(screen, fitHUDText(s, size, bounds.W-20), bounds.X+10, bounds.Y+(bounds.H-size)/2-1, size, clr)
}

// IsInsideTopBar prevents route editing through the two-row overlay.
func (h *HUD) IsInsideTopBar(p sim.Vec2) bool { return p.Y >= 0 && p.Y < hudTopHeight }
