package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

// Exercise the relocated scene renderers through the real Game.Draw entry point,
// including menu, driving, planning and settings overlays on every map.
func TestGameDrawAllModes(t *testing.T) {
	screen := ebiten.NewImage(1280, 720)
	g := NewGame(1280, 720)
	g.Draw(screen)
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		t.Run(theme.Name(), func(t *testing.T) {
			g.EnterCity(theme)
			g.Draw(screen)
			g.IsPlanMode = true
			g.Draw(screen)
			g.Settings.IsOpen = true
			g.Draw(screen)
			g.Settings.IsOpen = false
		})
	}
}
