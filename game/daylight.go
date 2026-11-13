package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (g *Game) syncAutoHeadlights() {
	night := g.World.Clock.IsNight()
	if g.TrafficMgr != nil {
		for _, c := range g.TrafficMgr.Cars {
			c.HeadlightsOn = night
		}
	}
	if g.AIBusMgr != nil {
		for _, a := range g.AIBusMgr.Buses {
			a.Bus.HeadlightsOn = night
		}
	}
}
func (g *Game) ToggleHeadlights() { g.Bus.HeadlightsOn = !g.Bus.HeadlightsOn }
func (g *Game) clockPanelBounds() sim.Rect {
	return sim.Rect{X: float64(g.Width) - 300, Y: 104, W: 285, H: 64}
}
func (g *Game) headlightButtonBounds() sim.Rect {
	r := g.clockPanelBounds()
	r.Y += 34
	r.H = 28
	return r
}
func (g *Game) updateClockControls() bool {
	if inpututil.IsKeyJustPressed(ebiten.KeyJ) {
		g.ToggleHeadlights()
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) || inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonRight) {
		x, y := ebiten.CursorPosition()
		p := sim.Vec2{X: float64(x), Y: float64(y)}
		if g.clockPanelBounds().Contains(p) {
			if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) && g.headlightButtonBounds().Contains(p) {
				g.ToggleHeadlights()
			}
			return true
		}
	}
	return false
}
func (g *Game) drawClockHUD(screen *ebiten.Image) {
	r := g.clockPanelBounds()
	r.H = 30
	panelBox(screen, r, sim.RGBA(15, 23, 42, 240))
	state := locale.T("白天", "DAY")
	col := sim.RGBA(253, 224, 71, 255)
	if g.World.Clock.IsNight() {
		state = locale.T("夜晚", "NIGHT")
		col = sim.RGBA(165, 180, 252, 255)
	}
	textui.DrawText(screen, g.World.Clock.TimeString()+" · "+state, r.X+14, r.Y+7, 15, col)
	b := g.headlightButtonBounds()
	panelBox(screen, b, sim.RGBA(15, 23, 42, 240))
	label := locale.T("[J] 车灯：关闭", "[J] Headlights: OFF")
	color := sim.RGBA(148, 163, 184, 255)
	if g.Bus.HeadlightsOn {
		label = locale.T("[J] 车灯：开启", "[J] Headlights: ON")
		color = sim.RGBA(125, 211, 252, 255)
	}
	vector.StrokeRect(screen, float32(b.X), float32(b.Y), float32(b.W), float32(b.H), 1, color, true)
	textui.DrawText(screen, label, b.X+14, b.Y+7, 13, color)
}
