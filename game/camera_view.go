package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) updateDrivingViewHotkey() {
	if !inpututil.IsKeyJustPressed(ebiten.KeyX) {
		return
	}
	switch g.Camera.ToggleDrivingView() {
	case 2:
		g.HUD.AddToast(locale.T("视野已扩大：长宽各 2 倍；按 X 切换 4 倍", "Wide view: 2× width and height. Press X for 4×."), false)
	case 4:
		g.HUD.AddToast(locale.T("视野已扩大：长宽各 4 倍，面积 16 倍；按 X 恢复", "Wide view: 4× width and height, 16× area. Press X to restore."), false)
	default:
		g.HUD.AddToast(locale.T("已恢复默认视野；按 X 切换 2 倍", "Default view restored. Press X for 2×."), false)
	}
}
