package ui

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (h *HUD) drawPlanningInstructions(screen *ebiten.Image, route *sim.Route, world *sim.World, settings *Settings) {
	screenH := screen.Bounds().Dy()
	screenW := screen.Bounds().Dx()

	// Responsive panel width: adapt gracefully to screen width (up to 1160px)
	panelW := math.Min(float64(screenW)-64.0, 1160.0)
	if panelW < 640.0 {
		panelW = float64(screenW) - 24.0
	}
	panelX := (float64(screenW) - panelW) / 2.0
	maxContentW := panelW - 44.0

	// Get formatted route lines wrapped nicely within available panel width
	routeLines := route.GetFormattedRouteLines(world, maxContentW, 12.0)
	numRouteLines := len(routeLines)
	if numRouteLines < 1 {
		numRouteLines = 1
	}

	// Dynamic height based on number of wrapped route lines (with extra spacing for interactive buttons)
	panelH := 88.0 + float64(numRouteLines)*18.0
	panelY := float64(screenH) - panelH - 12.0
	h.PlanPanelBounds = sim.Rect{X: panelX, Y: panelY, W: panelW, H: panelH}

	// Card background with rich glassmorphism border
	vector.DrawFilledRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		sim.RGBA(15, 23, 42, 235), true)
	vector.StrokeRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		1.5, sim.RGBA(168, 85, 247, 240), true)

	// Top title bar: Title on left, Status badges on right
	titleText := locale.T("【线路规划面板 ⏸️游戏运行与客流生成已暂停】点击站点加入路线 (TAB返回驾驶)",
		"[Route Planning Panel - ⏸️Paused] Click stops to add to route (TAB to resume)")
	textui.DrawText(screen, titleText, panelX+20, panelY+12, 13, sim.RGBA(192, 132, 252, 255))

	// Status badges (Loop status & Fleet count) on the right side of top bar
	rightBadgeX := panelX + panelW - 20.0

	// Loop closure badge
	if len(route.StopIDs) >= 2 {
		if route.StopIDs[0] == route.StopIDs[len(route.StopIDs)-1] {
			loopStr := locale.T("[ 🔄 已形成闭环线路 ]", "[ 🔄 Closed Loop Route ]")
			loopW, _ := textui.MeasureText(loopStr, 11.0)
			rightBadgeX -= loopW
			textui.DrawText(screen, loopStr, rightBadgeX, panelY+13, 11, sim.RGBA(52, 211, 153, 255))
		} else {
			openStr := locale.T("[ ⚠️ 线路尚未闭环 (首尾站点不同) ]", "[ ⚠️ Open Route (Start/End differ) ]")
			openW, _ := textui.MeasureText(openStr, 11.0)
			rightBadgeX -= openW
			textui.DrawText(screen, openStr, rightBadgeX, panelY+13, 11, sim.RGBA(251, 191, 36, 255))
		}
	}

	// Second line: Stop Rules guidance
	rulesText := locale.T("[T] 线路与 AI 限速 · [H] 其他线路 · 右键/Shift+点站名编辑 · 当前："+route.Name,
		"[T] Lines / AI limits · [H] Other lines · Right-click / Shift+click stop: rename · Editing: "+route.Name)
	textui.DrawText(screen, rulesText, panelX+20, panelY+32, 11, sim.RGBA(250, 204, 21, 230))

	// Third line(s): The wrapped route lines
	currY := panelY + 50.0
	for lineIdx, lineStr := range routeLines {
		var lineColor color.Color = color.White
		if lineIdx > 0 {
			lineColor = sim.RGBA(226, 232, 240, 240) // Continuation line color
		}
		textui.DrawText(screen, lineStr, panelX+20, currY, 12, lineColor)
		currY += 18.0
	}

	// Bottom line: Action buttons on the right, shortcut key guidance on the left
	mx, my := ebiten.CursorPosition()
	mousePos := sim.Vec2{X: float64(mx), Y: float64(my)}

	clearBtnW := 96.0
	clearBtnH := 24.0
	clearBtnX := panelX + panelW - 20.0 - clearBtnW
	clearBtnY := currY - 3.0
	h.PlanClearBtnBounds = sim.Rect{X: clearBtnX, Y: clearBtnY, W: clearBtnW, H: clearBtnH}

	undoBtnW := 142.0
	undoBtnH := 24.0
	undoBtnX := clearBtnX - 10.0 - undoBtnW
	undoBtnY := currY - 3.0
	h.PlanUndoBtnBounds = sim.Rect{X: undoBtnX, Y: undoBtnY, W: undoBtnW, H: undoBtnH}

	hasStops := len(route.StopIDs) > 0

	// Draw Undo Button (Z / Backspace)
	undoHover := hasStops && h.PlanUndoBtnBounds.Contains(mousePos)
	undoBg := sim.RGBA(30, 41, 59, 230)
	undoBorder := sim.RGBA(245, 158, 11, 200)
	undoTextCol := sim.RGBA(251, 191, 36, 255)
	if !hasStops {
		undoBorder = sim.RGBA(100, 116, 139, 120)
		undoTextCol = sim.RGBA(100, 116, 139, 180)
	} else if undoHover {
		undoBg = sim.RGBA(245, 158, 11, 70)
		undoBorder = sim.RGBA(251, 191, 36, 255)
		undoTextCol = sim.RGBA(254, 240, 138, 255)
	}
	vector.DrawFilledRect(screen, float32(undoBtnX), float32(undoBtnY), float32(undoBtnW), float32(undoBtnH), undoBg, true)
	vector.StrokeRect(screen, float32(undoBtnX), float32(undoBtnY), float32(undoBtnW), float32(undoBtnH), 1.5, undoBorder, true)
	undoLabel := locale.T("↶ 撤销上一站 (Z)", "↶ Undo Stop (Z)")
	textui.DrawText(screen, undoLabel, undoBtnX+10.0, undoBtnY+5.0, 11.5, undoTextCol)

	// Draw Clear Button (C)
	clearHover := hasStops && h.PlanClearBtnBounds.Contains(mousePos)
	clearBg := sim.RGBA(30, 41, 59, 230)
	clearBorder := sim.RGBA(239, 68, 68, 200)
	clearTextCol := sim.RGBA(248, 113, 113, 255)
	if !hasStops {
		clearBorder = sim.RGBA(100, 116, 139, 120)
		clearTextCol = sim.RGBA(100, 116, 139, 180)
	} else if clearHover {
		clearBg = sim.RGBA(239, 68, 68, 70)
		clearBorder = sim.RGBA(248, 113, 113, 255)
		clearTextCol = sim.RGBA(254, 202, 202, 255)
	}
	vector.DrawFilledRect(screen, float32(clearBtnX), float32(clearBtnY), float32(clearBtnW), float32(clearBtnH), clearBg, true)
	vector.StrokeRect(screen, float32(clearBtnX), float32(clearBtnY), float32(clearBtnW), float32(clearBtnH), 1.5, clearBorder, true)
	clearLabel := locale.T("🗑️ 清空 (C)", "🗑️ Clear (C)")
	textui.DrawText(screen, clearLabel, clearBtnX+14.0, clearBtnY+5.0, 11.5, clearTextCol)

	// Shortcut action keys on the left
	shortcuts := locale.T("[Z/退格] 撤销上一站  |  [C] 清空  |  [TAB] 返回驾驶执行路线  |  [P] AI自动驾驶  |  [B] 狂暴模式",
		"[Z/Backspace] Undo Stop  |  [C] Clear  |  [TAB] Resume Driving  |  [P] Autopilot  |  [B] Berserk")
	if undoBtnX-(panelX+20.0) < 480.0 {
		shortcuts = locale.T("[Z/退格] 撤销上一站  |  [C] 清空路线  |  [TAB] 返回驾驶",
			"[Z/Backspace] Undo Stop  |  [C] Clear  |  [TAB] Resume Driving")
	}
	textui.DrawText(screen, shortcuts, panelX+20, currY+3.0, 11.5, sim.RGBA(148, 163, 184, 255))
}
