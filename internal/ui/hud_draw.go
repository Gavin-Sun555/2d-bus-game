package ui

import (
	"fmt"
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (h *HUD) Draw(screen *ebiten.Image, bus *sim.Bus, route *sim.Route, world *sim.World, isPlanMode bool, cash int, activeStop *sim.BusStop, settings *Settings, stopBellRung bool, passengersAlighting int, cam *sim.Camera, tls ...*sim.TrafficLightSystem) {
	w := screen.Bounds().Dx()
	_ = screen.Bounds().Dy()

	var activeTLS *sim.TrafficLightSystem
	if len(tls) > 0 && tls[0] != nil {
		activeTLS = tls[0]
	}

	// 1. Top Status Banner (Glassmorphism dark slate bar)
	vector.DrawFilledRect(screen, 0, 0, float32(w), 56, sim.RGBA(15, 23, 42, 220), false)
	vector.StrokeLine(screen, 0, 56, float32(w), 56, 1.5, sim.RGBA(51, 65, 85, 255), false)

	// Mode Indicator Badge
	modeText := locale.T(" [驾驶模式 DRIVE] (TAB 规划)", " [DRIVE MODE] (TAB Route)")
	modeBg := sim.RGBA(14, 165, 233, 240) // Cyan
	if isPlanMode {
		modeText = locale.T(" [线路规划模式 PLAN] (⏸️游戏已暂停)", " [PLAN MODE] (⏸️PAUSED)")
		modeBg = sim.RGBA(168, 85, 247, 240) // Purple
	}
	vector.DrawFilledRect(screen, 16, 12, 240, 32, modeBg, true)
	textui.DrawText(screen, modeText, 22, 20, 13, color.White)

	// Economy & Capacity Badges
	cashText := locale.Tf("营收: $%d", "Revenue: $%d", cash)
	textui.DrawText(screen, cashText, 246, 20, 13, sim.RGBA(250, 204, 21, 255))

	paxText := locale.Tf("乘客: %d/%d", "Pax: %d/%d", bus.PassengerCount, bus.Capacity)
	textui.DrawText(screen, paxText, 345, 20, 13, sim.RGBA(56, 189, 248, 255))

	// Current target stop & Stop bell status
	if targetID := route.GetCurrentTargetStopID(); targetID >= 0 && targetID < len(world.Stops) {
		stop := world.Stops[targetID]
		distToStop := bus.Pos.Distance(stop.Pos)
		distMeters := int(math.Round(distToStop * 0.5))
		targetMsg := locale.Tf("下一站: [%s] 距%dm (候车 %d人)", "Next: [%s] %dm (Wait: %d)", stop.LocalizedName(), distMeters, stop.WaitingPassengers)
		textui.DrawText(screen, targetMsg, 445, 20, 12, sim.RGBA(226, 232, 240, 255))

		// Stop Bell badge
		bellX := 660.0
		if stopBellRung {
			bellText := locale.Tf("🔔 下车铃: %d人准备下车", "🔔 Bell: %d to alight", passengersAlighting)
			vector.DrawFilledRect(screen, float32(bellX), 12, 175, 32, sim.RGBA(180, 83, 9, 230), true)
			vector.StrokeRect(screen, float32(bellX), 12, 175, 32, 1.5, sim.RGBA(250, 204, 21, 255), true)
			textui.DrawText(screen, bellText, bellX+10, 20, 12, sim.RGBA(254, 240, 138, 255))
		} else {
			bellText := locale.T("🔕 下车铃: 无人按铃", "🔕 Bell: None")
			vector.DrawFilledRect(screen, float32(bellX), 12, 150, 32, sim.RGBA(30, 41, 59, 220), true)
			vector.StrokeRect(screen, float32(bellX), 12, 150, 32, 1.0, sim.RGBA(100, 116, 139, 200), true)
			textui.DrawText(screen, bellText, bellX+12, 20, 12, sim.RGBA(148, 163, 184, 255))
		}
	} else {
		textui.DrawText(screen, locale.T("未选择目标站点", "No target stop selected"), 445, 20, 13, sim.RGBA(148, 163, 184, 255))
	}

	// Map Theme Selector Badge
	// Main Menu / Change Map Badge
	mapBtnX := float32(w - 530)
	mapBtnW := float32(165)
	h.MenuBtnBounds = sim.Rect{X: float64(mapBtnX), Y: 12, W: float64(mapBtnW), H: 32}
	h.MapThemeBtnBounds = h.MenuBtnBounds
	menuBtnStr := locale.T("🏠 主菜单/换图", "🏠 Menu / Maps")
	menuBtnColor := sim.RGBA(168, 85, 247, 255)
	if settings != nil {
		switch settings.MapTheme {
		case sim.MapThemeCoastalBay:
			menuBtnStr = locale.T("🏠 海滨港湾 [换图]", "🏠 Coastal Bay [Maps]")
			menuBtnColor = sim.RGBA(6, 182, 212, 255)
		case sim.MapThemeRiverValley:
			menuBtnStr = locale.T("🏠 溪谷江城 [换图]", "🏠 River Valley [Maps]")
			menuBtnColor = sim.RGBA(16, 185, 129, 255)
		case sim.MapThemeEmeraldLake:
			menuBtnStr = locale.T("🏠 湖畔水乡 [换图]", "🏠 Emerald Lake [Maps]")
			menuBtnColor = sim.RGBA(34, 197, 94, 255)
		case sim.MapThemeTwinCities:
			menuBtnStr = locale.T("🏠 高新双城 [换图]", "🏠 Twin Districts [Maps]")
			menuBtnColor = sim.RGBA(217, 70, 239, 255)
		case sim.MapThemeCustom:
			menuBtnStr = locale.T("🏠 自定义图 [换图]", "🏠 Custom Map [Maps]")
			menuBtnColor = sim.RGBA(245, 158, 11, 255)
		default:
			menuBtnStr = locale.T("🏠 现代都会 [换图]", "🏠 Metropolis [Maps]")
			menuBtnColor = sim.RGBA(59, 130, 246, 255)
		}
	}
	vector.DrawFilledRect(screen, mapBtnX, 12, mapBtnW, 32, sim.RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, mapBtnX, 12, mapBtnW, 32, 1.5, menuBtnColor, true)
	textui.DrawText(screen, menuBtnStr, float64(mapBtnX)+12, 20, 12, menuBtnColor)

	// Camera Perspective Badge (Switchable between North-Up, Heading-Up, and South-Up)
	camBtnX := float32(w - 355)
	camBtnW := float32(165)
	h.CameraBtnBounds = sim.Rect{X: float64(camBtnX), Y: 12, W: float64(camBtnW), H: 32}
	camModeStr := locale.T("🎥 视角: 正北 [V]", "🎥 Cam: North [V]")
	camModeColor := sim.RGBA(148, 163, 184, 255)
	if cam != nil {
		switch cam.Mode {
		case sim.CameraModeHeadingUp:
			camModeStr = locale.T("🎥 视角: 跟随 [V]", "🎥 Cam: Follow [V]")
			camModeColor = sim.RGBA(56, 189, 248, 255)
		case sim.CameraModeSouthUp:
			camModeStr = locale.T("🎥 视角: 正南 [V]", "🎥 Cam: South [V]")
			camModeColor = sim.RGBA(251, 146, 60, 255)
		}
	}
	vector.DrawFilledRect(screen, camBtnX, 12, camBtnW, 32, sim.RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, camBtnX, 12, camBtnW, 32, 1.5, camModeColor, true)
	textui.DrawText(screen, camModeStr, float64(camBtnX)+12, 20, 12, camModeColor)

	// Settings button / violation status on top right
	fineStatusText := locale.T("[违章: 开启]", "[Fines: ON]")
	fineStatusColor := sim.RGBA(34, 197, 94, 255)
	if !settings.TrafficFinesEnabled {
		fineStatusText = locale.T("[违章: 关闭]", "[Fines: OFF]")
		fineStatusColor = sim.RGBA(239, 68, 68, 255)
	}
	settingsBtnX := float32(w - 180)
	vector.DrawFilledRect(screen, settingsBtnX, 12, 165, 32, sim.RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, settingsBtnX, 12, 165, 32, 1.5, fineStatusColor, true)
	settingsBtnLabel := fmt.Sprintf("%s [ESC]", fineStatusText)
	textui.DrawText(screen, settingsBtnLabel, float64(settingsBtnX)+14, 20, 12, fineStatusColor)

	// 2. Bottom Dashboard (in Drive Mode)
	if !isPlanMode {
		var targetStop *sim.BusStop
		if targetID := route.GetCurrentTargetStopID(); targetID >= 0 && targetID < len(world.Stops) {
			targetStop = world.Stops[targetID]
		}
		h.drawDriveDashboard(screen, bus, activeStop, targetStop, stopBellRung, passengersAlighting, activeTLS, settings)
	} else {
		h.drawPlanningInstructions(screen, route, world, settings)
	}

	// 3. Render Toasts
	h.drawToasts(screen)
}

func (h *HUD) drawToasts(screen *ebiten.Image) {
	toastX := 24.0
	toastY := 70.0
	for _, t := range h.Toasts {
		alpha := float32(t.Timer / t.MaxTime)
		if alpha > 1 {
			alpha = 1
		}
		bg := sim.RGBA(30, 41, 59, uint8(alpha*220))
		border := sim.RGBA(59, 130, 246, uint8(alpha*255))
		toastW := 380.0

		if t.IsSuccess {
			border = sim.RGBA(34, 197, 94, uint8(alpha*255))
		} else if t.IsWarning {
			bg = sim.RGBA(127, 29, 29, uint8(alpha*235))     // Deep Crimson
			border = sim.RGBA(239, 68, 68, uint8(alpha*255)) // Glowing Red Alert
			toastW = 440.0
		}

		vector.DrawFilledRect(screen, float32(toastX), float32(toastY), float32(toastW), 32, bg, true)
		vector.StrokeRect(screen, float32(toastX), float32(toastY), float32(toastW), 32, 1.5, border, true)
		textui.DrawText(screen, t.Text, toastX+14, toastY+8, 12, color.White)
		toastY += 38.0
	}
}

// drawMiniTurnArrow draws a crisp, antialiased vector left-turn arrow (↰) for the traffic signal HUD
func drawMiniTurnArrow(screen *ebiten.Image, cx, cy float32, clr color.Color) {
	// Vertical stem from bottom
	vector.StrokeLine(screen, cx+2.5, cy+4.5, cx+2.5, cy-2.0, 1.5, clr, true)
	// Horizontal branch turning left
	vector.StrokeLine(screen, cx+2.5, cy-2.0, cx-3.5, cy-2.0, 1.5, clr, true)
	// Left arrowhead barbs
	vector.StrokeLine(screen, cx-3.5, cy-2.0, cx-0.5, cy-5.0, 1.5, clr, true)
	vector.StrokeLine(screen, cx-3.5, cy-2.0, cx-0.5, cy+1.0, 1.5, clr, true)
}

// drawMiniStraightArrow draws a crisp, antialiased vector straight arrow (↑) for the traffic signal HUD
func drawMiniStraightArrow(screen *ebiten.Image, cx, cy float32, clr color.Color) {
	// Vertical stem
	vector.StrokeLine(screen, cx, cy+4.5, cx, cy-3.5, 1.5, clr, true)
	// Arrowhead barbs
	vector.StrokeLine(screen, cx, cy-3.5, cx-3.0, cy-0.5, 1.5, clr, true)
	vector.StrokeLine(screen, cx, cy-3.5, cx+3.0, cy-0.5, 1.5, clr, true)
}
