package ui

import (
	"fmt"
	"image/color"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (s *Settings) Draw(screen *ebiten.Image, cam ...*sim.Camera) {
	if !s.IsOpen {
		return
	}

	if len(cam) > 0 && cam[0] != nil {
		s.CameraViewMode = cam[0].Mode
	}

	sw := float64(screen.Bounds().Dx())
	sh := float64(screen.Bounds().Dy())

	// Dark semi-transparent backdrop
	vector.DrawFilledRect(screen, 0, 0, float32(sw), float32(sh), sim.RGBA(0, 0, 0, 160), false)

	// Modal card
	modalW := 580.0
	modalH := 624.0
	modalX := (sw - modalW) / 2
	modalY := (sh - modalH) / 2

	// Modal shadow and background
	vector.DrawFilledRect(screen, float32(modalX+6), float32(modalY+6), float32(modalW), float32(modalH),
		sim.RGBA(2, 6, 23, 140), true)
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		sim.RGBA(15, 23, 42, 245), true)
	vector.StrokeRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		2, sim.RGBA(59, 130, 246, 255), true)

	// Modal Header
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), 44,
		sim.RGBA(30, 41, 59, 255), true)
	headerTitle := locale.T("=== 游戏设置与公交自动化运营配置 (Settings & Fleet) ===", "=== Game Settings & Fleet Automation (Settings) ===")
	textui.DrawText(screen, headerTitle, modalX+24, modalY+14, 14, color.White)

	// Option 1: Traffic Fine Toggle
	opt1Y := modalY + 52
	textui.DrawText(screen, locale.T("1. 红绿灯违章抓拍与罚款机制:", "1. Red Light & Violation Fines:"), modalX+26, opt1Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.fineToggleBounds = sim.Rect{X: modalX + 340, Y: opt1Y, W: 210, H: 28}

	fineBtnColor := sim.RGBA(34, 197, 94, 255) // Green (Enabled)
	fineBtnText := locale.T("【已开启 (扣$50)】", "【Enabled (-$50)】")
	if !s.TrafficFinesEnabled {
		fineBtnColor = sim.RGBA(239, 68, 68, 255) // Red (Disabled)
		fineBtnText = locale.T("【已关闭 (无罚款)】", "【Disabled (No Fine)】")
	}
	vector.DrawFilledRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), 1.5, fineBtnColor, true)
	textui.DrawText(screen, fineBtnText, s.fineToggleBounds.X+36, s.fineToggleBounds.Y+6, 12, fineBtnColor)

	// Option 2: AI Traffic Density
	opt2Y := modalY + 86
	textui.DrawText(screen, locale.T("2. AI 私家车交通流密度:", "2. AI Traffic Flow Density:"), modalX+26, opt2Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.densityToggleBounds = sim.Rect{X: modalX + 340, Y: opt2Y, W: 210, H: 28}

	labels := [sim.TrafficDensityLevels]string{
		locale.T("低", "Low"), locale.T("中", "Medium"),
		locale.T("高", "High"), locale.T("拥堵", "Congested"),
	}
	level := s.TrafficDensity
	if level < 1 || level > sim.TrafficDensityLevels {
		level = 2
	}
	densityText := fmt.Sprintf(locale.T("%s (%d辆)", "%s (%d vehicles)"), labels[level-1], sim.TrafficCarCount(level))

	vector.DrawFilledRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), 1.5, sim.RGBA(56, 189, 248, 255), true)
	textui.DrawText(screen, densityText, s.densityToggleBounds.X+32, s.densityToggleBounds.Y+6, 12, sim.RGBA(56, 189, 248, 255))

	// Option 3: Camera View Perspective
	opt3Y := modalY + 120
	textui.DrawText(screen, locale.T("3. 驾驶摄像机视角 (Camera):", "3. Driving Camera Perspective:"), modalX+26, opt3Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.cameraToggleBounds = sim.Rect{X: modalX + 340, Y: opt3Y, W: 210, H: 28}

	camViewText := locale.T("【固定正北 (标准)】", "【Fixed North-Up】")
	camViewColor := sim.RGBA(148, 163, 184, 255)
	switch s.CameraViewMode {
	case sim.CameraModeHeadingUp:
		camViewText = locale.T("【车头朝上 (跟随)】", "【Heading-Up (Chase)】")
		camViewColor = sim.RGBA(56, 189, 248, 255)
	case sim.CameraModeSouthUp:
		camViewText = locale.T("【固定正南 (倒置)】", "【Fixed South-Up】")
		camViewColor = sim.RGBA(251, 146, 60, 255)
	}
	vector.DrawFilledRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), 1.5, camViewColor, true)
	textui.DrawText(screen, camViewText, s.cameraToggleBounds.X+32, s.cameraToggleBounds.Y+6, 12, camViewColor)

	// Option 4: AI Autopilot for Player Bus
	opt4Y := modalY + 154
	textui.DrawText(screen, locale.T("4. 玩家公交 AI 自动驾驶 (代驾):", "4. Player Bus AI Autopilot:"), modalX+26, opt4Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.autopilotBounds = sim.Rect{X: modalX + 340, Y: opt4Y, W: 210, H: 28}

	apText := locale.T("【🤖 AI自动驾驶: 标准巡航 (P)】", "【🤖 Autopilot: Standard (P)】")
	apColor := sim.RGBA(6, 182, 212, 255)
	if !s.AutopilotEnabled {
		apText = locale.T("【👤 手动驾驶模式: 已关闭 (P)】", "【👤 Manual Mode: OFF (P)】")
		apColor = sim.RGBA(148, 163, 184, 255)
	} else if s.AutopilotBerserk {
		apText = locale.Tf("【⚡ AI狂暴极速: %d km/h (P)】", "【⚡ Autopilot: BSK %d (P)】", s.AutopilotBerserkMaxSpeed)
		apColor = sim.RGBA(249, 115, 22, 255)
	}
	vector.DrawFilledRect(screen, float32(s.autopilotBounds.X), float32(s.autopilotBounds.Y),
		float32(s.autopilotBounds.W), float32(s.autopilotBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.autopilotBounds.X), float32(s.autopilotBounds.Y),
		float32(s.autopilotBounds.W), float32(s.autopilotBounds.H), 1.5, apColor, true)
	textui.DrawText(screen, apText, s.autopilotBounds.X+12, s.autopilotBounds.Y+6, 12, apColor)

	// Option 4b: Player Bus Top Speed Limit (Manual Driving & Berserk Mode: 60, 80, 100, 120 km/h)
	opt4bY := modalY + 188
	speedLabelCol := sim.RGBA(226, 232, 240, 255)
	textui.DrawText(screen, locale.T("   └ 公交最高限速 (手动极速/狂暴模式):", "   └ Bus Top Speed (Manual & Berserk):"), modalX+26, opt4bY+7, 13, speedLabelCol)

	speeds := []int{60, 80, 100, 120}
	btnW := 48.0
	btnH := 28.0
	gap := 6.0
	for idx, spd := range speeds {
		bx := modalX + 340 + float64(idx)*(btnW+gap)
		bRect := sim.Rect{X: bx, Y: opt4bY, W: btnW, H: btnH}
		switch spd {
		case 60:
			s.berserkSpeed60Bounds = bRect
		case 80:
			s.berserkSpeed80Bounds = bRect
		case 100:
			s.berserkSpeed100Bounds = bRect
		case 120:
			s.berserkSpeed120Bounds = bRect
		}

		isSelected := (s.AutopilotBerserkMaxSpeed == spd)
		bgCol := sim.RGBA(30, 41, 59, 255)
		borderCol := sim.RGBA(71, 85, 105, 200)
		textCol := sim.RGBA(148, 163, 184, 255)
		if isSelected {
			bgCol = sim.RGBA(234, 88, 12, 230)
			borderCol = sim.RGBA(253, 186, 116, 255)
			textCol = sim.RGBA(255, 255, 255, 255)
		}
		vector.DrawFilledRect(screen, float32(bx), float32(opt4bY), float32(btnW), float32(btnH), bgCol, true)
		vector.StrokeRect(screen, float32(bx), float32(opt4bY), float32(btnW), float32(btnH), 1.5, borderCol, true)
		textui.DrawText(screen, fmt.Sprintf("%d", spd), bx+14, opt4bY+6, 12, textCol)
	}

	// Option 5: Configurable Route AI Buses Count
	opt5Y := modalY + 222
	textui.DrawText(screen, locale.T("5. 各线路 AI 公交配置:", "5. AI fleets by line:"), modalX+26, opt5Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.aiBusToggleBounds = sim.Rect{X: modalX + 340, Y: opt5Y, W: 210, H: 28}

	fleetBtnColor := sim.RGBA(16, 185, 129, 255)
	fleetBtnText := locale.Tf("%d 辆 · 线路管理 [T]", "%d buses · Lines [T]", s.AIBusCount)
	vector.DrawFilledRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), 1.5, fleetBtnColor, true)
	textui.DrawText(screen, fleetBtnText, s.aiBusToggleBounds.X+36, s.aiBusToggleBounds.Y+6, 12, fleetBtnColor)

	// Option 6: Map Theme & City Selection
	opt6Y := modalY + 256
	textui.DrawText(screen, locale.T("6. 城市地图与风貌 (Map Theme):", "6. City Map & Theme:"), modalX+26, opt6Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.themeToggleBounds = sim.Rect{X: modalX + 340, Y: opt6Y, W: 210, H: 28}

	themeBtnColor := sim.RGBA(168, 85, 247, 255)
	if s.MapTheme == sim.MapThemeCoastalBay {
		themeBtnColor = sim.RGBA(6, 182, 212, 255)
	} else if s.MapTheme == sim.MapThemeRiverValley {
		themeBtnColor = sim.RGBA(16, 185, 129, 255)
	} else if s.MapTheme == sim.MapThemeEmeraldLake {
		themeBtnColor = sim.RGBA(34, 197, 94, 255)
	} else if s.MapTheme == sim.MapThemeTwinCities {
		themeBtnColor = sim.RGBA(217, 70, 239, 255)
	} else if s.MapTheme == sim.MapThemeCustom {
		themeBtnColor = sim.RGBA(245, 158, 11, 255)
	}
	vector.DrawFilledRect(screen, float32(s.themeToggleBounds.X), float32(s.themeToggleBounds.Y),
		float32(s.themeToggleBounds.W), float32(s.themeToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.themeToggleBounds.X), float32(s.themeToggleBounds.Y),
		float32(s.themeToggleBounds.W), float32(s.themeToggleBounds.H), 1.5, themeBtnColor, true)
	textui.DrawText(screen, locale.T("🏠 退入主菜单更换", "🏠 Return to Menu to Change"), s.themeToggleBounds.X+24, s.themeToggleBounds.Y+6, 12, themeBtnColor)

	// Option 7: Interface Language (Chinese / English)
	opt7Y := modalY + 290
	textui.DrawText(screen, locale.T("7. 界面语言 / Language:", "7. Interface Language:"), modalX+26, opt7Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.langToggleBounds = sim.Rect{X: modalX + 340, Y: opt7Y, W: 210, H: 28}

	langBtnColor := sim.RGBA(56, 189, 248, 255)
	langBtnText := "【🌐 简体中文 (Chinese)】"
	if s.Language == locale.LanguageEN {
		langBtnText = "【🌐 English (英文)】"
	}
	vector.DrawFilledRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), 1.5, langBtnColor, true)
	textui.DrawText(screen, langBtnText, s.langToggleBounds.X+20, s.langToggleBounds.Y+6, 12, langBtnColor)

	s.drawAISpeedRanges(screen, modalX, modalY+332)
	textui.DrawText(screen, locale.T("每辆车按 10 km/h 一档随机分配，生成后固定。", "Limits are sampled in 10 km/h steps and fixed at spawn."), modalX+26, modalY+414, 12, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("最低档保持跟车；其余车辆可安全超车。", "The lowest tier follows; faster vehicles may pass safely."), modalX+26, modalY+436, 12, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("调整范围对新生成的车辆生效。", "Range changes apply to newly spawned vehicles."), modalX+26, modalY+458, 12, sim.RGBA(148, 163, 184, 255))

	textui.DrawText(screen, locale.T("AI 公交数量与限速统一在 [T] 线路管理中设定。", "Set AI bus counts and limits in [T] Transit Lines."), modalX+26, modalY+392, 12, sim.RGBA(148, 163, 184, 255))
	s.saveBounds = sim.Rect{X: modalX + 26, Y: modalY + 478, W: 256, H: 30}
	s.loadBounds = sim.Rect{X: modalX + 298, Y: modalY + 478, W: 256, H: 30}
	operationsButton(screen, s.saveBounds, locale.T("保存 / 覆盖存档 [F5]", "Save / Overwrite [F5]"), false)
	operationsButton(screen, s.loadBounds, locale.T("读取存档 [F9]", "Load save [F9]"), false)
	s.operationsBounds = sim.Rect{X: modalX + 26, Y: modalY + 516, W: 528, H: 32}
	operationsButton(screen, s.operationsBounds, locale.T("票价设置与运营统计 [O]", "Fares & operating statistics [O]"), true)

	// Bottom Buttons: Return to Menu & Resume
	s.menuButtonBounds = sim.Rect{X: modalX + 26, Y: modalY + modalH - 46, W: 250, H: 34}
	vector.DrawFilledRect(screen, float32(s.menuButtonBounds.X), float32(s.menuButtonBounds.Y),
		float32(s.menuButtonBounds.W), float32(s.menuButtonBounds.H), sim.RGBA(99, 102, 241, 240), true)
	textui.DrawText(screen, locale.T("🏠 返回主菜单 (选择地图)", "🏠 Return to Main Menu (Maps)"), s.menuButtonBounds.X+24, s.menuButtonBounds.Y+9, 13, color.White)

	s.closeButtonBounds = sim.Rect{X: modalX + modalW - 230, Y: modalY + modalH - 46, W: 204, H: 34}
	vector.DrawFilledRect(screen, float32(s.closeButtonBounds.X), float32(s.closeButtonBounds.Y),
		float32(s.closeButtonBounds.W), float32(s.closeButtonBounds.H), sim.RGBA(59, 130, 246, 255), true)
	textui.DrawText(screen, locale.T("继续驾驶 [ESC]", "Resume Driving [ESC]"), s.closeButtonBounds.X+36, s.closeButtonBounds.Y+9, 13, color.White)
}
