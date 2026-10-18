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

	densityText := locale.T("中等密度 (24辆)", "Medium (24 vehicles)")
	switch s.TrafficDensity {
	case 1:
		densityText = locale.T("低密度 (12辆)", "Low (12 vehicles)")
	case 2:
		densityText = locale.T("中等密度 (24辆)", "Medium (24 vehicles)")
	case 3:
		densityText = locale.T("高密度 (36辆)", "High (36 vehicles)")
	case 4:
		densityText = locale.T("超高密度 (72辆)", "Very High (72 vehicles)")
	case 5:
		densityText = locale.T("极限车流 (128辆)", "Extreme Jam (128 vehicles)")
	}
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
	textui.DrawText(screen, locale.T("5. 路线 AI 公交车数量配置:", "5. Route AI Bus Fleet Size:"), modalX+26, opt5Y+7, 13, sim.RGBA(226, 232, 240, 255))
	s.aiBusToggleBounds = sim.Rect{X: modalX + 340, Y: opt5Y, W: 210, H: 28}

	fleetBtnColor := sim.RGBA(16, 185, 129, 255)
	fleetBtnText := locale.Tf("【AI公交车队: %d 辆】", "【AI Fleet: %d Buses】", s.AIBusCount)
	if s.AIBusCount == 0 {
		fleetBtnColor = sim.RGBA(148, 163, 184, 255)
		fleetBtnText = locale.T("【无AI公交 (0 辆)】", "【No AI Buses (0)】")
	}
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

	// Rules & Customization Explanation Info Box
	infoY := modalY + 328
	infoH := 240.0
	vector.DrawFilledRect(screen, float32(modalX+26), float32(infoY), float32(modalW-52), float32(infoH),
		sim.RGBA(15, 23, 42, 200), true)
	vector.StrokeRect(screen, float32(modalX+26), float32(infoY), float32(modalW-52), float32(infoH),
		1, sim.RGBA(71, 85, 105, 255), true)

	if s.Language == locale.LanguageEN {
		textui.DrawText(screen, "• Traffic Network, Autopilot & Operating Rules:", modalX+36, infoY+10, 12, sim.RGBA(148, 163, 184, 255))
		textui.DrawText(screen, fmt.Sprintf("  - Active Map: [%s] (6 distinct city themes, return to menu to switch);", s.MapTheme.DisplayName()), modalX+36, infoY+28, 12, sim.RGBA(192, 132, 252, 255))
		textui.DrawText(screen, "  - Top Speed (60/80/100/120 km/h): Unified limit for manual driving and Berserk mode; Shift+B to cycle;", modalX+36, infoY+46, 11, sim.RGBA(6, 182, 212, 255))
		textui.DrawText(screen, "  - Route AI Fleet: 0-8 autonomous buses operating routes and serving passengers;", modalX+36, infoY+64, 12, sim.RGBA(52, 211, 153, 255))
		textui.DrawText(screen, "  - Camera Perspectives: Heading-Up Follow / Fixed North-Up / Fixed South-Up ([V]);", modalX+36, infoY+82, 12, sim.RGBA(251, 146, 60, 255))
		textui.DrawText(screen, "  - Road Hierarchy: 4-lane avenues, 2-lane streets, grade-separated overpasses;", modalX+36, infoY+100, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - Standard 52px wide city roadway lanes provide comfortable clearance and turning;", modalX+36, infoY+118, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - Wide intersection geometries allow realistic turning without barrier clipping;", modalX+36, infoY+136, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - Press [L] anytime to quickly toggle interface language (English / Chinese);", modalX+36, infoY+154, 12, sim.RGBA(56, 189, 248, 255))

		descFine := fmt.Sprintf("  - Fines Status: Violations deduct $%d (red light crossing / wrong-way).", s.FineAmount)
		descFineColor := sim.RGBA(248, 113, 113, 255)
		if !s.TrafficFinesEnabled {
			descFine = "  - Fines Status: Violations waived (warning alert only)."
			descFineColor = sim.RGBA(74, 222, 128, 255)
		}
		textui.DrawText(screen, descFine, modalX+36, infoY+176, 12, descFineColor)
	} else {
		textui.DrawText(screen, "• 交通网络调度、自动驾驶与车辆规则说明:", modalX+36, infoY+10, 12, sim.RGBA(148, 163, 184, 255))
		textui.DrawText(screen, fmt.Sprintf("  - 当前地图: 【%s】 (6大独立城市风貌，含自定义地图，需退入主菜单切换)；", s.MapTheme.DisplayName()), modalX+36, infoY+28, 12, sim.RGBA(192, 132, 252, 255))
		textui.DrawText(screen, "  - 公交最高限速 (60/80/100/120 km/h): 手动极速与AI狂暴巡航均生效，支持按 Shift+B 快速切速；", modalX+36, infoY+46, 11, sim.RGBA(6, 182, 212, 255))
		textui.DrawText(screen, "  - 路线 AI 公交: 调度 0~8 辆独立 AI 公交车，在城市环线自主运营、上下客流；", modalX+36, infoY+64, 12, sim.RGBA(52, 211, 153, 255))
		textui.DrawText(screen, "  - 视角切换: 支持【车头朝上跟随】/【固定正北标准】/【固定正南倒置】，按 [V] 键切换；", modalX+36, infoY+82, 12, sim.RGBA(251, 146, 60, 255))
		textui.DrawText(screen, "  - 城市支持4车道快速路、2车道城市路、立体跨线桥及大江大湖水道；", modalX+36, infoY+100, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - 默认采用 52px 宽敞市区物理车道，提供充裕的行驶与平稳转弯空间；", modalX+36, infoY+118, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - 全城道路与路口几何尺寸全面加宽，显著降低公交转弯难度；", modalX+36, infoY+136, 12, sim.RGBA(203, 213, 225, 255))
		textui.DrawText(screen, "  - 随时按键盘 [L] 键可快速在中文与英文界面之间一键切换；", modalX+36, infoY+154, 12, sim.RGBA(56, 189, 248, 255))

		descFine := fmt.Sprintf("  - 罚款状态: 违章抓拍扣除 $%d (红灯压线/对侧车道判断)。", s.FineAmount)
		descFineColor := sim.RGBA(248, 113, 113, 255)
		if !s.TrafficFinesEnabled {
			descFine = "  - 罚款状态: 违章已豁免扣款 (仅弹出警示提示)。"
			descFineColor = sim.RGBA(74, 222, 128, 255)
		}
		textui.DrawText(screen, descFine, modalX+36, infoY+176, 12, descFineColor)
	}

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
