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

	if s.PassengerTimingOpen {
		s.drawPassengerTiming(screen)
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
	modalH := 680.0
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
	opt1Y := modalY + 48
	textui.DrawText(screen, locale.T("1. 红绿灯违章抓拍与罚款机制:", "1. Red Light & Violation Fines:"), modalX+26, opt1Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.fineToggleBounds = sim.Rect{X: modalX + 340, Y: opt1Y, W: 210, H: 26}

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
	textui.DrawText(screen, fineBtnText, s.fineToggleBounds.X+36, s.fineToggleBounds.Y+5, 12, fineBtnColor)

	// Option 2: AI Traffic Density
	opt2Y := modalY + 79
	textui.DrawText(screen, locale.T("2. AI 私家车交通流密度:", "2. AI Traffic Flow Density:"), modalX+26, opt2Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.densityToggleBounds = sim.Rect{X: modalX + 340, Y: opt2Y, W: 210, H: 26}

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
	textui.DrawText(screen, densityText, s.densityToggleBounds.X+32, s.densityToggleBounds.Y+5, 12, sim.RGBA(56, 189, 248, 255))

	// Option 3: Camera View Perspective
	opt3Y := modalY + 110
	textui.DrawText(screen, locale.T("3. 驾驶摄像机视角 (Camera):", "3. Driving Camera Perspective:"), modalX+26, opt3Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.cameraToggleBounds = sim.Rect{X: modalX + 340, Y: opt3Y, W: 210, H: 26}

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
	textui.DrawText(screen, camViewText, s.cameraToggleBounds.X+32, s.cameraToggleBounds.Y+5, 12, camViewColor)

	// Option 4: AI Autopilot for Player Bus
	opt4Y := modalY + 141
	textui.DrawText(screen, locale.T("4. 玩家公交 AI 自动驾驶 (代驾):", "4. Player Bus AI Autopilot:"), modalX+26, opt4Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.autopilotBounds = sim.Rect{X: modalX + 340, Y: opt4Y, W: 210, H: 26}

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
	textui.DrawText(screen, apText, s.autopilotBounds.X+12, s.autopilotBounds.Y+5, 12, apColor)

	// Option 4b: Player Bus Top Speed Limit (Manual Driving & Berserk Mode: 60, 80, 100, 120 km/h)
	opt4bY := modalY + 172
	speedLabelCol := sim.RGBA(226, 232, 240, 255)
	textui.DrawText(screen, locale.T("   └ 公交最高限速 (手动极速/狂暴模式):", "   └ Bus Top Speed (Manual & Berserk):"), modalX+26, opt4bY+6, 13, speedLabelCol)

	speeds := []int{60, 80, 100, 120}
	btnW := 48.0
	btnH := 26.0
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
		textui.DrawText(screen, fmt.Sprintf("%d", spd), bx+14, opt4bY+5, 12, textCol)
	}

	// Option 5: Configurable Route AI Buses Count
	opt5Y := modalY + 203
	textui.DrawText(screen, locale.T("5. 各线路 AI 公交配置:", "5. AI fleets by line:"), modalX+26, opt5Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.aiBusToggleBounds = sim.Rect{X: modalX + 340, Y: opt5Y, W: 210, H: 26}

	fleetBtnColor := sim.RGBA(16, 185, 129, 255)
	fleetBtnText := locale.Tf("%d 辆 · 线路管理 [T]", "%d buses · Lines [T]", s.AIBusCount)
	vector.DrawFilledRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), 1.5, fleetBtnColor, true)
	textui.DrawText(screen, fleetBtnText, s.aiBusToggleBounds.X+36, s.aiBusToggleBounds.Y+5, 12, fleetBtnColor)

	// Option 6: Interface Language (Chinese / English)
	opt6Y := modalY + 234
	textui.DrawText(screen, locale.T("6. 界面语言 / Language:", "6. Interface Language:"), modalX+26, opt6Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.langToggleBounds = sim.Rect{X: modalX + 340, Y: opt6Y, W: 210, H: 26}

	langBtnColor := sim.RGBA(56, 189, 248, 255)
	langBtnText := "【🌐 简体中文 (Chinese)】"
	if s.Language == locale.LanguageEN {
		langBtnText = "【🌐 English (英文)】"
	}
	vector.DrawFilledRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), 1.5, langBtnColor, true)
	textui.DrawText(screen, langBtnText, s.langToggleBounds.X+20, s.langToggleBounds.Y+5, 12, langBtnColor)

	// Option 7: Bus announcements mode
	opt7Y := modalY + 265
	textui.DrawText(screen, locale.T("7. 公交语音报站:", "7. Bus announcements:"), modalX+26, opt7Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.announcementsBounds = sim.Rect{X: modalX + 340, Y: opt7Y, W: 210, H: 26}
	operationsButton(screen, s.announcementsBounds, s.Announcements.Label(), false)

	// Option 8: Bus announcement voice
	opt8Y := modalY + 296
	textui.DrawText(screen, locale.T("8. 报站声音:", "8. Announcement voice:"), modalX+26, opt8Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.announcementVoiceBounds = sim.Rect{X: modalX + 340, Y: opt8Y, W: 210, H: 26}
	operationsButton(screen, s.announcementVoiceBounds, s.AnnouncementVoiceLabel(), false)

	// Option 9: Bus Passenger Spawn Rate
	opt9Y := modalY + 327
	textui.DrawText(screen, locale.T("9. 公交车站乘客生成速度:", "9. Passenger Spawn Rate:"), modalX+26, opt9Y+6, 13, sim.RGBA(226, 232, 240, 255))
	s.passengerSpawnRateBounds = sim.Rect{X: modalX + 340, Y: opt9Y, W: 210, H: 26}

	rateBtnColor := sim.RGBA(56, 189, 248, 255) // Sky Blue (Moderate)
	if s.PassengerSpawnRate == sim.PassengerSpawnFew {
		rateBtnColor = sim.RGBA(148, 163, 184, 255) // Slate (Few)
	} else if s.PassengerSpawnRate == sim.PassengerSpawnMany {
		rateBtnColor = sim.RGBA(245, 158, 11, 255) // Amber (Many)
	}
	vector.DrawFilledRect(screen, float32(s.passengerSpawnRateBounds.X), float32(s.passengerSpawnRateBounds.Y),
		float32(s.passengerSpawnRateBounds.W), float32(s.passengerSpawnRateBounds.H), sim.RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.passengerSpawnRateBounds.X), float32(s.passengerSpawnRateBounds.Y),
		float32(s.passengerSpawnRateBounds.W), float32(s.passengerSpawnRateBounds.H), 1.5, rateBtnColor, true)
	textui.DrawText(screen, s.PassengerSpawnRate.Label(), s.passengerSpawnRateBounds.X+38, s.passengerSpawnRateBounds.Y+5, 12, rateBtnColor)

	s.drawAISpeedRanges(screen, modalX, modalY+358)
	volumeLabels := []string{locale.T("引擎 / 路噪音量", "Engine / road volume"), locale.T("报站音量", "Announcement volume"), locale.T("喇叭音量", "Horn volume")}
	volumes := s.Volumes()
	values := []int{volumes.Engine, volumes.Announcements, volumes.Horn}
	for i, label := range volumeLabels {
		y := modalY + 398 + float64(i)*28
		textui.DrawText(screen, label, modalX+26, y+4, 13, sim.RGBA(226, 232, 240, 255))
		for d, caption := range []string{"-", "+"} {
			x := modalX + 340 + float64(d)*174
			s.volumeBounds[i][d] = sim.Rect{X: x, Y: y, W: 36, H: 24}
			operationsButton(screen, s.volumeBounds[i][d], caption, false)
		}
		textui.DrawText(screen, fmt.Sprintf("%d%%", values[i]), modalX+418, y+4, 13, sim.RGBA(56, 189, 248, 255))
	}

	s.saveBounds = sim.Rect{X: modalX + 26, Y: modalY + 490, W: 256, H: 28}
	s.loadBounds = sim.Rect{X: modalX + 298, Y: modalY + 490, W: 256, H: 28}
	operationsButton(screen, s.saveBounds, locale.T("选择存档并保存 [F5]", "Save to slot [F5]"), false)
	operationsButton(screen, s.loadBounds, locale.T("选择存档并读取 [F9]", "Load from slot [F9]"), false)
	s.operationsBounds = sim.Rect{X: modalX + 26, Y: modalY + 524, W: 528, H: 30}
	operationsButton(screen, s.operationsBounds, locale.T("票价设置与运营统计 [O]", "Fares & operating statistics [O]"), true)

	s.passengerTimingBounds = sim.Rect{X: modalX + 26, Y: modalY + 560, W: 256, H: 26}
	operationsButton(screen, s.passengerTimingBounds, locale.T("各车型上下客速度…", "Boarding / alighting times…"), false)
	s.busSignalPriorityBounds = sim.Rect{X: modalX + 298, Y: modalY + 560, W: 256, H: 26}
	priorityLabel := locale.T("公交信号优先：关闭", "Bus signal priority: OFF")
	if s.BusSignalPriority {
		priorityLabel = locale.T("公交信号优先：开启", "Bus signal priority: ON")
	}
	operationsButton(screen, s.busSignalPriorityBounds, priorityLabel, s.BusSignalPriority)

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
