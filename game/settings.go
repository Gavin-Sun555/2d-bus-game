package game

import (
	"fmt"
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Settings manages user game configuration
type Settings struct {
	IsOpen                bool
	TrafficFinesEnabled   bool
	FineAmount            int
	TrafficDensity        int        // 1: Low (12), 2: Med (24), 3: High (36), 4: Very High (72), 5: Extreme Jam (128)
	LaneWidthLevel        int        // 0: 42px (标准紧凑), 1: 52px (宽敞舒适), 2: 64px (开阔大道), 3: 78px (超宽城际)
	CameraViewMode        CameraMode // 0: CameraModeNorthUp, 1: CameraModeHeadingUp
	MapTheme              MapTheme   // Active city map
	ThemeChanged          bool
	ReturnToMenuRequested bool

	// AI Autopilot & Route AI Buses Configurations
	AutopilotEnabled         bool
	AutopilotBerserk         bool // Berserk mode: straightaways top speed boost while obeying traffic rules
	AutopilotBerserkMaxSpeed int  // Berserk top speed in km/h: 60, 80, 100, 120 (default: 80)
	AIBusCount               int  // Number of AI buses on the route (0 ~ 8)
	AIBusFleetChanged        bool

	// UI Localization Language
	Language Language

	// Button bounds for click detection
	fineToggleBounds      Rect
	densityToggleBounds   Rect
	cameraToggleBounds    Rect
	autopilotBounds       Rect
	berserkSpeed60Bounds  Rect
	berserkSpeed80Bounds  Rect
	berserkSpeed100Bounds Rect
	berserkSpeed120Bounds Rect
	aiBusToggleBounds     Rect
	themeToggleBounds     Rect
	langToggleBounds      Rect
	menuButtonBounds      Rect
	closeButtonBounds     Rect
}

type Rect struct {
	X float64
	Y float64
	W float64
	H float64
}

func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.X && p.X <= r.X+r.W && p.Y >= r.Y && p.Y <= r.Y+r.H
}

func NewSettings() *Settings {
	return &Settings{
		IsOpen:              false,
		TrafficFinesEnabled: true,
		FineAmount:          50,
		TrafficDensity:      2,                  // Default: Medium
		LaneWidthLevel:      1,                  // Default: 52px (宽敞市区 Comfortable Wide)
		CameraViewMode:      CameraModeNorthUp,  // Default: North-Up standard
		MapTheme:            MapThemeMetropolis, // Default: Modern Metropolis
		AutopilotEnabled:         false,              // Manual driving by default
		AutopilotBerserk:         false,              // Standard mode by default
		AutopilotBerserkMaxSpeed: 80,                 // Default: 80 km/h (445 px/s)
		AIBusCount:               0,                  // 0 AI buses by default
		Language:                 CurrentLanguage,    // Default to CurrentLanguage
	}
}

func (s *Settings) GetBerserkCruiseSpeed() float64 {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		return 333.0
	case 80:
		return 445.0
	case 100:
		return 555.0
	case 120:
		return 667.0
	default:
		return 445.0
	}
}

// GetBusMaxSpeed returns the vehicle physics max forward speed in px/s based on the configured speed limit.
// In manual driving mode, the bus accelerates up to this limit (reaching 60, 80, 100, or 120 km/h).
func (s *Settings) GetBusMaxSpeed() float64 {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		return 334.0 // 334 * 0.18 = 60.12 km/h
	case 80:
		return 445.0 // 445 * 0.18 = 80.1 km/h
	case 100:
		return 556.0 // 556 * 0.18 = 100.08 km/h
	case 120:
		return 667.0 // 667 * 0.18 = 120.06 km/h
	default:
		return 445.0
	}
}

func (s *Settings) SetBerserkMaxSpeed(speed int) {
	if speed == 60 || speed == 80 || speed == 100 || speed == 120 {
		s.AutopilotBerserkMaxSpeed = speed
	}
}

func (s *Settings) SetMaxSpeed(speed int) {
	s.SetBerserkMaxSpeed(speed)
}

func (s *Settings) CycleBerserkMaxSpeed() int {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		s.AutopilotBerserkMaxSpeed = 80
	case 80:
		s.AutopilotBerserkMaxSpeed = 100
	case 100:
		s.AutopilotBerserkMaxSpeed = 120
	case 120:
		s.AutopilotBerserkMaxSpeed = 60
	default:
		s.AutopilotBerserkMaxSpeed = 80
	}
	return s.AutopilotBerserkMaxSpeed
}

func (s *Settings) CycleMaxSpeed() int {
	return s.CycleBerserkMaxSpeed()
}

func (s *Settings) GetLaneWidth() float64 {
	return 52.0 // 默认固定为宽敞市区 (52px)
}

func (s *Settings) Toggle() {
	s.IsOpen = !s.IsOpen
}

func (s *Settings) Update(cam ...*Camera) {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		s.Toggle()
		return
	}

	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		s.Language = ToggleLanguage()
	}

	if !s.IsOpen {
		return
	}

	var activeCam *Camera
	if len(cam) > 0 && cam[0] != nil {
		activeCam = cam[0]
		s.CameraViewMode = activeCam.Mode
	}

	// Handle mouse clicks on buttons
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mPos := Vec2{X: float64(mx), Y: float64(my)}

		if s.menuButtonBounds.Contains(mPos) {
			s.ReturnToMenuRequested = true
			s.IsOpen = false
		} else if s.fineToggleBounds.Contains(mPos) {
			s.TrafficFinesEnabled = !s.TrafficFinesEnabled
		} else if s.densityToggleBounds.Contains(mPos) {
			s.TrafficDensity = (s.TrafficDensity % 5) + 1
		} else if s.cameraToggleBounds.Contains(mPos) {
			switch s.CameraViewMode {
			case CameraModeNorthUp:
				s.CameraViewMode = CameraModeHeadingUp
			case CameraModeHeadingUp:
				s.CameraViewMode = CameraModeSouthUp
			case CameraModeSouthUp:
				s.CameraViewMode = CameraModeNorthUp
			default:
				s.CameraViewMode = CameraModeNorthUp
			}
			if activeCam != nil {
				activeCam.SetMode(s.CameraViewMode)
			}
		} else if s.autopilotBounds.Contains(mPos) {
			if !s.AutopilotEnabled {
				s.AutopilotEnabled = true
				s.AutopilotBerserk = false
			} else if !s.AutopilotBerserk {
				s.AutopilotBerserk = true
			} else {
				s.AutopilotEnabled = false
				s.AutopilotBerserk = false
			}
		} else if s.berserkSpeed60Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 60
		} else if s.berserkSpeed80Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 80
		} else if s.berserkSpeed100Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 100
		} else if s.berserkSpeed120Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 120
		} else if s.aiBusToggleBounds.Contains(mPos) {
			s.AIBusCount = (s.AIBusCount + 1) % 9
			s.AIBusFleetChanged = true
		} else if s.themeToggleBounds.Contains(mPos) {
			s.ReturnToMenuRequested = true
			s.IsOpen = false
		} else if s.langToggleBounds.Contains(mPos) {
			s.Language = ToggleLanguage()
		} else if s.closeButtonBounds.Contains(mPos) {
			s.IsOpen = false
		}
	}
}

func (s *Settings) Draw(screen *ebiten.Image, cam ...*Camera) {
	if !s.IsOpen {
		return
	}

	if len(cam) > 0 && cam[0] != nil {
		s.CameraViewMode = cam[0].Mode
	}

	sw := float64(screen.Bounds().Dx())
	sh := float64(screen.Bounds().Dy())

	// Dark semi-transparent backdrop
	vector.DrawFilledRect(screen, 0, 0, float32(sw), float32(sh), RGBA(0, 0, 0, 160), false)

	// Modal card
	modalW := 580.0
	modalH := 624.0
	modalX := (sw - modalW) / 2
	modalY := (sh - modalH) / 2

	// Modal shadow and background
	vector.DrawFilledRect(screen, float32(modalX+6), float32(modalY+6), float32(modalW), float32(modalH),
		RGBA(2, 6, 23, 140), true)
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		RGBA(15, 23, 42, 245), true)
	vector.StrokeRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		2, RGBA(59, 130, 246, 255), true)

	// Modal Header
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), 44,
		RGBA(30, 41, 59, 255), true)
	headerTitle := T("=== 游戏设置与公交自动化运营配置 (Settings & Fleet) ===", "=== Game Settings & Fleet Automation (Settings) ===")
	DrawText(screen, headerTitle, modalX+24, modalY+14, 14, color.White)

	// Option 1: Traffic Fine Toggle
	opt1Y := modalY + 52
	DrawText(screen, T("1. 红绿灯违章抓拍与罚款机制:", "1. Red Light & Violation Fines:"), modalX+26, opt1Y+7, 13, RGBA(226, 232, 240, 255))
	s.fineToggleBounds = Rect{X: modalX + 340, Y: opt1Y, W: 210, H: 28}

	fineBtnColor := RGBA(34, 197, 94, 255) // Green (Enabled)
	fineBtnText := T("【已开启 (扣$50)】", "【Enabled (-$50)】")
	if !s.TrafficFinesEnabled {
		fineBtnColor = RGBA(239, 68, 68, 255) // Red (Disabled)
		fineBtnText = T("【已关闭 (无罚款)】", "【Disabled (No Fine)】")
	}
	vector.DrawFilledRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), 1.5, fineBtnColor, true)
	DrawText(screen, fineBtnText, s.fineToggleBounds.X+36, s.fineToggleBounds.Y+6, 12, fineBtnColor)

	// Option 2: AI Traffic Density
	opt2Y := modalY + 86
	DrawText(screen, T("2. AI 私家车交通流密度:", "2. AI Traffic Flow Density:"), modalX+26, opt2Y+7, 13, RGBA(226, 232, 240, 255))
	s.densityToggleBounds = Rect{X: modalX + 340, Y: opt2Y, W: 210, H: 28}

	densityText := T("中等密度 (24辆)", "Medium (24 vehicles)")
	switch s.TrafficDensity {
	case 1:
		densityText = T("低密度 (12辆)", "Low (12 vehicles)")
	case 2:
		densityText = T("中等密度 (24辆)", "Medium (24 vehicles)")
	case 3:
		densityText = T("高密度 (36辆)", "High (36 vehicles)")
	case 4:
		densityText = T("超高密度 (72辆)", "Very High (72 vehicles)")
	case 5:
		densityText = T("极限车流 (128辆)", "Extreme Jam (128 vehicles)")
	}
	vector.DrawFilledRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), 1.5, RGBA(56, 189, 248, 255), true)
	DrawText(screen, densityText, s.densityToggleBounds.X+32, s.densityToggleBounds.Y+6, 12, RGBA(56, 189, 248, 255))

	// Option 3: Camera View Perspective
	opt3Y := modalY + 120
	DrawText(screen, T("3. 驾驶摄像机视角 (Camera):", "3. Driving Camera Perspective:"), modalX+26, opt3Y+7, 13, RGBA(226, 232, 240, 255))
	s.cameraToggleBounds = Rect{X: modalX + 340, Y: opt3Y, W: 210, H: 28}

	camViewText := T("【固定正北 (标准)】", "【Fixed North-Up】")
	camViewColor := RGBA(148, 163, 184, 255)
	switch s.CameraViewMode {
	case CameraModeHeadingUp:
		camViewText = T("【车头朝上 (跟随)】", "【Heading-Up (Chase)】")
		camViewColor = RGBA(56, 189, 248, 255)
	case CameraModeSouthUp:
		camViewText = T("【固定正南 (倒置)】", "【Fixed South-Up】")
		camViewColor = RGBA(251, 146, 60, 255)
	}
	vector.DrawFilledRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), 1.5, camViewColor, true)
	DrawText(screen, camViewText, s.cameraToggleBounds.X+32, s.cameraToggleBounds.Y+6, 12, camViewColor)

	// Option 4: AI Autopilot for Player Bus
	opt4Y := modalY + 154
	DrawText(screen, T("4. 玩家公交 AI 自动驾驶 (代驾):", "4. Player Bus AI Autopilot:"), modalX+26, opt4Y+7, 13, RGBA(226, 232, 240, 255))
	s.autopilotBounds = Rect{X: modalX + 340, Y: opt4Y, W: 210, H: 28}

	apText := T("【🤖 AI自动驾驶: 标准巡航 (P)】", "【🤖 Autopilot: Standard (P)】")
	apColor := RGBA(6, 182, 212, 255)
	if !s.AutopilotEnabled {
		apText = T("【👤 手动驾驶模式: 已关闭 (P)】", "【👤 Manual Mode: OFF (P)】")
		apColor = RGBA(148, 163, 184, 255)
	} else if s.AutopilotBerserk {
		apText = Tf("【⚡ AI狂暴极速: %d km/h (P)】", "【⚡ Autopilot: BSK %d (P)】", s.AutopilotBerserkMaxSpeed)
		apColor = RGBA(249, 115, 22, 255)
	}
	vector.DrawFilledRect(screen, float32(s.autopilotBounds.X), float32(s.autopilotBounds.Y),
		float32(s.autopilotBounds.W), float32(s.autopilotBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.autopilotBounds.X), float32(s.autopilotBounds.Y),
		float32(s.autopilotBounds.W), float32(s.autopilotBounds.H), 1.5, apColor, true)
	DrawText(screen, apText, s.autopilotBounds.X+12, s.autopilotBounds.Y+6, 12, apColor)

	// Option 4b: Player Bus Top Speed Limit (Manual Driving & Berserk Mode: 60, 80, 100, 120 km/h)
	opt4bY := modalY + 188
	speedLabelCol := RGBA(226, 232, 240, 255)
	DrawText(screen, T("   └ 公交最高限速 (手动极速/狂暴模式):", "   └ Bus Top Speed (Manual & Berserk):"), modalX+26, opt4bY+7, 13, speedLabelCol)

	speeds := []int{60, 80, 100, 120}
	btnW := 48.0
	btnH := 28.0
	gap := 6.0
	for idx, spd := range speeds {
		bx := modalX + 340 + float64(idx)*(btnW+gap)
		bRect := Rect{X: bx, Y: opt4bY, W: btnW, H: btnH}
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
		bgCol := RGBA(30, 41, 59, 255)
		borderCol := RGBA(71, 85, 105, 200)
		textCol := RGBA(148, 163, 184, 255)
		if isSelected {
			bgCol = RGBA(234, 88, 12, 230)
			borderCol = RGBA(253, 186, 116, 255)
			textCol = RGBA(255, 255, 255, 255)
		}
		vector.DrawFilledRect(screen, float32(bx), float32(opt4bY), float32(btnW), float32(btnH), bgCol, true)
		vector.StrokeRect(screen, float32(bx), float32(opt4bY), float32(btnW), float32(btnH), 1.5, borderCol, true)
		DrawText(screen, fmt.Sprintf("%d", spd), bx+14, opt4bY+6, 12, textCol)
	}

	// Option 5: Configurable Route AI Buses Count
	opt5Y := modalY + 222
	DrawText(screen, T("5. 路线 AI 公交车数量配置:", "5. Route AI Bus Fleet Size:"), modalX+26, opt5Y+7, 13, RGBA(226, 232, 240, 255))
	s.aiBusToggleBounds = Rect{X: modalX + 340, Y: opt5Y, W: 210, H: 28}

	fleetBtnColor := RGBA(16, 185, 129, 255)
	fleetBtnText := Tf("【AI公交车队: %d 辆】", "【AI Fleet: %d Buses】", s.AIBusCount)
	if s.AIBusCount == 0 {
		fleetBtnColor = RGBA(148, 163, 184, 255)
		fleetBtnText = T("【无AI公交 (0 辆)】", "【No AI Buses (0)】")
	}
	vector.DrawFilledRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.aiBusToggleBounds.X), float32(s.aiBusToggleBounds.Y),
		float32(s.aiBusToggleBounds.W), float32(s.aiBusToggleBounds.H), 1.5, fleetBtnColor, true)
	DrawText(screen, fleetBtnText, s.aiBusToggleBounds.X+36, s.aiBusToggleBounds.Y+6, 12, fleetBtnColor)

	// Option 6: Map Theme & City Selection
	opt6Y := modalY + 256
	DrawText(screen, T("6. 城市地图与风貌 (Map Theme):", "6. City Map & Theme:"), modalX+26, opt6Y+7, 13, RGBA(226, 232, 240, 255))
	s.themeToggleBounds = Rect{X: modalX + 340, Y: opt6Y, W: 210, H: 28}

	themeBtnColor := RGBA(168, 85, 247, 255)
	if s.MapTheme == MapThemeCoastalBay {
		themeBtnColor = RGBA(6, 182, 212, 255)
	} else if s.MapTheme == MapThemeRiverValley {
		themeBtnColor = RGBA(16, 185, 129, 255)
	} else if s.MapTheme == MapThemeEmeraldLake {
		themeBtnColor = RGBA(34, 197, 94, 255)
	} else if s.MapTheme == MapThemeTwinCities {
		themeBtnColor = RGBA(217, 70, 239, 255)
	} else if s.MapTheme == MapThemeCustom {
		themeBtnColor = RGBA(245, 158, 11, 255)
	}
	vector.DrawFilledRect(screen, float32(s.themeToggleBounds.X), float32(s.themeToggleBounds.Y),
		float32(s.themeToggleBounds.W), float32(s.themeToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.themeToggleBounds.X), float32(s.themeToggleBounds.Y),
		float32(s.themeToggleBounds.W), float32(s.themeToggleBounds.H), 1.5, themeBtnColor, true)
	DrawText(screen, T("🏠 退入主菜单更换", "🏠 Return to Menu to Change"), s.themeToggleBounds.X+24, s.themeToggleBounds.Y+6, 12, themeBtnColor)

	// Option 7: Interface Language (Chinese / English)
	opt7Y := modalY + 290
	DrawText(screen, T("7. 界面语言 / Language:", "7. Interface Language:"), modalX+26, opt7Y+7, 13, RGBA(226, 232, 240, 255))
	s.langToggleBounds = Rect{X: modalX + 340, Y: opt7Y, W: 210, H: 28}

	langBtnColor := RGBA(56, 189, 248, 255)
	langBtnText := "【🌐 简体中文 (Chinese)】"
	if s.Language == LanguageEN {
		langBtnText = "【🌐 English (英文)】"
	}
	vector.DrawFilledRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.langToggleBounds.X), float32(s.langToggleBounds.Y),
		float32(s.langToggleBounds.W), float32(s.langToggleBounds.H), 1.5, langBtnColor, true)
	DrawText(screen, langBtnText, s.langToggleBounds.X+20, s.langToggleBounds.Y+6, 12, langBtnColor)

	// Rules & Customization Explanation Info Box
	infoY := modalY + 328
	infoH := 240.0
	vector.DrawFilledRect(screen, float32(modalX+26), float32(infoY), float32(modalW-52), float32(infoH),
		RGBA(15, 23, 42, 200), true)
	vector.StrokeRect(screen, float32(modalX+26), float32(infoY), float32(modalW-52), float32(infoH),
		1, RGBA(71, 85, 105, 255), true)

	if s.Language == LanguageEN {
		DrawText(screen, "• Traffic Network, Autopilot & Operating Rules:", modalX+36, infoY+10, 12, RGBA(148, 163, 184, 255))
		DrawText(screen, fmt.Sprintf("  - Active Map: [%s] (6 distinct city themes, return to menu to switch);", s.MapTheme.DisplayName()), modalX+36, infoY+28, 12, RGBA(192, 132, 252, 255))
		DrawText(screen, "  - Top Speed (60/80/100/120 km/h): Unified limit for manual driving and Berserk mode; Shift+B to cycle;", modalX+36, infoY+46, 11, RGBA(6, 182, 212, 255))
		DrawText(screen, "  - Route AI Fleet: 0-8 autonomous buses operating routes and serving passengers;", modalX+36, infoY+64, 12, RGBA(52, 211, 153, 255))
		DrawText(screen, "  - Camera Perspectives: Heading-Up Follow / Fixed North-Up / Fixed South-Up ([V]);", modalX+36, infoY+82, 12, RGBA(251, 146, 60, 255))
		DrawText(screen, "  - Road Hierarchy: 4-lane avenues, 2-lane streets, grade-separated overpasses;", modalX+36, infoY+100, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - Standard 52px wide city roadway lanes provide comfortable clearance and turning;", modalX+36, infoY+118, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - Wide intersection geometries allow realistic turning without barrier clipping;", modalX+36, infoY+136, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - Press [L] anytime to quickly toggle interface language (English / Chinese);", modalX+36, infoY+154, 12, RGBA(56, 189, 248, 255))

		descFine := fmt.Sprintf("  - Fines Status: Violations deduct $%d (red light crossing / wrong-way).", s.FineAmount)
		descFineColor := RGBA(248, 113, 113, 255)
		if !s.TrafficFinesEnabled {
			descFine = "  - Fines Status: Violations waived (warning alert only)."
			descFineColor = RGBA(74, 222, 128, 255)
		}
		DrawText(screen, descFine, modalX+36, infoY+176, 12, descFineColor)
	} else {
		DrawText(screen, "• 交通网络调度、自动驾驶与车辆规则说明:", modalX+36, infoY+10, 12, RGBA(148, 163, 184, 255))
		DrawText(screen, fmt.Sprintf("  - 当前地图: 【%s】 (6大独立城市风貌，含自定义地图，需退入主菜单切换)；", s.MapTheme.DisplayName()), modalX+36, infoY+28, 12, RGBA(192, 132, 252, 255))
		DrawText(screen, "  - 公交最高限速 (60/80/100/120 km/h): 手动极速与AI狂暴巡航均生效，支持按 Shift+B 快速切速；", modalX+36, infoY+46, 11, RGBA(6, 182, 212, 255))
		DrawText(screen, "  - 路线 AI 公交: 调度 0~8 辆独立 AI 公交车，在城市环线自主运营、上下客流；", modalX+36, infoY+64, 12, RGBA(52, 211, 153, 255))
		DrawText(screen, "  - 视角切换: 支持【车头朝上跟随】/【固定正北标准】/【固定正南倒置】，按 [V] 键切换；", modalX+36, infoY+82, 12, RGBA(251, 146, 60, 255))
		DrawText(screen, "  - 城市支持4车道快速路、2车道城市路、立体跨线桥及大江大湖水道；", modalX+36, infoY+100, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - 默认采用 52px 宽敞市区物理车道，提供充裕的行驶与平稳转弯空间；", modalX+36, infoY+118, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - 全城道路与路口几何尺寸全面加宽，显著降低公交转弯难度；", modalX+36, infoY+136, 12, RGBA(203, 213, 225, 255))
		DrawText(screen, "  - 随时按键盘 [L] 键可快速在中文与英文界面之间一键切换；", modalX+36, infoY+154, 12, RGBA(56, 189, 248, 255))

		descFine := fmt.Sprintf("  - 罚款状态: 违章抓拍扣除 $%d (红灯压线/对侧车道判断)。", s.FineAmount)
		descFineColor := RGBA(248, 113, 113, 255)
		if !s.TrafficFinesEnabled {
			descFine = "  - 罚款状态: 违章已豁免扣款 (仅弹出警示提示)。"
			descFineColor = RGBA(74, 222, 128, 255)
		}
		DrawText(screen, descFine, modalX+36, infoY+176, 12, descFineColor)
	}

	// Bottom Buttons: Return to Menu & Resume
	s.menuButtonBounds = Rect{X: modalX + 26, Y: modalY + modalH - 46, W: 250, H: 34}
	vector.DrawFilledRect(screen, float32(s.menuButtonBounds.X), float32(s.menuButtonBounds.Y),
		float32(s.menuButtonBounds.W), float32(s.menuButtonBounds.H), RGBA(99, 102, 241, 240), true)
	DrawText(screen, T("🏠 返回主菜单 (选择地图)", "🏠 Return to Main Menu (Maps)"), s.menuButtonBounds.X+24, s.menuButtonBounds.Y+9, 13, color.White)

	s.closeButtonBounds = Rect{X: modalX + modalW - 230, Y: modalY + modalH - 46, W: 204, H: 34}
	vector.DrawFilledRect(screen, float32(s.closeButtonBounds.X), float32(s.closeButtonBounds.Y),
		float32(s.closeButtonBounds.W), float32(s.closeButtonBounds.H), RGBA(59, 130, 246, 255), true)
	DrawText(screen, T("继续驾驶 [ESC]", "Resume Driving [ESC]"), s.closeButtonBounds.X+36, s.closeButtonBounds.Y+9, 13, color.White)
}
