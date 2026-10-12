package game

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type ToastMessage struct {
	Text      string
	Timer     float64
	MaxTime   float64
	IsSuccess bool
	IsWarning bool
}

// HUD handles the on-screen dashboard and user interface
type HUD struct {
	Toasts            []ToastMessage
	GearRBounds           Rect
	GearNBounds           Rect
	GearDBounds           Rect
	AutopilotBounds       Rect
	CameraBtnBounds       Rect
	MapThemeBtnBounds     Rect
	MenuBtnBounds         Rect
	ReturnToMenuRequested bool
}

func NewHUD() *HUD {
	return &HUD{
		Toasts: make([]ToastMessage, 0),
	}
}

// HandleMenuBtnClick detects mouse click on the top-bar main menu / map change button
func (h *HUD) HandleMenuBtnClick(mousePos Vec2) bool {
	if h.MenuBtnBounds.Contains(mousePos) || h.MapThemeBtnBounds.Contains(mousePos) {
		h.ReturnToMenuRequested = true
		return true
	}
	return false
}

// HandleMapThemeBtnClick detects mouse click on the top-bar map theme button
func (h *HUD) HandleMapThemeBtnClick(mousePos Vec2, settings *Settings) bool {
	return h.HandleMenuBtnClick(mousePos)
}

// HandleCameraBtnClick detects mouse click on the top-bar camera perspective button
func (h *HUD) HandleCameraBtnClick(mousePos Vec2, cam *Camera) (toggled bool, notice string) {
	if cam == nil {
		return false, ""
	}
	if h.CameraBtnBounds.Contains(mousePos) {
		newMode := cam.ToggleMode()
		switch newMode {
		case CameraModeHeadingUp:
			return true, T("已切换视角: 【车头朝上 (跟随视角)】 - 视野随转向旋转", "Camera Mode: [Heading-Up Follow] - Rotates with steering")
		case CameraModeSouthUp:
			return true, T("已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上", "Camera Mode: [Fixed South-Up] - Map fixed south")
		default:
			return true, T("已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上", "Camera Mode: [Fixed North-Up] - Standard map north")
		}
	}
	return false, ""
}

// HandleGearClick detects mouse click on the dashboard gear selector badges
func (h *HUD) HandleGearClick(mousePos Vec2, bus *Bus) (shifted bool, notice string) {
	if bus == nil {
		return false, ""
	}
	if h.GearRBounds.Contains(mousePos) {
		if bus.ShiftTo(GearReverse) {
			return true, T("已挂入倒挡 (R) [按 W/↑ 倒车，S/↓ 刹车]", "Shifted to Reverse (R) [W/↑ Reverse, S/↓ Brake]")
		}
		return false, T("车速过快，变速箱保护锁止！请先踩刹车减速", "Speed too high! Gearbox lock active. Please brake first")
	} else if h.GearNBounds.Contains(mousePos) {
		if bus.ShiftTo(GearNeutral) {
			return true, T("已挂入空挡 (N)", "Shifted to Neutral (N)")
		}
	} else if h.GearDBounds.Contains(mousePos) {
		if bus.ShiftTo(GearDrive) {
			return true, T("已挂入前进挡 (D)", "Shifted to Drive (D)")
		}
		return false, T("倒车速度过快，无法切入前进挡！请先踩刹车减速", "Reverse speed too high! Cannot shift to D. Please brake first")
	}
	return false, ""
}

// HandleAutopilotClick detects mouse click on the dashboard autopilot button
func (h *HUD) HandleAutopilotClick(mousePos Vec2, settings *Settings) (toggled bool, notice string) {
	if settings == nil {
		return false, ""
	}
	if h.AutopilotBounds.Contains(mousePos) {
		settings.AutopilotEnabled = !settings.AutopilotEnabled
		if settings.AutopilotEnabled {
			return true, T("已开启 AI 自动巡航驾驶 (P键随时切换)", "AI Autopilot ON (Press P anytime to toggle)")
		}
		return true, T("已切换回手动驾驶模式 (P键随时切换)", "Manual Driving Mode ON (Press P anytime to toggle)")
	}
	return false, ""
}

func (h *HUD) AddToast(text string, isSuccess bool) {
	h.Toasts = append(h.Toasts, ToastMessage{
		Text:      text,
		Timer:     3.5,
		MaxTime:   3.5,
		IsSuccess: isSuccess,
		IsWarning: false,
	})
	if len(h.Toasts) > 4 {
		h.Toasts = h.Toasts[len(h.Toasts)-4:]
	}
}

func (h *HUD) AddWarningToast(text string) {
	h.Toasts = append(h.Toasts, ToastMessage{
		Text:      text,
		Timer:     4.5,
		MaxTime:   4.5,
		IsSuccess: false,
		IsWarning: true,
	})
	if len(h.Toasts) > 4 {
		h.Toasts = h.Toasts[len(h.Toasts)-4:]
	}
}

func (h *HUD) Update(dt float64) {
	valid := h.Toasts[:0]
	for i := range h.Toasts {
		h.Toasts[i].Timer -= dt
		if h.Toasts[i].Timer > 0 {
			valid = append(valid, h.Toasts[i])
		}
	}
	h.Toasts = valid
}

func (h *HUD) Draw(screen *ebiten.Image, bus *Bus, route *Route, world *World, isPlanMode bool, cash int, activeStop *BusStop, settings *Settings, stopBellRung bool, passengersAlighting int, cam *Camera, tls ...*TrafficLightSystem) {
	w := screen.Bounds().Dx()
	_ = screen.Bounds().Dy()

	var activeTLS *TrafficLightSystem
	if len(tls) > 0 && tls[0] != nil {
		activeTLS = tls[0]
	}

	// 1. Top Status Banner (Glassmorphism dark slate bar)
	vector.DrawFilledRect(screen, 0, 0, float32(w), 56, RGBA(15, 23, 42, 220), false)
	vector.StrokeLine(screen, 0, 56, float32(w), 56, 1.5, RGBA(51, 65, 85, 255), false)

	// Mode Indicator Badge
	modeText := T(" [驾驶模式 DRIVE] (TAB 规划)", " [DRIVE MODE] (TAB Route)")
	modeBg := RGBA(14, 165, 233, 240) // Cyan
	if isPlanMode {
		modeText = T(" [线路规划模式 PLAN] (TAB 驾驶)", " [PLAN MODE] (TAB Drive)")
		modeBg = RGBA(168, 85, 247, 240) // Purple
	}
	vector.DrawFilledRect(screen, 16, 12, 220, 32, modeBg, true)
	DrawText(screen, modeText, 22, 20, 13, color.White)

	// Economy & Capacity Badges
	cashText := Tf("营收: $%d", "Revenue: $%d", cash)
	DrawText(screen, cashText, 246, 20, 13, RGBA(250, 204, 21, 255))

	paxText := Tf("乘客: %d/%d", "Pax: %d/%d", bus.PassengerCount, bus.Capacity)
	DrawText(screen, paxText, 345, 20, 13, RGBA(56, 189, 248, 255))

	// Current target stop & Stop bell status
	if targetID := route.GetCurrentTargetStopID(); targetID >= 0 && targetID < len(world.Stops) {
		stop := world.Stops[targetID]
		distToStop := bus.Pos.Distance(stop.Pos)
		distMeters := int(math.Round(distToStop * 0.5))
		targetMsg := Tf("下一站: [%s] 距%dm (候车 %d人)", "Next: [%s] %dm (Wait: %d)", stop.LocalizedName(), distMeters, stop.WaitingPassengers)
		DrawText(screen, targetMsg, 445, 20, 12, RGBA(226, 232, 240, 255))

		// Stop Bell badge
		bellX := 660.0
		if stopBellRung {
			bellText := Tf("🔔 下车铃: %d人准备下车", "🔔 Bell: %d to alight", passengersAlighting)
			vector.DrawFilledRect(screen, float32(bellX), 12, 175, 32, RGBA(180, 83, 9, 230), true)
			vector.StrokeRect(screen, float32(bellX), 12, 175, 32, 1.5, RGBA(250, 204, 21, 255), true)
			DrawText(screen, bellText, bellX+10, 20, 12, RGBA(254, 240, 138, 255))
		} else {
			bellText := T("🔕 下车铃: 无人按铃", "🔕 Bell: None")
			vector.DrawFilledRect(screen, float32(bellX), 12, 150, 32, RGBA(30, 41, 59, 220), true)
			vector.StrokeRect(screen, float32(bellX), 12, 150, 32, 1.0, RGBA(100, 116, 139, 200), true)
			DrawText(screen, bellText, bellX+12, 20, 12, RGBA(148, 163, 184, 255))
		}
	} else {
		DrawText(screen, T("未选择目标站点", "No target stop selected"), 445, 20, 13, RGBA(148, 163, 184, 255))
	}

	// Map Theme Selector Badge
	// Main Menu / Change Map Badge
	mapBtnX := float32(w - 530)
	mapBtnW := float32(165)
	h.MenuBtnBounds = Rect{X: float64(mapBtnX), Y: 12, W: float64(mapBtnW), H: 32}
	h.MapThemeBtnBounds = h.MenuBtnBounds
	menuBtnStr := T("🏠 主菜单/换图", "🏠 Menu / Maps")
	menuBtnColor := RGBA(168, 85, 247, 255)
	if settings != nil {
		switch settings.MapTheme {
		case MapThemeCoastalBay:
			menuBtnStr = T("🏠 海滨港湾 [换图]", "🏠 Coastal Bay [Maps]")
			menuBtnColor = RGBA(6, 182, 212, 255)
		case MapThemeRiverValley:
			menuBtnStr = T("🏠 溪谷江城 [换图]", "🏠 River Valley [Maps]")
			menuBtnColor = RGBA(16, 185, 129, 255)
		case MapThemeEmeraldLake:
			menuBtnStr = T("🏠 湖畔水乡 [换图]", "🏠 Emerald Lake [Maps]")
			menuBtnColor = RGBA(34, 197, 94, 255)
		case MapThemeTwinCities:
			menuBtnStr = T("🏠 高新双城 [换图]", "🏠 Twin Districts [Maps]")
			menuBtnColor = RGBA(217, 70, 239, 255)
		case MapThemeCustom:
			menuBtnStr = T("🏠 自定义图 [换图]", "🏠 Custom Map [Maps]")
			menuBtnColor = RGBA(245, 158, 11, 255)
		default:
			menuBtnStr = T("🏠 现代都会 [换图]", "🏠 Metropolis [Maps]")
			menuBtnColor = RGBA(59, 130, 246, 255)
		}
	}
	vector.DrawFilledRect(screen, mapBtnX, 12, mapBtnW, 32, RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, mapBtnX, 12, mapBtnW, 32, 1.5, menuBtnColor, true)
	DrawText(screen, menuBtnStr, float64(mapBtnX)+12, 20, 12, menuBtnColor)

	// Camera Perspective Badge (Switchable between North-Up, Heading-Up, and South-Up)
	camBtnX := float32(w - 355)
	camBtnW := float32(165)
	h.CameraBtnBounds = Rect{X: float64(camBtnX), Y: 12, W: float64(camBtnW), H: 32}
	camModeStr := T("🎥 视角: 正北 [V]", "🎥 Cam: North [V]")
	camModeColor := RGBA(148, 163, 184, 255)
	if cam != nil {
		switch cam.Mode {
		case CameraModeHeadingUp:
			camModeStr = T("🎥 视角: 跟随 [V]", "🎥 Cam: Follow [V]")
			camModeColor = RGBA(56, 189, 248, 255)
		case CameraModeSouthUp:
			camModeStr = T("🎥 视角: 正南 [V]", "🎥 Cam: South [V]")
			camModeColor = RGBA(251, 146, 60, 255)
		}
	}
	vector.DrawFilledRect(screen, camBtnX, 12, camBtnW, 32, RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, camBtnX, 12, camBtnW, 32, 1.5, camModeColor, true)
	DrawText(screen, camModeStr, float64(camBtnX)+12, 20, 12, camModeColor)

	// Settings button / violation status on top right
	fineStatusText := T("[违章: 开启]", "[Fines: ON]")
	fineStatusColor := RGBA(34, 197, 94, 255)
	if !settings.TrafficFinesEnabled {
		fineStatusText = T("[违章: 关闭]", "[Fines: OFF]")
		fineStatusColor = RGBA(239, 68, 68, 255)
	}
	settingsBtnX := float32(w - 180)
	vector.DrawFilledRect(screen, settingsBtnX, 12, 165, 32, RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, settingsBtnX, 12, 165, 32, 1.5, fineStatusColor, true)
	settingsBtnLabel := fmt.Sprintf("%s [ESC]", fineStatusText)
	DrawText(screen, settingsBtnLabel, float64(settingsBtnX)+14, 20, 12, fineStatusColor)

	// 2. Bottom Dashboard (in Drive Mode)
	if !isPlanMode {
		var targetStop *BusStop
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

func (h *HUD) drawDriveDashboard(screen *ebiten.Image, bus *Bus, activeStop *BusStop, targetStop *BusStop, stopBellRung bool, passengersAlighting int, tls *TrafficLightSystem, settings *Settings) {
	screenH := screen.Bounds().Dy()

	// Bottom-left Speedometer & Telemetry
	dashX := 24.0
	dashY := float64(screenH - 100)

	// Approaching intersection traffic signal status banner
	if tls != nil {
		lw := 42.0
		if settings != nil {
			lw = settings.GetLaneWidth()
		}
		hasInter, thruSig, leftSig, isMajor, _, dist := tls.GetApproachingSignalInfo(bus.Pos, bus.Heading, lw)
		if hasInter && dist < 240.0 {
			sigBoxW := float32(318.0)
			sigBoxY := dashY - 40.0
			sigYF32 := float32(sigBoxY)
			sigBoxH := float32(34.0)

			// --- Frosted Glass Effect (毛玻璃特效) ---
			// 1. Ambient soft drop shadow
			vector.DrawFilledRect(screen, float32(dashX-2), sigYF32-2, sigBoxW+4, sigBoxH+4, RGBA(2, 6, 23, 110), true)
			// 2. Translucent deep-slate frosted glass body
			vector.DrawFilledRect(screen, float32(dashX), sigYF32, sigBoxW, sigBoxH, RGBA(15, 23, 42, 195), true)
			// 3. Frosted specular reflection sheen across top half
			vector.DrawFilledRect(screen, float32(dashX+1), sigYF32+1, sigBoxW-2, sigBoxH*0.46, RGBA(255, 255, 255, 20), true)
			// 4. Subtle frosted inner rim
			vector.StrokeRect(screen, float32(dashX+1), sigYF32+1, sigBoxW-2, sigBoxH-2, 0.8, RGBA(148, 163, 184, 75), true)
			// 5. Crisp frosted glass outer border
			vector.StrokeRect(screen, float32(dashX), sigYF32, sigBoxW, sigBoxH, 1.2, RGBA(255, 255, 255, 70), true)

			lampY := sigYF32 + 17
			if isMajor {
				// Frosted dividers between signal sections
				vector.StrokeLine(screen, float32(dashX+105), sigYF32+6, float32(dashX+105), sigYF32+sigBoxH-6, 1.0, RGBA(255, 255, 255, 45), true)
				vector.StrokeLine(screen, float32(dashX+206), sigYF32+6, float32(dashX+206), sigYF32+sigBoxH-6, 1.0, RGBA(255, 255, 255, 45), true)

				// Major intersection: show both Left-Turn and Straight signals
				lCol := RGBA(248, 113, 113, 255)
				lDot := RGBA(248, 40, 40, 255)
				lText := T("左转↰ 红", "Left↰ RED")
				if leftSig == SignalYellow {
					lCol = RGBA(250, 204, 21, 255)
					lDot = RGBA(250, 204, 21, 255)
					lText = T("左转↰ 黄", "Left↰ YEL")
				} else if leftSig == SignalGreen {
					lCol = RGBA(74, 222, 128, 255)
					lDot = RGBA(34, 197, 94, 255)
					lText = T("左转↰ 绿", "Left↰ GRN")
				}

				tCol := RGBA(248, 113, 113, 255)
				tDot := RGBA(248, 40, 40, 255)
				tText := T("直行↑ 红", "Thru↑ RED")
				if thruSig == SignalYellow {
					tCol = RGBA(250, 204, 21, 255)
					tDot = RGBA(250, 204, 21, 255)
					tText = T("直行↑ 黄", "Thru↑ YEL")
				} else if thruSig == SignalGreen {
					tCol = RGBA(74, 222, 128, 255)
					tDot = RGBA(34, 197, 94, 255)
					tText = T("直行↑ 绿", "Thru↑ GRN")
				}

				// Soft frosted glass bloom halo behind active lamps
				vector.DrawFilledCircle(screen, float32(dashX+16), lampY, 8, RGBA(lDot.R, lDot.G, lDot.B, 70), true)
				vector.DrawFilledCircle(screen, float32(dashX+16), lampY, 5, lDot, true)
				DrawText(screen, lText, dashX+26, sigBoxY+11, 11, lCol)

				vector.DrawFilledCircle(screen, float32(dashX+118), lampY, 8, RGBA(tDot.R, tDot.G, tDot.B, 70), true)
				vector.DrawFilledCircle(screen, float32(dashX+118), lampY, 5, tDot, true)
				DrawText(screen, tText, dashX+128, sigBoxY+11, 11, tCol)

				distText := Tf("(距线 %.0fm)", "(%.0fm to line)", math.Max(0, dist))
				DrawText(screen, distText, dashX+214, sigBoxY+11, 11, RGBA(203, 213, 225, 255))
			} else {
				// Standard 3-lamp banner for minor intersections
				rCol := RGBA(70, 15, 15, 255)
				yCol := RGBA(70, 60, 15, 255)
				gCol := RGBA(15, 60, 25, 255)

				statusText := ""
				statusColor := color.RGBA{R: 255, G: 255, B: 255, A: 255}
				switch thruSig {
				case SignalRed:
					rCol = RGBA(248, 40, 40, 255)
					statusText = Tf("前方信号: 红灯 (停止线前等待 %.0fm)", "Signal: RED (Stop before line %.0fm)", math.Max(0, dist))
					statusColor = RGBA(248, 113, 113, 255)
				case SignalYellow:
					yCol = RGBA(250, 204, 21, 255)
					statusText = Tf("前方信号: 黄灯 (减速准备 %.0fm)", "Signal: YELLOW (Prepare to stop %.0fm)", math.Max(0, dist))
					statusColor = RGBA(250, 204, 21, 255)
				case SignalGreen:
					gCol = RGBA(34, 197, 94, 255)
					statusText = Tf("前方信号: 绿灯 (准许通行 %.0fm)", "Signal: GREEN (Proceed %.0fm)", math.Max(0, dist))
					statusColor = RGBA(74, 222, 128, 255)
				}

				vector.DrawFilledCircle(screen, float32(dashX+16), lampY, 5, rCol, true)
				vector.DrawFilledCircle(screen, float32(dashX+30), lampY, 5, yCol, true)
				vector.DrawFilledCircle(screen, float32(dashX+44), lampY, 5, gCol, true)
				DrawText(screen, statusText, dashX+58, sigBoxY+11, 11, statusColor)
			}
		}
	}

	// Dark round cluster background
	vector.DrawFilledRect(screen, float32(dashX), float32(dashY), 350, 80, RGBA(15, 23, 42, 230), true)
	vector.StrokeRect(screen, float32(dashX), float32(dashY), 350, 80, 1.5, RGBA(51, 65, 85, 255), true)

	// Speed calculation (convert px/s to intuitive km/h)
	speedKmh := int(math.Abs(bus.Speed) * 0.18)
	speedStr := fmt.Sprintf("%3d", speedKmh)
	DrawText(screen, speedStr, dashX+14, dashY+16, 16, color.White)
	DrawText(screen, "KM/H", dashX+52, dashY+20, 11, RGBA(148, 163, 184, 255))

	// Speed bar
	barRatio := math.Abs(bus.Speed) / bus.MaxSpeedFwd
	barColor := RGBA(34, 197, 94, 255)
	if barRatio > 0.75 {
		barColor = RGBA(239, 68, 68, 255)
	} else if barRatio > 0.45 {
		barColor = RGBA(234, 179, 8, 255)
	}
	vector.DrawFilledRect(screen, float32(dashX+14), float32(dashY+48), float32(80*barRatio), 7, barColor, true)
	vector.StrokeRect(screen, float32(dashX+14), float32(dashY+48), 80, 7, 1, RGBA(100, 116, 139, 200), true)

	// Vertical cluster divider
	vector.StrokeLine(screen, float32(dashX+104), float32(dashY+8), float32(dashX+104), float32(dashY+72), 1, RGBA(51, 65, 85, 200), true)

	// Interactive Gear Selector Badges: [ R ] [ N ] [ D ]
	btnW := 24.0
	btnH := 20.0
	h.GearRBounds = Rect{X: dashX + 114, Y: dashY + 12, W: btnW, H: btnH}
	h.GearNBounds = Rect{X: dashX + 142, Y: dashY + 12, W: btnW, H: btnH}
	h.GearDBounds = Rect{X: dashX + 170, Y: dashY + 12, W: btnW, H: btnH}

	// 1. R (Reverse)
	if bus.Gear == GearReverse {
		vector.DrawFilledRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), RGBA(239, 68, 68, 240), true)
		vector.StrokeRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), 1.5, RGBA(254, 202, 202, 255), true)
		DrawText(screen, "R", h.GearRBounds.X+7, h.GearRBounds.Y+4, 12, color.White)
	} else {
		vector.DrawFilledRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), 1.0, RGBA(71, 85, 105, 180), true)
		DrawText(screen, "R", h.GearRBounds.X+7, h.GearRBounds.Y+4, 12, RGBA(148, 163, 184, 255))
	}

	// 2. N (Neutral)
	if bus.Gear == GearNeutral {
		vector.DrawFilledRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), RGBA(234, 179, 8, 240), true)
		vector.StrokeRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), 1.5, RGBA(254, 240, 138, 255), true)
		DrawText(screen, "N", h.GearNBounds.X+7, h.GearNBounds.Y+4, 12, RGBA(15, 23, 42, 255))
	} else {
		vector.DrawFilledRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), 1.0, RGBA(71, 85, 105, 180), true)
		DrawText(screen, "N", h.GearNBounds.X+7, h.GearNBounds.Y+4, 12, RGBA(148, 163, 184, 255))
	}

	// 3. D (Drive / Forward)
	if bus.Gear == GearDrive || bus.Gear >= Gear1 {
		vector.DrawFilledRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), RGBA(34, 197, 94, 240), true)
		vector.StrokeRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), 1.5, RGBA(187, 247, 208, 255), true)
		label := bus.Gear.String()
		DrawText(screen, label, h.GearDBounds.X+7, h.GearDBounds.Y+4, 12, color.White)
	} else {
		vector.DrawFilledRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), 1.0, RGBA(71, 85, 105, 180), true)
		DrawText(screen, "D", h.GearDBounds.X+7, h.GearDBounds.Y+4, 12, RGBA(148, 163, 184, 255))
	}

	// Status text next to gear buttons
	statusStr := bus.Gear.Name()
	statusCol := RGBA(74, 222, 128, 255)
	if bus.DoorsOpen {
		statusStr = T("【开门P】", "[DOOR P]")
		statusCol = RGBA(250, 204, 21, 255)
	} else if bus.IsHandbraking {
		statusStr = T("【!手刹!】", "[!PARK!]")
		statusCol = RGBA(239, 68, 68, 255)
	} else if bus.IsBraking {
		statusStr = T("【脚刹】", "[BRAKE]")
		statusCol = RGBA(248, 113, 113, 255)
	} else if bus.Gear == GearReverse {
		statusCol = RGBA(248, 113, 113, 255)
	} else if bus.Gear == GearNeutral {
		statusCol = RGBA(250, 204, 21, 255)
	}
	DrawText(screen, statusStr, dashX+200, dashY+16, 11, statusCol)

	// Interactive AI Autopilot Badge: [🤖 AI巡航(P)] / [🤖 手动(P)]
	apW := 88.0
	apH := 20.0
	h.AutopilotBounds = Rect{X: dashX + 252, Y: dashY + 12, W: apW, H: apH}
	if settings != nil && settings.AutopilotEnabled {
		vector.DrawFilledRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), RGBA(6, 182, 212, 235), true)
		vector.StrokeRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), 1.5, RGBA(165, 243, 252, 255), true)
		DrawText(screen, T("🤖 AI巡航(P)", "🤖 Autopilot(P)"), h.AutopilotBounds.X+6, h.AutopilotBounds.Y+4, 11, RGBA(15, 23, 42, 255))
	} else {
		vector.DrawFilledRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), 1.0, RGBA(100, 116, 139, 180), true)
		DrawText(screen, T("🤖 手动(P)", "🤖 Manual(P)"), h.AutopilotBounds.X+10, h.AutopilotBounds.Y+4, 11, RGBA(148, 163, 184, 255))
	}

	// Gear shortcut prompt row
	DrawText(screen, T("换挡: [Shift/Ctrl]升降 [R/N/G]切挡 [P]AI", "Shift: [Shift/Ctrl] [R/N/G] [P]AI"), dashX+114, dashY+38, 10, RGBA(148, 163, 184, 220))

	// Health and Steering info row
	durCol := RGBA(34, 197, 94, 255)
	if bus.Durability < 35 {
		durCol = RGBA(239, 68, 68, 255)
	} else if bus.Durability < 70 {
		durCol = RGBA(234, 179, 8, 255)
	}
	durStr := Tf("车况:%.0f%%", "Health:%.0f%%", bus.Durability)
	DrawText(screen, durStr, dashX+114, dashY+58, 10, durCol)

	if bus.OnShoulder {
		vector.DrawFilledRect(screen, float32(dashX+182), float32(dashY+54), 126, 18, RGBA(234, 179, 8, 220), true)
		DrawText(screen, T("[ 路肩缓冲减速 ]", "[ Shoulder Slow ]"), dashX+190, dashY+57, 10, RGBA(15, 23, 42, 255))
	} else {
		steerStr := Tf("转向: %.0f°", "Steer: %.0f°", bus.SteeringAngle*180/math.Pi)
		DrawText(screen, steerStr, dashX+182, dashY+58, 10, RGBA(203, 213, 225, 255))
		steerRatio := bus.SteeringAngle / bus.MaxSteerAngle
		vector.StrokeLine(screen, float32(dashX+240), float32(dashY+63), float32(dashX+240+steerRatio*20), float32(dashY+63), 2.0, RGBA(56, 189, 248, 255), true)
	}

	// Prompt Banners at Bottom Center
	if activeStop != nil {
		promptW := 440.0
		promptH := 46.0
		promptX := float64(screen.Bounds().Dx())/2 - promptW/2
		promptY := float64(screenH) - 80.0

		vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			RGBA(30, 41, 59, 240), true)
		vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			2, RGBA(250, 204, 21, 255), true)

		promptText := Tf("已停靠在 [%s]! 按 [E] 开门上下客", "Stopped at [%s]! Press [E] to open doors", activeStop.LocalizedName())
		if bus.DoorsOpen {
			promptText = T("上下客完成! 按 [E] 关门准备发车 >>", "Boarding complete! Press [E] to close doors >>")
		}
		DrawText(screen, promptText, promptX+20, promptY+15, 13, RGBA(250, 204, 21, 255))
	} else if targetStop != nil {
		dist := bus.Pos.Distance(targetStop.Pos)
		if dist <= 280.0 {
			promptH := 46.0
			promptY := float64(screenH) - 80.0

			canSkip := targetStop.WaitingPassengers == 0 && !stopBellRung
			if canSkip {
				// Eligible for Skip Stop (飞站)!
				promptW := 560.0
				promptX := float64(screen.Bounds().Dx())/2 - promptW/2

				vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					RGBA(6, 78, 59, 235), true)
				vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					2, RGBA(52, 211, 153, 255), true)

				promptText := Tf("【满足飞站条件】[%s] 站台无人且无下车铃！按 [F] 飞站 ⏩ (或直接驶过)", "[Express Skip] [%s] No waiting pax & no bell! Press [F] to skip ⏩", targetStop.LocalizedName())
				DrawText(screen, promptText, promptX+16, promptY+15, 13, RGBA(209, 250, 229, 255))
			} else {
				// Must Stop
				promptW := 530.0
				promptX := float64(screen.Bounds().Dx())/2 - promptW/2

				vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					RGBA(30, 41, 59, 235), true)
				vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					2, RGBA(234, 179, 8, 255), true)

				promptText := Tf("【须靠站】[%s] 站台 %d 人候车 / 车内 %d 人下车，请减速靠站", "[Must Stop] [%s] %d waiting / %d alighting, please decelerate to stop bay", targetStop.LocalizedName(), targetStop.WaitingPassengers, passengersAlighting)
				DrawText(screen, promptText, promptX+16, promptY+15, 13, RGBA(254, 240, 138, 255))
			}
		}
	}
}

func (h *HUD) drawPlanningInstructions(screen *ebiten.Image, route *Route, world *World, settings *Settings) {
	screenH := screen.Bounds().Dy()
	screenW := screen.Bounds().Dx()

	// Planning helper card at bottom center
	panelW := 740.0
	panelH := 92.0
	panelX := float64(screenW)/2 - panelW/2
	panelY := float64(screenH) - 108.0

	vector.DrawFilledRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		RGBA(15, 23, 42, 230), true)
	vector.StrokeRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		1.5, RGBA(168, 85, 247, 240), true)

	DrawText(screen, T("【线路规划面板】鼠标左键点击任意站点图标将其加入行驶路线", "[Route Planning Panel] Left click station icons to add to driving route"), panelX+20, panelY+12, 13, RGBA(192, 132, 252, 255))
	fleetText := ""
	if settings != nil {
		fleetText = Tf(" | 路线AI车队: %d 辆 (ESC设置修改)", " | Route AI Fleet: %d buses (ESC to edit)", settings.AIBusCount)
	}
	DrawText(screen, Tf("【靠站规则】公交车站均设于道路顺行右侧，系统已自动按顺行右侧靠站规划路线%s", "[Stop Rules] Stations placed on right side of roadway. Route safely planned for right-side bays%s", fleetText), panelX+20, panelY+32, 11, RGBA(250, 204, 21, 255))
	routeDesc := Tf("当前路线: %s", "Active Route: %s", route.GetRouteDescription(world))
	DrawText(screen, routeDesc, panelX+20, panelY+50, 12, color.White)
	DrawText(screen, T("[C键] 清空路线重新规划  |  [TAB键] 返回驾驶并执行当前路线  |  [P键] 开启AI自动巡航驾驶", "[C] Clear & Redo Route  |  [TAB] Resume Driving with Route  |  [P] Toggle AI Autopilot"), panelX+20, panelY+70, 12, RGBA(148, 163, 184, 255))
}

func (h *HUD) drawToasts(screen *ebiten.Image) {
	toastX := 24.0
	toastY := 70.0
	for _, t := range h.Toasts {
		alpha := float32(t.Timer / t.MaxTime)
		if alpha > 1 {
			alpha = 1
		}
		bg := RGBA(30, 41, 59, uint8(alpha*220))
		border := RGBA(59, 130, 246, uint8(alpha*255))
		toastW := 380.0

		if t.IsSuccess {
			border = RGBA(34, 197, 94, uint8(alpha*255))
		} else if t.IsWarning {
			bg = RGBA(127, 29, 29, uint8(alpha*235))      // Deep Crimson
			border = RGBA(239, 68, 68, uint8(alpha*255))  // Glowing Red Alert
			toastW = 440.0
		}

		vector.DrawFilledRect(screen, float32(toastX), float32(toastY), float32(toastW), 32, bg, true)
		vector.StrokeRect(screen, float32(toastX), float32(toastY), float32(toastW), 32, 1.5, border, true)
		DrawText(screen, t.Text, toastX+14, toastY+8, 12, color.White)
		toastY += 38.0
	}
}
