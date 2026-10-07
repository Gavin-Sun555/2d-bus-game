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
	Toasts          []ToastMessage
	GearRBounds     Rect
	GearNBounds     Rect
	GearDBounds     Rect
	CameraBtnBounds Rect
}

func NewHUD() *HUD {
	return &HUD{
		Toasts: make([]ToastMessage, 0),
	}
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
			return true, "已切换视角: 【车头朝上 (跟随视角)】 - 车头始终朝上，视野随转向旋转"
		case CameraModeSouthUp:
			return true, "已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上"
		default:
			return true, "已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上"
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
			return true, "已挂入倒挡 (R) [按 W/↑ 倒车，S/↓ 刹车]"
		}
		return false, "车速过快，变速箱保护锁止！请先踩刹车减速"
	} else if h.GearNBounds.Contains(mousePos) {
		if bus.ShiftTo(GearNeutral) {
			return true, "已挂入空挡 (N)"
		}
	} else if h.GearDBounds.Contains(mousePos) {
		if bus.ShiftTo(GearDrive) {
			return true, "已挂入前进挡 (D)"
		}
		return false, "倒车速度过快，无法直接切入前进挡！请先踩刹车减速"
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
	modeText := " [驾驶模式 DRIVE] (TAB 规划)"
	modeBg := RGBA(14, 165, 233, 240) // Cyan
	if isPlanMode {
		modeText = " [线路规划模式 PLAN] (TAB 驾驶)"
		modeBg = RGBA(168, 85, 247, 240) // Purple
	}
	vector.DrawFilledRect(screen, 16, 12, 220, 32, modeBg, true)
	DrawText(screen, modeText, 22, 20, 13, color.White)

	// Economy & Capacity Badges
	cashText := fmt.Sprintf("营收: $%d", cash)
	DrawText(screen, cashText, 246, 20, 13, RGBA(250, 204, 21, 255))

	paxText := fmt.Sprintf("乘客: %d/%d", bus.PassengerCount, bus.Capacity)
	DrawText(screen, paxText, 345, 20, 13, RGBA(56, 189, 248, 255))

	// Current target stop & Stop bell status
	if targetID := route.GetCurrentTargetStopID(); targetID >= 0 && targetID < len(world.Stops) {
		stop := world.Stops[targetID]
		targetMsg := fmt.Sprintf("下一站: [%s] (候车 %d人)", stop.Name, stop.WaitingPassengers)
		DrawText(screen, targetMsg, 445, 20, 12, RGBA(226, 232, 240, 255))

		// Stop Bell badge
		bellX := 660.0
		if stopBellRung {
			bellText := fmt.Sprintf("🔔 下车铃: %d人准备下车", passengersAlighting)
			vector.DrawFilledRect(screen, float32(bellX), 12, 175, 32, RGBA(180, 83, 9, 230), true)
			vector.StrokeRect(screen, float32(bellX), 12, 175, 32, 1.5, RGBA(250, 204, 21, 255), true)
			DrawText(screen, bellText, bellX+10, 20, 12, RGBA(254, 240, 138, 255))
		} else {
			bellText := "🔕 下车铃: 无人按铃"
			vector.DrawFilledRect(screen, float32(bellX), 12, 150, 32, RGBA(30, 41, 59, 220), true)
			vector.StrokeRect(screen, float32(bellX), 12, 150, 32, 1.0, RGBA(100, 116, 139, 200), true)
			DrawText(screen, bellText, bellX+12, 20, 12, RGBA(148, 163, 184, 255))
		}
	} else {
		DrawText(screen, "未选择目标站点", 445, 20, 13, RGBA(148, 163, 184, 255))
	}

	// Camera Perspective Badge (Switchable between North-Up, Heading-Up, and South-Up)
	camBtnX := float32(w - 380)
	camBtnW := float32(175)
	h.CameraBtnBounds = Rect{X: float64(camBtnX), Y: 12, W: float64(camBtnW), H: 32}
	camModeStr := "🎥 视角: 正北固定 [V]"
	camModeColor := RGBA(148, 163, 184, 255)
	if cam != nil {
		switch cam.Mode {
		case CameraModeHeadingUp:
			camModeStr = "🎥 视角: 车头朝上 [V]"
			camModeColor = RGBA(56, 189, 248, 255)
		case CameraModeSouthUp:
			camModeStr = "🎥 视角: 正南固定 [V]"
			camModeColor = RGBA(251, 146, 60, 255)
		}
	}
	vector.DrawFilledRect(screen, camBtnX, 12, camBtnW, 32, RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, camBtnX, 12, camBtnW, 32, 1.5, camModeColor, true)
	DrawText(screen, camModeStr, float64(camBtnX)+12, 20, 12, camModeColor)

	// Settings button / violation status on top right
	fineStatusText := "[违章: 开启]"
	fineStatusColor := RGBA(34, 197, 94, 255)
	if !settings.TrafficFinesEnabled {
		fineStatusText = "[违章: 关闭]"
		fineStatusColor = RGBA(239, 68, 68, 255)
	}
	settingsBtnX := float32(w - 195)
	vector.DrawFilledRect(screen, settingsBtnX, 12, 180, 32, RGBA(30, 41, 59, 240), true)
	vector.StrokeRect(screen, settingsBtnX, 12, 180, 32, 1.5, fineStatusColor, true)
	settingsBtnLabel := fmt.Sprintf("%s [ESC]", fineStatusText)
	DrawText(screen, settingsBtnLabel, float64(settingsBtnX)+14, 20, 13, fineStatusColor)

	// 2. Bottom Dashboard (in Drive Mode)
	if !isPlanMode {
		var targetStop *BusStop
		if targetID := route.GetCurrentTargetStopID(); targetID >= 0 && targetID < len(world.Stops) {
			targetStop = world.Stops[targetID]
		}
		h.drawDriveDashboard(screen, bus, activeStop, targetStop, stopBellRung, passengersAlighting, activeTLS, settings)
	} else {
		h.drawPlanningInstructions(screen, route, world)
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
				lText := "左转↰ 红"
				if leftSig == SignalYellow {
					lCol = RGBA(250, 204, 21, 255)
					lDot = RGBA(250, 204, 21, 255)
					lText = "左转↰ 黄"
				} else if leftSig == SignalGreen {
					lCol = RGBA(74, 222, 128, 255)
					lDot = RGBA(34, 197, 94, 255)
					lText = "左转↰ 绿"
				}

				tCol := RGBA(248, 113, 113, 255)
				tDot := RGBA(248, 40, 40, 255)
				tText := "直行↑ 红"
				if thruSig == SignalYellow {
					tCol = RGBA(250, 204, 21, 255)
					tDot = RGBA(250, 204, 21, 255)
					tText = "直行↑ 黄"
				} else if thruSig == SignalGreen {
					tCol = RGBA(74, 222, 128, 255)
					tDot = RGBA(34, 197, 94, 255)
					tText = "直行↑ 绿"
				}

				// Soft frosted glass bloom halo behind active lamps
				vector.DrawFilledCircle(screen, float32(dashX+16), lampY, 8, RGBA(lDot.R, lDot.G, lDot.B, 70), true)
				vector.DrawFilledCircle(screen, float32(dashX+16), lampY, 5, lDot, true)
				DrawText(screen, lText, dashX+26, sigBoxY+11, 11, lCol)

				vector.DrawFilledCircle(screen, float32(dashX+118), lampY, 8, RGBA(tDot.R, tDot.G, tDot.B, 70), true)
				vector.DrawFilledCircle(screen, float32(dashX+118), lampY, 5, tDot, true)
				DrawText(screen, tText, dashX+128, sigBoxY+11, 11, tCol)

				distText := fmt.Sprintf("(距线 %.0fm)", math.Max(0, dist))
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
					statusText = fmt.Sprintf("前方信号: 红灯 (停止线前等待 %.0fm)", math.Max(0, dist))
					statusColor = RGBA(248, 113, 113, 255)
				case SignalYellow:
					yCol = RGBA(250, 204, 21, 255)
					statusText = fmt.Sprintf("前方信号: 黄灯 (减速准备 %.0fm)", math.Max(0, dist))
					statusColor = RGBA(250, 204, 21, 255)
				case SignalGreen:
					gCol = RGBA(34, 197, 94, 255)
					statusText = fmt.Sprintf("前方信号: 绿灯 (准许通行 %.0fm)", math.Max(0, dist))
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
	vector.DrawFilledRect(screen, float32(dashX), float32(dashY), 320, 80, RGBA(15, 23, 42, 230), true)
	vector.StrokeRect(screen, float32(dashX), float32(dashY), 320, 80, 1.5, RGBA(51, 65, 85, 255), true)

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
		statusStr = "【开门P】"
		statusCol = RGBA(250, 204, 21, 255)
	} else if bus.IsHandbraking {
		statusStr = "【!手刹!】"
		statusCol = RGBA(239, 68, 68, 255)
	} else if bus.IsBraking {
		statusStr = "【脚刹】"
		statusCol = RGBA(248, 113, 113, 255)
	} else if bus.Gear == GearReverse {
		statusCol = RGBA(248, 113, 113, 255)
	} else if bus.Gear == GearNeutral {
		statusCol = RGBA(250, 204, 21, 255)
	}
	DrawText(screen, statusStr, dashX+202, dashY+16, 11, statusCol)

	// Gear shortcut prompt row
	DrawText(screen, "换挡: [Shift/Ctrl]升降 [R][N][G]切挡", dashX+114, dashY+38, 10, RGBA(148, 163, 184, 220))

	// Health and Steering info row
	durCol := RGBA(34, 197, 94, 255)
	if bus.Durability < 35 {
		durCol = RGBA(239, 68, 68, 255)
	} else if bus.Durability < 70 {
		durCol = RGBA(234, 179, 8, 255)
	}
	durStr := fmt.Sprintf("车况:%.0f%%", bus.Durability)
	DrawText(screen, durStr, dashX+114, dashY+58, 10, durCol)

	if bus.OnShoulder {
		vector.DrawFilledRect(screen, float32(dashX+182), float32(dashY+54), 126, 18, RGBA(234, 179, 8, 220), true)
		DrawText(screen, "[ 路肩缓冲减速 ]", dashX+190, dashY+57, 10, RGBA(15, 23, 42, 255))
	} else {
		steerStr := fmt.Sprintf("转向: %.0f°", bus.SteeringAngle*180/math.Pi)
		DrawText(screen, steerStr, dashX+182, dashY+58, 10, RGBA(203, 213, 225, 255))
		steerRatio := bus.SteeringAngle / bus.MaxSteerAngle
		vector.StrokeLine(screen, float32(dashX+240), float32(dashY+63), float32(dashX+240+steerRatio*20), float32(dashY+63), 2.0, RGBA(56, 189, 248, 255), true)
	}

	// Prompt Banners at Bottom Center
	if activeStop != nil {
		promptW := 420.0
		promptH := 46.0
		promptX := float64(screen.Bounds().Dx())/2 - promptW/2
		promptY := float64(screenH) - 80.0

		vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			RGBA(30, 41, 59, 240), true)
		vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			2, RGBA(250, 204, 21, 255), true)

		promptText := fmt.Sprintf("已停靠在 [%s]! 按 [E] 开门上下客", activeStop.Name)
		if bus.DoorsOpen {
			promptText = fmt.Sprintf("上下客完成! 按 [E] 关门准备发车 >>")
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

				promptText := fmt.Sprintf("【满足飞站条件】[%s] 站台无人且无下车铃！按 [F] 飞站 ⏩ (或直接驶过)", targetStop.Name)
				DrawText(screen, promptText, promptX+16, promptY+15, 13, RGBA(209, 250, 229, 255))
			} else {
				// Must Stop
				promptW := 530.0
				promptX := float64(screen.Bounds().Dx())/2 - promptW/2

				vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					RGBA(30, 41, 59, 235), true)
				vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					2, RGBA(234, 179, 8, 255), true)

				promptText := fmt.Sprintf("【须靠站】[%s] 站台 %d 人候车 / 车内 %d 人下车，请减速靠站",
					targetStop.Name, targetStop.WaitingPassengers, passengersAlighting)
				DrawText(screen, promptText, promptX+16, promptY+15, 13, RGBA(254, 240, 138, 255))
			}
		}
	}
}

func (h *HUD) drawPlanningInstructions(screen *ebiten.Image, route *Route, world *World) {
	screenH := screen.Bounds().Dy()
	screenW := screen.Bounds().Dx()

	// Planning helper card at bottom center
	panelW := 680.0
	panelH := 88.0
	panelX := float64(screenW)/2 - panelW/2
	panelY := float64(screenH) - 106.0

	vector.DrawFilledRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		RGBA(15, 23, 42, 230), true)
	vector.StrokeRect(screen, float32(panelX), float32(panelY), float32(panelW), float32(panelH),
		1.5, RGBA(168, 85, 247, 240), true)

	DrawText(screen, "【线路规划面板】鼠标左键点击任意站点图标将其加入行驶路线", panelX+20, panelY+12, 13, RGBA(192, 132, 252, 255))
	DrawText(screen, "【靠站规则】公交车站均设于道路顺行右侧，系统已自动按顺行右侧靠站规划最佳街道路线", panelX+20, panelY+30, 11, RGBA(250, 204, 21, 255))
	routeDesc := fmt.Sprintf("当前路线: %s", route.GetRouteDescription(world))
	DrawText(screen, routeDesc, panelX+20, panelY+48, 12, color.White)
	DrawText(screen, "[C键] 清空路线重新规划  |  [TAB键] 返回驾驶并执行当前路线", panelX+20, panelY+68, 12, RGBA(148, 163, 184, 255))
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
