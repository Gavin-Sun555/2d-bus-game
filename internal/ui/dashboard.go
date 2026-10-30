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

func (h *HUD) drawDriveDashboard(screen *ebiten.Image, bus *sim.Bus, activeStop *sim.BusStop, targetStop *sim.BusStop, stopBellRung bool, passengersAlighting int, tls *sim.TrafficLightSystem, settings *Settings) {
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
		hasInter, thruSig, leftSig, isMajor, remainingSec, dist := tls.GetApproachingSignalInfo(bus.Pos, bus.Heading, lw)
		if hasInter && dist < 240.0 {
			sigBoxW := float32(350.0) // Align cleanly with 350px speedometer dashboard below
			sigBoxH := float32(38.0)
			sigBoxY := dashY - float64(sigBoxH) - 8.0 // 8px breathing space above dashboard
			sigYF32 := float32(sigBoxY)

			// --- Sleek Glassmorphism Panel (毛玻璃悬浮状态栏) ---
			// 1. Soft ambient drop shadow
			vector.DrawFilledRect(screen, float32(dashX-2), sigYF32-2, sigBoxW+4, sigBoxH+4, sim.NRGBA(2, 6, 23, 140), true)
			// 2. Translucent deep-slate backdrop
			vector.DrawFilledRect(screen, float32(dashX), sigYF32, sigBoxW, sigBoxH, sim.NRGBA(15, 23, 42, 235), true)
			// 3. Subtle dark slate outer border
			vector.StrokeRect(screen, float32(dashX), sigYF32, sigBoxW, sigBoxH, 1.2, sim.NRGBA(71, 85, 105, 180), true)
			// 4. Elegant top-edge specular highlight rim (1px line, non-intrusive)
			vector.StrokeLine(screen, float32(dashX)+2, sigYF32+1, float32(dashX)+sigBoxW-2, sigYF32+1, 1.0, sim.NRGBA(255, 255, 255, 60), true)

			lampY := sigYF32 + 19.0

			// Section 3: Distance to Stop Line & Phase Countdown (common to both major & minor)
			distCol := sim.RGBA(203, 213, 225, 255)
			if dist < 25.0 {
				distCol = sim.RGBA(250, 204, 21, 255) // Caution highlight when close to stop line
			}
			distText := locale.Tf("距线 %.0fm", "%.0fm to line", math.Max(0, dist))
			if remainingSec > 0 && remainingSec < 99 {
				distText = locale.Tf("距线 %.0fm (%ds)", "%.0fm (%ds)", math.Max(0, dist), int(remainingSec+0.5))
			}

			// Draw mini stop-line icon
			vector.DrawFilledRect(screen, float32(dashX+242), sigYF32+12, 2.5, 14, sim.RGBA(248, 250, 252, 240), true)
			vector.StrokeLine(screen, float32(dashX+238), sigYF32+19, float32(dashX+242), sigYF32+19, 1.5, sim.RGBA(148, 163, 184, 200), true)
			textui.DrawText(screen, distText, dashX+250, sigBoxY+13, 11, distCol)

			if isMajor {
				// Section dividers
				vector.StrokeLine(screen, float32(dashX+116), sigYF32+7, float32(dashX+116), sigYF32+sigBoxH-7, 1.0, sim.NRGBA(71, 85, 105, 160), true)
				vector.StrokeLine(screen, float32(dashX+232), sigYF32+7, float32(dashX+232), sigYF32+sigBoxH-7, 1.0, sim.NRGBA(71, 85, 105, 160), true)

				// === 1. Left-Turn Lane Indicator (左转道信号) ===
				lDot := sim.RGBA(248, 40, 40, 255)
				lCol := sim.RGBA(248, 113, 113, 255)
				lState := locale.T("红灯", "RED")
				if leftSig == sim.SignalYellow {
					lDot = sim.RGBA(250, 204, 21, 255)
					lCol = sim.RGBA(250, 204, 21, 255)
					lState = locale.T("黄灯", "YEL")
				} else if leftSig == sim.SignalGreen {
					lDot = sim.RGBA(34, 197, 94, 255)
					lCol = sim.RGBA(74, 222, 128, 255)
					lState = locale.T("绿灯", "GRN")
				}

				// Left lamp halo & circle
				vector.DrawFilledCircle(screen, float32(dashX+17), lampY, 7.5, sim.NRGBA(lDot.R, lDot.G, lDot.B, 65), true)
				vector.DrawFilledCircle(screen, float32(dashX+17), lampY, 4.5, lDot, true)
				vector.StrokeCircle(screen, float32(dashX+17), lampY, 4.5, 0.8, sim.NRGBA(255, 255, 255, 80), true)
				// Crisp vector left-turn arrow (↰)
				drawMiniTurnArrow(screen, float32(dashX+30), lampY, lDot)
				// Labels
				textui.DrawText(screen, locale.T("左转", "Left"), dashX+40, sigBoxY+13, 11, sim.RGBA(226, 232, 240, 255))
				textui.DrawText(screen, lState, dashX+68, sigBoxY+13, 11, lCol)

				// === 2. Straight Through Lane Indicator (直行道信号) ===
				tDot := sim.RGBA(248, 40, 40, 255)
				tCol := sim.RGBA(248, 113, 113, 255)
				tState := locale.T("红灯", "RED")
				if thruSig == sim.SignalYellow {
					tDot = sim.RGBA(250, 204, 21, 255)
					tCol = sim.RGBA(250, 204, 21, 255)
					tState = locale.T("黄灯", "YEL")
				} else if thruSig == sim.SignalGreen {
					tDot = sim.RGBA(34, 197, 94, 255)
					tCol = sim.RGBA(74, 222, 128, 255)
					tState = locale.T("绿灯", "GRN")
				}

				// Straight lamp halo & circle
				vector.DrawFilledCircle(screen, float32(dashX+130), lampY, 7.5, sim.NRGBA(tDot.R, tDot.G, tDot.B, 65), true)
				vector.DrawFilledCircle(screen, float32(dashX+130), lampY, 4.5, tDot, true)
				vector.StrokeCircle(screen, float32(dashX+130), lampY, 4.5, 0.8, sim.NRGBA(255, 255, 255, 80), true)
				// Crisp vector straight arrow (↑)
				drawMiniStraightArrow(screen, float32(dashX+143), lampY, tDot)
				// Labels
				textui.DrawText(screen, locale.T("直行", "Thru"), dashX+153, sigBoxY+13, 11, sim.RGBA(226, 232, 240, 255))
				textui.DrawText(screen, tState, dashX+181, sigBoxY+13, 11, tCol)
			} else {
				// Section dividers
				vector.StrokeLine(screen, float32(dashX+70), sigYF32+7, float32(dashX+70), sigYF32+sigBoxH-7, 1.0, sim.NRGBA(71, 85, 105, 160), true)
				vector.StrokeLine(screen, float32(dashX+232), sigYF32+7, float32(dashX+232), sigYF32+sigBoxH-7, 1.0, sim.NRGBA(71, 85, 105, 160), true)

				// Standard 3-lamp housing for minor intersection
				vector.DrawFilledRect(screen, float32(dashX+10), sigYF32+10, 52, 18, sim.NRGBA(15, 23, 42, 255), true)
				vector.StrokeRect(screen, float32(dashX+10), sigYF32+10, 52, 18, 1.0, sim.NRGBA(51, 65, 85, 255), true)

				rCol := sim.RGBA(60, 20, 20, 255)
				yCol := sim.RGBA(60, 50, 15, 255)
				gCol := sim.RGBA(15, 50, 25, 255)

				statusLabel := ""
				statusCol := color.RGBA{R: 255, G: 255, B: 255, A: 255}
				switch thruSig {
				case sim.SignalRed:
					rCol = sim.RGBA(248, 40, 40, 255)
					statusLabel = locale.T("单车道混合: 红灯等待", "Single Lane: RED (Stop)")
					statusCol = sim.RGBA(248, 113, 113, 255)
					vector.DrawFilledCircle(screen, float32(dashX+19), lampY, 7.5, sim.NRGBA(rCol.R, rCol.G, rCol.B, 65), true)
				case sim.SignalYellow:
					yCol = sim.RGBA(250, 204, 21, 255)
					statusLabel = locale.T("单车道混合: 黄灯减速", "Single Lane: YELLOW (Wait)")
					statusCol = sim.RGBA(250, 204, 21, 255)
					vector.DrawFilledCircle(screen, float32(dashX+36), lampY, 7.5, sim.NRGBA(yCol.R, yCol.G, yCol.B, 65), true)
				case sim.SignalGreen:
					gCol = sim.RGBA(34, 197, 94, 255)
					statusLabel = locale.T("单车道混合: 准许通行", "Single Lane: GREEN (Proceed)")
					statusCol = sim.RGBA(74, 222, 128, 255)
					vector.DrawFilledCircle(screen, float32(dashX+53), lampY, 7.5, sim.NRGBA(gCol.R, gCol.G, gCol.B, 65), true)
				}

				vector.DrawFilledCircle(screen, float32(dashX+19), lampY, 4.5, rCol, true)
				vector.DrawFilledCircle(screen, float32(dashX+36), lampY, 4.5, yCol, true)
				vector.DrawFilledCircle(screen, float32(dashX+53), lampY, 4.5, gCol, true)

				textui.DrawText(screen, statusLabel, dashX+78, sigBoxY+13, 11, statusCol)
			}
		}
	}

	// Dark round cluster background
	vector.DrawFilledRect(screen, float32(dashX), float32(dashY), 350, 80, sim.RGBA(15, 23, 42, 230), true)
	vector.StrokeRect(screen, float32(dashX), float32(dashY), 350, 80, 1.5, sim.RGBA(51, 65, 85, 255), true)

	// Speed calculation (convert px/s to intuitive km/h)
	speedKmh := int(math.Abs(bus.Speed) * 0.18)
	speedStr := fmt.Sprintf("%3d", speedKmh)
	textui.DrawText(screen, speedStr, dashX+14, dashY+16, 16, color.White)
	textui.DrawText(screen, "KM/H", dashX+52, dashY+20, 11, sim.RGBA(148, 163, 184, 255))

	// Speed bar
	barRatio := math.Min(1.0, math.Abs(bus.Speed)/math.Max(1.0, bus.MaxSpeedFwd))
	barColor := sim.RGBA(34, 197, 94, 255)
	if barRatio > 0.75 {
		barColor = sim.RGBA(239, 68, 68, 255)
	} else if barRatio > 0.45 {
		barColor = sim.RGBA(234, 179, 8, 255)
	}
	vector.DrawFilledRect(screen, float32(dashX+14), float32(dashY+48), float32(80*barRatio), 7, barColor, true)
	vector.StrokeRect(screen, float32(dashX+14), float32(dashY+48), 80, 7, 1, sim.RGBA(100, 116, 139, 200), true)

	// Vertical cluster divider
	vector.StrokeLine(screen, float32(dashX+104), float32(dashY+8), float32(dashX+104), float32(dashY+72), 1, sim.RGBA(51, 65, 85, 200), true)

	// Interactive Gear Selector Badges: [ R ] [ N ] [ D ]
	btnW := 24.0
	btnH := 20.0
	h.GearRBounds = sim.Rect{X: dashX + 114, Y: dashY + 12, W: btnW, H: btnH}
	h.GearNBounds = sim.Rect{X: dashX + 142, Y: dashY + 12, W: btnW, H: btnH}
	h.GearDBounds = sim.Rect{X: dashX + 170, Y: dashY + 12, W: btnW, H: btnH}

	// 1. R (Reverse)
	if bus.Gear == sim.GearReverse {
		vector.DrawFilledRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), sim.RGBA(239, 68, 68, 240), true)
		vector.StrokeRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), 1.5, sim.RGBA(254, 202, 202, 255), true)
		textui.DrawText(screen, "R", h.GearRBounds.X+7, h.GearRBounds.Y+4, 12, color.White)
	} else {
		vector.DrawFilledRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), sim.RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearRBounds.X), float32(h.GearRBounds.Y), float32(btnW), float32(btnH), 1.0, sim.RGBA(71, 85, 105, 180), true)
		textui.DrawText(screen, "R", h.GearRBounds.X+7, h.GearRBounds.Y+4, 12, sim.RGBA(148, 163, 184, 255))
	}

	// 2. N (Neutral)
	if bus.Gear == sim.GearNeutral {
		vector.DrawFilledRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), sim.RGBA(234, 179, 8, 240), true)
		vector.StrokeRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), 1.5, sim.RGBA(254, 240, 138, 255), true)
		textui.DrawText(screen, "N", h.GearNBounds.X+7, h.GearNBounds.Y+4, 12, sim.RGBA(15, 23, 42, 255))
	} else {
		vector.DrawFilledRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), sim.RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearNBounds.X), float32(h.GearNBounds.Y), float32(btnW), float32(btnH), 1.0, sim.RGBA(71, 85, 105, 180), true)
		textui.DrawText(screen, "N", h.GearNBounds.X+7, h.GearNBounds.Y+4, 12, sim.RGBA(148, 163, 184, 255))
	}

	// 3. D (Drive / Forward)
	if bus.Gear == sim.GearDrive || bus.Gear >= sim.Gear1 {
		vector.DrawFilledRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), sim.RGBA(34, 197, 94, 240), true)
		vector.StrokeRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), 1.5, sim.RGBA(187, 247, 208, 255), true)
		label := bus.Gear.String()
		textui.DrawText(screen, label, h.GearDBounds.X+7, h.GearDBounds.Y+4, 12, color.White)
	} else {
		vector.DrawFilledRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), sim.RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.GearDBounds.X), float32(h.GearDBounds.Y), float32(btnW), float32(btnH), 1.0, sim.RGBA(71, 85, 105, 180), true)
		textui.DrawText(screen, "D", h.GearDBounds.X+7, h.GearDBounds.Y+4, 12, sim.RGBA(148, 163, 184, 255))
	}

	// Status text next to gear buttons
	statusStr := bus.Gear.Name()
	statusCol := sim.RGBA(74, 222, 128, 255)
	if bus.DoorsOpen {
		statusStr = locale.T("【开门P】", "[DOOR P]")
		statusCol = sim.RGBA(250, 204, 21, 255)
	} else if bus.IsHandbraking {
		statusStr = locale.T("【!手刹!】", "[!PARK!]")
		statusCol = sim.RGBA(239, 68, 68, 255)
	} else if bus.IsBraking {
		statusStr = locale.T("【脚刹】", "[BRAKE]")
		statusCol = sim.RGBA(248, 113, 113, 255)
	} else if bus.Gear == sim.GearReverse {
		statusCol = sim.RGBA(248, 113, 113, 255)
	} else if bus.Gear == sim.GearNeutral {
		statusCol = sim.RGBA(250, 204, 21, 255)
	}
	textui.DrawText(screen, statusStr, dashX+200, dashY+16, 11, statusCol)

	// Interactive AI Autopilot Badge: [⚡ 狂暴80(P)] / [🤖 AI巡航(P)] / [🤖 手动(P)]
	apW := 96.0
	apH := 20.0
	h.AutopilotBounds = sim.Rect{X: dashX + 246, Y: dashY + 12, W: apW, H: apH}
	if settings != nil && settings.AutopilotEnabled {
		if settings.AutopilotBerserk {
			vector.DrawFilledRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), sim.RGBA(234, 88, 12, 240), true)
			vector.StrokeRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), 1.5, sim.RGBA(253, 186, 116, 255), true)
			bskText := fmt.Sprintf(locale.T("⚡ 狂暴%d(P)", "⚡ BSK%d(P)"), settings.AutopilotBerserkMaxSpeed)
			textui.DrawText(screen, bskText, h.AutopilotBounds.X+6, h.AutopilotBounds.Y+4, 11, color.White)
		} else {
			vector.DrawFilledRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), sim.RGBA(6, 182, 212, 235), true)
			vector.StrokeRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), 1.5, sim.RGBA(165, 243, 252, 255), true)
			textui.DrawText(screen, locale.T("🤖 AI巡航(P)", "🤖 Autopilot(P)"), h.AutopilotBounds.X+8, h.AutopilotBounds.Y+4, 11, sim.RGBA(15, 23, 42, 255))
		}
	} else {
		vector.DrawFilledRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), sim.RGBA(30, 41, 59, 200), true)
		vector.StrokeRect(screen, float32(h.AutopilotBounds.X), float32(h.AutopilotBounds.Y), float32(apW), float32(apH), 1.0, sim.RGBA(100, 116, 139, 180), true)
		manText := locale.T("👤 手动(P)", "👤 Manual(P)")
		if settings != nil {
			manText = fmt.Sprintf(locale.T("👤 手动%d(P)", "👤 Man%d(P)"), settings.AutopilotBerserkMaxSpeed)
		}
		textui.DrawText(screen, manText, h.AutopilotBounds.X+8, h.AutopilotBounds.Y+4, 11, sim.RGBA(148, 163, 184, 255))
	}

	// Gear shortcut prompt row
	textui.DrawText(screen, locale.T("换挡: [Shift/Ctrl]升降 [R/N/G]切挡 [P]AI [B]狂暴 [Shift+B]切速", "Shift: [Shift/Ctrl] [R/N/G] [P]AI [B]Berserk [Shift+B]Speed"), dashX+70, dashY+38, 10, sim.RGBA(148, 163, 184, 220))

	// Health and Steering info row
	textui.DrawText(screen, locale.T("[H]长鸣 [F8]静音", "[H]Hold [F8]Mute"), dashX+12, dashY+60, 9, sim.RGBA(148, 163, 184, 220))
	durCol := sim.RGBA(34, 197, 94, 255)
	if bus.Durability < 35 {
		durCol = sim.RGBA(239, 68, 68, 255)
	} else if bus.Durability < 70 {
		durCol = sim.RGBA(234, 179, 8, 255)
	}
	durStr := locale.Tf("车况:%.0f%%", "Health:%.0f%%", bus.Durability)
	textui.DrawText(screen, durStr, dashX+114, dashY+58, 10, durCol)

	if bus.OnShoulder {
		vector.DrawFilledRect(screen, float32(dashX+182), float32(dashY+54), 126, 18, sim.RGBA(234, 179, 8, 220), true)
		textui.DrawText(screen, locale.T("[ 路肩缓冲减速 ]", "[ Shoulder Slow ]"), dashX+190, dashY+57, 10, sim.RGBA(15, 23, 42, 255))
	} else {
		steerStr := locale.Tf("转向: %.0f°", "Steer: %.0f°", bus.SteeringAngle*180/math.Pi)
		textui.DrawText(screen, steerStr, dashX+182, dashY+58, 10, sim.RGBA(203, 213, 225, 255))
		steerRatio := bus.SteeringAngle / bus.MaxSteerAngle
		vector.StrokeLine(screen, float32(dashX+240), float32(dashY+63), float32(dashX+240+steerRatio*20), float32(dashY+63), 2.0, sim.RGBA(56, 189, 248, 255), true)
	}

	// Prompt Banners at Bottom Center
	if activeStop != nil {
		promptW := 440.0
		promptH := 46.0
		promptX := float64(screen.Bounds().Dx())/2 - promptW/2
		promptY := float64(screenH) - 80.0

		vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			sim.RGBA(30, 41, 59, 240), true)
		vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
			2, sim.RGBA(250, 204, 21, 255), true)

		promptText := locale.Tf("已停靠在 [%s]! 按 [E] 开门上下客", "Stopped at [%s]! Press [E] to open doors", activeStop.LocalizedName())
		if bus.DoorsOpen {
			promptText = locale.T("上下客完成! 按 [E] 关门准备发车 >>", "Boarding complete! Press [E] to close doors >>")
			if remaining := bus.PassengerServiceRemaining(); remaining > 0 {
			promptText = locale.Tf("上下客中，还需约 %.0f 秒，暂不能关门", "Passenger exchange: %.0f s left; keep doors open", math.Ceil(remaining))
			}
		}
		textui.DrawText(screen, promptText, promptX+20, promptY+15, 13, sim.RGBA(250, 204, 21, 255))
	} else if targetStop != nil {
		dist := bus.Pos.Distance(targetStop.Pos)
		if dist <= 380.0 {
			promptH := 46.0
			promptY := float64(screenH) - 80.0

			occupied := h.TargetStopOccupied
			canSkip := !stopBellRung && (targetStop.WaitingPassengers == 0 || occupied)
			if canSkip {
				// Eligible for Skip Stop (飞站)!
				promptW := 600.0
				promptX := float64(screen.Bounds().Dx())/2 - promptW/2

				vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					sim.RGBA(6, 78, 59, 235), true)
				vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					2, sim.RGBA(52, 211, 153, 255), true)

				var promptText string
				if occupied {
					promptText = locale.Tf("【满足飞站条件】[%s] 前方站台已有车靠站且无下车铃！按 [F] 飞站 ⏩ (或直接驶过)", "[Express Skip] [%s] Stop occupied by bus & no bell! Press [F] to skip ⏩", targetStop.LocalizedName())
				} else {
					promptText = locale.Tf("【满足飞站条件】[%s] 站台无人且无下车铃！按 [F] 飞站 ⏩ (或直接驶过)", "[Express Skip] [%s] No waiting pax & no bell! Press [F] to skip ⏩", targetStop.LocalizedName())
				}
				textui.DrawText(screen, promptText, promptX+16, promptY+15, 13, sim.RGBA(209, 250, 229, 255))
			} else {
				// Must Stop
				promptW := 550.0
				promptX := float64(screen.Bounds().Dx())/2 - promptW/2

				vector.DrawFilledRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					sim.RGBA(30, 41, 59, 235), true)
				vector.StrokeRect(screen, float32(promptX), float32(promptY), float32(promptW), float32(promptH),
					2, sim.RGBA(234, 179, 8, 255), true)

				bellStatus := locale.T("无人按铃", "No stop request")
				if stopBellRung {
					bellStatus = locale.T("下车铃已响", "Stop requested")
				}
				promptText := locale.Tf("【须靠站】[%s] 站台 %d 人候车 / %s，请减速靠站", "[Must Stop] [%s] %d waiting / %s, please decelerate", targetStop.LocalizedName(), targetStop.WaitingPassengers, bellStatus)
				textui.DrawText(screen, promptText, promptX+16, promptY+15, 13, sim.RGBA(254, 240, 138, 255))
			}
		}
	}
}
