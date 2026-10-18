package ui

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
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
	Toasts                []ToastMessage
	GearRBounds           sim.Rect
	GearNBounds           sim.Rect
	GearDBounds           sim.Rect
	AutopilotBounds       sim.Rect
	CameraBtnBounds       sim.Rect
	MapThemeBtnBounds     sim.Rect
	MenuBtnBounds         sim.Rect
	PlanUndoBtnBounds     sim.Rect
	PlanClearBtnBounds    sim.Rect
	PlanPanelBounds       sim.Rect
	ReturnToMenuRequested bool
	TargetStopOccupied    bool
}

func NewHUD() *HUD {
	return &HUD{
		Toasts: make([]ToastMessage, 0),
	}
}

// HandleModeBtnClick detects mouse click on the top-bar mode badge (Drive Mode <-> Plan Mode)
func (h *HUD) HandleModeBtnClick(mousePos sim.Vec2) bool {
	// Top-left mode badge is at x: 16 ~ 236, y: 12 ~ 44
	return mousePos.X >= 16 && mousePos.X <= 236 && mousePos.Y >= 12 && mousePos.Y <= 44
}

// HandleMenuBtnClick detects mouse click on the top-bar main menu / map change button
func (h *HUD) HandleMenuBtnClick(mousePos sim.Vec2) bool {
	if h.MenuBtnBounds.Contains(mousePos) || h.MapThemeBtnBounds.Contains(mousePos) {
		h.ReturnToMenuRequested = true
		return true
	}
	return false
}

// HandleMapThemeBtnClick detects mouse click on the top-bar map theme button
func (h *HUD) HandleMapThemeBtnClick(mousePos sim.Vec2, settings *Settings) bool {
	return h.HandleMenuBtnClick(mousePos)
}

// HandlePlanUndoClick detects mouse click on the undo stop button in route planning mode
func (h *HUD) HandlePlanUndoClick(mousePos sim.Vec2) bool {
	return h.PlanUndoBtnBounds.Contains(mousePos)
}

// HandlePlanClearClick detects mouse click on the clear route button in route planning mode
func (h *HUD) HandlePlanClearClick(mousePos sim.Vec2) bool {
	return h.PlanClearBtnBounds.Contains(mousePos)
}

// IsInsidePlanPanel returns true if the mouse position is inside the planning panel
func (h *HUD) IsInsidePlanPanel(mousePos sim.Vec2) bool {
	return h.PlanPanelBounds.Contains(mousePos)
}

// HandleCameraBtnClick detects mouse click on the top-bar camera perspective button
func (h *HUD) HandleCameraBtnClick(mousePos sim.Vec2, cam *sim.Camera) (toggled bool, notice string) {
	if cam == nil {
		return false, ""
	}
	if h.CameraBtnBounds.Contains(mousePos) {
		newMode := cam.ToggleMode()
		switch newMode {
		case sim.CameraModeHeadingUp:
			return true, locale.T("已切换视角: 【车头朝上 (跟随视角)】 - 视野随转向旋转", "Camera Mode: [Heading-Up Follow] - Rotates with steering")
		case sim.CameraModeSouthUp:
			return true, locale.T("已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上", "Camera Mode: [Fixed South-Up] - Map fixed south")
		default:
			return true, locale.T("已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上", "Camera Mode: [Fixed North-Up] - Standard map north")
		}
	}
	return false, ""
}

// HandleGearClick detects mouse click on the dashboard gear selector badges
func (h *HUD) HandleGearClick(mousePos sim.Vec2, bus *sim.Bus) (shifted bool, notice string) {
	if bus == nil {
		return false, ""
	}
	if h.GearRBounds.Contains(mousePos) {
		if bus.ShiftTo(sim.GearReverse) {
			return true, locale.T("已挂入倒挡 (R) [按 W/↑ 倒车，S/↓ 刹车]", "Shifted to Reverse (R) [W/↑ Reverse, S/↓ Brake]")
		}
		return false, locale.T("车速过快，变速箱保护锁止！请先踩刹车减速", "Speed too high! Gearbox lock active. Please brake first")
	} else if h.GearNBounds.Contains(mousePos) {
		if bus.ShiftTo(sim.GearNeutral) {
			return true, locale.T("已挂入空挡 (N)", "Shifted to Neutral (N)")
		}
	} else if h.GearDBounds.Contains(mousePos) {
		if bus.ShiftTo(sim.GearDrive) {
			return true, locale.T("已挂入前进挡 (D)", "Shifted to Drive (D)")
		}
		return false, locale.T("倒车速度过快，无法切入前进挡！请先踩刹车减速", "Reverse speed too high! Cannot shift to D. Please brake first")
	}
	return false, ""
}

// HandleAutopilotClick detects mouse click on the dashboard autopilot button
func (h *HUD) HandleAutopilotClick(mousePos sim.Vec2, settings *Settings) (toggled bool, notice string) {
	if settings == nil {
		return false, ""
	}
	if h.AutopilotBounds.Contains(mousePos) {
		settings.AutopilotEnabled = !settings.AutopilotEnabled
		if settings.AutopilotEnabled {
			if settings.AutopilotBerserk {
				return true, fmt.Sprintf(locale.T("已开启 AI 狂暴模式 (最高%d km/h, P键关闭/Shift+B切速/B键切标准)", "AI Berserk Mode ON (%d km/h, P to exit/Shift+B for speed/B for Standard)"), settings.AutopilotBerserkMaxSpeed)
			}
			return true, locale.T("已开启 AI 自动巡航驾驶 (P键随时切换, B键开启狂暴模式)", "AI Autopilot ON (Press P anytime, B for Berserk)")
		}
		return true, locale.T("已切换回手动驾驶模式 (P键随时切换)", "Manual Driving Mode ON (Press P anytime to toggle)")
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
