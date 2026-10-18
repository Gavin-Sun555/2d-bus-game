package ui

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestHUDInteractionHandlers(t *testing.T) {
	hud := NewHUD()
	cam := sim.NewCamera(1280, 720)
	bus := sim.NewBus(0, 0, 0)
	bus.Gear = sim.GearNeutral

	// 1. Menu Button Click
	hud.MenuBtnBounds = sim.Rect{X: 10, Y: 10, W: 60, H: 30}
	if !hud.HandleMenuBtnClick(sim.Vec2{X: 25, Y: 25}) {
		t.Errorf("HandleMenuBtnClick should return true when clicking inside bounds")
	}
	if !hud.ReturnToMenuRequested {
		t.Errorf("ReturnToMenuRequested should be true")
	}
	if hud.HandleMenuBtnClick(sim.Vec2{X: 500, Y: 500}) {
		t.Errorf("HandleMenuBtnClick should return false when clicking outside")
	}

	// 2. Camera Toggle Button Click
	hud.CameraBtnBounds = sim.Rect{X: 80, Y: 10, W: 60, H: 30}
	toggled, notice := hud.HandleCameraBtnClick(sim.Vec2{X: 95, Y: 25}, cam)
	if !toggled || notice == "" {
		t.Errorf("HandleCameraBtnClick should toggle camera mode and return notice")
	}

	// 3. Gear Badges Click
	hud.GearDBounds = sim.Rect{X: 200, Y: 600, W: 40, H: 40}
	shifted, gearNotice := hud.HandleGearClick(sim.Vec2{X: 220, Y: 620}, bus)
	if !shifted || bus.Gear != sim.GearDrive || gearNotice == "" {
		t.Errorf("HandleGearClick on D badge should shift bus to Drive, got %s", bus.Gear)
	}

	hud.GearNBounds = sim.Rect{X: 250, Y: 600, W: 40, H: 40}
	shiftedN, _ := hud.HandleGearClick(sim.Vec2{X: 270, Y: 620}, bus)
	if !shiftedN || bus.Gear != sim.GearNeutral {
		t.Errorf("HandleGearClick on N badge should shift bus to Neutral, got %s", bus.Gear)
	}

	// 4. Toast Cap: max 4 toasts
	for i := 0; i < 10; i++ {
		hud.AddToast("Test Toast", true)
	}
	if len(hud.Toasts) > 4 {
		t.Errorf("HUD toast list exceeded maximum display cap of 4, got %d", len(hud.Toasts))
	}
}

func TestSettingsInteractionAndToggles(t *testing.T) {
	s := NewSettings()

	// Toggle Open/Close
	if s.IsOpen {
		t.Errorf("Settings should be closed by default")
	}
	s.Toggle()
	if !s.IsOpen {
		t.Errorf("Settings should be open after Toggle")
	}
	s.Toggle()
	if s.IsOpen {
		t.Errorf("Settings should be closed after second Toggle")
	}

	// Lane width is fixed to 52.0 (宽敞市区 Comfortable Wide) as default
	if s.GetLaneWidth() != 52.0 {
		t.Errorf("Expected fixed lane width of 52.0 (宽敞市区), got %.1f", s.GetLaneWidth())
	}
}
