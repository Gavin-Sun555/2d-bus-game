package game

import (
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func TestHUDDisplaysSmallVsLargeRoadNetworkAccurately(t *testing.T) {
	g := NewGame(1440, 900)

	// Case 1: Small road network (default) in Coastal Bay
	g.Settings.SetLargeRoadNetwork(sim.MapThemeCoastalBay, false)
	g.EnterCity(sim.MapThemeCoastalBay)

	if g.World.LargeGrid {
		t.Fatalf("Expected Coastal Bay LargeGrid=false")
	}

	screen := ebiten.NewImage(1440, 900)
	g.Draw(screen)

	// In Coastal Bay small road network, HUD menu button must not say (大路网), should say (小路网)
	btnText := g.HUD.MenuBtnString()
	if strings.Contains(btnText, "大路网") {
		t.Fatalf("Expected HUD button NOT to contain '大路网' for small grid, got: %s", btnText)
	}
	if !strings.Contains(btnText, "小路网") {
		t.Fatalf("Expected HUD button to contain '小路网', got: %s", btnText)
	}

	// Case 2: Large road network in Metropolis
	g.Settings.SetLargeRoadNetwork(sim.MapThemeMetropolis, true)
	g.EnterCity(sim.MapThemeMetropolis)

	if !g.World.LargeGrid {
		t.Fatalf("Expected Metropolis LargeGrid=true")
	}

	g.Draw(screen)
	btnText = g.HUD.MenuBtnString()
	if !strings.Contains(btnText, "大路网") {
		t.Fatalf("Expected HUD button to contain '大路网' for large grid, got: %s", btnText)
	}
	if strings.Contains(btnText, "小路网") {
		t.Fatalf("Expected HUD button NOT to contain '小路网' for large grid, got: %s", btnText)
	}
}

func TestSavedSlot2DisplaysSmallRoadNetwork(t *testing.T) {
	savePath := findSlot2Jam1814File()
	if savePath == "" {
		t.Skip("slot2 snapshot not found; skipping")
	}

	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	if g.World.LargeGrid {
		t.Fatalf("Expected slot2 snapshot World.LargeGrid to be false")
	}

	screen := ebiten.NewImage(1440, 900)
	g.Draw(screen)

	btnText := g.HUD.MenuBtnString()
	if strings.Contains(btnText, "大路网") {
		t.Fatalf("Expected slot2 HUD button NOT to say '大路网', got: %s", btnText)
	}
	if !strings.Contains(btnText, "小路网") {
		t.Fatalf("Expected slot2 HUD button to say '小路网', got: %s", btnText)
	}
}

func TestMenuIndependentLargeRoadNetworkPerMap(t *testing.T) {
	g := NewGame(1440, 900)
	g.State = StateMainMenu

	screen := ebiten.NewImage(1440, 900)
	g.Draw(screen)

	// Initially, all maps are small road network (false)
	for _, th := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeTwinCities} {
		if g.Settings.IsLargeRoadNetwork(th) {
			t.Fatalf("Expected theme %v initially IsLargeRoadNetwork=false", th)
		}
	}

	// Click Card 1 (Coastal Bay) large grid button
	b1 := g.menuCardLargeGridBounds[1]
	mPos1 := sim.Vec2{X: b1.X + b1.W/2, Y: b1.Y + b1.H/2}
	// Simulate click
	g.Settings.ToggleLargeRoadNetwork(sim.MapThemeCoastalBay)
	g.SelectedMapTheme = sim.MapThemeCoastalBay
	_ = mPos1

	// Verify ONLY Coastal Bay is large road network
	if !g.Settings.IsLargeRoadNetwork(sim.MapThemeCoastalBay) {
		t.Fatalf("Expected Coastal Bay to be Large Grid")
	}
	if g.Settings.IsLargeRoadNetwork(sim.MapThemeMetropolis) {
		t.Fatalf("Expected Metropolis to still be Small Grid")
	}
	if g.Settings.IsLargeRoadNetwork(sim.MapThemeRiverValley) {
		t.Fatalf("Expected River Valley to still be Small Grid")
	}
	if g.Settings.IsLargeRoadNetwork(sim.MapThemeTwinCities) {
		t.Fatalf("Expected Twin Cities to still be Small Grid")
	}

	// Now press [G] while Coastal Bay is selected -> toggles back to false
	g.Settings.ToggleLargeRoadNetwork(g.SelectedMapTheme)
	if g.Settings.IsLargeRoadNetwork(sim.MapThemeCoastalBay) {
		t.Fatalf("Expected Coastal Bay to toggle back to Small Grid via [G]")
	}

	// Select Metropolis and toggle [G] -> only Metropolis is true
	g.SelectedMapTheme = sim.MapThemeMetropolis
	g.Settings.ToggleLargeRoadNetwork(g.SelectedMapTheme)
	if !g.Settings.IsLargeRoadNetwork(sim.MapThemeMetropolis) {
		t.Fatalf("Expected Metropolis to be Large Grid")
	}
	if g.Settings.IsLargeRoadNetwork(sim.MapThemeRiverValley) {
		t.Fatalf("Expected River Valley to remain Small Grid")
	}

	// Entering Metropolis should yield LargeGrid=true
	g.EnterCity(sim.MapThemeMetropolis)
	if !g.World.LargeGrid {
		t.Fatalf("Expected Metropolis World.LargeGrid=true")
	}

	// Returning to menu and entering River Valley should yield LargeGrid=false
	g.State = StateMainMenu
	g.EnterCity(sim.MapThemeRiverValley)
	if g.World.LargeGrid {
		t.Fatalf("Expected River Valley World.LargeGrid=false")
	}
}
