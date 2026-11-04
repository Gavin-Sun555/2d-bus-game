package game

import (
	"math"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func TestGameMetropolisLargeRoadNetworkIntegration(t *testing.T) {
	g := NewGame(1440, 900)

	// 1. Initial defaults
	if g.Settings.LargeRoadNetwork {
		t.Fatalf("Expected default LargeRoadNetwork=false")
	}

	// 2. Main menu toggle via card button
	g.State = StateMainMenu
	screen := ebiten.NewImage(1440, 900)
	g.Draw(screen)

	if g.menuLargeGridBounds.W <= 0 || g.menuLargeGridBounds.H <= 0 {
		t.Fatalf("menuLargeGridBounds should be populated on Draw: %+v", g.menuLargeGridBounds)
	}
	for i := 0; i < 4; i++ {
		b := g.menuCardLargeGridBounds[i]
		if b.W <= 0 || b.H <= 0 {
			t.Fatalf("menuCardLargeGridBounds[%d] should be populated on Draw: %+v", i, b)
		}
	}

	// Simulate clicking the Large Grid button on Card 0
	g.Settings.SetLargeRoadNetwork(sim.MapThemeMetropolis, true)
	g.EnterCity(sim.MapThemeMetropolis)

	if !g.World.LargeGrid {
		t.Fatalf("Expected g.World.LargeGrid=true after EnterCity in LargeRoadNetwork mode")
	}
	if g.World.GridScale() != sim.MapGridScale*4.0 {
		t.Fatalf("Expected GridScale=%.1f, got %.1f", sim.MapGridScale*4.0, g.World.GridScale())
	}

	// Verify toast message on entering large grid Metropolis
	foundToast := false
	for _, toast := range g.HUD.Toasts {
		if strings.Contains(toast.Text, "大路网模式") {
			foundToast = true
			break
		}
	}
	if !foundToast {
		t.Fatalf("Expected toast mentioning '大路网模式', got: %+v", g.HUD.Toasts)
	}

	// 3. Verify HUD Top Bar badge text
	g.Draw(screen)
	// In Metropolis with LargeRoadNetwork, HUD menu button text should include "大路网"

	// 4. Verify Planning Mode Zoom
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatalf("Expected planning mode active")
	}
	if math.Abs(g.Camera.Zoom-0.012) > 0.001 {
		t.Fatalf("Expected planning mode initial zoom 0.012 for large grid, got %.4f", g.Camera.Zoom)
	}
	g.TogglePlanMode()

	// 5. Switch to Standard mode via Settings
	g.Settings.LargeRoadNetwork = false
	g.Settings.LargeRoadNetworkChanged = true
	g.SetMapTheme(sim.MapThemeMetropolis)

	if g.World.LargeGrid {
		t.Fatalf("Expected g.World.LargeGrid=false after switching back to standard")
	}
	if g.World.GridScale() != sim.MapGridScale {
		t.Fatalf("Expected GridScale=%.1f, got %.1f", sim.MapGridScale, g.World.GridScale())
	}

	// Planning mode zoom for standard grid
	g.TogglePlanMode()
	if math.Abs(g.Camera.Zoom-0.04) > 0.001 {
		t.Fatalf("Expected planning mode initial zoom 0.04 for standard grid, got %.4f", g.Camera.Zoom)
	}
	g.TogglePlanMode()

	// 6. Verify that Map 2 (Coastal Bay), Map 3 (River Valley), and Map 4 (Twin Districts / Emerald Lake)
	// also scale up to 4x (12.0) when LargeRoadNetwork is true, while Custom Map does not.
	otherBuiltinMaps := []sim.MapTheme{
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
	}

	for _, theme := range otherBuiltinMaps {
		g.Settings.SetLargeRoadNetwork(theme, true)
		g.EnterCity(theme)
		if !g.World.LargeGrid {
			t.Fatalf("Expected LargeGrid=true on theme %v when LargeRoadNetwork is true", theme)
		}
		if g.World.GridScale() != sim.MapGridScale*4.0 {
			t.Fatalf("Expected GridScale=%.1f on theme %v, got %.1f", sim.MapGridScale*4.0, theme, g.World.GridScale())
		}

		// Switch back to standard
		g.Settings.SetLargeRoadNetwork(theme, false)
		g.SetMapTheme(theme)
		if g.World.LargeGrid {
			t.Fatalf("Expected LargeGrid=false on theme %v when LargeRoadNetwork is false", theme)
		}
		if g.World.GridScale() != sim.MapGridScale {
			t.Fatalf("Expected GridScale=%.1f on theme %v, got %.1f", sim.MapGridScale, theme, g.World.GridScale())
		}
	}

	// 7. Custom map does NOT scale up
	g.Settings.SetLargeRoadNetwork(sim.MapThemeCustom, true)
	g.EnterCity(sim.MapThemeCustom)
	if g.World.LargeGrid {
		t.Fatalf("Expected LargeGrid=false on Custom Map even if LargeRoadNetwork setting is true")
	}
	if g.World.GridScale() != sim.MapGridScale {
		t.Fatalf("Expected GridScale=%.1f on Custom Map, got %.1f", sim.MapGridScale, g.World.GridScale())
	}
}
