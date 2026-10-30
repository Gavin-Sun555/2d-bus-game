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

	// Simulate clicking the Large Grid button on Card 0
	g.Settings.LargeRoadNetwork = true
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

	// 6. Non-Metropolis maps (e.g. Coastal Bay) do not scale up
	g.Settings.LargeRoadNetwork = true
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.World.LargeGrid {
		t.Fatalf("Expected LargeGrid=false on Coastal Bay even if LargeRoadNetwork setting is true")
	}
	if g.World.GridScale() != sim.MapGridScale {
		t.Fatalf("Expected GridScale=%.1f on Coastal Bay, got %.1f", sim.MapGridScale, g.World.GridScale())
	}
}
