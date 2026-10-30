package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Verify that setting player vehicle to an articulated bus at game start
// immediately updates the vehicle model, rather than waiting for lap 2.
func TestPlayerBusModelAppliedImmediatelyAtRouteStart(t *testing.T) {
	g := NewGame(1280, 720)
	if !g.isPlayerAtRouteStart() {
		t.Fatal("fresh game should be at route start")
	}
	if g.Bus.Model != sim.BusStandard {
		t.Fatalf("expected standard bus initially, got %v", g.Bus.Model)
	}

	// Change player bus model at route start
	c := g.LineFleets[g.PlayerLine]
	c.PlayerModel = sim.BusArticulated
	g.SetLineFleet(g.PlayerLine, c)

	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("expected articulated bus immediately at route start, got %v", g.Bus.Model)
	}
	if g.Bus.PendingModel != nil {
		t.Fatalf("pending model should be nil when changed at route start, got %v", *g.Bus.PendingModel)
	}
	if !g.Bus.IsArticulated() {
		t.Fatal("bus should report IsArticulated() == true")
	}
	if g.Settings.GetPlayerModel() != sim.BusArticulated {
		t.Fatalf("settings should remember player model %v, got %v", sim.BusArticulated, g.Settings.GetPlayerModel())
	}
}

// Verify that setting player vehicle while en route defers replacement to terminus.
func TestPlayerBusModelEnRouteDefersToTerminus(t *testing.T) {
	g := NewGame(1280, 720)
	// Start with articulated bus
	c := g.LineFleets[g.PlayerLine]
	c.PlayerModel = sim.BusArticulated
	g.SetLineFleet(g.PlayerLine, c)
	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("expected articulated bus at start, got %v", g.Bus.Model)
	}

	// Advance route index to simulate being en route
	g.Route.CurrentIndex = 1
	if g.isPlayerAtRouteStart() {
		t.Fatal("should not be at route start when CurrentIndex == 1")
	}

	// Change model to double-decker while en route
	c.PlayerModel = sim.BusDoubleDecker
	g.SetLineFleet(g.PlayerLine, c)

	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("model should remain articulated while en route, got %v", g.Bus.Model)
	}
	if g.Bus.PendingModel == nil || *g.Bus.PendingModel != sim.BusDoubleDecker {
		t.Fatalf("expected pending model BusDoubleDecker, got %v", g.Bus.PendingModel)
	}
}

// Verify that entering a city or switching themes initializes with the configured player model.
func TestCityStartupWithConfiguredPlayerModel(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.PlayerModel = sim.BusArticulated

	// Entering city should spawn an articulated bus directly
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("expected articulated bus on EnterCity, got %v", g.Bus.Model)
	}
	if g.Bus.PendingModel != nil {
		t.Fatalf("pending model should be nil on fresh city entry, got %v", *g.Bus.PendingModel)
	}
	if !g.Bus.IsArticulated() {
		t.Fatal("expected IsArticulated == true")
	}
}

// Verify that clicking button 24 (玩家用车) at game start cycles immediately.
func TestNetworkAction24CyclesPlayerModelImmediatelyAtStart(t *testing.T) {
	g := NewGame(1280, 720)
	g.NetworkSelected = 0

	// Action 24 cycles PlayerModel: Standard -> DoubleDecker -> Articulated -> Standard
	g.handleNetworkAction(24)
	if g.Bus.Model != sim.BusDoubleDecker || g.Bus.PendingModel != nil {
		t.Fatalf("expected immediate switch to DoubleDecker, got Model=%v, Pending=%v", g.Bus.Model, g.Bus.PendingModel)
	}

	g.handleNetworkAction(24)
	if g.Bus.Model != sim.BusArticulated || g.Bus.PendingModel != nil {
		t.Fatalf("expected immediate switch to Articulated, got Model=%v, Pending=%v", g.Bus.Model, g.Bus.PendingModel)
	}

	g.handleNetworkAction(24)
	if g.Bus.Model != sim.BusStandard || g.Bus.PendingModel != nil {
		t.Fatalf("expected immediate switch back to Standard, got Model=%v, Pending=%v", g.Bus.Model, g.Bus.PendingModel)
	}
}

// Verify that saving and loading at route start preserves and aligns the player model.
func TestSavedGameAtRouteStartPreservesConfiguredModel(t *testing.T) {
	g := saveTestGame(t)
	c := traffic.LineFleetSettings{PlayerModel: sim.BusArticulated}
	g.SetLineFleet(0, c)
	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("expected articulated bus before save, got %v", g.Bus.Model)
	}

	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}

	if g.Bus.Model != sim.BusArticulated {
		t.Fatalf("expected articulated bus after load, got %v", g.Bus.Model)
	}
	if g.Bus.PendingModel != nil {
		t.Fatalf("pending model should be nil after load, got %v", *g.Bus.PendingModel)
	}
}
