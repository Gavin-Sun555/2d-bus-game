package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

// Optional replay of the backed-up 2026-10-26 harbor station jam. Never saves.
func TestCurrentHarborStationRecovery(t *testing.T) {
	path := os.Getenv("BUS_HARBOR_JAM_SAVE")
	if path == "" {
		t.Skip("set BUS_HARBOR_JAM_SAVE")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	stopID := g.Route.GetCurrentTargetStopID()
	initial := g.Bus.Durability
	health := map[int]float64{}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	playerBoarded, leaderBoarded := false, false
	playerFirst, leaderFirst := -1, -1
	start := g.Bus.Pos
	for tick := 0; tick < 7200; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if !playerBoarded && g.Bus.DoorsOpen && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == stopID {
			playerBoarded = true
			playerFirst = tick
		}
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus.Durability < health[ab.ID] {
				t.Fatalf("AI %d lost health at tick%d", ab.ID, tick)
			}
			if ab.ID == 1 && !leaderBoarded && ab.Bus.DoorsOpen && ab.Route.GetCurrentTargetStopID() == stopID {
				leaderBoarded = true
				leaderFirst = tick
			}
		}
		if g.Bus.Durability < initial {
			t.Fatalf("player lost health at tick%d", tick)
		}
	}
	t.Logf("leader boarded at %.2fs, player at %.2fs; displacement %.1fpx; target %d -> %d; all bus health unchanged", float64(leaderFirst)/60, float64(playerFirst)/60, g.Bus.Pos.Distance(start), stopID, g.Route.GetCurrentTargetStopID())
	if !playerBoarded || !leaderBoarded || g.Route.GetCurrentTargetStopID() == stopID {
		t.Fatal("station service failed to resume")
	}
}
