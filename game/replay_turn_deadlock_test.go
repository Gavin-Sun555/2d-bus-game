package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedTurnDeadlockResumes(t *testing.T) {
	path := os.Getenv("BUS_TURN_DEADLOCK_SAVE")
	if path == "" {
		t.Skip("BUS_TURN_DEADLOCK_SAVE not set")
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(copyPath, data, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	start, health := g.Bus.Pos, g.Bus.Durability
	for tick := 0; tick < 7200; tick++ {
		before := g.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.1 || g.Bus.Durability < health {
			t.Fatalf("relocated or damaged tick%d pos%v", tick, g.Bus.Pos)
		}
		if tick%1200 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, 52)
			t.Logf("%.0fs player pos%v h%.3f speed%.1f limits%v blockers%+v recovery%v timer%.1f", float64(tick)/60, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, d.Limits, d.Blockers, g.AutopilotDriver.HeadonRecoveryActive(), g.AutopilotDriver.HeadonDeadlockTimer())
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(start) < 900 || a.ID == 9 || a.ID == 53 || a.ID == 55 || a.ID == 16 || a.ID == 74 {
					d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, g.AIBusMgr.GetObstacles(a.ID), 52)
					t.Logf("AI%d pos%v h%.2f speed%.1f limits%v", a.ID, a.Bus.Pos, a.Bus.Heading, a.Bus.Speed, d.Limits)
				}
			}
		}
		if g.Bus.Pos.Distance(start) > 1200 {
			t.Logf("resumed at %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatalf("stayed blocked at %v", g.Bus.Pos)
}
