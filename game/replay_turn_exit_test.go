package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedTurnExitStaysOnRoad(t *testing.T) {
	path := os.Getenv("BUS_TURN_EXIT_SAVE")
	if path == "" {
		t.Skip("BUS_TURN_EXIT_SAVE not set")
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
	settled := false
	for tick := 0; tick < 3600; tick++ {
		before := g.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.1 || g.Bus.Durability < health {
			t.Fatalf("relocated or damaged tick%d pos%v", tick, g.Bus.Pos)
		}
		if tick%120 == 0 {
			t.Logf("%.1fs pos%v h%.2f speed%.1f offset%.1f/%.1f pursuit%v", float64(tick)/60, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset, g.AutopilotDriver.LastPursuitPt)
		}
		if g.Bus.Pos.Y < 101 && math.Abs(g.Bus.Heading) < .08 {
			settled = true
		}
		if settled && g.Bus.Pos.Y > 104 {
			t.Fatalf("returned to shoulder tick%d pos%v", tick, g.Bus.Pos)
		}
		if settled && g.Bus.Pos.X > start.X+1600 {
			return
		}
	}
	t.Fatalf("did not recover and drive on road: pos%v speed%.1f", g.Bus.Pos, g.Bus.Speed)
}
