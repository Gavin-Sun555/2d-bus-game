package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"os"
	"path/filepath"
	"testing"
)

func TestSlot2InitialPlayerSpawnRepair(t *testing.T) {
	path := os.Getenv("BUS_INITIAL_SPAWN_REPLAY")
	if path == "" {
		t.Skip("BUS_INITIAL_SPAWN_REPLAY not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	start := g.Bus.Pos
	t.Logf("repaired player at %v stop %d", start, g.Route.GetCurrentTargetStopID())
	if !traffic.SafeInitialBusPosition(g.Bus, g.World, g.Settings.GetLaneWidth()) {
		t.Fatal("player still has unsafe initial position")
	}
	health := g.Bus.Durability
	for tick := 0; tick < 1800; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatal("repaired player damaged")
		}
		if g.Bus.Pos.Distance(start) > 500 {
			t.Logf("player resumed after %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("player failed to resume service")
}
