package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func TestSavedSlot1Entry1058QueueReleases(t *testing.T) {
	path := os.Getenv("BUS_ENTRY_1058_SAVE")
	if path == "" {
		t.Skip("BUS_ENTRY_1058_SAVE not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(copyPath, raw, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	type watcher struct {
		b           *sim.Bus
		start, last sim.Vec2
		health      float64
	}
	watch := map[int]*watcher{}
	watch[0] = &watcher{g.Bus, g.Bus.Pos, g.Bus.Pos, g.Bus.Durability}
	for _, a := range g.AIBusMgr.Buses {
		switch a.ID {
		case 61, 72, 47, 46, 48, 45, 70, 42:
			watch[a.ID] = &watcher{a.Bus, a.Bus.Pos, a.Bus.Pos, a.Bus.Durability}
			a.Driver.BeforeRecovery = func() { t.Fatal("queued bus repositioned") }
		}
	}
	if len(watch) != 9 {
		t.Fatal("expected saved queue missing")
	}
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if watch[e.ID] != nil {
			t.Fatalf("bus%d reset", e.ID)
		}
	}
	for tick := 0; tick < 10800; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		cleared := true
		for id, w := range watch {
			if w.b.Pos.Distance(w.last) > math.Abs(w.b.Speed)/60+.1 || w.b.Durability < w.health {
				t.Fatalf("bus%d relocated or damaged at t%.2f", id, float64(tick)/60)
			}
			w.last = w.b.Pos
			need := 200.
			if id == 0 || id == 61 {
				need = 1000
			}
			if w.b.Pos.Distance(w.start) < need {
				cleared = false
			}
		}
		if cleared {
			t.Logf("player and eight queued buses released t%.2fs", float64(tick+1)/60)
			return
		}
	}
	for id, w := range watch {
		t.Logf("bus%d distance%.1f pos%v", id, w.b.Pos.Distance(w.start), w.b.Pos)
	}
	t.Fatal("queue remained blocked")
}
