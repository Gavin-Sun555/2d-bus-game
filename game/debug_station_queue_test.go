package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Replays an untouched copy of a captured station queue through normal Update.
func TestLatestDebugStationQueueClears(t *testing.T) {
	path := os.Getenv("BUS_CURRENT_DEBUG")
	if path == "" {
		t.Skip("BUS_CURRENT_DEBUG not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	if err := os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	starts, health := map[int]sim.Vec2{}, map[int]float64{}
	released := map[int]bool{}
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 25 && a.ID != 28 && a.ID != 31 && a.ID != 32 && a.ID != 66 {
			continue
		}
		id := a.ID
		starts[id], health[id] = a.Bus.Pos, a.Bus.Durability
		a.Driver.BeforeRecovery = func() { t.Fatalf("bus%d repositioned", id) }
	}
	g.AIBusMgr.BeforeRecovery = func(event sim.TrafficRecoveryEvent) {
		if _, ok := starts[event.ID]; ok {
			t.Fatalf("bus%d reset", event.ID)
		}
	}
	for tick := 0; tick < 45*60; tick++ {
		before := map[int]sim.Vec2{}
		for _, a := range g.AIBusMgr.Buses {
			if _, ok := starts[a.ID]; ok {
				before[a.ID] = a.Bus.Pos
			}
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, a := range g.AIBusMgr.Buses {
			start, ok := starts[a.ID]
			if !ok {
				continue
			}
			if a.Bus.Pos.Distance(before[a.ID]) > math.Abs(a.Bus.Speed)/60+.05 || a.Bus.Durability < health[a.ID] {
				t.Fatalf("bus%d jumped or took damage", a.ID)
			}
			if a.Bus.Pos.Distance(start) > 100 && !released[a.ID] {
				t.Logf("bus%d cleared 100px in %.2fs", a.ID, float64(tick+1)/60)
				released[a.ID] = true
			}
		}
		if len(released) == len(starts) {
			return
		}
	}
	t.Fatalf("station queue did not release all buses: %v", released)
}
