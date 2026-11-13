package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// Opt-in replay of the Nov 10 slot 1 station wedge, using a temporary save copy.
func TestSlot1MissedTaperQueueReleases(t *testing.T) {
	source := os.Getenv("BUS_SLOT1_TAPER_REPLAY")
	if source == "" {
		t.Skip("BUS_SLOT1_TAPER_REPLAY not set")
	}
	raw, err := os.ReadFile(source)
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
	sim.SeedSimulation(5)
	buses := map[int]*sim.Bus{0: g.Bus}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 15 || ab.ID == 53 {
			buses[ab.ID] = ab.Bus
			ab.Driver.BeforeRecovery = func() { t.Fatal("recovery repositioned bus") }
		}
	}
	if len(buses) != 3 {
		t.Fatal("expected station queue missing")
	}
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("recovery repositioned player") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if _, ok := buses[e.ID]; ok {
			t.Fatal("queued bus reset")
		}
	}
	starts := map[int]sim.Vec2{}
	health := map[int]float64{}
	released := map[int]bool{}
	for id, b := range buses {
		starts[id] = b.Pos
		health[id] = b.Durability
	}
	for tick := 0; tick < 1800; tick++ {
		before := map[int]sim.Vec2{}
		for id, b := range buses {
			before[id] = b.Pos
		}
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, b := range buses {
			if b.Pos.Distance(before[id]) > math.Abs(b.Speed)/60+.05 || b.Durability < health[id] {
				t.Fatalf("bus%d repositioned or damaged at %.2fs", id, float64(tick+1)/60)
			}
			if !released[id] && b.Pos.Distance(starts[id]) > 250 {
				released[id] = true
				t.Logf("bus%d cleared queue after %.2fs", id, float64(tick+1)/60)
			}
		}
		if len(released) == 3 {
			return
		}
	}
	t.Fatalf("queue did not clear: %v", released)
}
