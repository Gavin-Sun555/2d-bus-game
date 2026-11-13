package game

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Replay the 2026-11-12 12:36 autosave, taken while two Line 2 station jams
// on the western boulevard were already stationary.
//
// Stop 228: bus 56 stopped in the curb lane level with the front berth of an
// empty bay. No berth curve fits from there, so the dock planner held it on
// the brake indefinitely and the whole lane queued behind it. It must give up
// that stop, keep its passengers aboard and continue.
//
// Stop 232: bus 9 missed the berth with its nose turned into the curb past the
// bay. Its only recovery is a short reverse, but bus 82 had stopped against
// its tail. The follower must back off so bus 9 can straighten and leave.
func TestSavedAutosaveWestBoulevardStationJamsClear(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-noon", "save-auto-02.json"))
	if p == "" {
		t.Skip("2026-11-12 west boulevard autosave not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	buses := map[int]*traffic.AIBus{}
	for _, ab := range g.AIBusMgr.Buses {
		switch ab.ID {
		case 9, 13, 56, 76, 82, 83:
			buses[ab.ID] = ab
		}
	}
	if len(buses) != 6 {
		t.Fatalf("autosave is missing jammed buses: found %d", len(buses))
	}
	health := map[int]float64{}
	for id, ab := range buses {
		health[id] = ab.Bus.Durability
	}
	cleared := func(id int) bool {
		y := buses[id].Bus.Pos.Y
		if id == 56 || id == 83 {
			return y > -10300
		}
		return y > 4200
	}
	for tick := 0; tick < 60*45; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		done := true
		for id, ab := range buses {
			if ab.Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
			done = done && cleared(id)
		}
		if done {
			return
		}
	}
	for id, ab := range buses {
		if !cleared(id) {
			t.Errorf("bus %d still jammed: pos%v heading%.2f speed%.1f", id, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Speed)
		}
	}
}

// Replay the 2026-11-12 13:41 bus 79 capture. The compact Line 1 bus overran
// the end of stop 211's bay with its nose turned 0.32 rad into the curb and
// its arrival curve exhausted. The dock planner held it on the brake for good
// because the missed-stop recovery only covered long buses. It must give up
// the stop, keep its passengers aboard and rejoin the road.
func TestSavedCompactBusPastBayEndContinues(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-bus79", "congestion-20261112-134135.690151000-000001-bus79.json"))
	if p == "" {
		t.Skip("2026-11-12 bus 79 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	var bus *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 79 {
			bus = ab
		}
	}
	if bus == nil {
		t.Fatal("capture is missing bus 79")
	}
	health := bus.Bus.Durability
	for tick := 0; tick < 60*20; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus 79 damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X > 40000 && math.Abs(bus.Bus.Pos.Y-21782) < 30 {
			return
		}
	}
	t.Fatalf("bus 79 still stuck past stop 211: pos%v heading%.2f speed%.1f", bus.Bus.Pos, bus.Bus.Heading, bus.Bus.Speed)
}
