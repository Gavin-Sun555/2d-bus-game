package game

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Replay the 2026-11-12 18:46 slot 2 bus 99 capture at the CBD central
// roundabout. Two articulated Line buses waited side by side at the west
// entrance: bus 99 in the curb lane held the long-vehicle reservation, bus 98
// in the inner lane was held for it. Bus 99 still yielded to bus 98 because
// the entry separation test swept 98's straight whole-body envelope along the
// curve, so neither moved and all four approaches queued. Both must enter.
func TestSavedSlot2ParallelArticulatedRoundaboutEntryClears(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184608.434635000-000008-bus99.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 bus 99 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	rb := g.World.Roundabouts[0]
	for _, r := range g.World.Roundabouts {
		if r.Center.Length() < rb.Center.Length() {
			rb = r
		}
	}
	buses := map[int]*traffic.AIBus{}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 98 || ab.ID == 99 {
			buses[ab.ID] = ab
		}
	}
	if len(buses) != 2 {
		t.Fatalf("capture is missing the waiting buses: found %d", len(buses))
	}
	health := map[int]float64{}
	entered := map[int]bool{}
	for id, ab := range buses {
		health[id] = ab.Bus.Durability
	}
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, ab := range buses {
			if ab.Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
			if ab.Bus.Pos.Distance(rb.Center) < rb.Radius {
				entered[id] = true
			}
		}
		if entered[98] && entered[99] {
			return
		}
	}
	for id, ab := range buses {
		if !entered[id] {
			t.Errorf("bus %d never entered the roundabout: pos%v speed%.1f state %v", id, ab.Bus.Pos, ab.Bus.Speed, ab.Driver.State)
		}
	}
}

// Replay the 2026-11-12 18:43 slot 2 capture taken 3 s after loading. Bus 114's
// route turns left at junction (11700, 0) straight out of the curb lane, which
// is also its approach-merge corridor. The merge handed back to the route only
// while the route was within 4 px of that corridor; the articulated bus
// straightened only after the route had begun curving into the turn, so the
// merge kept it on the straight curb lane through the junction, its lane
// offset ran away and it stalled off-route before stop 16. It must turn.
func TestSavedSlot2CurbLaneLeftTurnHandsOffToRoute(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184339.418177000-000001-car141.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 early capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	var bus *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 114 {
			bus = ab
		}
	}
	if bus == nil {
		t.Fatal("capture is missing bus 114")
	}
	health := bus.Bus.Durability
	for tick := 0; tick < 60*20; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if math.Abs(bus.Driver.LaneOffset) > 120 {
			t.Fatalf("lane offset ran away at tick %d: %.1f pos%v", tick, bus.Driver.LaneOffset, bus.Bus.Pos)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus 114 damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X > 11950 && bus.Bus.Pos.Y < 300 {
			return
		}
		if bus.Bus.Pos.Y > 400 {
			t.Fatalf("bus 114 went straight through the junction: pos%v", bus.Bus.Pos)
		}
	}
	t.Fatalf("bus 114 did not complete its left turn: pos%v speed%.1f", bus.Bus.Pos, bus.Bus.Speed)
}

// Replay the 2026-11-12 18:45 slot 2 bus 119 capture. The articulated bus had
// served stop 22 and stopped at the end of its bay with its nose turned 0.2 rad
// toward the road. Its commanded steering swept the trailer 0.1 px into the
// shelter, so it held forever, although a gentler steering was clear. The
// checked departure crawl must take it out without damage.
func TestSavedSlot2ArticulatedBayExitGrazingShelterCrawlsOut(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184532.481490000-000007-bus119.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 bus 119 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	var bus *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 119 {
			bus = ab
		}
	}
	if bus == nil {
		t.Fatal("capture is missing bus 119")
	}
	start, health := bus.Bus.Pos, bus.Bus.Durability
	for tick := 0; tick < 60*15; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus 119 damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X-start.X > 400 {
			return
		}
	}
	t.Fatalf("bus 119 still held at the bay exit: pos%v speed%.1f", bus.Bus.Pos, bus.Bus.Speed)
}
