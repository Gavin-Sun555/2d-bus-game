package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func loadSlot1LeftExitReplay(t *testing.T) *Game {
	t.Helper()
	path := os.Getenv("BUS_SLOT1_LEFT_EXIT_SAVE")
	if path == "" {
		t.Skip("BUS_SLOT1_LEFT_EXIT_SAVE not set")
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
	return g
}

func TestSavedSlot1LeftExitUsesAvailableRoad(t *testing.T) {
	g := loadSlot1LeftExitReplay(t)
	var err error
	for tick := 0; tick < 3; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 43 && a.ID != 72 && a.ID != 71 && a.ID != 42 {
			continue
		}
		obs := g.AIBusMgr.GetLocalObstacles(a.Bus.Pos, a.Bus.Speed, nil)
		filtered := obs[:0]
		for _, ob := range obs {
			if ob.Bus != a.Bus {
				filtered = append(filtered, ob)
			}
		}
		d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, filtered, 52)
		d.Path = nil
		t.Logf("AI%d pos%v diagnosis%+v", a.ID, a.Bus.Pos, d)
	}
	buses := map[int]*sim.Bus{0: g.Bus}
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 43 || a.ID == 72 {
			buses[a.ID] = a.Bus
		}
	}
	if len(buses) != 3 {
		t.Fatal("expected player and two line-six buses")
	}
	health := map[int]float64{}
	last := map[int]sim.Vec2{}
	passed := map[int]bool{}
	for id, bus := range buses {
		health[id], last[id] = bus.Durability, bus.Pos
	}
	for tick := 0; tick < 7200; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, bus := range buses {
			if bus.Durability < health[id] || bus.Pos.Distance(last[id]) > math.Abs(bus.Speed)/60+.05 {
				t.Fatalf("bus%d contact or relocation at tick%d", id, tick)
			}
			last[id] = bus.Pos
			if bus.Pos.Y > 8000 && bus.Pos.X > -62560 {
				if !passed[id] {
					t.Logf("bus%d left via general lane in %.2fs pos%v", id, float64(tick+1)/60, bus.Pos)
				}
				passed[id] = true
			}
		}
		if len(passed) == 3 {
			return
		}
	}
	for id, bus := range buses {
		t.Logf("bus%d final pos%v speed%.1f", id, bus.Pos, bus.Speed)
	}
	t.Fatalf("left-turn queue did not clear: %v", passed)
}

func TestSavedSlot1DownstreamMergeResumesWithoutContact(t *testing.T) {
	g := loadSlot1LeftExitReplay(t)
	var leader, follower *sim.Bus
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 71 {
			leader = a.Bus
		}
		if a.ID == 44 {
			follower = a.Bus
		}
	}
	if leader == nil || follower == nil {
		t.Fatal("saved downstream pair missing")
	}
	start, h1, h2 := leader.Pos, leader.Durability, follower.Durability
	for tick := 0; tick < 900; tick++ {
		before := leader.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if leader.Durability < h1 || follower.Durability < h2 || leader.Pos.Distance(before) > math.Abs(leader.Speed)/60+.05 {
			t.Fatalf("downstream pair contact or relocation tick%d", tick)
		}
		if leader.Pos.Y > start.Y+200 {
			t.Logf("downstream leader resumed in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("downstream merge still deadlocked")
}
