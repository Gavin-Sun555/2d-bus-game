package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// Replay both sides of the Nov 11 bus85/bus88 station lock through departure.
// Loading a capture into a temporary path keeps the user's debug save intact.
func TestLateYawedStationPairReleases(t *testing.T) {
	path := os.Getenv("BUS_DEBUG_PAIR_REPLAY")
	if path == "" {
		t.Skip("BUS_DEBUG_PAIR_REPLAY not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	wanted := map[int]bool{85: true, 88: true}
	if value := os.Getenv("BUS_DEBUG_PAIR_IDS"); value != "" {
		wanted = map[int]bool{}
		for _, part := range strings.Split(value, ",") {
			id, e := strconv.Atoi(part)
			if e != nil {
				t.Fatal(e)
			}
			wanted[id] = true
		}
	}
	buses := map[int]*sim.Bus{}
	for _, a := range g.AIBusMgr.Buses {
		if wanted[a.ID] {
			buses[a.ID] = a.Bus
			a.Driver.BeforeRecovery = func() { t.Fatal("bus repositioned") }
		}
	}
	if len(buses) != len(wanted) {
		t.Fatal("station pair missing")
	}
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if _, ok := buses[e.ID]; ok {
			t.Fatal("bus reset")
		}
	}
	starts := map[int]sim.Vec2{}
	health := map[int]float64{}
	passengers := map[int]int{}
	released := map[int]bool{}
	served := map[int]bool{}
	for id, b := range buses {
		starts[id] = b.Pos
		health[id] = b.Durability
		passengers[id] = b.PassengerCount
	}
	for tick := 0; tick < 5400; tick++ {
		before := map[int]sim.Vec2{}
		for id, b := range buses {
			before[id] = b.Pos
		}
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, b := range buses {
			if b.Durability < health[id] || b.Pos.Distance(before[id]) > math.Abs(b.Speed)/60+.05 {
				t.Fatalf("bus%d damaged or repositioned at %.2fs", id, float64(tick+1)/60)
			}
			served[id] = served[id] || b.DoorsOpen
			if !released[id] && !served[id] && b.PassengerCount != passengers[id] {
				t.Fatalf("bus%d lost passengers before leaving stop", id)
			}
			if !released[id] && b.Pos.Distance(starts[id]) > 1000 {
				released[id] = true
				t.Logf("bus%d cleared station after %.2fs", id, float64(tick+1)/60)
			}
		}
		if len(released) == len(wanted) {
			return
		}
	}
	t.Fatalf("station queue remained blocked: %v", released)
}
