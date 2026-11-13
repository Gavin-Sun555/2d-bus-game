package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Read an isolated copy of the November 7 slot-one/slot-five junction queue.
func TestSavedJunctionAndStationQueueResume(t *testing.T) {
	path := os.Getenv("BUS_JUNCTION_QUEUE_SAVE")
	if path == "" {
		t.Skip("BUS_JUNCTION_QUEUE_SAVE not set")
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(copyPath, data, 0600); err != nil {
		t.Fatal(err)
	}
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	starts := map[int]sim.Vec2{}
	last := map[int]sim.Vec2{}
	conditions := map[int]float64{}
	passengers := 0
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 10 || a.ID == 11 || a.ID == 71 {
			starts[a.ID], last[a.ID], conditions[a.ID] = a.Bus.Pos, a.Bus.Pos, a.Bus.Durability
			if a.ID == 10 {
				passengers = a.Bus.PassengerCount
			}
		}
	}
	progressed := map[int]bool{}
	health := g.Bus.Durability
	for tick := 0; tick < 10800; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatalf("player contact at tick%d", tick)
		}
		for _, a := range g.AIBusMgr.Buses {
			if _, ok := starts[a.ID]; !ok {
				continue
			}
			if a.Bus.Pos.Distance(last[a.ID]) > math.Abs(a.Bus.Speed)/60+.1 || a.Bus.Durability < conditions[a.ID] {
				t.Fatalf("AI%d relocated or damaged at tick%d", a.ID, tick)
			}
			last[a.ID] = a.Bus.Pos
			for _, other := range g.AIBusMgr.Buses {
				if other.ID == a.ID || a.Bus.Pos.Distance(other.Bus.Pos) > a.Bus.OverallLength()+other.Bus.OverallLength() {
					continue
				}
				if hit, _, depth := sim.CheckBusesCollision(a.Bus, other.Bus); hit && depth > 1e-6 {
					t.Fatalf("AI%d contacted AI%d at tick%d", a.ID, other.ID, tick)
				}
			}
			if (a.ID == 10 && a.Route.GetCurrentTargetStopID() != 232) || (a.ID == 11 && a.Bus.Pos.Y > starts[a.ID].Y+650) || (a.ID == 71 && a.Bus.Pos.X > -61700) {
				if !progressed[a.ID] {
					if a.ID == 10 && a.Bus.PassengerCount != passengers {
						t.Fatal("missed-arrival recovery lost passengers")
					}
					t.Logf("AI%d resumed at %.2fs pos%v", a.ID, float64(tick)/60, a.Bus.Pos)
				}
				progressed[a.ID] = true
			}
		}
		if progressed[10] && progressed[11] && progressed[71] {
			return
		}
		if tick%1200 == 0 {
			for _, a := range g.AIBusMgr.Buses {
				if _, ok := starts[a.ID]; ok {
					t.Logf("tick%d AI%d pos%v target%d speed%.2f", tick, a.ID, a.Bus.Pos, a.Route.GetCurrentTargetStopID(), a.Bus.Speed)
				}
			}
		}
	}
	t.Fatalf("queue did not resume: %v", progressed)
}
