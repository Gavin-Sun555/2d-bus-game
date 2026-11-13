package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedAI34LeavesStraddledLane(t *testing.T) {
	path := os.Getenv("BUS_AI34_SAVE")
	if path == "" {
		t.Skip("BUS_AI34_SAVE not set")
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
	var bus *sim.Bus
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 34 {
			bus = a.Bus
		}
	}
	if bus == nil {
		t.Fatal("AI34 not present")
	}
	start, health, passengers := bus.Pos, bus.Durability, bus.PassengerCount
	for tick := 0; tick < 7200; tick++ {
		before := bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		repositioned := bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.1
		if repositioned && (math.Abs(bus.Pos.Y-before.Y) > .001 || math.Abs(bus.Pos.X-before.X) > 156 || math.Abs(sim.NormalizeAngle(bus.Heading-math.Pi/2)) > .01) {
			t.Fatalf("non-local recovery at tick%d", tick)
		}
		if bus.Durability < health || bus.PassengerCount != passengers {
			t.Fatalf("AI34 relocated, damaged or lost passengers tick%d pos%v", tick, bus.Pos)
		}
		if tick%1200 == 0 {
			for _, a := range g.AIBusMgr.Buses {
				if a.ID == 34 {
					obs := g.AIBusMgr.GetLocalObstacles(bus.Pos, bus.Speed, nil)
					filtered := obs[:0]
					for _, o := range obs {
						if o.Bus != bus {
							filtered = append(filtered, o)
						}
					}
					d := a.Driver.DiagnoseStop(bus, a.Route, g.World, g.TrafficLights, filtered, 52)
					t.Logf("t%.0f pos%v speed%.1f target%.1f limits%v blockers%v", float64(tick)/60, bus.Pos, bus.Speed, a.Driver.TargetLaneOffset, d.Limits, d.Blockers)
				}
			}
		}
		if bus.Pos.Distance(start) > 1200 {
			t.Logf("AI34 resumed at %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatalf("AI34 remained stuck at %v", bus.Pos)
}
