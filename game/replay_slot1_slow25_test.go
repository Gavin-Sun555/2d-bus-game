package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedSlot1ClearLaneReleases25KPHLimit(t *testing.T) {
	p := os.Getenv("BUS_SLOW25_SAVE")
	if p == "" {
		t.Skip("opt in")
	}
	raw, err := os.ReadFile(p)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err := os.WriteFile(copyPath, raw, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, copyPath, 1280, 720)
	health, passengers, cars, buses := g.Bus.Durability, g.Bus.PassengerCount, len(g.TrafficMgr.Cars), len(g.AIBusMgr.Buses)
	for tick := 0; tick < 240; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if tick == 0 && g.AutopilotDriver.LastTargetSpeed < sim.SpeedKPH(80) {
			t.Fatal("stale 25 kph limit retained")
		}
		if g.Bus.Durability != health || g.Bus.PassengerCount != passengers || len(g.TrafficMgr.Cars) != cars || len(g.AIBusMgr.Buses) != buses || g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatal("traffic removed, journey changed or vehicle repositioned")
		}
		if tick == 179 && g.Bus.Speed < sim.SpeedKPH(100) {
			t.Fatalf("clear road remains slow: %.1f kph", g.Bus.Speed*.18)
		}
		if tick%120 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, 52)
			t.Logf("tick%d pos%v speed%.2f target%.2f limits%v", tick, g.Bus.Pos, g.Bus.Speed*.18, g.AutopilotDriver.LastTargetSpeed*.18, d.Limits)
		}
	}
}
