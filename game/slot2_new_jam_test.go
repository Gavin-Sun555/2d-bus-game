package game

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func findSlot2Jam1501File() string {
	candidates := []string{
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1501", "slot2.json"),
		filepath.Join("dist", "slot2-jam-2026-11-02-1501", "slot2.json"),
		"/Users/yansun/personal/2d-bus-game/dist/slot2-jam-2026-11-02-1501/slot2.json",
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

func TestSlot2CarBusMergeDeadlockResolution(t *testing.T) {
	savePath := findSlot2Jam1501File()
	if savePath == "" {
		t.Skip("slot2 snapshot not found; skipping")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame failed: %v", err)
	}

	var car31566 *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == 31566 {
			car31566 = c
			break
		}
	}

	var bus8 *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 8 {
			bus8 = ab
			break
		}
	}

	if car31566 == nil || bus8 == nil {
		t.Fatalf("car 31566 (%v) or bus 8 (%v) not found", car31566, bus8)
	}

	initCarX := car31566.Pos.X
	initBusX := bus8.Bus.Pos.X

	// Step simulation for 500 ticks (~8.3s)
	for tick := 1; tick <= 500; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at tick %d: %v", tick, err)
		}
	}

	// Car 31566 should have accelerated and driven past Bus 8 (heading West, so X becomes more negative)
	if car31566.Pos.X >= initCarX-200.0 {
		t.Fatalf("Car 31566 should have moved forward significantly, got X=%.2f (init=%.2f)", car31566.Pos.X, initCarX)
	}
	if car31566.Speed < 50.0 {
		t.Fatalf("Car 31566 should be cruising at speed, got Spd=%.2f", car31566.Speed)
	}


	// Bus 8 should have moved forward significantly (it merges into Lane 2 and proceeds to Station 1)
	if bus8.Bus.Pos.X >= initBusX-100.0 {
		t.Fatalf("Bus 8 should have advanced and merged, got X=%.2f (init=%.2f)", bus8.Bus.Pos.X, initBusX)
	}
}
