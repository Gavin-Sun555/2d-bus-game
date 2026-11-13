package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedPlayerLaneChoice(t *testing.T) {
	path := os.Getenv("BUS_LANE_CHOICE_SAVE")
	if path == "" {
		t.Skip("BUS_LANE_CHOICE_SAVE not set")
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(copyPath, data, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	initial, health, passengers := g.Bus.Pos, g.Bus.Durability, g.Bus.PassengerCount
	stable := 0
	for tick := 0; tick < 7200; tick++ {
		before := g.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatalf("bus relocated at tick%d", tick)
		}
		if g.Bus.Durability < health || g.Bus.PassengerCount != passengers {
			t.Fatalf("contact or lost passengers at tick%d", tick)
		}
		centered := true
		for _, part := range g.Bus.BodyParts() {
			for _, corner := range sim.BodyCorners(part) {
				lateral := -corner.Y - 28800
				if lateral < 104 || lateral > 156 {
					centered = false
				}
			}
		}
		if centered && math.Abs(sim.NormalizeAngle(g.Bus.Heading-math.Pi)) < .1 {
			stable++
		} else {
			stable = 0
		}
		if stable >= 120 && initial.X-g.Bus.Pos.X > 1000 {
			t.Logf("curb lane maintained at %.2fs distance%.0f", float64(tick)/60, initial.X-g.Bus.Pos.X)
			return
		}
		if tick%600 == 0 {
			t.Logf("tick%d pos%v speed%.1f mid%v", tick, g.Bus.Pos, g.Bus.Speed, g.AutopilotDriver.Snapshot().MidLaneRecovery)
		}
	}
	t.Fatalf("did not settle into the clear curb lane: pos%v speed%.1f", g.Bus.Pos, g.Bus.Speed)
}
