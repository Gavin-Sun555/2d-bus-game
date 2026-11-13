package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedPlayerMedianReturn(t *testing.T) {
	path := os.Getenv("BUS_MEDIAN_RETURN_SAVE")
	if path == "" {
		t.Skip("BUS_MEDIAN_RETURN_SAVE not set")
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
	start := g.Bus.Pos
	for tick := 0; tick < 3600; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Pos.Distance(start) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatalf("player relocated tick%d", tick)
		}
		start = g.Bus.Pos
		if g.Bus.Durability < health || g.Bus.PassengerCount != passengers {
			t.Fatalf("recovery damaged bus or lost passengers tick%d", tick)
		}
		onOwnSide := true
		for _, part := range g.Bus.BodyParts() {
			for _, corner := range sim.BodyCorners(part) {
				if corner.X > -62400 {
					onOwnSide = false
				}
			}
		}
		if onOwnSide && g.Bus.Pos.Distance(initial) > 400 && math.Abs(sim.NormalizeAngle(g.Bus.Heading-math.Pi/2)) < .1 {
			t.Logf("returned at %.2fs pos%v speed%.1f", float64(tick)/60, g.Bus.Pos, g.Bus.Speed)
			return
		}
		if tick%600 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, 52)
			d.Path = nil
			t.Logf("tick%d pos%v h%.3f speed%.1f diagnosis%+v", tick, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, d)
		}
	}
	t.Fatalf("player remained stranded pos%v heading%.3f speed%.1f", g.Bus.Pos, g.Bus.Heading, g.Bus.Speed)
}
