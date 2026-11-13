package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedPlayerKeepsInnerLeftLane(t *testing.T) {
	path := os.Getenv("BUS_LEFT_LANE_SAVE")
	if path == "" {
		t.Skip("BUS_LEFT_LANE_SAVE not set")
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
	health := g.Bus.Durability
	junction := sim.Vec2{X: -62400}
	geometry := g.World.JunctionAt(junction, 52)
	selected := false
	for tick := 0; tick < 7200; tick++ {
		before := g.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 || g.Bus.Durability < health {
			t.Fatalf("relocation or contact at tick%d", tick)
		}
		snapshot := g.AutopilotDriver.Snapshot()
		if g.Bus.Pos.Y < -geometry.HalfY-100 {
			if snapshot.BusLaneTurnChosen {
				selected = true
			}
			if selected && (!snapshot.OrdinaryBusLeft || math.Abs(g.Bus.Pos.X+62426) > 12) {
				t.Fatalf("abandoned inner left lane at %.2fs pos%v choice%v offset%.1f", float64(tick)/60, g.Bus.Pos, snapshot.OrdinaryBusLeft, g.AutopilotDriver.TargetLaneOffset)
			}
		}
		if selected && g.Bus.Pos.X > junction.X+geometry.HalfX+150 {
			t.Logf("completed left turn without changing approach lane at %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatalf("did not complete left turn pos%v speed%.1f selected%v", g.Bus.Pos, g.Bus.Speed, selected)
}
