package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func loadNov8Entry(t *testing.T) *Game {
	t.Helper()
	p := os.Getenv("BUS_NOV8_ENTRY_SAVE")
	if p == "" {
		t.Skip("BUS_NOV8_ENTRY_SAVE not set")
	}
	raw, err := os.ReadFile(p)
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
func TestSavedNov8EntryQueueRecovers(t *testing.T) {
	g := loadNov8Entry(t)
	for tick := 0; tick < 3; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		if a.ID != 30 && a.ID != 28 && a.ID != 26 {
			continue
		}
		obs := g.AIBusMgr.GetLocalObstacles(a.Bus.Pos, a.Bus.Speed, nil)
		filtered := obs[:0]
		for _, o := range obs {
			if o.Bus != a.Bus {
				filtered = append(filtered, o)
			}
		}
		d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, filtered, 52)
		d.Path = nil
		t.Logf("AI%d pos%v state%+v diag%+v", a.ID, a.Bus.Pos, a.Driver.State, d)
	}
	var root *sim.Bus
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 30 {
			root = a.Bus
		}
	}
	if root == nil {
		t.Fatal("root AI30 missing")
	}
	var sidecar *sim.TrafficCar
	for _, car := range g.TrafficMgr.Cars {
		if car.ID == 102389 {
			sidecar = car
		}
	}
	if sidecar == nil {
		t.Fatal("sidecar missing")
	}
	carStart := sidecar.Pos
	start, health := root.Pos, root.Durability
	for tick := 0; tick < 7200; tick++ {
		before := root.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if root.Durability < health || root.Pos.Distance(before) > math.Abs(root.Speed)/60+.05 {
			t.Fatalf("contact or relocation tick%d", tick)
		}
		live := false
		for _, car := range g.TrafficMgr.Cars {
			if car == sidecar {
				live = true
				break
			}
		}
		if !live || sidecar.Pos.Distance(carStart) > sidecar.TargetSpeed*float64(tick+1)/60+100 {
			t.Fatal("sidecar was reset instead of released")
		}
		if root.Pos.Distance(start) > 1000 && sidecar.Pos.X > carStart.X+100 {
			t.Logf("root cleared in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("root stayed jammed %v", root.Pos)

}
