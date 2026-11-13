package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedSlot1RetriesOvertakeFromBaseLane(t *testing.T) {
	path := os.Getenv("BUS_SLOT1_OVERTAKE_SAVE")
	if path == "" {
		t.Skip("BUS_SLOT1_OVERTAKE_SAVE not set")
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
	var lead *sim.TrafficCar
	for _, car := range g.TrafficMgr.Cars {
		if car.ID == 101148 {
			lead = car
		}
	}
	if lead == nil {
		t.Fatal("November 7 15:06 slot-one lead car missing")
	}
	health, passengers := g.Bus.Durability, g.Bus.PassengerCount
	changed := false
	for tick := 0; tick < 1200; tick++ {
		before := g.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health || g.Bus.PassengerCount != passengers || g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatalf("contact, lost passengers or relocation tick%d", tick)
		}
		for _, part := range g.Bus.BodyParts() {
			if part.HalfLen <= 0 {
				continue
			}
			for _, corner := range sim.BodyCorners(part) {
				if corner.Y < -.1 || corner.Y > 104.1 {
					t.Fatalf("left carriageway tick%d corner%v", tick, corner)
				}
			}
		}
		changed = changed || g.AutopilotDriver.TargetLaneOffset < -40
		if tick == 60 && !changed {
			t.Fatalf("did not retry clear passing lane: state%d offset%.1f/%.1f", g.AutopilotDriver.OvertakeState, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset)
		}
		if g.Bus.Pos.X-lead.Pos.X > g.Bus.OverallLength()+lead.Length/2+30 {
			t.Logf("passed original lead in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("did not pass original lead: bus%v lead%v speed%.1f", g.Bus.Pos, lead.Pos, g.Bus.Speed)
}
