package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"strconv"
	"testing"
)

func TestSavedStationEntryQueueClears(t *testing.T) {
	path := os.Getenv("BUS_STATION_ENTRY_QUEUE_SAVE")
	if path == "" {
		t.Skip("copied 2026-10-29 10:09 save required")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	seed := int64(5)
	if v := os.Getenv("BUS_STATION_ENTRY_QUEUE_SEED"); v != "" {
		var err error
		seed, err = strconv.ParseInt(v, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
	}
	sim.SeedSimulation(seed)
	start := g.Bus.Pos
	positions := map[int]sim.Vec2{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, a := range g.AIBusMgr.Buses {
		positions[a.ID] = a.Bus.Pos
		health[a.ID] = a.Bus.Durability
	}
	servedFront := false
	for tick := 0; tick < 14400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health[0] {
			t.Fatalf("player contact %.2fs health %.0f", float64(tick)/60, g.Bus.Durability)
		}
		if tick%1800 == 0 {
			t.Logf("%.0fs player distance%.1f pos%v", float64(tick)/60, g.Bus.Pos.Distance(start), g.Bus.Pos)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < health[a.ID] {
				t.Fatalf("AI%d contact %.2fs", a.ID, float64(tick)/60)
			}
			if a.ID == 7 && a.Bus.DoorsOpen && a.Driver.DockedStopID == 232 {
				servedFront = true
			}
			if a.ID == 7 && tick%1800 == 0 {
				d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, g.AIBusMgr.GetObstacles(a.ID), 52)
				t.Logf("AI7 moved%.1f pos%v state%v remaining%.1f targetSpd%.1f limits%v", a.Bus.Pos.Distance(positions[a.ID]), a.Bus.Pos, a.Driver.State, a.Driver.LastRemaining, a.Driver.LastTargetSpeed, d.Limits)
			}
		}
		if tick == 3599 && !servedFront {
			t.Fatal("front bus did not reach its empty boarding area within 60s")
		}
		allMoved := g.Bus.Pos.Distance(start) > 4000 && servedFront
		for _, a := range g.AIBusMgr.Buses {
			allMoved = allMoved && a.Bus.Pos.Distance(positions[a.ID]) > 800
		}
		if allMoved {
			t.Logf("whole queue cleared %.2fs; player distance%.1f; front served%v", float64(tick+1)/60, g.Bus.Pos.Distance(start), servedFront)
			return
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		t.Logf("AI%d distance%.1f state%v", a.ID, a.Bus.Pos.Distance(positions[a.ID]), a.Driver.State)
	}
	t.Fatal("queue remained blocked")
}
