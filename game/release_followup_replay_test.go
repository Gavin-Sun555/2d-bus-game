package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"strconv"
	"testing"
)

func TestReleaseFollowupReplay(t *testing.T) {
	path := os.Getenv("BUS_FOLLOWUP_SAVE")
	if path == "" {
		t.Skip("BUS_FOLLOWUP_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	ticks := 600
	if v := os.Getenv("BUS_FOLLOWUP_TICKS"); v != "" {
		var err error
		ticks, err = strconv.Atoi(v)
		if err != nil || ticks < 1 {
			t.Fatal("invalid ticks")
		}
	}
	distance := 0.
	aiDistances := map[int]float64{}
	for tick := 0; tick < ticks; tick++ {
		before := g.Bus.Pos
		aiPositions := map[int]sim.Vec2{}
		for _, ab := range g.AIBusMgr.Buses {
			aiPositions[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		distance += g.Bus.Pos.Distance(before)
		for _, ab := range g.AIBusMgr.Buses {
			aiStep := ab.Bus.Pos.Distance(aiPositions[ab.ID])
			if aiStep > ab.Bus.MaxSpeedFwd/60+20 {
				t.Fatalf("AI%d discontinuity tick%d distance%.1f", ab.ID, tick, aiStep)
			}
			aiDistances[ab.ID] += aiStep
			if ab.Bus.Durability < 100 {
				t.Fatalf("AI%d contact tick%d pos%v health%.1f", ab.ID, tick, ab.Bus.Pos, ab.Bus.Durability)
			}
		}
		if g.Bus.Durability < 100 {
			logNearbyVehicles(t, g)
			t.Fatalf("contact tick%d pos%v health%.1f", tick, g.Bus.Pos, g.Bus.Durability)
		}
		if tick%300 == 0 || tick == ticks-1 || (ticks < 300 && tick%10 == 0) {
			diag := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			diag.Path = nil
			t.Logf("tick%d pos%v speed%.1f distance%.0f diag%+v", tick, g.Bus.Pos, g.Bus.Speed, distance, diag)
			t.Logf("tail%.4f steer%.4f body%+v", g.Bus.TailHeading(), g.Bus.SteeringAngle, g.Bus.BodyParts())
			logStoppedFleet(t, g)
		}
	}
	if path := os.Getenv("BUS_FOLLOWUP_CAPTURE"); path != "" {
		g.savePathOverride = path
		if err := g.SaveGame(); err != nil {
			t.Fatal(err)
		}
	}
	t.Logf("distance%.0f health%.0f AI distances%v", distance, g.Bus.Durability, aiDistances)
}
