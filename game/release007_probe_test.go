package game

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"strconv"
	"testing"
)

func TestRelease007ScenarioProbe(t *testing.T) {
	if os.Getenv("BUSGAME_QUALIFY_PROBE") != "1" {
		t.Skip("local qualification diagnosis")
	}
	sim.SeedSimulation(91)
	g := NewGame(1280, 720)
	g.Settings.AICarMaxSpeed = 100
	g.Settings.AIBusMaxSpeed = 80
	g.Settings.AIBusCount = 8
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	g.EnterCity(sim.MapThemeMetropolis)
	distance := 0.
	steps := 7200
	if value := os.Getenv("BUSGAME_QUALIFY_PROBE_STEPS"); value != "" {
		n, err := strconv.Atoi(value)
		if err != nil || n < 1 {
			t.Fatal("invalid BUSGAME_QUALIFY_PROBE_STEPS")
		}
		steps = n
	}
	for tick := 0; tick < steps; tick++ {
		previous := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		distance += previous.Distance(g.Bus.Pos)
		if tick == 2100 && os.Getenv("BUSGAME_QUALIFY_STALL_SAVE") != "" {
			g.savePathOverride = os.Getenv("BUSGAME_QUALIFY_STALL_SAVE")
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
		}
		if tick%300 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			d.Path = nil
			data, _ := json.Marshal(struct {
				Tick            int
				Pos             sim.Vec2
				Speed, Distance float64
				Diagnosis       any
			}{tick, g.Bus.Pos, g.Bus.Speed, distance, d})
			t.Log(string(data))
			if g.Bus.Speed < 1 {
				for _, ab := range g.AIBusMgr.Buses {
					if ab.Bus.Pos.Distance(g.Bus.Pos) > 1300 {
						continue
					}
					d := ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, g.AIBusMgr.GetObstacles(ab.ID), g.Settings.GetLaneWidth())
					d.Path = nil
					t.Logf("AI%d pos%v h%.3f speed%.2f target%d diag%+v", ab.ID, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Speed, ab.Route.GetCurrentTargetStopID(), d)
				}
				for _, c := range g.TrafficMgr.GetActiveCars(4) {
					if c.Pos.Distance(g.Bus.Pos) < 230 {
						t.Logf("car%d pos%v h%.3f speed%.2f turn%+v", c.ID, c.Pos, c.Heading, c.Speed, c.JunctionTurn)
					}
				}
			}
		}
	}
}
