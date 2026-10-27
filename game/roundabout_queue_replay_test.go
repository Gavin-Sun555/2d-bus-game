package game

import (
	"encoding/json"
	"os"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestArticulatedRoundaboutQueueReplay(t *testing.T) {
	path := os.Getenv("BUS_RING_REPLAY")
	if path == "" && os.Getenv("BUS_RING_FRESH") != "1" {
		t.Skip("BUS_RING_REPLAY not set")
	}
	sim.SeedSimulation(705)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if os.Getenv("BUS_RING_FRESH") == "1" {
		g.Settings.AutopilotEnabled = true
		g.Settings.AutopilotBerserk = true
		g.Settings.AutopilotBerserkMaxSpeed = 120
		g.Settings.TrafficDensity = 4
		g.EnterCity(sim.MapThemeCustom)
		g.SetLineFleet(0, traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 3, 3}, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 80}})
		g.Bus.SetModel(sim.BusArticulated)
		g.ResetBusesToRouteStart()
	} else if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	ticks := 3600
	if s := os.Getenv("BUS_RING_TICKS"); s != "" {
		n, e := strconv.Atoi(s)
		if e != nil || n < 1 {
			t.Fatal("invalid ticks")
		}
		ticks = n
	}
	distance := 0.
	for tick := 0; tick < ticks; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		distance += g.Bus.Pos.Distance(before)
		if g.Bus.Durability < 100 {
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(g.Bus.Pos) < 160 {
					t.Logf("contact near car%d pos%v h%.3f speed%.2f curve%+v", c.ID, c.Pos, c.Heading, c.Speed, c.RoundaboutCurve)
				}
			}
			t.Fatalf("contact tick%d pos%v health%.1f", tick, g.Bus.Pos, g.Bus.Durability)
		}
		if tick%300 == 0 {
			diag := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			diag.Path = nil
			t.Logf("tick%d pos%v speed%.1f distance%.0f steer%.3f pursuit%v diag%+v", tick, g.Bus.Pos, g.Bus.Speed, distance, g.Bus.SteeringAngle, g.AutopilotDriver.LastPursuitPt, diag)
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(g.Bus.Pos) < 240 {
					t.Logf("car%d pos%v heading%.3f speed%.1f ring%v r%.1f angle%.3f exit%.3f traveled%.3f curve%v", c.ID, c.Pos, c.Heading, c.Speed, c.InRoundabout, c.RoundaboutRadius, c.RoundaboutAngle, c.RoundaboutExitAngle, c.RoundaboutTraveled, c.RoundaboutCurve != nil)
				}
			}
		}
		if tick == ticks-1 && os.Getenv("BUS_RING_CAPTURE") != "" {
			type capture struct {
				Bus        *sim.Bus
				Cars       []*sim.TrafficCar
				Roundabout sim.Roundabout
			}
			f := capture{Bus: g.Bus, Roundabout: g.World.Roundabouts[0]}
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(f.Roundabout.Center) < 600 {
					f.Cars = append(f.Cars, c)
				}
			}
			if saved := os.Getenv("BUS_RING_CAPTURE_SAVE"); saved != "" {
				g.savePathOverride = saved
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			b, e := json.MarshalIndent(f, "", "  ")
			if e != nil {
				t.Fatal(e)
			}
			if e = os.WriteFile(os.Getenv("BUS_RING_CAPTURE"), b, 0644); e != nil {
				t.Fatal(e)
			}
		}
	}
	if distance < 3000 {
		t.Fatalf("ring queue stalled: distance%.1f pos%v", distance, g.Bus.Pos)
	}
	t.Logf("distance%.1f health%.0f", distance, g.Bus.Durability)
}
