package game

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"os"
	"strconv"
	"testing"
)

// Optional qualification captures are kept outside source control. Never writes
// the user's save; BUS_QUALIFICATION_REPLAY selects the captured input.
func TestQualificationProgressReplay(t *testing.T) {
	path := os.Getenv("BUS_QUALIFICATION_REPLAY")
	if path == "" {
		t.Skip("set BUS_QUALIFICATION_REPLAY")
	}
	sim.SeedSimulation(5)
	if os.Getenv("BUS_QUALIFICATION_FRESH_LAKE") == "1" {
		sim.SeedSimulation(503)
	}
	g := NewGame(1280, 720)
	if os.Getenv("BUS_QUALIFICATION_NATIVE") == "1" {
		var s struct {
			World  *sim.World
			Player *sim.Bus
			Driver autopilot.DriverSnapshot
			Route  *sim.Route
			Fleet  []struct {
				ID     int
				Bus    *sim.Bus
				Route  *sim.Route
				Driver autopilot.DriverSnapshot
			}
			Cars   []*sim.TrafficCar
			Lights *sim.TrafficLightSystem
		}
		b, e := os.ReadFile(path)
		if e != nil {
			t.Fatal(e)
		}
		if e = json.Unmarshal(b, &s); e != nil {
			t.Fatal(e)
		}
		g.Settings.AutopilotEnabled = true
		g.Settings.AutopilotBerserk = true
		g.Settings.TrafficDensity = 4
		g.EnterCity(sim.MapThemeMetropolis)
		g.SetTransitLineCount(6)
		for i := 0; i < 6; i++ {
			g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
		}
		g.World = s.World
		g.Bus = s.Player
		g.Route = s.Route
		g.Lines[0] = s.Route
		g.TrafficLights = s.Lights
		g.AutopilotDriver, e = s.Driver.Restore(s.World)
		if e != nil {
			t.Fatal(e)
		}
		g.TrafficMgr.Cars = s.Cars
		g.AIBusMgr.Buses = nil
		for _, a := range s.Fleet {
			d, e := a.Driver.Restore(s.World)
			if e != nil {
				t.Fatal(e)
			}
			g.AIBusMgr.Buses = append(g.AIBusMgr.Buses, &traffic.AIBus{LineIndex: (a.ID - 1) / 8, ID: a.ID, Bus: a.Bus, Route: a.Route, Driver: d, MaxSpeedKPH: 80, CanOvertake: true, LastExchangedStopID: -1})
		}
	} else if os.Getenv("BUS_QUALIFICATION_FRESH_LAKE") == "1" {
		g.Settings.AutopilotEnabled = true
		g.Settings.AutopilotBerserk = true
		g.Settings.AutopilotBerserkMaxSpeed = 120
		g.Settings.TrafficDensity = 4
		g.Settings.AIBusCount = 8
		g.EnterCity(sim.MapThemeEmeraldLake)
	} else {
		g.savePathOverride = path
		if e := g.LoadGame(); e != nil {
			t.Fatal(e)
		}
	}
	leaderID := 37
	if raw := os.Getenv("BUS_QUALIFICATION_LEADER"); raw != "" {
		n, err := strconv.Atoi(raw)
		if err != nil || n < 1 {
			t.Fatal("invalid tracked leader")
		}
		leaderID = n
	}
	distance := 0.
	ticks := 7200
	if raw := os.Getenv("BUS_QUALIFICATION_TICKS"); raw != "" {
		n, e := strconv.Atoi(raw)
		if e != nil || n < 1 {
			t.Fatal("invalid replay ticks")
		}
		ticks = n
	}
	initialTarget := g.Route.GetCurrentTargetStopID()
	anchor := g.Bus.Pos
	idle, maxIdle := 0., 0.
	fleetHealth := map[int]float64{}
	leaderDistance := 0.
	for _, a := range g.AIBusMgr.Buses {
		fleetHealth[a.ID] = a.Bus.Durability
	}
	health := g.Bus.Durability
	for tick := 0; tick < ticks; tick++ {
		p := g.Bus.Pos
		var leaderBefore sim.Vec2
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == leaderID {
				leaderBefore = a.Bus.Pos
			}
		}
		if e := g.Update(); e != nil {
			t.Fatal(e)
		}
		distance += g.Bus.Pos.Distance(p)
		if g.Bus.Pos.Distance(anchor) > 8 {
			anchor = g.Bus.Pos
			idle = 0
		} else {
			idle += 1. / 60
			maxIdle = max(maxIdle, idle)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < fleetHealth[a.ID] {
				t.Fatalf("AI %d contact tick%d", a.ID, tick)
			}
			if a.ID == leaderID {
				leaderDistance += a.Bus.Pos.Distance(leaderBefore)
			}
		}
		if g.Bus.Durability < health {
			t.Fatalf("contact tick%d", tick)
		}
		if tick%600 == 0 || tick == ticks-1 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			d.Path = nil
			t.Logf("tick%d traveled%.1f pos%v diagnosis%+v", tick, distance, g.Bus.Pos, d)
			if os.Getenv("BUS_QUALIFICATION_NATIVE") == "1" {
				for _, a := range g.AIBusMgr.Buses {
					if a.ID == leaderID {
						d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, nil, 52)
						d.Path = nil
						t.Logf("AI%d pos%v h%.3f tail%.3f steer%.3f boundary%+v diagnosis%+v", a.ID, a.Bus.Pos, a.Bus.Heading, a.Bus.TailHeading(), a.Bus.SteeringAngle, a.Bus.CheckBoundaries(g.World, 52), d)
					}
				}
			}
		}
	}
	if os.Getenv("BUS_QUALIFICATION_CAPTURE") != "" {
		g.savePathOverride = os.Getenv("BUS_QUALIFICATION_CAPTURE")
		if e := g.SaveGame(); e != nil {
			t.Fatal(e)
		}
	}
	minimum := 3000.
	if os.Getenv("BUS_QUALIFICATION_NATIVE") == "1" {
		minimum = 1000
		if g.Route.GetCurrentTargetStopID() == initialTarget || leaderDistance < 500 {
			t.Fatalf("original queue did not clear: target%d leader%.0fpx", g.Route.GetCurrentTargetStopID(), leaderDistance)
		}
	}
	if os.Getenv("BUS_QUALIFICATION_FRESH_LAKE") == "1" {
		minimum = 10000
	}
	t.Logf("REPLAY_SUMMARY player%.1fpx leader%.1fpx longestWait%.2fs target%d->%d health%.0f", distance, leaderDistance, maxIdle, initialTarget, g.Route.GetCurrentTargetStopID(), g.Bus.Durability)
	if distance < minimum || maxIdle > 120 {
		t.Fatalf("recovery failed: %.0fpx (minimum%.0f), longest wait%.1fs", distance, minimum, maxIdle)
	}
}
