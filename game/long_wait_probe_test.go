package game

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"
)

// Opt-in deterministic replay of the fullscreen soak; writes only isolated artifacts.
func TestLongWaitDiagnosis(t *testing.T) {
	out := os.Getenv("BUS_LONG_WAIT_OUTPUT")
	if out == "" {
		t.Skip("set BUS_LONG_WAIT_OUTPUT")
	}
	if err := os.MkdirAll(out, 0755); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.Settings.TrafficDensity = 4
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.IsOpen = false
	g.SetTransitLineCount(6)
	for i := 0; i < 6; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
	}
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.savePathOverride = filepath.Join(out, "wait-state.json")
	file, err := os.Create(filepath.Join(out, "diagnosis.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	encoder := json.NewEncoder(file)
	distance := 0.
	for tick := 1; tick <= 36000; tick++ {
		pos := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		distance += pos.Distance(g.Bus.Pos)
		if tick%3000 == 0 {
			t.Logf("tick=%d distance=%.2f cars=%d pos=%v target=%d", tick, distance, len(g.TrafficMgr.GetActiveCars(4)), g.Bus.Pos, g.Route.GetCurrentTargetStopID())
		}
		if tick == 30000 || tick == 33000 || tick == 36000 {
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			data, _ := os.ReadFile(g.savePathOverride)
			if err := os.WriteFile(filepath.Join(out, "state-"+strconv.Itoa(tick)+".json"), data, 0600); err != nil {
				t.Fatal(err)
			}
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			if err := encoder.Encode(map[string]any{"tick": tick, "bus": g.Bus, "diagnosis": d, "snapshot": g.AutopilotDriver.Snapshot()}); err != nil {
				t.Fatal(err)
			}
			for _, ab := range g.AIBusMgr.Buses {
				if ab.Bus.Pos.Distance(g.Bus.Pos) > 1800 {
					continue
				}
				obs := g.AIBusMgr.GetObstacles(ab.ID)
				obs = append(obs, autopilot.ObserveBus(g.Bus))
				for _, c := range g.TrafficMgr.GetActiveCars(4) {
					obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
				}
				diag := ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
				encoder.Encode(map[string]any{"tick": tick, "id": ab.ID, "bus": ab.Bus, "diagnosis": diag, "snapshot": ab.Driver.Snapshot()})
			}
		}
	}
}

func TestLongWaitIsolateLeader(t *testing.T) {
	path := os.Getenv("BUS_LONG_WAIT_STATE")
	if path == "" {
		t.Skip("set BUS_LONG_WAIT_STATE")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID != 8 {
			continue
		}
		b := ab.Bus
		lw := g.Settings.GetLaneWidth()
		d := ab.Driver.DiagnoseStop(b, ab.Route, g.World, g.TrafficLights, nil, lw)
		d.Path = nil
		t.Logf("empty obstacles limits=%v current boundary=%+v bus=%+v", d.Limits, b.CheckBoundaries(g.World, lw), struct {
			Pos                     sim.Vec2
			Heading, Tail, Steering float64
		}{b.Pos, b.Heading, b.TailHeading(), b.SteeringAngle})
		_, _, steer, _, _, _ := ab.Driver.ComputeControls(b, ab.Route, g.World, g.TrafficLights, nil, lw, 1./60)
		for _, v := range []float64{steer, -.3, -.1, 0, .1, .3} {
			copy := *b
			for tick := 1; tick <= 16; tick++ {
				wheel := copy.SteeringAngle + sim.Clamp(v*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
				angle := copy.Heading + 16/copy.WheelBase*math.Tan(wheel)*.05
				copy = copy.AtPose(copy.Pos.Add(sim.UnitFromAngle(angle).Mul(16*.05)), angle)
				copy.SteeringAngle = wheel
				if tick%4 == 0 {
					zone := copy.CheckBoundaries(g.World, lw)
					t.Logf("steer %.2f tick%d pose%v zone%+v", v, tick, copy.Pos, zone)
				}
			}
		}
	}
}

func TestLongWaitRecoveryReplay(t *testing.T) {
	path := os.Getenv("BUS_LONG_WAIT_STATE")
	if path == "" {
		t.Skip("set BUS_LONG_WAIT_STATE")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	initialTarget := g.Route.GetCurrentTargetStopID()
	openedTarget, servedTarget := false, false
	traveled := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	firstAI, firstPlayer := -1., -1.
	for tick := 0; tick < 7200; tick++ {
		pos := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, ab := range g.AIBusMgr.Buses {
			pos[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		traveled[0] += pos[0].Distance(g.Bus.Pos)
		openedTarget = openedTarget || (g.Bus.DoorsOpen && g.AutopilotDriver.DockedStopID == initialTarget)
		servedTarget = servedTarget || (openedTarget && g.Route.GetCurrentTargetStopID() != initialTarget)
		for _, ab := range g.AIBusMgr.Buses {
			traveled[ab.ID] += pos[ab.ID].Distance(ab.Bus.Pos)
			if ab.Bus.Durability < health[ab.ID] {
				t.Fatalf("AI%d damage %.2f -> %.2f", ab.ID, health[ab.ID], ab.Bus.Durability)
			}
		}
		if g.Bus.Durability < health[0] {
			t.Fatal("player damage")
		}
		if firstAI < 0 && traveled[8] > 50 {
			firstAI = float64(tick) / 60
		}
		if firstPlayer < 0 && traveled[0] > 100 {
			firstPlayer = float64(tick) / 60
		}
		if tick%1200 == 0 {
			t.Logf("tick%d player %.1f AI8 %.1f cars%d", tick, traveled[0], traveled[8], len(g.TrafficMgr.GetActiveCars(4)))
		}
	}
	t.Logf("player %.1f AI8 %.1f first progress player %.2fs AI8 %.2fs", traveled[0], traveled[8], firstPlayer, firstAI)
	g.savePathOverride = filepath.Join(filepath.Dir(path), "post-recovery-state.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	diagnostics := []any{map[string]any{"id": 0, "bus": g.Bus, "diagnosis": g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth()), "snapshot": g.AutopilotDriver.Snapshot()}}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.Bus.Pos.Distance(g.Bus.Pos) > 1800 {
			continue
		}
		obs := g.AIBusMgr.GetObstacles(ab.ID)
		obs = append(obs, autopilot.ObserveBus(g.Bus))
		for _, c := range g.TrafficMgr.GetActiveCars(4) {
			obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
		}
		diagnostics = append(diagnostics, map[string]any{"id": ab.ID, "bus": ab.Bus, "diagnosis": ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth()), "snapshot": ab.Driver.Snapshot()})
	}
	data, err := json.Marshal(diagnostics)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(filepath.Dir(path), "post-recovery-diagnosis.json"), data, 0600); err != nil {
		t.Fatal(err)
	}
	t.Logf("original target %d served=%v opened=%v", initialTarget, servedTarget, openedTarget)
	if traveled[8] < 600 || traveled[0] < 300 || !servedTarget {
		t.Fatal("queue did not resume")
	}
}
