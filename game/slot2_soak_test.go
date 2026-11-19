package game

import (
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestSlot2TrafficSoak replays an explicit, isolated save at the game's 60 Hz.
// It observes every vehicle, including the player, rather than relying on the
// debug snapshot retention limit or the AI's eventual reset to hide a jam.
func TestSlot2TrafficSoak(t *testing.T) {
	path, out := os.Getenv("BUS_SLOT2_SAVE"), os.Getenv("BUS_SLOT2_OUTPUT")
	if path == "" || out == "" {
		t.Skip("set BUS_SLOT2_SAVE and BUS_SLOT2_OUTPUT")
	}
	if err := os.MkdirAll(out, 0700); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	freshTimers := os.Getenv("BUS_SLOT2_FRESH_RECOVERY") == "1"
	if freshTimers {
		for _, c := range g.TrafficMgr.Cars {
			c.DeadlockTimer = 0
			c.DeadlockTracker = sim.StationaryTracker{}
		}
		for _, a := range g.AIBusMgr.Buses {
			a.DeadlockTimer = 0
			a.DeadlockTracker = sim.StationaryTracker{}
		}
	}
	if !g.Settings.AutopilotEnabled {
		t.Fatal("soak requires the saved autopilot")
	}
	seconds := 600
	if s := os.Getenv("BUS_SLOT2_SECONDS"); s != "" {
		var err error
		seconds, err = strconv.Atoi(s)
		if err != nil || seconds < 1 {
			t.Fatal("invalid BUS_SLOT2_SECONDS")
		}
	}
	f, err := os.Create(filepath.Join(out, "events.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	enc := json.NewEncoder(f)
	tick, incidents := 0, 0
	capture := func(reason string, kind string, id int, pos sim.Vec2) {
		incidents++
		name := fmt.Sprintf("%06d-%s-%s%d.json", tick, reason, kind, id)
		g.savePathOverride = filepath.Join(out, name)
		if err := g.SaveGame(); err != nil {
			t.Fatal(err)
		}
		if err := enc.Encode(map[string]any{"tick": tick, "reason": reason, "kind": kind, "id": id, "position": pos, "capture": name}); err != nil {
			t.Fatal(err)
		}
		t.Logf("t=%.1f %s %s%d pos%v", float64(tick)/60, reason, kind, id, pos)
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { capture(e.Reason, e.Kind, e.ID, e.Pos) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { capture(e.Reason, e.Kind, e.ID, e.Pos) }
	g.AutopilotDriver.BeforeRecovery = func() { capture("local-recovery", "player", 0, g.Bus.Pos) }
	for _, a := range g.AIBusMgr.Buses {
		a := a
		a.Driver.BeforeRecovery = func() { capture("local-recovery", "bus", a.ID, a.Bus.Pos) }
	}
	type progress struct {
		anchor                sim.Vec2
		still                 int
		maxStill              int
		distance              float64
		last                  sim.Vec2
		health                float64
		areaAnchor            sim.Vec2
		confined, maxConfined int
	}
	watches := map[debugVehicleKey]*progress{}
	observe := func(k debugVehicleKey, pos sim.Vec2, health float64, service bool) {
		w := watches[k]
		if w == nil {
			w = &progress{anchor: pos, areaAnchor: pos, last: pos, health: health}
			watches[k] = w
		}
		step := pos.Distance(w.last)
		w.distance += step
		w.last = pos
		if math.IsNaN(pos.X) || math.IsNaN(pos.Y) || math.IsInf(pos.X, 0) || math.IsInf(pos.Y, 0) {
			t.Fatal("invalid position", k)
		}
		if health < w.health {
			capture("damage", k.Kind, k.ID, pos)
		}
		w.health = health
		if service || pos.Distance(w.anchor) > 10 {
			w.anchor, w.still = pos, 0
		} else {
			w.still++
		}
		w.maxStill = max(w.maxStill, w.still)
		// Repeated short reverse/forward attempts are movement, but not a
		// released bottleneck. Observe progress outside the local 80px area
		// separately so rocking cannot hide a deadlock from the 10px watch.
		if service || pos.Distance(w.areaAnchor) > 80 {
			w.areaAnchor, w.confined = pos, 0
		} else {
			w.confined++
		}
		w.maxConfined = max(w.maxConfined, w.confined)
		if w.confined == 120*60 || w.confined == 180*60 {
			capture(fmt.Sprintf("confined-%ds", w.confined/60), k.Kind, k.ID, pos)
		}
		// A normal signal cycle can exceed 45 seconds. Keep that observation
		// in the log; only waits exceeding two full cycles are incidents.
		if w.still == 60*45 {
			_ = enc.Encode(map[string]any{"tick": tick, "observation": "stationary-45s", "kind": k.Kind, "id": k.ID, "position": pos})
		}
		deadline := max(90, int(math.Ceil(g.TrafficLights.DeadlockTimeoutAt(pos)))) * 60
		if w.still == deadline || w.still == deadline+60*60 {
			capture(fmt.Sprintf("stationary-%ds", w.still/60), k.Kind, k.ID, pos)
		}
	}
	diagnose := func(id int, bus *sim.Bus, driver *autopilot.AutopilotDriver, route *sim.Route, obs []autopilot.VehicleObstacle) {
		d := driver.DiagnoseStop(bus, route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
		d.Path = nil
		if err := enc.Encode(map[string]any{"tick": tick, "id": id, "position": bus.Pos, "heading": bus.Heading, "speed": bus.Speed, "diagnosis": d, "driver": driver.Snapshot()}); err != nil {
			t.Fatal(err)
		}
		if id == 0 {
			t.Logf("t=%.1f player pos%v speed%.1f state%s limits%v", float64(tick)/60, bus.Pos, bus.Speed, d.State, d.Limits)
		}
	}
	t.Logf("source=%s seconds=%d cars=%d buses=%d", path, seconds, len(g.TrafficMgr.Cars), len(g.AIBusMgr.Buses))
	tracePhase := map[int]sim.LightPhase{}
	traceCars := map[int]bool{}
	for _, value := range strings.Split(os.Getenv("BUS_SLOT2_TRACE_CARS"), ",") {
		if id, err := strconv.Atoi(strings.TrimSpace(value)); err == nil {
			traceCars[id] = true
		}
	}
	for tick = 1; tick <= seconds*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		observe(debugVehicleKey{"player", 0}, g.Bus.Pos, g.Bus.Durability, g.Bus.DoorsOpen || g.Bus.PassengerServiceRemaining() > 0)
		for _, a := range g.AIBusMgr.Buses {
			observe(debugVehicleKey{"bus", a.ID}, a.Bus.Pos, a.Bus.Durability, a.Bus.DoorsOpen || a.Bus.PassengerServiceRemaining() > 0 || a.Bus.TerminalReplacementHold)
		}
		for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			observe(debugVehicleKey{"car", c.ID}, c.Pos, 100, false)
			if traceCars[c.ID] {
				var nearest *sim.TrafficIntersection
				for _, it := range g.TrafficLights.Intersections {
					if nearest == nil || it.Center.Distance(c.Pos) < nearest.Center.Distance(c.Pos) {
						nearest = it
					}
				}
				if old, ok := tracePhase[c.ID]; !ok || old != nearest.Phase {
					tracePhase[c.ID] = nearest.Phase
					_ = enc.Encode(map[string]any{"trace": true, "tick": tick, "car": c.ID, "position": c.Pos, "speed": c.Speed, "junction": nearest.Center, "phase": nearest.Phase, "timer": nearest.Timer, "wait": nearest.PriorityPhaseWait, "queue": nearest.BusPriorityQueue})
				}
			}
		}
		if tick == 1 || tick%1800 == 0 {
			diagnose(0, g.Bus, g.AutopilotDriver, g.Route, g.obstacles)
			for _, a := range g.AIBusMgr.Buses {
				if math.Abs(a.Bus.Speed) >= 3 || a.Bus.DoorsOpen {
					continue
				}
				obs := append(g.AIBusMgr.GetObstacles(a.ID), autopilot.ObserveBus(g.Bus))
				for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
					if c.Pos.Distance(a.Bus.Pos) < 1600 {
						obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
					}
				}
				diagnose(a.ID, a.Bus, a.Driver, a.Route, obs)
			}
		}
	}
	g.savePathOverride = filepath.Join(out, "end-state.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	for k, w := range watches {
		if k.Kind != "car" || w.still > 60*45 {
			_ = enc.Encode(map[string]any{"summary": true, "kind": k.Kind, "id": k.ID, "distance": w.distance, "stationary_seconds": float64(w.still) / 60})
		}
	}
	source, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	maxWait := map[string]float64{}
	maxConfined := map[string]float64{}
	distance := map[string]float64{}
	for k, w := range watches {
		maxWait[k.Kind] = max(maxWait[k.Kind], float64(w.maxStill)/60)
		maxConfined[k.Kind] = max(maxConfined[k.Kind], float64(w.maxConfined)/60)
		if k.Kind != "car" {
			distance[fmt.Sprintf("%s%d", k.Kind, k.ID)] = w.distance
		}
	}
	report := map[string]any{"source": path, "source_sha256": fmt.Sprintf("%x", sha256.Sum256(source)), "seed": 5, "fresh_recovery_timers": freshTimers, "simulation_seconds": seconds, "ticks": seconds * 60, "cars": len(g.TrafficMgr.Cars), "ai_buses": len(g.AIBusMgr.Buses), "incidents": incidents, "max_stationary_seconds": maxWait, "bus_distance_pixels": distance}
	report["max_confined_seconds"] = maxConfined
	raw, err := json.MarshalIndent(report, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(out, "report.json"), raw, 0600); err != nil {
		t.Fatal(err)
	}
	t.Logf("completed %.1f simulation seconds; %d incident captures", float64(seconds), incidents)
	if incidents > 0 {
		t.Fatalf("%d incidents require review in %s", incidents, out)
	}
}
