package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"
)

func TestDebugCaptureSceneReleases(t *testing.T) {
	path := os.Getenv("BUS_DEBUG_REPLAY")
	if path == "" {
		t.Skip("BUS_DEBUG_REPLAY not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	e, err := savefile.Read[SaveMetadata](path)
	if err != nil || e.Metadata == nil || e.Metadata.Debug == nil {
		t.Fatal("debug metadata missing", err)
	}
	meta := e.Metadata.Debug
	if meta.VehicleKind != "car" {
		t.Skip("car replay requires car capture")
	}
	p := filepath.Join(t.TempDir(), "save.json")
	if err = os.WriteFile(p, raw, 0600); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if err = g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	// Captures are taken at the timeout boundary. Give the repaired controller
	// a fresh observation window, while forbidding reset as proof of release.
	if os.Getenv("BUS_DEBUG_KEEP_TIMERS") != "1" {
		for _, c := range g.TrafficMgr.Cars {
			c.DeadlockTimer = 0
			c.DeadlockTracker = sim.StationaryTracker{}
		}
		for _, a := range g.AIBusMgr.Buses {
			a.DeadlockTimer = 0
			a.DeadlockTracker = sim.StationaryTracker{}
		}
	}
	var car *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == meta.VehicleID {
			car = c
		}
	}
	if car == nil {
		t.Fatal("trigger missing")
	}
	g.TrafficMgr.BeforeRecovery = func(ev sim.TrafficRecoveryEvent) {
		if ev.ID == car.ID {
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(car.Pos) < 500 {
					t.Logf("near reset bus%d pos%v h%.4f v%.6f", a.ID, a.Bus.Pos, a.Bus.Heading, a.Bus.Speed)
				}
			}
			t.Fatalf("trigger reset after %.1fsec pos%v heading%.4f", car.DeadlockTimer, car.Pos, car.Heading)
		}
	}
	type watchedBus struct {
		bus    *sim.Bus
		start  sim.Vec2
		health float64
	}
	var watched []watchedBus
	for _, a := range g.AIBusMgr.Buses {
		if a.Bus.Pos.Distance(meta.Position) < 500 {
			watched = append(watched, watchedBus{a.Bus, a.Bus.Pos, a.Bus.Durability})
			if os.Getenv("BUS_DEBUG_NO_LOCAL_RECOVERY") == "1" {
				id := a.ID
				a.Driver.BeforeRecovery = func() { t.Fatalf("nearby bus%d repositioned", id) }
			}
		}
	}
	traveled := 0.
	last := car.Pos
	phase := sim.LightPhase(-1)
	peak := 0.
	var inter *sim.TrafficIntersection
	for _, it := range g.TrafficLights.Intersections {
		if it.Center.Distance(*meta.Intersection) < 1 {
			inter = it
			break
		}
	}
	seconds := 150
	if raw := os.Getenv("BUS_DEBUG_REPLAY_SECONDS"); raw != "" {
		var err error
		seconds, err = strconv.Atoi(raw)
		if err != nil || seconds < 1 || seconds > 600 {
			t.Fatal("BUS_DEBUG_REPLAY_SECONDS must be 1..600")
		}
	}
	for tick := 0; tick < seconds*60; tick++ {
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, w := range watched {
			if w.bus.Durability < w.health {
				t.Fatal("nearby bus damaged during recovery")
			}
		}
		step := car.Pos.Distance(last)
		if step > math.Abs(car.Speed)/60+.1 {
			t.Fatalf("trigger relocated step%.1f", step)
		}
		traveled += step
		last = car.Pos
		peak = math.Max(peak, car.Speed)
		if inter != nil && phase != inter.Phase {
			t.Logf("t%.1f phase%d ->%d peak%.1f dist%.1f pos%v", float64(tick)/60, phase, inter.Phase, peak, traveled, car.Pos)
			phase = inter.Phase
			peak = 0
		}
		live := false
		for _, c := range g.TrafficMgr.Cars {
			if c == car {
				live = true
				break
			}
		}
		if traveled > 650 || (!live && traveled > 400) {
			for _, w := range watched {
				t.Logf("nearby bus displacement%.1f", w.bus.Pos.Distance(w.start))
			}
			t.Logf("released t%.2f distance%.1f", float64(tick+1)/60, traveled)
			return
		}
	}
	t.Fatalf("trigger did not clear replay window (%ds): %v distance%.1f", seconds, car.Pos, traveled)
}
