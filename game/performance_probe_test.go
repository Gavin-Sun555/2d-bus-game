package game

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"runtime/pprof"
	"slices"
	"strconv"
	"testing"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Opt-in profiling only: fixed scenes or a copied save, no writes to user saves.
// Warm-up is excluded from CPU and allocation profiles and tick statistics.
func TestPerformanceProbe(t *testing.T) {
	out := os.Getenv("BUS_PROBE_OUTPUT")
	if out == "" {
		t.Skip("set BUS_PROBE_OUTPUT for offline profiling")
	}
	if err := os.MkdirAll(out, 0755); err != nil {
		t.Fatal(err)
	}
	value := func(name string, fallback int) int {
		if s := os.Getenv(name); s != "" {
			n, err := strconv.Atoi(s)
			if err != nil || n < 0 {
				t.Fatalf("invalid %s", name)
			}
			return n
		}
		return fallback
	}
	ticks, warmup := value("BUS_PROBE_TICKS", 1800), value("BUS_PROBE_WARMUP", 300)
	if ticks < 1 {
		t.Fatal("ticks must be positive")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if saved := os.Getenv("BUS_PROBE_SAVE"); saved != "" {
		g.savePathOverride = saved
		if err := g.LoadGame(); err != nil {
			t.Fatal(err)
		}
	} else {
		g.Settings.TrafficDensity = 4
		g.Settings.AutopilotEnabled, g.Settings.AutopilotBerserk = true, true
		g.EnterCity(sim.MapTheme(value("BUS_PROBE_THEME", int(sim.MapThemeMetropolis))))
		n := value("BUS_PROBE_BUSES", 48)
		if n > 48 {
			t.Fatal("at most 48 AI buses")
		}
		g.SetTransitLineCount(max(1, (n+7)/8))
		for i := 0; i < g.LineCount; i++ {
			g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: min(8, max(0, n-i*8)), Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
		}
		g.Bus.SetModel(sim.BusArticulated)
		g.ResetBusesToRouteStart()
	}
	sim.SeedSimulation(5)
	g.Settings.IsOpen = false
	if !g.Settings.AutopilotEnabled || g.IsPlanMode {
		t.Fatal("probe requires active autopilot")
	}
	for i := 0; i < warmup; i++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	profile := func(name string) {
		f, err := os.Create(filepath.Join(out, name))
		if err != nil {
			t.Fatal(err)
		}
		if err := pprof.Lookup("allocs").WriteTo(f, 0); err != nil {
			t.Fatal(err)
		}
		if err := f.Close(); err != nil {
			t.Fatal(err)
		}
	}
	runtime.GC()
	profile("alloc-before.pprof")
	var cpu *os.File
	if os.Getenv("BUS_PROBE_CPU") == "1" {
		var err error
		cpu, err = os.Create(filepath.Join(out, "cpu.pprof"))
		if err != nil {
			t.Fatal(err)
		}
		if err := pprof.StartCPUProfile(cpu); err != nil {
			t.Fatal(err)
		}
		defer cpu.Close()
		defer pprof.StopCPUProfile()
	}
	type result struct {
		Ticks, Warmup, Buses, CarsMin, CarsMax, CarsFinal                             int
		MeanMS, P50MS, P95MS, P99MS, MaxMS, BytesPerTick, AllocsPerTick               float64
		GCCount                                                                       uint32
		GCPauseMS, PlayerHealthStart, PlayerHealthEnd, FleetMinHealth, PlayerDistance float64
		PoseHits, PoseMisses                                                          uint64
	}
	r := result{Ticks: ticks, Warmup: warmup, Buses: len(g.AIBusMgr.Buses), CarsMin: len(g.TrafficMgr.Cars), PlayerHealthStart: g.Bus.Durability, FleetMinHealth: 100}
	poseStats := func() (uint64, uint64) {
		h, m := g.Bus.PosePredictionStats()
		for _, ab := range g.AIBusMgr.Buses {
			a, b := ab.Bus.PosePredictionStats()
			h += a
			m += b
		}
		return h, m
	}
	h0, m0 := poseStats()
	samples := make([]float64, ticks)
	var before, after runtime.MemStats
	runtime.ReadMemStats(&before)
	for i := 0; i < ticks; i++ {
		pos := g.Bus.Pos
		start := time.Now()
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		ms := float64(time.Since(start).Nanoseconds()) / 1e6
		samples[i], r.MeanMS = ms, r.MeanMS+ms
		r.PlayerDistance += pos.Distance(g.Bus.Pos)
		cars := len(g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity))
		r.CarsMin, r.CarsMax, r.CarsFinal = min(r.CarsMin, cars), max(r.CarsMax, cars), cars
		if math.IsNaN(g.Bus.Pos.X) || math.IsNaN(g.Bus.Pos.Y) {
			t.Fatal("invalid player pose")
		}
		for _, ab := range g.AIBusMgr.Buses {
			r.FleetMinHealth = min(r.FleetMinHealth, ab.Bus.Durability)
		}
	}
	runtime.ReadMemStats(&after)
	r.PoseHits, r.PoseMisses = poseStats()
	r.PoseHits, r.PoseMisses = r.PoseHits-h0, r.PoseMisses-m0
	r.MeanMS /= float64(ticks)
	slices.Sort(samples)
	r.P50MS, r.P95MS, r.P99MS, r.MaxMS = samples[(ticks-1)/2], samples[(ticks-1)*95/100], samples[(ticks-1)*99/100], samples[ticks-1]
	r.BytesPerTick, r.AllocsPerTick = float64(after.TotalAlloc-before.TotalAlloc)/float64(ticks), float64(after.Mallocs-before.Mallocs)/float64(ticks)
	r.GCCount, r.GCPauseMS = after.NumGC-before.NumGC, float64(after.PauseTotalNs-before.PauseTotalNs)/1e6
	r.PlayerHealthEnd = g.Bus.Durability
	runtime.GC()
	profile("alloc-after.pprof")
	data, err := json.MarshalIndent(r, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(out, "metrics.json"), data, 0644); err != nil {
		t.Fatal(err)
	}
	t.Log(string(data))
}
