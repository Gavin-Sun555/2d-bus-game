package game

import (
	"bufio"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"runtime/pprof"
	"slices"
	"strconv"
	"testing"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// Fixed seed, warm-up and frame count keep before/after profiles comparable.
// This measures simulation updates; Draw/GPU work is measured by release-check.
func BenchmarkArticulatedCongestedCity(b *testing.B) {
	sim.SeedSimulation(20261023)
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	for tick := 0; tick < 600; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.ReportAllocs()
	b.ResetTimer()
	for tick := 0; tick < b.N; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
	b.ReportMetric(g.Bus.Durability, "health")
}

// Optional deterministic trajectory trace for profiling changes. No user save
// is read or written. Compare hashes from separate before/after builds.
func TestArticulatedPerformanceTrace(t *testing.T) {
	path := os.Getenv("BUS_PERF_TRACE")
	if path == "" {
		t.Skip("BUS_PERF_TRACE not set")
	}
	sim.SeedSimulation(20261023)
	g := saveTestGame(t)
	if os.Getenv("BUS_PERF_FLEET") == "48" {
		g.SetTransitLineCount(6)
		for i := 0; i < 6; i++ {
			g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
		}
	}
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	f, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	writer := bufio.NewWriter(f)
	defer writer.Flush()
	for tick := 0; tick < 1800; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		h := sha256.New()
		put := func(values ...float64) {
			for _, v := range values {
				var b [8]byte
				binary.LittleEndian.PutUint64(b[:], math.Float64bits(v))
				h.Write(b[:])
			}
		}
		busState := func(b *sim.Bus, r *sim.Route, d *autopilot.AutopilotDriver) {
			put(b.Pos.X, b.Pos.Y, b.Heading, b.Speed, b.TailHeading(), b.SteeringAngle, b.DoorProgress, b.Durability, float64(b.PassengerCount), float64(r.CurrentIndex), float64(r.TotalTrips), float64(d.State), d.LastTargetSpeed, d.LastPathS, d.LaneOffset)
		}
		busState(g.Bus, g.Route, g.AutopilotDriver)
		for _, ab := range g.AIBusMgr.Buses {
			busState(ab.Bus, ab.Route, ab.Driver)
		}
		for _, c := range g.TrafficMgr.Cars {
			put(float64(c.ID), c.Pos.X, c.Pos.Y, c.Speed, c.Heading, c.TargetSpeed, float64(c.LaneIndex))
			if c.JunctionTurn != nil {
				put(float64(c.JunctionTurn.Segment), c.JunctionTurn.Progress)
			}
		}
		put(float64(g.Cash))
		for _, s := range g.World.Stops {
			put(float64(s.WaitingPassengers))
		}
		fmt.Fprintf(writer, "%d %x\n", tick, h.Sum(nil))
	}
}

func BenchmarkArticulatedMaximumFleet(b *testing.B) {
	sim.SeedSimulation(20261024)
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(6)
	for i := 0; i < 6; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
	}
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	for tick := 0; tick < 300; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.ReportAllocs()
	b.ResetTimer()
	for tick := 0; tick < b.N; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	var hits, misses uint64
	hits, misses = g.Bus.PosePredictionStats()
	for _, ab := range g.AIBusMgr.Buses {
		h, m := ab.Bus.PosePredictionStats()
		hits, misses = hits+h, misses+m
	}
	if hits+misses > 0 {
		b.ReportMetric(100*float64(hits)/float64(hits+misses), "pose-hit%")
	}
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
	b.ReportMetric(g.Bus.Durability, "health")
}

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
		defer pprof.StopCPUProfile() // Also stop on a failed replay.
	}
	type result struct {
		Ticks, Warmup, Buses, CarsMin, CarsMax, CarsFinal                             int
		MeanMS, P50MS, P95MS, P99MS, MaxMS, BytesPerTick, AllocsPerTick               float64
		GCCount                                                                       uint32
		GCPauseMS, PlayerHealthStart, PlayerHealthEnd, FleetMinHealth, PlayerDistance float64
		PoseHits, PoseMisses                                                          uint64
		EndState                                                                      string
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
	if cpu != nil {
		pprof.StopCPUProfile()
	}
	r.EndState = performanceStateHash(g)
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

// Full production Update profiling; opt-in, synthetic dispatch, no user saves.
func TestHighSpeedGamePerformance(t *testing.T) {
	out := os.Getenv("BUS_HIGH_OUTPUT")
	if out == "" {
		t.Skip("BUS_HIGH_OUTPUT not set")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	kph := 120
	if raw := os.Getenv("BUS_HIGH_KPH"); raw != "" {
		requested, err := strconv.Atoi(raw)
		if err != nil {
			t.Fatal(err)
		}
		kph = requested
	}
	g.Settings.SetBerserkMaxSpeed(kph)
	if g.Settings.BerserkSpeedKPH() != kph {
		t.Fatalf("invalid high-speed probe preset %d", kph)
	}
	busKPH := 80
	if os.Getenv("BUS_HIGH_TRAFFIC") == "1" {
		busKPH = sim.TrafficMaxSpeedKPH
		g.Settings.AICarMaxSpeed = sim.TrafficMaxSpeedKPH
	}
	g.Settings.TrafficDensity = 4
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.IsOpen = false
	g.SetTransitLineCount(1)
	if os.Getenv("BUS_HIGH_LONG_ROUTE") == "1" {
		g.Route.StopIDs = []int{199, 213, 199}
		g.Route.CurrentIndex = 0
	}
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: busKPH, MaxKPH: busKPH}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	type sample struct {
		Tick    int
		Group   string
		KPH, MS float64
	}
	samples := make([]sample, 0, 7200)
	hash := sha256.New()
	var b [8]byte
	put := func(v float64) {
		binary.LittleEndian.PutUint64(b[:], math.Float64bits(v))
		hash.Write(b[:])
	}
	var before, after runtime.MemStats
	for tick := 0; tick < 7500; tick++ {
		if tick == 300 && os.Getenv("BUS_HIGH_LANE_SCENE") == "1" {
			g.Route.StopIDs = []int{199, 213, 199}
			g.Route.CurrentIndex = 1
			g.Bus.Pos = sim.Vec2{X: -11000, Y: 5582}
			g.Bus.Heading = 0
			g.Bus.Speed = sim.SpeedKPH(kph)
			g.Bus.SteeringAngle = 0
			g.Bus.DoorsOpen = false
			g.Bus.DoorProgress = 0
			g.Bus.ResetArticulation()
			g.AutopilotDriver.PrepareInitialLeg(g.Bus, g.World, g.Route, 52)
			lead := g.AIBusMgr.Buses[0]
			lead.Route.StopIDs = []int{199, 213, 199}
			lead.Route.CurrentIndex = 1
			// Match the closing runway at the start of each speed comparison.
			closing := sim.SpeedKPH(kph) - sim.SpeedKPH(40)
			gap := 800.0
			if kph > 120 {
				gap = closing*closing/(2*130) + 240
			}
			lead.Bus.Pos = sim.Vec2{X: g.Bus.Pos.X + gap, Y: 5582}
			lead.Bus.Heading = 0
			lead.Bus.Speed = sim.SpeedKPH(40)
			lead.Bus.SteeringAngle = 0
			lead.Bus.DoorsOpen = false
			lead.Bus.DoorProgress = 0
			lead.Bus.ResetArticulation()
			lead.Driver.PrepareInitialLeg(lead.Bus, g.World, lead.Route, 52)
		}
		if tick == 300 {
			runtime.GC()
			runtime.ReadMemStats(&before)
		}
		speed := g.Bus.Speed * 3.6 / 20
		group := "straight"
		if g.World.InsideJunction(g.Bus.Pos, 52, g.Bus.Length/2) {
			group = "junction"
		}
		if g.AutopilotDriver.OvertakeState == 1 || math.Abs(g.AutopilotDriver.TargetLaneOffset-g.AutopilotDriver.LaneOffset) > .5 {
			group = "lane-change"
		}
		start := time.Now()
		err := g.Update()
		ms := float64(time.Since(start).Nanoseconds()) / 1e6
		if err != nil {
			t.Fatal(err)
		}
		if tick >= 300 {
			samples = append(samples, sample{tick, group, speed, ms})
		}
		busHash := func(b *sim.Bus) {
			put(b.Pos.X)
			put(b.Pos.Y)
			put(b.Heading)
			put(b.TailHeading())
			put(b.Speed)
			put(b.Durability)
		}
		busHash(g.Bus)
		for _, ab := range g.AIBusMgr.Buses {
			busHash(ab.Bus)
		}
		for _, c := range g.TrafficMgr.Cars {
			put(c.Pos.X)
			put(c.Pos.Y)
			put(c.Heading)
			put(c.Speed)
		}
		for _, it := range g.TrafficLights.Intersections {
			put(float64(it.Phase))
			put(it.Timer)
		}
		put(float64(g.Route.CurrentIndex))
		put(float64(g.Bus.PassengerCount))
		put(float64(g.Cash))
	}
	runtime.ReadMemStats(&after)
	type stats struct {
		N                         int
		Mean, P50, P99, P999, Max float64
		Over16, Over33            int
	}
	groups := map[string][]float64{}
	for _, s := range samples {
		groups["all"] = append(groups["all"], s.MS)
		if s.KPH >= float64(kph-10) && s.KPH <= float64(kph)+.01 {
			key := fmt.Sprintf("%d-%d/%s", kph-10, kph, s.Group)
			groups[key] = append(groups[key], s.MS)
		}
	}
	summary := map[string]stats{}
	for group, values := range groups {
		s := stats{N: len(values)}
		for _, ms := range values {
			s.Mean += ms
			if ms > 16.667 {
				s.Over16++
			}
			if ms > 33.334 {
				s.Over33++
			}
		}
		s.Mean /= float64(s.N)
		slices.Sort(values)
		s.P50 = values[(s.N-1)/2]
		s.P99 = values[(s.N-1)*99/100]
		s.P999 = values[(s.N-1)*999/1000]
		s.Max = values[s.N-1]
		summary[group] = s
	}
	data, _ := json.MarshalIndent(map[string]any{"groups": summary, "hash": hex.EncodeToString(hash.Sum(nil)), "bytesPerTick": float64(after.TotalAlloc-before.TotalAlloc) / 7200, "allocsPerTick": float64(after.Mallocs-before.Mallocs) / 7200, "health": g.Bus.Durability, "samples": samples}, "", "  ")
	if err := os.WriteFile(out, data, 0644); err != nil {
		t.Fatal(err)
	}
}

func findSlot2ProfileSave() string {
	return firstFixture(
		filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-02.json"),
		filepath.Join("..", "dist", "slot2-headon-2026-11-02-1227", "slot2.json"),
	)
}

func BenchmarkSaveSlot2Update(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.AIBusMgr.Buses)), "buses")
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
}

func BenchmarkSaveSlot2Draw(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}
	screen := ebiten.NewImage(1440, 900)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.Draw(screen)
	}
	b.StopTimer()
}

func BenchmarkSaveSlot2FullFrame(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}
	screen := ebiten.NewImage(1440, 900)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
		g.Draw(screen)
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.AIBusMgr.Buses)), "buses")
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
}
