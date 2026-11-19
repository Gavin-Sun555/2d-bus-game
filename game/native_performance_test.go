package game

import (
	"context"
	"crypto/sha256"
	"encoding/csv"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"runtime/pprof"
	"runtime/trace"
	"slices"
	"strconv"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type nativeFrameSample struct {
	tick, updates                                         int
	gap, update, maxUpdate, draw, finalDraw, beforeUpdate time.Duration
}

type nativePerf struct {
	game                                 *Game
	warm                                 *GraphicsWarmupGame
	ticks, total, warmup, draws, focused int
	dropped                              int
	simulationSeconds                    float64
	updates, submits, intervals          []float64
	lastDraw, started                    time.Time
	groups                               map[string][]float64
	frames                               []nativeFrameSample
	firstUpdate, lastFinal               time.Time
	updateDuration, maxUpdate            time.Duration
	updatesSinceDraw                     int
	minimalDraw                          bool
}

func (p *nativePerf) Layout(w, h int) (int, int)              { return p.warm.Layout(w, h) }
func (p *nativePerf) LayoutF(w, h float64) (float64, float64) { return p.warm.LayoutF(w, h) }
func (p *nativePerf) Update() error {
	if p.ticks >= p.total {
		return ebiten.Termination
	}
	if !p.warm.ready {
		return p.warm.Update()
	}
	before := p.game.World.Clock.ElapsedMinutes
	start := time.Now()
	if p.updatesSinceDraw == 0 {
		p.firstUpdate = start
	}
	var region *trace.Region
	if trace.IsEnabled() {
		region = trace.StartRegion(context.Background(), "simulation")
	}
	err := p.warm.Update()
	if region != nil {
		region.End()
	}
	elapsed := time.Since(start)
	p.updatesSinceDraw++
	p.updateDuration += elapsed
	p.maxUpdate = max(p.maxUpdate, elapsed)
	if err == nil && p.game.World.Clock.ElapsedMinutes == before {
		if p.game.State != StatePlaying || p.game.Settings.IsOpen || p.game.IsPlanMode {
			return fmt.Errorf("simulation paused at tick %d", p.ticks)
		}
		if p.ticks >= p.warmup {
			p.dropped++
		}
		p.ticks++
		return nil
	}
	ms := float64(elapsed.Nanoseconds()) / 1e6
	if p.ticks >= p.warmup {
		p.simulationSeconds += (p.game.World.Clock.ElapsedMinutes - before) * 60
		if p.started.IsZero() {
			p.started = start
		}
		p.updates = append(p.updates, ms)
		group := "road"
		if p.game.World.InsideJunction(p.game.Bus.Pos, 52, p.game.Bus.Length/2) {
			group = "junction"
		} else if p.game.Bus.Speed >= sim.SpeedKPH(140) {
			group = "high-speed"
		}
		p.groups[group] = append(p.groups[group], ms)
		near := 0
		for _, c := range p.game.TrafficMgr.Cars {
			d := c.Pos.Sub(p.game.Bus.Pos)
			if d.Dot(d) < 800*800 {
				near++
			}
		}
		if near >= 12 {
			p.groups["dense-traffic"] = append(p.groups["dense-traffic"], ms)
		}
	}
	p.ticks++
	return err
}
func (p *nativePerf) Draw(screen *ebiten.Image) {
	start := time.Now()
	if p.minimalDraw && p.warm.ready {
		screen.Clear()
	} else {
		p.warm.Draw(screen)
	}
	duration := time.Since(start)
	if p.ticks > p.warmup {
		p.submits = append(p.submits, float64(duration.Nanoseconds())/1e6)
		if !p.lastDraw.IsZero() {
			gap := start.Sub(p.lastDraw)
			p.intervals = append(p.intervals, float64(gap.Nanoseconds())/1e6)
			row := nativeFrameSample{tick: p.ticks, updates: p.updatesSinceDraw, gap: gap, update: p.updateDuration, maxUpdate: p.maxUpdate, draw: duration}
			if p.updatesSinceDraw > 0 && !p.lastFinal.IsZero() {
				row.beforeUpdate = p.firstUpdate.Sub(p.lastFinal)
			}
			p.frames = append(p.frames, row)
		}
		p.lastDraw = start
		p.draws++
		if ebiten.IsFocused() {
			p.focused++
		}
	}
	p.updatesSinceDraw, p.updateDuration, p.maxUpdate = 0, 0, 0
}

func (p *nativePerf) DrawFinalScreen(screen ebiten.FinalScreen, offscreen *ebiten.Image, geoM ebiten.GeoM) {
	start := time.Now()
	ebiten.DefaultDrawFinalScreen(screen, offscreen, geoM)
	p.lastFinal = time.Now()
	if len(p.frames) > 0 {
		p.frames[len(p.frames)-1].finalDraw = p.lastFinal.Sub(start)
	}
}

func writeNativeFrames(path string, samples []nativeFrameSample) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := csv.NewWriter(f)
	w.Write([]string{"tick", "updates", "gap_ms", "update_ms", "max_update_ms", "draw_ms", "final_draw_ms", "previous_final_to_first_update_ms"})
	ms := func(d time.Duration) string {
		return strconv.FormatFloat(float64(d)/float64(time.Millisecond), 'f', 4, 64)
	}
	for _, row := range samples {
		w.Write([]string{strconv.Itoa(row.tick), strconv.Itoa(row.updates), ms(row.gap), ms(row.update), ms(row.maxUpdate), ms(row.draw), ms(row.finalDraw), ms(row.beforeUpdate)})
	}
	w.Flush()
	return w.Error()
}

type perfDistribution struct {
	Count                              int
	MeanMS, P50MS, P95MS, P99MS, MaxMS float64
}

func summarizePerf(samples []float64) perfDistribution {
	r := perfDistribution{Count: len(samples)}
	if len(samples) == 0 {
		return r
	}
	for _, x := range samples {
		r.MeanMS += x
	}
	r.MeanMS /= float64(len(samples))
	v := slices.Clone(samples)
	slices.Sort(v)
	r.P50MS, r.P95MS, r.P99MS, r.MaxMS = v[(len(v)-1)/2], v[(len(v)-1)*95/100], v[(len(v)-1)*99/100], v[len(v)-1]
	return r
}
func performanceStateHash(g *Game) string {
	// Hash the serialized city, including traffic, passengers and operations.
	// The replay fixes the window size and the snapshot excludes private caches.
	s := g.snapshotCity()
	raw, err := json.Marshal(s)
	if err != nil {
		panic(fmt.Sprintf("performance state: %v", err))
	}
	return fmt.Sprintf("%x", sha256.Sum256(raw))
}
func runNativePerformance() int {
	fail := func(err error) int { fmt.Fprintln(os.Stderr, err); return 1 }
	out := os.Getenv("BUS_NATIVE_OUTPUT")
	if err := os.MkdirAll(out, 0755); err != nil {
		return fail(err)
	}
	source, err := os.ReadFile(os.Getenv("BUS_NATIVE_SAVE"))
	if err != nil {
		return fail(err)
	}
	copyPath := filepath.Join(out, "input-save.json")
	if err = os.WriteFile(copyPath, source, 0600); err != nil {
		return fail(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = copyPath
	if err = g.LoadGame(); err != nil {
		return fail(err)
	}
	sim.SeedSimulation(5)
	g.Settings.IsOpen = false
	if speed := os.Getenv("BUS_NATIVE_KPH"); speed != "" {
		kph, err := strconv.Atoi(speed)
		if err != nil {
			return fail(err)
		}
		g.Settings.SetBerserkMaxSpeed(kph)
	}
	if !g.Settings.AutopilotEnabled || g.IsPlanMode {
		return fail(fmt.Errorf("native probe requires active autopilot"))
	}
	ticks := 1800
	if s := os.Getenv("BUS_NATIVE_TICKS"); s != "" {
		ticks, err = strconv.Atoi(s)
		if err != nil || ticks < 121 {
			return fail(fmt.Errorf("invalid tick count"))
		}
	}
	p := &nativePerf{game: g, warm: WithGraphicsWarmup(g), total: ticks, warmup: 120, groups: map[string][]float64{}, frames: make([]nativeFrameSample, 0, ticks), minimalDraw: os.Getenv("BUS_NATIVE_MINIMAL_DRAW") == "1"}
	if os.Getenv("BUS_NATIVE_PROFILE") == "1" {
		cpu, err := os.Create(filepath.Join(out, "cpu.pprof"))
		if err != nil {
			return fail(err)
		}
		defer cpu.Close()
		if err := pprof.StartCPUProfile(cpu); err != nil {
			return fail(err)
		}
		defer pprof.StopCPUProfile()
		f, err := os.Create(filepath.Join(out, "runtime.trace"))
		if err != nil {
			return fail(err)
		}
		defer f.Close()
		if err := trace.Start(f); err != nil {
			return fail(err)
		}
		defer trace.Stop()
	}
	ebiten.SetWindowSize(1470, 915)
	ebiten.SetWindowTitle("Bus game · isolated performance replay")
	ebiten.SetRunnableOnUnfocused(true)
	ebiten.SetVsyncEnabled(true)
	err = ebiten.RunGameWithOptions(p, &ebiten.RunGameOptions{InitUnfocused: os.Getenv("BUS_NATIVE_FOCUS") != "1"})
	if err != nil {
		return fail(err)
	}
	wall := time.Since(p.started).Seconds()
	groups := map[string]perfDistribution{}
	for k, v := range p.groups {
		groups[k] = summarizePerf(v)
	}
	report := struct {
		Update, DrawSubmit, DrawInterval perfDistribution
		Groups                           map[string]perfDistribution
		UpdateTPS, DrawFPS               float64
		FocusedDraws, Cars, Buses        int
		EndState                         string
		DroppedCatchUpTicks              int
		SimulationSeconds, WallSeconds   float64
	}{summarizePerf(p.updates), summarizePerf(p.submits), summarizePerf(p.intervals), groups, float64(len(p.updates)) / wall, float64(p.draws) / wall, p.focused, len(g.TrafficMgr.Cars), len(g.AIBusMgr.Buses), performanceStateHash(g), p.dropped, p.simulationSeconds, wall}
	raw, err := json.MarshalIndent(report, "", "  ")
	if err != nil {
		return fail(err)
	}
	if err = os.WriteFile(filepath.Join(out, "native-metrics.json"), raw, 0644); err != nil {
		return fail(err)
	}
	if err = writeNativeFrames(filepath.Join(out, "native-frames.csv"), p.frames); err != nil {
		return fail(err)
	}
	fmt.Println(string(raw))
	if os.Getenv("BUS_NATIVE_REQUIRE_REALTIME") == "1" {
		if report.UpdateTPS < 59 || report.DrawFPS < 55 || p.dropped != 0 || math.Abs(p.simulationSeconds-wall) > .1 {
			return fail(fmt.Errorf("frame pacing failed: simulation %.2f TPS, rendering %.2f FPS, skipped %d, simulation/wall %.3f/%.3fs", report.UpdateTPS, report.DrawFPS, p.dropped, p.simulationSeconds, wall))
		}
	}
	return 0
}
