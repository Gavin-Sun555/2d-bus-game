// Command release-check runs real Update/Draw frames and saves release screenshots.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/png"
	"math"
	"os"
	"path/filepath"
	"runtime/pprof"
	"slices"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/hajimehoshi/ebiten/v2"
)

type measurement struct {
	PosePredictionHits, PosePredictionMisses           uint64
	poseHitsStart, poseMissesStart                     uint64
	Vsync                                              bool
	SingleThread                                       bool
	GraphicsLibrary                                    string
	CarsMin, CarsMax, CarsFinal, FleetBuses            int
	PlayerDamageSteps                                  int
	PlayerFinalDurability, FleetMinDurability          float64
	MeasuredWallMS, MeanUpdateTPS, MeanDrawFPS         float64
	measuredAt                                         time.Time
	UpdateP50MS, UpdateP95MS, UpdateP99MS, UpdateMaxMS float64
	updateSamples                                      []float64
	Scene                                              string
	Frames                                             int
	UpdateFrames                                       int
	UpdateMS, DrawSubmitMS                             float64
}
type check struct {
	vsync                bool
	singleThread         bool
	busModel             sim.BusModel
	warmupFrames         int
	previousPlayerHealth float64
	fleetBuses           int
	g                    *game.Game
	output               string
	scene, frame, drawn  int
	results              []measurement
	current              measurement
	err                  error
	frames               int
	captured             bool
	onlyScene            bool
	freeze               bool
}

func (c *check) configure() {
	c.frame, c.drawn, c.captured = 0, 0, false
	if c.onlyScene && c.scene >= 25 {
		c.g.Settings.TrafficDensity = 4
		c.g.Settings.AIBusCount = 8
		c.g.EnterCity(sim.MapThemeCustom)
		if c.scene >= 26 {
			c.g.Settings.IsOpen = true
		}
	}
	c.previousPlayerHealth = 100
	c.current = measurement{Scene: fmt.Sprintf("scene-%02d", c.scene), SingleThread: c.singleThread, Vsync: c.vsync}
	switch {
	case c.scene == 31:
		c.g.Settings.TrafficDensity = 4
		c.g.Settings.AutopilotEnabled = true
		c.g.Settings.AutopilotBerserk = true
		c.g.EnterCity(sim.MapThemeMetropolis)
		c.g.Settings.IsOpen = false
		lineCount := max(1, (c.fleetBuses+7)/8)
		c.g.SetTransitLineCount(lineCount)
		for i := 0; i < lineCount; i++ {
			c.g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: min(8, max(0, c.fleetBuses-i*8)), Model: c.busModel, PlayerModel: c.busModel, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
		}
		if c.busModel != sim.BusStandard {
			c.g.Bus.SetModel(c.busModel)
			c.g.ResetBusesToRouteStart()
		}
		c.g.Camera.DrivingViewLevel = 2
		c.g.Camera.RestoreDrivingZoom()
		c.current.Scene = fmt.Sprintf("max-traffic-%d-cars-%d-buses-model%d", sim.TrafficCarCount(4), c.fleetBuses, c.busModel)
	case c.scene == 30:
		c.g.EnterCity(sim.MapThemeMetropolis)
		c.g.Settings.IsOpen = false
		c.g.SetTransitLineCount(6)
		c.g.NetworkOpen = true
		c.current.Scene = "transit-network"
	case c.scene == 0:
		c.g.State = game.StateMainMenu
	case c.scene <= 24:
		index := c.scene - 1
		c.g.Settings.TrafficDensity = index%4 + 1
		c.g.Settings.AIBusCount = 8
		c.g.Settings.AutopilotEnabled = true
		c.g.Settings.AutopilotBerserk = index%2 == 1
		// Include the legacy lake map for old-save qualification exactly once.
		themes := []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities, sim.MapThemeCustom}
		c.g.EnterCity(themes[index/4])
		c.g.Camera.Zoom = .6
		c.current.Scene = fmt.Sprintf("map%d-cars%d", index/4, sim.TrafficCarCount(index%4+1))
	case c.scene == 25:
		c.g.IsPlanMode = true
		c.g.Camera.Zoom = .075
		c.current.Scene = "planning"
	case c.scene == 26:
		c.g.IsPlanMode = false
		c.g.Settings.IsOpen = true
		c.current.Scene = "settings-zh"
	case c.scene == 28 || c.scene == 29:
		locale.SetLanguage(locale.LanguageZH)
		c.g.Settings.IsOpen = false
		c.g.IsPlanMode = false
		c.g.Camera.DrivingViewLevel = c.scene - 27
		c.g.Camera.RestoreDrivingZoom()
		c.current.Scene = fmt.Sprintf("driving-view-%dx", c.g.Camera.DrivingViewMultiplier())
	case c.scene == 27:
		locale.SetLanguage(locale.LanguageEN)
		c.g.Settings.Language = locale.LanguageEN
		c.current.Scene = "settings-en"
	}
}
func (c *check) Update() error {
	if c.err != nil {
		return c.err
	}
	if c.frame >= c.frames && c.drawn >= c.frames {
		if !c.current.measuredAt.IsZero() {
			c.current.MeasuredWallMS = float64(time.Since(c.current.measuredAt).Nanoseconds()) / 1e6
			c.current.MeanUpdateTPS = float64(c.current.UpdateFrames) * 1000 / c.current.MeasuredWallMS
			c.current.MeanDrawFPS = float64(c.current.Frames) * 1000 / c.current.MeasuredWallMS
		}
		if n := len(c.current.updateSamples); n > 0 {
			samples := slices.Clone(c.current.updateSamples)
			slices.Sort(samples)
			percentile := func(q float64) float64 { return samples[max(0, int(math.Ceil(q*float64(n)))-1)] }
			c.current.UpdateP50MS = percentile(.5)
			c.current.UpdateP95MS = percentile(.95)
			c.current.UpdateP99MS = percentile(.99)
			c.current.UpdateMaxMS = samples[n-1]
		}
		c.results = append(c.results, c.current)
		c.scene++
		if c.scene >= 31 || c.onlyScene {
			data, err := json.MarshalIndent(c.results, "", "  ")
			if err != nil {
				return err
			}
			if err = os.WriteFile(filepath.Join(c.output, "frames.json"), data, 0644); err != nil {
				return err
			}
			return ebiten.Termination
		}
		c.configure()
	}
	if c.freeze {
		c.frame++
		return nil
	}
	start := time.Now()
	err := c.g.Update()
	if c.frame >= c.warmupFrames {
		if c.current.measuredAt.IsZero() {
			c.current.measuredAt = start
		}
		c.current.UpdateFrames++
		elapsed := float64(time.Since(start).Nanoseconds()) / 1e6
		c.current.UpdateMS += elapsed
		c.current.updateSamples = append(c.current.updateSamples, elapsed)
		cars := c.g.TrafficMgr.GetActiveCars(c.g.Settings.TrafficDensity)
		count := len(cars)
		if c.current.UpdateFrames == 1 {
			var graphics ebiten.DebugInfo
			ebiten.ReadDebugInfo(&graphics)
			c.current.GraphicsLibrary = graphics.GraphicsLibrary.String()
			c.current.CarsMin = count
			c.previousPlayerHealth = c.g.Bus.Durability
		}
		c.current.CarsMin = min(c.current.CarsMin, count)
		c.current.CarsMax = max(c.current.CarsMax, count)
		c.current.CarsFinal = count
		if c.g.Bus.Durability < c.previousPlayerHealth {
			c.current.PlayerDamageSteps++
		}
		c.previousPlayerHealth = c.g.Bus.Durability
		c.current.PlayerFinalDurability = c.g.Bus.Durability
		c.current.FleetBuses = len(c.g.AIBusMgr.Buses)
		c.current.FleetMinDurability = 100
		finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
		for _, car := range cars {
			if !finite(car.Pos.X) || !finite(car.Pos.Y) || !finite(car.Speed) || !finite(car.Heading) {
				return fmt.Errorf("non-finite car %d", car.ID)
			}
		}
		if !finite(c.g.Bus.Pos.X) || !finite(c.g.Bus.Pos.Y) || !finite(c.g.Bus.Speed) {
			return fmt.Errorf("non-finite player")
		}
		for _, bus := range c.g.AIBusMgr.Buses {
			c.current.FleetMinDurability = math.Min(c.current.FleetMinDurability, bus.Bus.Durability)
			if !finite(bus.Bus.Pos.X) || !finite(bus.Bus.Pos.Y) || !finite(bus.Bus.Speed) {
				return fmt.Errorf("non-finite bus %d", bus.ID)
			}
		}
		hits, misses := c.g.Bus.PosePredictionStats()
		for _, bus := range c.g.AIBusMgr.Buses {
			h, m := bus.Bus.PosePredictionStats()
			hits, misses = hits+h, misses+m
		}
		if c.current.UpdateFrames == 1 {
			c.current.poseHitsStart, c.current.poseMissesStart = hits, misses
		}
		c.current.PosePredictionHits = hits - c.current.poseHitsStart
		c.current.PosePredictionMisses = misses - c.current.poseMissesStart
	}
	c.frame++
	return err
}
func (c *check) Draw(screen *ebiten.Image) {
	start := time.Now()
	c.g.Draw(screen)
	if c.frame > c.warmupFrames {
		c.current.DrawSubmitMS += float64(time.Since(start).Nanoseconds()) / 1e6
		c.current.Frames++
	}
	c.drawn = c.frame
	if c.frame >= c.frames && !c.captured {
		c.captured = true
		pixels := make([]byte, 1280*720*4)
		screen.ReadPixels(pixels)
		f, err := os.Create(filepath.Join(c.output, c.current.Scene+".png"))
		if err != nil {
			c.err = err
			return
		}
		err = png.Encode(f, &image.RGBA{Pix: pixels, Stride: 1280 * 4, Rect: image.Rect(0, 0, 1280, 720)})
		closeErr := f.Close()
		if err == nil {
			err = closeErr
		}
		c.err = err
	}
}
func (c *check) Layout(_, _ int) (int, int) { return 1280, 720 }
func main() {
	output := flag.String("output", "dist/release-0.0.7/visual", "Screenshot and frame measurement directory")
	frames := flag.Int("frames", 60, "Frames per scene (at least 11; first ten warm up)")
	rotation := flag.Float64("rotation", 0, "Camera rotation in radians for frozen visual checks")
	freeze := flag.Bool("freeze", false, "Keep simulation static for pixel-identical renderer comparisons")
	only := flag.Int("only-scene", -1, "Run one numbered scene (0-31) for controlled profiling")
	buses := flag.Int("buses", 48, "AI bus count for scene 31 (0-48)")
	model := flag.Int("bus-model", 0, "Bus model for scene 31: 0 standard, 1 double-decker, 2 articulated")
	warmup := flag.Int("warmup", 10, "Warm-up update count excluded from timing")
	profile := flag.String("cpuprofile", "", "Optional CPU profile file")
	singleThread := flag.Bool("single-thread", false, "Use Ebitengine main-thread event/render loop for controlled platform comparison")
	vsync := flag.Bool("vsync", true, "Enable display synchronization for controlled frame-pacing comparison")
	flag.Parse()
	if *profile != "" {
		f, err := os.Create(*profile)
		if err != nil {
			panic(err)
		}
		if err = pprof.StartCPUProfile(f); err != nil {
			panic(err)
		}
		defer f.Close()
		defer pprof.StopCPUProfile()
	}
	if *buses < 0 || *buses > 48 {
		panic("invalid bus count")
	}
	if *warmup < 0 || *warmup >= *frames || *model < 0 || *model >= int(sim.BusModelCount) {
		panic("invalid warmup or model")
	}
	if *frames < 11 {
		panic("frames must be at least 11")
	}
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	sim.SeedSimulation(5)
	c := &check{vsync: *vsync, singleThread: *singleThread, g: game.NewGame(1280, 720), output: *output, frames: *frames, fleetBuses: *buses, busModel: sim.BusModel(*model), warmupFrames: *warmup}
	if *only >= 0 {
		if *only > 31 {
			panic("invalid scene")
		}
		c.scene = *only
		c.onlyScene = true
	}
	c.freeze = *freeze
	c.configure()
	c.g.Camera.Rotation, c.g.Camera.TargetRotation = *rotation, *rotation
	ebiten.SetRunnableOnUnfocused(true)
	ebiten.SetVsyncEnabled(*vsync)
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("0.0.7 release graphics check")
	if err := ebiten.RunGameWithOptions(c, &ebiten.RunGameOptions{SingleThread: *singleThread}); err != nil {
		panic(err)
	}
	if c.err != nil {
		panic(c.err)
	}
	fmt.Println("Completed release graphics check; screenshots and CPU submission timings:", *output)
}
