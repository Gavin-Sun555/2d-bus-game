// Command release-check runs real Update/Draw frames and saves release screenshots.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/png"
	"os"
	"path/filepath"
	"runtime/pprof"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

type measurement struct {
	Scene                  string
	Frames                 int
	UpdateFrames           int
	UpdateMS, DrawSubmitMS float64
}
type check struct {
	g                   *game.Game
	output              string
	scene, frame, drawn int
	results             []measurement
	current             measurement
	err                 error
	frames              int
	captured            bool
	onlyScene           bool
	freeze              bool
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
	c.current = measurement{Scene: fmt.Sprintf("scene-%02d", c.scene)}
	switch {
	case c.scene == 0:
		c.g.State = game.StateMainMenu
	case c.scene <= 24:
		index := c.scene - 1
		c.g.Settings.TrafficDensity = index%4 + 1
		c.g.Settings.AIBusCount = 8
		c.g.Settings.AutopilotEnabled = true
		c.g.Settings.AutopilotBerserk = index%2 == 1
		c.g.EnterCity(sim.MapTheme(index / 4))
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
		c.results = append(c.results, c.current)
		c.scene++
		if c.scene >= 28 || c.onlyScene {
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
	if c.frame >= 10 {
		c.current.UpdateFrames++
		c.current.UpdateMS += float64(time.Since(start).Nanoseconds()) / 1e6
	}
	c.frame++
	return err
}
func (c *check) Draw(screen *ebiten.Image) {
	start := time.Now()
	c.g.Draw(screen)
	if c.frame > 10 {
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
	output := flag.String("output", "dist/release-0.0.5/visual", "Screenshot and frame measurement directory")
	frames := flag.Int("frames", 60, "Frames per scene (at least 11; first ten warm up)")
	freeze := flag.Bool("freeze", false, "Keep simulation static for pixel-identical renderer comparisons")
	only := flag.Int("only-scene", -1, "Run one numbered scene (0-27) for controlled profiling")
	profile := flag.String("cpuprofile", "", "Optional CPU profile file")
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
	if *frames < 11 {
		panic("frames must be at least 11")
	}
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	sim.SeedSimulation(5)
	c := &check{g: game.NewGame(1280, 720), output: *output, frames: *frames}
	if *only >= 0 {
		if *only > 27 {
			panic("invalid scene")
		}
		c.scene = *only
		c.onlyScene = true
	}
	c.freeze = *freeze
	c.configure()
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("0.0.5 release graphics check")
	if err := ebiten.RunGame(c); err != nil {
		panic(err)
	}
	if c.err != nil {
		panic(c.err)
	}
	fmt.Println("Completed release graphics check; screenshots and CPU submission timings:", *output)
}
