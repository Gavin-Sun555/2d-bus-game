package game

import (
	"fmt"
	"image"
	"image/png"
	"os"
	"path/filepath"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
	"github.com/hajimehoshi/ebiten/v2"
)

// Native QA is opt-in and always uses a temporary preference directory. It
// never opens, loads or writes the user's save/preferences.
type stopNamePreview struct {
	g           *Game
	output      string
	done        bool
	err         error
	interactive bool
	start       time.Time
}

func (p *stopNamePreview) Layout(w, h int) (int, int) { return 1280, 720 }
func (p *stopNamePreview) Update() error {
	if p.interactive {
		if time.Since(p.start) > 3*time.Minute {
			return ebiten.Termination
		}

		return p.g.Update()
	}
	if p.done {
		return ebiten.Termination
	}
	return nil
}
func (p *stopNamePreview) Draw(screen *ebiten.Image) {
	if p.interactive {
		p.g.Draw(screen)
		return
	}
	if p.done {
		return
	}
	p.done = true
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.SetLanguage(lang)
		for _, width := range []int{1280, 800} {
			g := p.g
			g.Width = width
			g.Height = 720
			g.Camera.ScreenWidth = width
			e := g.stopNameEditor
			e.fields = [2]string{"人民广场公交换乘枢纽总站", "Central Square Bus Interchange"}
			e.focused = 0
			e.caret = len(e.fields[0])
			e.err = ""
			if width == 800 {
				e.focused = 1
				e.caret = len(e.fields[1])
				e.err = "英文站名最多30字符（含空格） / English name: max 30 characters incl. spaces"
			}
			canvas := ebiten.NewImage(width, 720)
			if os.Getenv("BUSGAME_SETTINGS_PREVIEW") == "1" {
				g.Settings.IsOpen = true
				g.Settings.PassengerTimingOpen = os.Getenv("BUSGAME_PASSENGER_TIMING_PREVIEW") == "1"
				g.Settings.Language = lang
				g.Settings.Announcements = ui.AnnouncementsBilingual
				g.Settings.Draw(canvas, g.Camera)
			} else {
				g.Draw(canvas)
			}
			img := image.NewRGBA(canvas.Bounds())
			canvas.ReadPixels(img.Pix)
			path := filepath.Join(p.output, fmt.Sprintf("editor-%d-%v.png", width, lang))
			f, err := os.Create(path)
			if err != nil {
				p.err = err
				return
			}
			err = png.Encode(f, img)
			f.Close()
			canvas.Dispose()
			if err != nil {
				p.err = err
				return
			}
		}
	}
}
func runStopNamePreview(output string) int {
	if err := os.MkdirAll(output, 0700); err != nil {
		fmt.Println(err)
		return 1
	}
	dir, err := os.MkdirTemp("", "bus-station-names-qa-")
	if err != nil {
		fmt.Println(err)
		return 1
	}
	defer os.RemoveAll(dir)
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.TogglePlanMode()
	if err := g.loadStopNames(filepath.Join(dir, "station_names.json")); err != nil {
		fmt.Println(err)
		return 1
	}
	g.openStopNameEditor(278)
	interactive := os.Getenv("BUSGAME_STATION_NAME_INTERACTIVE") == "1"
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("Station name editor QA")
	ebiten.SetRunnableOnUnfocused(true)
	p := &stopNamePreview{g: g, output: output, interactive: interactive, start: time.Now()}
	if err := ebiten.RunGame(p); err != nil {
		fmt.Println(err)
		return 1
	}
	if p.err != nil {
		fmt.Println(p.err)
		return 1
	}
	return 0
}
