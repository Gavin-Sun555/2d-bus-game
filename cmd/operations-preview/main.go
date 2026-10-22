// Command operations-preview renders the real operations panel for visual QA.
package main

import (
	"flag"
	"fmt"
	"image"
	"image/png"
	"os"
	"path/filepath"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
	"github.com/hajimehoshi/ebiten/v2"
)

type preview struct {
	output string
	done   bool
	err    error
}

func (p *preview) Layout(_, _ int) (int, int) { return 1280, 720 }
func (p *preview) Update() error {
	if p.done {
		return ebiten.Termination
	}
	return nil
}
func (p *preview) Draw(screen *ebiten.Image) {
	if p.done {
		return
	}
	p.done = true
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	r := sim.GetStandardRouteForMap(w.Theme, w)
	o := sim.NewTransitOperations(w, r, 52, sim.DefaultFarePolicy())
	for id := 0; id <= 8; id++ {
		v := o.Vehicle(id)
		v.Revenue = 123 + id
		v.Boarded = 50
		v.Alighted = 34
		v.CompletedLaps = 1
		v.CompletedKM = 5
		v.CompletedSeconds = 900
		v.KM = 5.5
		o.Revenue += v.Revenue
	}
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.CurrentLanguage = lang
		for tab := 0; tab < 3; tab++ {
			panel := ui.NewOperationsPanel()
			panel.IsOpen = true
			panel.Tab = tab
			f := sim.DefaultFarePolicy()
			f.Mode = sim.FareByDistance
			screen.Clear()
			panel.Draw(screen, f, o, w)
			img := image.NewRGBA(screen.Bounds())
			screen.ReadPixels(img.Pix)
			file, err := os.Create(filepath.Join(p.output, fmt.Sprintf("operations-%d-%d.png", lang, tab)))
			if err != nil {
				p.err = err
				return
			}
			err = png.Encode(file, img)
			file.Close()
			if err != nil {
				p.err = err
				return
			}
		}
	}
}
func main() {
	output := flag.String("output", "dist/operations-preview", "PNG output directory")
	flag.Parse()
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	p := &preview{output: *output}
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("Operations panel visual review")
	if err := ebiten.RunGame(p); err != nil {
		panic(err)
	}
	if p.err != nil {
		panic(p.err)
	}
	fmt.Println("Saved operations previews to", *output)
}
