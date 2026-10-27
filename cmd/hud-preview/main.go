// Command hud-preview renders the real HUD for visual QA.
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
	bus := sim.NewBus(100, 100, 0)
	bus.Capacity = 85
	bus.PassengerCount = 84
	cam := sim.NewCamera(1280, 720)
	for _, width := range []int{1280, 1707} {
		for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
			locale.SetLanguage(lang)
			for _, plan := range []bool{false, true} {
				for _, bell := range []bool{false, true} {
					canvas := ebiten.NewImage(width, 720)
					canvas.Fill(sim.RGBA(28, 42, 54, 255))
					h := ui.NewHUD()
					h.AddToast(locale.T("视角已切换", "Camera updated"), true)
					settings := ui.NewSettings()
					settings.TrafficFinesEnabled = !plan
					h.Draw(canvas, bus, r, w, plan, 123456789, nil, settings, bell, 0, cam)
					img := image.NewRGBA(canvas.Bounds())
					canvas.ReadPixels(img.Pix)
					file, err := os.Create(filepath.Join(p.output, fmt.Sprintf("hud-%d-%d-plan%t-bell%t.png", width, lang, plan, bell)))
					if err != nil {
						p.err = err
						return
					}
					err = png.Encode(file, img)
					file.Close()
					canvas.Dispose()
					if err != nil {
						p.err = err
						return
					}
				}
			}
		}
	}
}

func main() {
	output := flag.String("output", "dist/hud-layout", "PNG output directory")
	flag.Parse()
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	p := &preview{output: *output}
	ebiten.SetRunnableOnUnfocused(true)
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("HUD visual review")
	if err := ebiten.RunGame(p); err != nil {
		panic(err)
	}
	if p.err != nil {
		panic(p.err)
	}
	fmt.Println("Saved HUD previews to", *output)
}
