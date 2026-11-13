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
	"github.com/hajimehoshi/ebiten/v2"
)

// Optional native UI review uses temporary saves, never the user's files.
type autosavePreview struct {
	g      *Game
	output string
	done   bool
	err    error
}

func (p *autosavePreview) Layout(w, h int) (int, int) { return 1280, 720 }
func (p *autosavePreview) Update() error {
	if p.done {
		return ebiten.Termination
	}
	return nil
}
func (p *autosavePreview) Draw(screen *ebiten.Image) {
	if p.done {
		return
	}
	p.done = true
	for mode := ModeFast; mode <= ModeCareer; mode++ {
		p.g.SelectMode(mode)
		if p.g.isCareer() {
			p.g.Career.PeakFleet = 8
		}
		for i := 0; i < 6; i++ {
			theme := sim.BuiltinMapThemes()[i%4]
			p.g.Settings.SetCenterBRT(theme, i%2 == 1)
			p.g.Settings.SetLargeRoadNetwork(theme, i%2 == 1)
			p.g.EnterCity(theme)
			if err := p.g.SaveGameSlotWithName(i, "晚高峰公交运营存档"); err != nil {
				p.err = err
				return
			}
			if err := p.g.AutoSave("interval", time.Now().Add(time.Duration(i-6)*AutoSaveInterval)); err != nil {
				p.err = err
				return
			}
		}
		for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
			locale.SetLanguage(lang)
			for _, auto := range []bool{false, true} {
				p.g.openSavePicker(true)
				p.g.setSavePickerSource(auto)
				canvas := ebiten.NewImage(1280, 720)
				p.g.Draw(canvas)
				img := image.NewRGBA(canvas.Bounds())
				canvas.ReadPixels(img.Pix)
				path := filepath.Join(p.output, fmt.Sprintf("%s-auto%v-lang%d.png", mode.String(), auto, lang))
				f, err := os.Create(path)
				if err != nil {
					p.err = err
					return
				}
				err = png.Encode(f, img)
				f.Close()
				canvas.Deallocate()
				if err != nil {
					p.err = err
					return
				}
			}
		}
		p.g.savePicker.open = false
	}
}
func runAutosavePreview(output string) int {
	if err := os.MkdirAll(output, 0700); err != nil {
		fmt.Println(err)
		return 1
	}
	dir, err := os.MkdirTemp("", "bus-autosave-preview-")
	if err != nil {
		fmt.Println(err)
		return 1
	}
	defer os.RemoveAll(dir)
	g := NewGame(1280, 720)
	g.savePathOverride = filepath.Join(dir, "save.json")
	p := &autosavePreview{g: g, output: output}
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("Autosave / metadata QA")
	ebiten.SetRunnableOnUnfocused(true)
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
