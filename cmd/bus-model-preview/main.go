// Command bus-model-preview captures the real paused fleet panel and model art.
package main

import (
	"flag"
	"fmt"
	"image"
	"image/png"
	"math"
	"os"
	"path/filepath"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"
	"github.com/hajimehoshi/ebiten/v2"
)

type preview struct {
	g      *game.Game
	output string
	done   bool
	err    error
}

func (p *preview) Update() error {
	if p.done {
		return ebiten.Termination
	}
	return nil
}
func (p *preview) Layout(int, int) (int, int) { return 1280, 720 }
func (p *preview) save(screen *ebiten.Image, name string) error {
	pixels := make([]byte, 1280*720*4)
	screen.ReadPixels(pixels)
	f, err := os.Create(filepath.Join(p.output, name))
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, &image.RGBA{Pix: pixels, Stride: 1280 * 4, Rect: image.Rect(0, 0, 1280, 720)})
}
func (p *preview) Draw(screen *ebiten.Image) {
	if p.done {
		return
	}
	p.done = true
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.SetLanguage(lang)
		p.g.Draw(screen)
		if err := p.save(screen, fmt.Sprintf("fleet-panel-%d.png", lang)); err != nil {
			p.err = err
			return
		}
	}
	locale.SetLanguage(locale.LanguageZH)
	screen.Fill(sim.RGBA(15, 23, 42, 255))
	cam := sim.NewCamera(1280, 720)
	cam.Zoom = 2.2
	textui.DrawText(screen, "公交车型 · 真实车身比例", 50, 45, 30, sim.RGBA(248, 250, 252, 255))
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		y := 180. + float64(model)*170
		b := sim.NewBus(105, (y-360)/cam.Zoom, 0)
		b.SetModel(model)
		view.DrawBus(b, screen, cam)
		textui.DrawText(screen, model.Name(), 60, y-15, 24, sim.RGBA(248, 250, 252, 255))
		textui.DrawText(screen, fmt.Sprintf("载客 %d 人 · 加速度 %.0f%% · 长度 %.0f", model.Capacity(), model.Acceleration()/230*100, b.OverallLength()), 60, y+32, 18, sim.RGBA(186, 230, 253, 255))
	}
	if err := p.save(screen, "models.png"); err != nil {
		p.err = err
		return
	}
	screen.Fill(sim.RGBA(15, 23, 42, 255))
	textui.DrawText(screen, "铰接转弯 · 前节带动尾节沿铰点跟随", 50, 45, 28, sim.RGBA(248, 250, 252, 255))
	for i := 0; i < 3; i++ {
		b := sim.NewBus(0, 0, 0)
		b.SetModel(sim.BusArticulated)
		for step := 0; step < i*60; step++ {
			b.Update(1./60, 1, 0, .35, false, false)
		}
		b.TrailerHeading = sim.NormalizeAngle(b.TrailerHeading - b.Heading)
		b.Heading = 0
		b.Pos = sim.Vec2{X: 110, Y: float64(i-1) * 85}
		view.DrawBus(b, screen, cam)
		textui.DrawText(screen, fmt.Sprintf("铰接角 %.1f°", math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading()))*180/math.Pi), 60, 180+float64(i)*185, 22, sim.RGBA(186, 230, 253, 255))
	}
	if err := p.save(screen, "articulation.png"); err != nil {
		p.err = err
		return
	}
	screen.Fill(sim.RGBA(15, 23, 42, 255))
	textui.DrawText(screen, "统一站台 · 按行驶方向靠前停", 50, 45, 28, sim.RGBA(248, 250, 252, 255))
	cam.Zoom = 2
	for row := 0; row < 2; row++ {
		stop := sim.NewBusStop(row, "双泊位站台", sim.Vec2{}, 0, 2, sim.RGBA(56, 189, 248, 255), 52, 0)
		stop.Pos = sim.Vec2{Y: float64(row)*170 - 100}
		stop.ShelterPos = stop.Pos.Add(sim.Vec2{Y: 36})
		view.DrawStop(stop, screen, cam, false, false)
		if row == 0 {
			p := stop.RearBerthPosition()
			rear := sim.NewBus(p.X, p.Y, 0)
			rear.SetModel(sim.BusDoubleDecker)
			view.DrawBus(rear, screen, cam)
			p = stop.FrontBerthPosition()
			front := sim.NewBus(p.X, p.Y, 0)
			view.DrawBus(front, screen, cam)
			textui.DrawText(screen, "两辆普通车／双层车同时上下客", 50, 300, 21, sim.RGBA(186, 230, 253, 255))
		} else {
			p := stop.FrontBerthPosition()
			b := sim.NewBus(p.X, p.Y, 0)
			b.SetModel(sim.BusArticulated)
			view.DrawBus(b, screen, cam)
			textui.DrawText(screen, "一辆铰接车占用整个停靠区", 50, 640, 21, sim.RGBA(186, 230, 253, 255))
		}
	}
	p.err = p.save(screen, "station-berths.png")
}
func main() {
	output := flag.String("output", "dist/bus-model-preview", "PNG output directory")
	flag.Parse()
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	g := game.NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(3)
	g.SetLineFleet(0, traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 2, 2}, PlayerModel: sim.BusDoubleDecker, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.SetLineFleet(1, traffic.LineFleetSettings{BusCount: 4, Model: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.NetworkOpen = true
	p := &preview{g: g, output: *output}
	ebiten.SetWindowPosition(-5000, -5000)
	ebiten.SetRunnableOnUnfocused(true)
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("Bus model visual review")
	if err := ebiten.RunGame(p); err != nil {
		panic(err)
	}
	if p.err != nil {
		panic(p.err)
	}
	fmt.Println("Saved fleet panels and bus models to", *output)
}
