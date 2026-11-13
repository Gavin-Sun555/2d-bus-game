// Command vehicle-style-preview captures the actual bus and car renderers.
package main

import (
	"flag"
	"image"
	"image/png"
	"os"
	"path/filepath"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type preview struct {
	out  string
	done bool
	err  error
}

func (p *preview) Update() error {
	if p.done {
		return ebiten.Termination
	}
	return nil
}
func (p *preview) Layout(int, int) (int, int) { return 1280, 720 }
func (p *preview) Draw(screen *ebiten.Image) {
	if p.done {
		return
	}
	p.done = true
	for _, night := range []bool{false, true} {
		screen.Fill(sim.RGBA(36, 48, 61, 255))
		for _, y := range []float32{190, 365, 540} {
			vector.DrawFilledRect(screen, 12, y-58, 1256, 116, sim.RGBA(46, 58, 70, 255), false)
		}
		cam := sim.NewCamera(1280, 720)
		cam.Zoom = 2.5
		point := func(x, y float64) sim.Vec2 { return sim.Vec2{X: (x - 640) / cam.Zoom, Y: (y - 360) / cam.Zoom} }
		buses := []*sim.Bus{}
		tm := &traffic.TrafficManager{}
		for i, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			y := 190 + float64(i)*175
			pos := point(510, y)
			b := sim.NewBus(pos.X, pos.Y, 0)
			b.SetModel(model)
			b.HeadlightsOn = night
			buses = append(buses, b)
			view.DrawBus(b, screen, cam)
			kind := []sim.CarType{sim.CarSedan, sim.CarSUV, sim.CarTaxi}[i]
			color := []simColor{{210, 65, 60}, {67, 127, 190}, {235, 186, 46}}[i]
			tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: point(1030, y), Type: kind, Length: 46, Width: 22, Color: sim.RGBA(color.r, color.g, color.b, 255), HeadlightsOn: night, IsBraking: i == 1})
		}
		view.DrawTraffic(tm, screen, cam, 1)
		w := &sim.World{}
		if night {
			w.Clock.ElapsedMinutes = 780
		}
		fleet := &traffic.AIBusManager{Buses: []*traffic.AIBus{{Bus: buses[1]}, {Bus: buses[2]}}}
		view.DrawDayNightLighting(screen, cam, w, 52, buses[0], tm, fleet, 1, false)
		title := "08:00 · DAY / HEADLIGHTS OFF"
		name := "vehicles-day.png"
		if night {
			title = "21:00 · NIGHT / HEADLIGHTS ON"
			name = "vehicles-night.png"
		}
		textui.DrawText(screen, title, 30, 35, 24, sim.RGBA(225, 237, 245, 255))
		for i, label := range []string{"STANDARD BUS", "DOUBLE-DECKER", "ARTICULATED BUS"} {
			textui.DrawText(screen, label, 30, 112+float64(i)*175, 15, sim.RGBA(191, 219, 238, 255))
		}
		for i, label := range []string{"SEDAN", "SUV · BRAKE LIGHTS", "TAXI"} {
			textui.DrawText(screen, label, 750, 112+float64(i)*175, 15, sim.RGBA(191, 219, 238, 255))
		}
		img := image.NewRGBA(screen.Bounds())
		screen.ReadPixels(img.Pix)
		f, err := os.Create(filepath.Join(p.out, name))
		if err != nil {
			p.err = err
			return
		}
		err = png.Encode(f, img)
		f.Close()
		if err != nil {
			p.err = err
			return
		}
	}
}

type simColor struct{ r, g, b uint8 }

func main() {
	out := flag.String("output", "dist/vehicle-style-preview", "PNG output directory")
	flag.Parse()
	if err := os.MkdirAll(*out, 0755); err != nil {
		panic(err)
	}
	p := &preview{out: *out}
	ebiten.SetWindowSize(1280, 720)
	ebiten.SetRunnableOnUnfocused(true)
	ebiten.SetWindowTitle("Vehicle style review")
	if err := ebiten.RunGame(p); err != nil {
		panic(err)
	}
	if p.err != nil {
		panic(p.err)
	}
}
