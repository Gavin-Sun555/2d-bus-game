// Command map-preview renders the actual game scenery into PNGs for visual review.
package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"

	xdraw "golang.org/x/image/draw"
	"os"
	"path/filepath"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/Gavin-Sun555/2d-bus-game/internal/view"
	"github.com/hajimehoshi/ebiten/v2"
)

type preview struct {
	output  string
	frame   int
	err     error
	worlds  []*sim.World
	details []*image.RGBA
}

func (p *preview) Update() error {
	if p.frame > 0 {
		return ebiten.Termination
	}
	return nil
}
func (p *preview) Layout(_, _ int) (int, int) { return 1100, 700 }
func (p *preview) Draw(screen *ebiten.Image) {
	if p.frame > 0 {
		return
	}
	centers := []sim.Vec2{
		{X: 1600, Y: 0},
		p.worlds[1].Stops[0].RoadCenter,
		p.worlds[2].Stops[4].RoadCenter,
		p.worlds[3].Stops[8].RoadCenter,
	}
	for i, w := range p.worlds {
		cam := sim.NewCamera(1100, 700)
		cam.Pos = centers[i]
		cam.Zoom = 0.52
		view.DrawWorld(w, screen, cam, -1, 52)
		textui.DrawText(screen, w.Theme.ShortName(), 30, 24, 24, sim.RGBA(245, 247, 239, 255))
		if err := p.save(screen, fmt.Sprintf("%02d-detail.png", i+1)); err != nil {
			p.err = err
			break
		}
		pixels := make([]byte, 1100*700*4)
		screen.ReadPixels(pixels)
		p.details = append(p.details, &image.RGBA{Pix: pixels, Stride: 4400, Rect: image.Rect(0, 0, 1100, 700)})

		// Save dedicated showcase views for landmarks
		switch w.Theme {
		case sim.MapThemeCoastalBay:
			views := []struct {
				name string
				stop int
				zoom float64
			}{
				{"coastal-west-harbor.png", 0, 0.65},
				{"coastal-golden-beach.png", 3, 0.65},
				{"coastal-deepwater-port.png", 5, 0.65},
				{"coastal-lighthouse.png", 11, 0.65},
			}
			for _, v := range views {
				if v.stop < len(w.Stops) {
					cam.Pos = w.Stops[v.stop].RoadCenter
					cam.Zoom = v.zoom
					view.DrawWorld(w, screen, cam, -1, 52)
					_ = p.save(screen, v.name)
				}
			}
		case sim.MapThemeRiverValley:
			views := []struct {
				name string
				stop int
				zoom float64
			}{
				{"river-hsr-hub.png", 4, 0.60},
				{"river-financial-towers.png", 5, 0.65},
				{"river-cable-bridgehead.png", 3, 0.65},
				{"river-north-forest.png", 0, 0.65},
			}
			for _, v := range views {
				if v.stop < len(w.Stops) {
					cam.Pos = w.Stops[v.stop].RoadCenter
					cam.Zoom = v.zoom
					view.DrawWorld(w, screen, cam, -1, 52)
					_ = p.save(screen, v.name)
				}
			}
		case sim.MapThemeTwinCities:
			views := []struct {
				name string
				stop int
				zoom float64
			}{
				{"twin-airport-terminal.png", 8, 0.60},
				{"twin-software-ring-campus.png", 4, 0.58},
				{"twin-giga-factory.png", 9, 0.60},
				{"twin-university-campus.png", 5, 0.60},
			}
			for _, v := range views {
				if v.stop < len(w.Stops) {
					cam.Pos = w.Stops[v.stop].RoadCenter
					cam.Zoom = v.zoom
					view.DrawWorld(w, screen, cam, -1, 52)
					_ = p.save(screen, v.name)
				}
			}
		}

		if i == 0 {
			for _, road := range []struct {
				name string
				x    float64
			}{{"eight-lane-normal", -2600 * sim.MapGridScale}, {"eight-lane-bus", 2600 * sim.MapGridScale}} {
				cam.Pos = sim.Vec2{X: road.x}
				cam.Zoom = .4
				view.DrawWorld(w, screen, cam, -1, 52)
				lights := sim.NewTrafficLightSystem()
				lights.BuildForWorld(w)
				for _, inter := range lights.Intersections {
					inter.Phase = sim.PhaseNSLeftGreen
				}
				view.DrawIntersectionStopLines(lights, screen, cam, 52)
				view.DrawTrafficLights(lights, screen, cam, 52, w.Railways())
				if err := p.save(screen, road.name+".png"); err != nil {
					p.err = err
					break
				}
			}
			if len(w.Roundabouts) > 0 {
				cam.Pos = w.Roundabouts[0].Center
				cam.Zoom = .9
				view.DrawWorld(w, screen, cam, -1, 52)
				if err := p.save(screen, "roundabout-yield-clearance.png"); err != nil {
					p.err = err
				}
			}
			brtWorld := &sim.World{CenterBRT: true}
			brtWorld.BuildMap(w.Theme, 52)
			var overpassStop, underpassStop *sim.BusStop
			for _, st := range brtWorld.Stops {
				if st == nil || !st.CenterBRT {
					continue
				}
				gx := int(math.Round(st.RoadCenter.X / 50.0))
				gy := int(math.Round(st.RoadCenter.Y / 50.0))
				h := gx*37 + gy*101
				if h < 0 {
					h = -h
				}
				if h%2 == 0 && overpassStop == nil {
					overpassStop = st
				} else if h%2 == 1 && underpassStop == nil {
					underpassStop = st
				}
			}
			if overpassStop != nil {
				cam.Pos = overpassStop.RoadCenter
				cam.Zoom = 0.85
				view.DrawWorld(brtWorld, screen, cam, -1, 52)
				view.DrawBRTOverpasses(brtWorld, screen, cam, false)
				if err := p.save(screen, "brt-overpass-station.png"); err != nil {
					p.err = err
				}
			}
			if underpassStop != nil {
				cam.Pos = underpassStop.RoadCenter
				cam.Zoom = 0.85
				view.DrawWorld(brtWorld, screen, cam, -1, 52)
				if err := p.save(screen, "brt-underpass-station.png"); err != nil {
					p.err = err
				}
			}
			cam.Pos = centers[i]
			cam.Zoom = .48
		}
		// Review world-anchored textures and bridge geometry with a rotated camera.
		cam.Rotation = math.Pi / 3
		view.DrawWorld(w, screen, cam, -1, 52)
		if err := p.save(screen, fmt.Sprintf("%02d-rotated.png", i+1)); err != nil {
			p.err = err
			break
		}
		cam.Rotation = 0
		cam.Pos = sim.Vec2{}
		cam.Zoom = 0.032
		view.DrawWorld(w, screen, cam, -1, 52, true)
		textui.DrawText(screen, w.Theme.ShortName(), 30, 24, 24, sim.RGBA(245, 247, 239, 255))
		if err := p.save(screen, fmt.Sprintf("%02d-map.png", i+1)); err != nil {
			p.err = err
			break
		}
	}
	if p.err == nil {
		p.err = p.saveComparison()
	}
	p.frame++
}
func (p *preview) save(screen *ebiten.Image, name string) error {
	pixels := make([]byte, 1100*700*4)
	screen.ReadPixels(pixels)
	img := &image.RGBA{Pix: pixels, Stride: 1100 * 4, Rect: image.Rect(0, 0, 1100, 700)}
	f, err := os.Create(filepath.Join(p.output, name))
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, img)
}
func (p *preview) saveComparison() error {
	canvas := image.NewRGBA(image.Rect(0, 0, 1100, 1050))
	xdraw.Draw(canvas, canvas.Bounds(), image.NewUniform(color.RGBA{R: 20, G: 29, B: 38, A: 255}), image.Point{}, xdraw.Src)
	for i, detail := range p.details {
		x, y := (i%2)*550, (i/2)*350
		if i == 4 {
			x = 275
		}
		xdraw.CatmullRom.Scale(canvas, image.Rect(x, y, x+550, y+350), detail, detail.Bounds(), xdraw.Src, nil)
	}
	f, err := os.Create(filepath.Join(p.output, "comparison.png"))
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, canvas)
}
func main() {
	output := flag.String("output", "dist/map-preview", "PNG output directory")
	flag.Parse()
	if err := os.MkdirAll(*output, 0755); err != nil {
		panic(err)
	}
	p := &preview{output: *output}
	for _, theme := range sim.BuiltinMapThemes() {
		p.worlds = append(p.worlds, sim.NewWorld(52, theme))
	}
	ebiten.SetWindowSize(1100, 700)
	ebiten.SetWindowTitle("Map visual review")
	if err := ebiten.RunGame(p); err != nil {
		panic(err)
	}
	if p.err != nil {
		panic(p.err)
	}
	fmt.Println("Saved four map overviews, driving details, rotated views and comparison to", *output)
}
