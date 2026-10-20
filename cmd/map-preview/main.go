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
	centers := []sim.Vec2{{X: 1600, Y: 0}, {X: 400, Y: 0}, {X: 3900, Y: -650}, {X: -4400, Y: 0}, {X: 4300, Y: 0}}
	for i, w := range p.worlds {
		cam := sim.NewCamera(1100, 700)
		cam.Pos = centers[i]
		cam.Zoom = 0.48
		view.DrawWorld(w, screen, cam, -1, 52)
		textui.DrawText(screen, w.Theme.ShortName(), 30, 24, 24, sim.RGBA(245, 247, 239, 255))
		if err := p.save(screen, fmt.Sprintf("%02d-detail.png", i+1)); err != nil {
			p.err = err
			break
		}
		pixels := make([]byte, 1100*700*4)
		screen.ReadPixels(pixels)
		p.details = append(p.details, &image.RGBA{Pix: pixels, Stride: 4400, Rect: image.Rect(0, 0, 1100, 700)})
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
				view.DrawTrafficLights(lights, screen, cam, 52)
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
		cam.Zoom = 0.065
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
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeTwinCities; theme++ {
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
	fmt.Println("Saved five map overviews, driving details, rotated views and comparison to", *output)
}
