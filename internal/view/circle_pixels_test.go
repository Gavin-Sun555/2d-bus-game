package view

import (
	"bytes"
	"fmt"
	"github.com/hajimehoshi/ebiten/v2"
	"image"
	"image/color"
	"image/png"
	"os"
	"path/filepath"
	"testing"
)

type circlePixelGame struct {
	done bool
	err  error
	out  string
}

func (g *circlePixelGame) Update() error {
	if g.done {
		return ebiten.Termination
	}
	return nil
}
func (*circlePixelGame) Layout(int, int) (int, int) { return 640, 480 }
func (g *circlePixelGame) Draw(screen *ebiten.Image) {
	if g.done {
		return
	}
	g.done = true
	target := ebiten.NewImage(640, 480)
	defer target.Deallocate()
	drawPass := func(reference bool) []byte {
		target.Fill(color.RGBA{R: 8, G: 12, B: 20, A: 255})
		radii := []float32{.01, .25, .5, 1, 2.49, 2.5, 5.99, 6, 6.001, 12, 31.5, 63.5, 64, 127.5, 128, 200}
		for zoomIndex, zoom := range []float32{.25, 1, 2, 4} {
			for i, r := range radii {
				rect := image.Rect((i%8)*80, (i/8+zoomIndex*2)*60, (i%8+1)*80, (i/8+zoomIndex*2+1)*60).Intersect(image.Rect(3, 5, 637, 475))
				dst := target.SubImage(rect).(*ebiten.Image)
				x, y := float32(rect.Min.X+28)+.125, float32(rect.Min.Y+28)+.75
				clr := color.NRGBA{R: uint8(27 + i*13), G: uint8(220 - i*9), B: uint8(40 + i*11), A: uint8(70 + i*11)}
				aa := (i+zoomIndex)%2 == 0
				if reference {
					v, idx := referenceFilledCircleMesh(x, y, r*zoom, clr, aa)
					dst.DrawTriangles(v, idx, getWhiteSubImage(), nil)
					v, idx = referenceStrokeCircleMesh(x+7, y-4, r*zoom, (float32(i%5)+.25)*zoom, clr, !aa)
					dst.DrawTriangles(v, idx, getWhiteSubImage(), nil)
				} else {
					filledCircle(dst, x, y, r*zoom, clr, aa)
					strokeCircle(dst, x+7, y-4, r*zoom, (float32(i%5)+.25)*zoom, clr, !aa)
				}
			}
		}
		pixels := make([]byte, 640*480*4)
		target.ReadPixels(pixels)
		return pixels
	}
	// Reuse the same render target and atlas position for both passes. Comparing
	// different atlas placements can introduce backend rounding unrelated to meshes.
	pa, pb := drawPass(true), drawPass(false)
	if !bytes.Equal(pa, pb) {
		n := 0
		for i := range pa {
			if pa[i] != pb[i] {
				n++
			}
		}
		g.err = fmt.Errorf("circle rendering changed %d channel bytes", n)
	}
	if g.out != "" {
		for name, p := range map[string][]byte{"original": pa, "reused": pb} {
			f, e := os.Create(filepath.Join(g.out, name+".png"))
			if e != nil {
				g.err = e
				continue
			}
			e = png.Encode(f, &image.RGBA{Pix: p, Stride: 640 * 4, Rect: image.Rect(0, 0, 640, 480)})
			f.Close()
			if e != nil {
				g.err = e
			}
		}
	}
	screen.DrawImage(target, nil)
}

var circlePixelResult *circlePixelGame

// AppKit requires the native rendering loop on the process main thread.
func TestMain(m *testing.M) {
	if os.Getenv("BUS_CIRCLE_PIXELS") == "1" {
		g := &circlePixelGame{out: os.Getenv("BUS_CIRCLE_OUTPUT")}
		ebiten.SetWindowSize(640, 480)
		ebiten.SetWindowTitle("Circle buffer pixel regression")
		ebiten.SetRunnableOnUnfocused(true)
		if e := ebiten.RunGame(g); e != nil {
			g.err = e
		}
		circlePixelResult = g
	}
	os.Exit(m.Run())
}
func TestCircleRenderedPixelsMatchOriginal(t *testing.T) {
	if os.Getenv("BUS_CIRCLE_PIXELS") != "1" {
		t.Skip("set BUS_CIRCLE_PIXELS=1 for native GPU pixel comparison")
	}
	if circlePixelResult == nil || !circlePixelResult.done {
		t.Fatal("pixel comparison did not run")
	}
	if circlePixelResult.err != nil {
		t.Fatal(circlePixelResult.err)
	}
}
