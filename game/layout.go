package game

import "math"

const (
	layoutBaseWidth  = 1280.
	layoutBaseHeight = 720.
)

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	w, h := g.LayoutF(float64(outsideWidth), float64(outsideHeight))
	return int(math.Ceil(w)), int(math.Ceil(h))
}

// Keep the UI's baseline scale, expanding the visible world to match the
// window/monitor aspect instead of letterboxing a fixed 16:9 canvas. LayoutF
// avoids fractional-size rounding bars on Retina and scaled Mac displays.
func (g *Game) LayoutF(outsideWidth, outsideHeight float64) (float64, float64) {
	w, h := layoutBaseWidth, layoutBaseHeight
	if outsideWidth > 0 && outsideHeight > 0 && !math.IsInf(outsideWidth, 0) && !math.IsInf(outsideHeight, 0) {
		if outsideWidth/outsideHeight >= layoutBaseWidth/layoutBaseHeight {
			w = layoutBaseHeight * outsideWidth / outsideHeight
		} else {
			h = layoutBaseWidth * outsideHeight / outsideWidth
		}
	} else if g.Width > 0 && g.Height > 0 {
		w, h = float64(g.Width), float64(g.Height)
	}
	g.Width, g.Height = int(math.Ceil(w)), int(math.Ceil(h))
	if g.Camera != nil {
		g.Camera.ScreenWidth, g.Camera.ScreenHeight = g.Width, g.Height
	}
	return w, h
}
