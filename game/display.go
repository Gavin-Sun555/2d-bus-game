package game

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

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

func (g *Game) updateDrivingViewHotkey() {
	if !inpututil.IsKeyJustPressed(ebiten.KeyX) {
		return
	}
	switch g.Camera.ToggleDrivingView() {
	case 2:
		g.HUD.AddToast(locale.T("视野已扩大：长宽各 2 倍；按 X 切换 4 倍", "Wide view: 2× width and height. Press X for 4×."), false)
	case 4:
		g.HUD.AddToast(locale.T("视野已扩大：长宽各 4 倍，面积 16 倍；按 X 恢复", "Wide view: 4× width and height, 16× area. Press X to restore."), false)
	default:
		g.HUD.AddToast(locale.T("已恢复默认视野；按 X 切换 2 倍", "Default view restored. Press X for 2×."), false)
	}
}

// WithGraphicsWarmup submits the vector curve shaders before simulation starts.
// Keeping this in the application wrapper leaves headless replay and Game.Update
// semantics unchanged. A new process warms once; map changes need no new pass.
func WithGraphicsWarmup(g *Game) *GraphicsWarmupGame {
	return &GraphicsWarmupGame{game: g}
}

type GraphicsWarmupGame struct {
	game  *Game
	ready bool
}

func (g *GraphicsWarmupGame) Update() error {
	if !g.ready {
		return nil
	}
	return g.game.Update()
}
func (g *GraphicsWarmupGame) Layout(w, h int) (int, int)              { return g.game.Layout(w, h) }
func (g *GraphicsWarmupGame) LayoutF(w, h float64) (float64, float64) { return g.game.LayoutF(w, h) }
func (g *GraphicsWarmupGame) Draw(screen *ebiten.Image) {
	if g.ready {
		g.game.Draw(screen)
		return
	}
	screen.Fill(color.RGBA{R: 15, G: 23, B: 42, A: 255})
	textui.DrawText(screen, "正在加载… / Loading…", 32, 32, 18, color.White)
	warmupCurvePipeline()
	g.ready = true
}

// Ebitengine creates the Bezier shader lazily and compiles its Metal library
// only on a real draw. ReadPixels submits and waits for this small offscreen
// draw, so setting ready cannot defer that compilation to the first city frame.
func warmupCurvePipeline() {
	sample := ebiten.NewImage(32, 32)
	var path vector.Path
	path.MoveTo(2, 26)
	path.QuadTo(16, 2, 30, 26)
	path.Close()
	op := vector.DrawPathOptions{AntiAlias: true}
	vector.FillPath(sample, &path, nil, &op)
	var pixels [32 * 32 * 4]byte
	sample.ReadPixels(pixels[:])
	sample.Deallocate()
}
