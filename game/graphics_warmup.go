package game

import (
	"image/color"

	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

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
