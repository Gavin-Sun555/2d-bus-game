package main

import (
	"log"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/hajimehoshi/ebiten/v2"
)

func main() {
	const (
		screenWidth  = 1280
		screenHeight = 720
	)

	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("2D Bus Simulator & Route Planner (Go + Ebitengine)")
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)

	g := game.NewGame(screenWidth, screenHeight)

	if err := ebiten.RunGame(g); err != nil {
		log.Fatalf("Game crashed: %v", err)
	}
}
