package main

import (
	"bytes"
	_ "embed"
	"image"
	_ "image/png"
	"log"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/hajimehoshi/ebiten/v2"
)

//go:embed assets/logo_512.png
var icon512Bytes []byte

//go:embed assets/icon_128.png
var icon128Bytes []byte

//go:embed assets/icon_64.png
var icon64Bytes []byte

//go:embed VERSION
var releaseVersion string

func main() {
	const (
		screenWidth  = 1280
		screenHeight = 720
	)

	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("2D Bus Simulator & Route Planner v" + strings.TrimSpace(releaseVersion) + " (Go + Ebitengine)")
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)

	// Set custom game logo as application window icon (multi-resolution for crisp Retina display)
	var icons []image.Image
	if img512, _, err := image.Decode(bytes.NewReader(icon512Bytes)); err == nil {
		icons = append(icons, img512)
	}
	if img128, _, err := image.Decode(bytes.NewReader(icon128Bytes)); err == nil {
		icons = append(icons, img128)
	}
	if img64, _, err := image.Decode(bytes.NewReader(icon64Bytes)); err == nil {
		icons = append(icons, img64)
	}
	if len(icons) > 0 {
		ebiten.SetWindowIcon(icons)
	}

	g := game.NewGame(screenWidth, screenHeight)

	if err := ebiten.RunGame(game.WithGraphicsWarmup(g)); err != nil {
		log.Fatalf("Game crashed: %v", err)
	}
}
