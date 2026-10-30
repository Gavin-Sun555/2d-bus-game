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
	if err := g.EnableStationNames(); err != nil {
		log.Printf("Station names unavailable: %v", err)
		g.HUD.AddWarningToast("无法读取自定义站名文件 / Cannot read custom station names")
	}
	if err := g.EnableAudio(); err != nil {
		log.Printf("Sound unavailable: %v", err)
	}
	defer g.CloseAudio()
	if err := g.EnableAnnouncements(); err != nil {
		log.Printf("Announcements unavailable: %v", err)
		g.HUD.AddWarningToast("本地报站语音不可用 / Local announcement voices unavailable")
	}
	defer g.CloseAnnouncements()

	if err := ebiten.RunGame(game.WithGraphicsWarmup(g)); err != nil {
		log.Fatalf("Game crashed: %v", err)
	}
}
