package main

import (
	"bytes"
	_ "embed"
	"flag"
	"image"
	_ "image/png"
	"log"
	"os"
	runtimedebug "runtime/debug"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/game"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
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
	debug := flag.Bool("debug", false, "save long congestion and pre-recovery traffic snapshots")
	flag.Parse()
	// Leave headroom between collections in large, busy cities. Explicit
	// runtime settings from the launch environment still take precedence.
	if _, set := os.LookupEnv("GOGC"); !set {
		runtimedebug.SetGCPercent(200)
	}
	if _, set := os.LookupEnv("GOMEMLIMIT"); !set {
		runtimedebug.SetMemoryLimit(1 << 30)
	}
	locale.InitSystemLanguage()
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
	if *debug {
		if err := g.EnableDebug(); err != nil {
			log.Fatalf("Enable debug mode: %v", err)
		}
	}
	g.ShowModeSelection()
	g.EnableAutosaves()
	ebiten.SetWindowClosingHandled(true)
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

	runErr := ebiten.RunGame(game.WithGraphicsWarmup(g))
	g.FlushDebug()
	if err := g.SaveOnExit(); err != nil {
		log.Printf("Exit autosave failed: %v", err)
	}
	if err := runErr; err != nil {
		log.Fatalf("Game crashed: %v", err)
	}
}
