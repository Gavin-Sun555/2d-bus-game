package game

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func TestViewportFillsWindowAndKeepsCameraState(t *testing.T) {
	g := &Game{Width: 1280, Height: 720, Camera: sim.NewCamera(1280, 720)}
	g.Camera.Pos = sim.Vec2{X: 1234, Y: -567}
	g.Camera.TargetPos = g.Camera.Pos
	g.Camera.Zoom = .5
	g.Camera.TargetZoom = .5
	g.Camera.Rotation = .3
	for _, size := range [][2]float64{{1280, 720}, {1470, 956}, {1512, 982}, {1728, 1117}, {1440, 900}, {1920, 1200}, {1920, 1080}, {2560, 1080}, {800, 600}, {640, 900}, {1470.5, 956.25}, {1280, 720}} {
		t.Run(fmt.Sprint(size), func(t *testing.T) {
			w, h := g.LayoutF(size[0], size[1])
			if math.Abs(w/h-size[0]/size[1]) > 1e-12 {
				t.Fatalf("letterboxed aspect: logical%v,%v outside%v", w, h, size)
			}
			if w < 1280-1e-9 || h < 720-1e-9 {
				t.Fatal("UI baseline clipped", w, h)
			}
			if g.Camera.ScreenWidth != int(math.Ceil(w)) || g.Camera.ScreenHeight != int(math.Ceil(h)) {
				t.Fatal("camera viewport mismatch")
			}
			if g.Camera.Pos != (sim.Vec2{X: 1234, Y: -567}) || g.Camera.Zoom != .5 || g.Camera.Rotation != .3 {
				t.Fatal("resize reset camera pose")
			}
			p := sim.Vec2{X: 1300, Y: -700}
			q := g.Camera.ScreenToWorld(g.Camera.WorldToScreen(p))
			if p.Distance(q) > 1e-9 {
				t.Fatal("planning coordinates changed", p, q)
			}
		})
	}
}

func TestViewportIgnoresTransientInvalidSize(t *testing.T) {
	g := &Game{Width: 1280, Height: 800, Camera: sim.NewCamera(1280, 800)}
	for _, s := range [][2]float64{{0, 0}, {0, 900}, {1440, 0}, {-1, 900}, {math.NaN(), 900}, {math.Inf(1), 900}} {
		w, h := g.LayoutF(s[0], s[1])
		if w != 1280 || h != 800 {
			t.Fatal("invalid size changed viewport", w, h)
		}
	}
	w, h := g.Layout(1440, 900)
	if w != 1280 || h != 800 {
		t.Fatal("integer layout mismatch", w, h)
	}
}

func TestViewportKeepsMenusAndHUDInsideCanvas(t *testing.T) {
	g := NewGame(1280, 720)
	inside := func(name string, r sim.Rect) {
		t.Helper()
		if r.W > 0 && r.H > 0 && (r.X < 0 || r.Y < 0 || r.X+r.W > float64(g.Width) || r.Y+r.H > float64(g.Height)) {
			t.Fatalf("%s outside %dx%d: %+v", name, g.Width, g.Height, r)
		}
	}
	for _, s := range [][2]int{{1440, 900}, {1470, 956}, {2560, 1080}, {800, 600}} {
		g.Layout(s[0], s[1])
		screen := ebiten.NewImage(g.Width, g.Height)
		g.State = StateMainMenu
		g.Draw(screen)
		inside("start", g.menuStartBounds)
		inside("language", g.menuLangBounds)
		inside("continue", g.menuContinueBounds)
		inside("large_grid", g.menuLargeGridBounds)
		for _, r := range g.menuCardBounds {
			inside("city", r)
		}
		g.EnterCity(sim.MapThemeMetropolis)
		g.Draw(screen)
		inside("menu", g.HUD.MenuBtnBounds)
		inside("camera", g.HUD.CameraBtnBounds)
		inside("settings", g.HUD.SettingsBtnBounds)
		inside("autopilot", g.HUD.AutopilotBounds)
		g.TogglePlanMode()
		g.Draw(screen)
		inside("planning", g.HUD.PlanPanelBounds)
		inside("undo", g.HUD.PlanUndoBtnBounds)
		inside("clear", g.HUD.PlanClearBtnBounds)
		g.NetworkOpen = true
		g.Draw(screen)
		for _, r := range g.networkButtons {
			inside("network", r)
		}
		g.NetworkOpen = false
		g.Settings.IsOpen = true
		g.Draw(screen)
		g.Settings.IsOpen = false
		g.OperationsPanel.IsOpen = true
		g.Draw(screen)
		g.OperationsPanel.IsOpen = false
		screen.Dispose()
	}
}
