package game

import (
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func TestCenterBRTSelectionResetAndSave(t *testing.T) {
	g := NewGame(1280, 720)
	screen := ebiten.NewImage(1280, 720)
	defer screen.Dispose()
	g.State = StateMainMenu
	g.Draw(screen)
	b := g.menuBRTBounds
	if b.W <= 0 || b.H <= 0 || !g.menuCardBounds[0].Contains(sim.Vec2{X: b.X + b.W/2, Y: b.Y + b.H/2}) {
		t.Fatal("BRT option missing from first map card")
	}
	if b.Y+b.H > g.menuCardLargeGridBounds[0].Y {
		t.Fatal("map options overlap")
	}
	for _, large := range []bool{false, true} {
		g.Settings.SetLargeRoadNetwork(sim.MapThemeMetropolis, large)
		g.Settings.SetCenterBRT(sim.MapThemeMetropolis, true)
		g.EnterCity(sim.MapThemeMetropolis)
		if !g.World.CenterBRT || g.World.LargeGrid != large {
			t.Fatal("map options are not independent")
		}
		g.Bus.Speed = 100
		g.Route.CurrentIndex = 2
		g.Settings.SetCenterBRT(sim.MapThemeMetropolis, false)
		g.EnterCity(sim.MapThemeMetropolis)
		if g.World.CenterBRT || g.Bus.Speed != 0 || g.Route.CurrentIndex != 0 {
			t.Fatal("changing BRT did not reset city")
		}
		g.Settings.SetCenterBRT(sim.MapThemeMetropolis, true)
		g.EnterCity(sim.MapThemeMetropolis)
	}
	g.savePathOverride = filepath.Join(t.TempDir(), "brt.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	g.Settings.SetCenterBRT(sim.MapThemeMetropolis, false)
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.World.CenterBRT {
		t.Fatal("BRT leaked into second map")
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if !g.Settings.CenterBRT || !g.World.CenterBRT {
		t.Fatal("save lost BRT variant")
	}
	count := 0
	for _, s := range g.World.Stops {
		if s.CenterBRT {
			count++
		}
	}
	if count == 0 {
		t.Fatal("save lost central stations")
	}
	g.Draw(screen)
}

func TestCustomMapCenterBRTOptionAndIndependentSave(t *testing.T) {
	g := NewGame(1280, 720)
	data := sim.GetDefaultCustomMapData()
	data.Roads[0].Lanes = 8
	data.Roads[0].BusLane = true
	g.World.CustomMap = data
	g.Settings.SetCenterBRT(sim.MapThemeMetropolis, true)
	g.Settings.SetCenterBRT(sim.MapThemeCustom, false)
	g.State = StateMainMenu
	screen := ebiten.NewImage(1280, 720)
	defer screen.Dispose()
	g.Draw(screen)
	if g.menuCardBRTBounds[0].W <= 0 || g.menuCardBRTBounds[4].W <= 0 {
		t.Fatal("eligible map missing BRT button")
	}
	for i, theme := range sim.SelectableMapThemes() {
		if g.menuCardBRTBounds[i].W > 0 != g.centerBRTAvailable(theme) {
			t.Fatal("BRT button availability does not match map", theme)
		}
	}
	g.EnterCity(sim.MapThemeCustom)
	if g.World.CenterBRT {
		t.Fatal("custom inherited Metropolis selection")
	}
	g.Settings.ToggleCenterBRT(sim.MapThemeCustom)
	g.Bus.Speed = 120
	g.Route.CurrentIndex = 2
	g.EnterCity(sim.MapThemeCustom)
	if !g.World.CenterBRT || g.Bus.Speed != 0 || g.Route.CurrentIndex != 0 {
		t.Fatal("custom BRT selection did not rebuild city")
	}
	if !g.Settings.IsCenterBRT(sim.MapThemeMetropolis) {
		t.Fatal("custom selection changed first map choice")
	}
	g.savePathOverride = filepath.Join(t.TempDir(), "custom-brt.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	g.Settings.SetCenterBRT(sim.MapThemeCustom, false)
	g.EnterCity(sim.MapThemeMetropolis)
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if !g.World.CenterBRT || !g.Settings.IsCenterBRT(sim.MapThemeCustom) || !g.Settings.IsCenterBRT(sim.MapThemeMetropolis) {
		t.Fatal("save lost independent map choices")
	}
	// Removing bus lanes must clear stale menu hit boxes on the next draw.
	g.World.CustomMap.Roads[0].BusLane = false
	g.State = StateMainMenu
	g.Draw(screen)
	if g.menuCardBRTBounds[4].W != 0 {
		t.Fatal("custom BRT button remained after bus lanes were removed")
	}
}
