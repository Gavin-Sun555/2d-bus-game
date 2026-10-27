package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestWideDrivingViewSurvivesPlanningAndRouteReset(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.Camera.ToggleDrivingView()
	g.TogglePlanMode()
	if g.Camera.Zoom != .04 {
		t.Fatal("planning view was not retained")
	}
	g.TogglePlanMode()
	if g.Camera.DrivingViewMultiplier() != 2 || g.Camera.Zoom != .5 || g.Camera.TargetZoom != .5 {
		t.Fatal("return from planning lost wide view")
	}
	g.ResetBusesToRouteStart()
	if g.Camera.Zoom != .5 {
		t.Fatal("route reset lost wide view")
	}
	g.TogglePlanMode()
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.Camera.Zoom != .5 {
		t.Fatal("city entry lost driving view")
	}
	g.Camera.ToggleDrivingView()
	g.TogglePlanMode()
	g.TogglePlanMode()
	g.ResetBusesToRouteStart()
	if g.Camera.DrivingViewMultiplier() != 4 || g.Camera.Zoom != .25 || g.Camera.TargetZoom != .25 {
		t.Fatal("4× view was lost after planning or route reset")
	}
	g.Camera.ToggleDrivingView()
	g.TogglePlanMode()
	g.TogglePlanMode()
	if g.Camera.Zoom != 1 {
		t.Fatal("default view did not restore")
	}
}
