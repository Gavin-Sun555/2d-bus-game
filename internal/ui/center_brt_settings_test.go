package ui

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestCenterBRTSettingsIndependentAndLegacy(t *testing.T) {
	s := &Settings{MapTheme: sim.MapThemeMetropolis, CenterBRT: true}
	if !s.IsCenterBRT(sim.MapThemeMetropolis) || s.IsCenterBRT(sim.MapThemeCustom) {
		t.Fatal("legacy preference leaked into another map")
	}
	s.ToggleCenterBRT(sim.MapThemeCustom)
	if !s.IsCenterBRT(sim.MapThemeMetropolis) || !s.IsCenterBRT(sim.MapThemeCustom) {
		t.Fatal("migration lost legacy choice")
	}
	s.ToggleCenterBRT(sim.MapThemeMetropolis)
	if s.IsCenterBRT(sim.MapThemeMetropolis) || !s.IsCenterBRT(sim.MapThemeCustom) {
		t.Fatal("map choices are not independent")
	}
	data, e := json.Marshal(s)
	if e != nil {
		t.Fatal(e)
	}
	var loaded Settings
	if e = json.Unmarshal(data, &loaded); e != nil {
		t.Fatal(e)
	}
	if loaded.IsCenterBRT(sim.MapThemeMetropolis) || !loaded.IsCenterBRT(sim.MapThemeCustom) {
		t.Fatal("map choices lost on serialization")
	}
}
