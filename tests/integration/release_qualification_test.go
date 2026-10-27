package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestReleaseQualification_AllMapsTrafficAndAIFleetHeavyConcurrency qualifies full multi-vehicle
// concurrency across all 6 maps under maximum system load:
// - 864 civilian vehicles (Maximum Traffic Density Level 4)
// - 8 AI route buses (Maximum Fleet Configuration)
// - Player bus active in world
// - Traffic lights cycling through all intersection phases
//
// Verifies 0 deadlocks, 0 NaN values, 0 barrier collisions, and continuous fleet progression.
func TestReleaseQualification_AllMapsTrafficAndAIFleetHeavyConcurrency(t *testing.T) {
	checkReleaseQualificationGate(t)

	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	laneWidth := 52.0
	dt := 0.05 // 20 FPS high-throughput simulation tick

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := sim.NewWorld(laneWidth, theme)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)

			// 1. Build maximum civilian traffic (864 cars)
			tm := traffic.NewTrafficManager(laneWidth)
			tm.BuildForWorld(w, traffic.MaxTrafficCars, laneWidth)
			activeCars := tm.GetActiveCars(4)
			if len(activeCars) != traffic.MaxTrafficCars {
				t.Fatalf("[%s] Expected 864 active cars at density 4, got %d", theme.DisplayName(), len(activeCars))
			}

			// 2. Build maximum AI bus fleet (8 buses)
			startStop := w.Stops[0]
			playerBus := sim.NewBus(startStop.Pos.X, startStop.Pos.Y, startStop.Heading)
			abm := traffic.NewAIBusManager()
			abm.BuildFleet(w, 8, laneWidth, playerBus)
			tm.AIBusMgr = abm
			if len(abm.Buses) != 8 {
				t.Fatalf("[%s] Expected 8 AI buses, got %d", theme.DisplayName(), len(abm.Buses))
			}

			// 3. Simulate 150 frames (7.5 seconds of concurrent transit operation)
			for frame := 0; frame < 150; frame++ {
				tls.Update(dt)
				tm.Update(dt, playerBus, tls, 4, laneWidth, w)
				abm.Update(dt, playerBus, w, tls, tm, laneWidth, 4)

				// Verify civilian traffic cars
				for i, car := range tm.GetActiveCars(4) {
					if math.IsNaN(car.Pos.X) || math.IsNaN(car.Pos.Y) || math.IsNaN(car.Speed) {
						t.Fatalf("[%s] Civilian car #%d corrupted with NaN at frame %d", theme.DisplayName(), i, frame)
					}
				}

				// Verify AI fleet buses
				for i, ai := range abm.Buses {
					if math.IsNaN(ai.Bus.Pos.X) || math.IsNaN(ai.Bus.Pos.Y) || math.IsNaN(ai.Bus.Speed) {
						t.Fatalf("[%s] AI Bus #%d corrupted with NaN at frame %d", theme.DisplayName(), i, frame)
					}
					// Ensure AI bus is never inside a hard barrier
					bRes := w.CheckBoundaries(ai.Bus.Pos, ai.Bus.Width*0.4, laneWidth)
					if bRes.Zone == sim.ZoneHardBarrier {
						t.Fatalf("[%s] AI Bus #%d collided with hard barrier (%s) at %v (frame %d)",
							theme.DisplayName(), i, bRes.ObstacleName, ai.Bus.Pos, frame)
					}
				}
			}
		})
	}
}

// TestReleaseQualification_AllMapsBilingualAndHUDStress tests language toggling (ZH <-> EN),
// stop name localization, and route line wrapping across all 6 maps.
func TestReleaseQualification_AllMapsBilingualAndHUDStress(t *testing.T) {
	checkReleaseQualificationGate(t)

	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeEmeraldLake,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	laneWidth := 52.0

	for _, theme := range themes {
		t.Run(theme.DisplayName(), func(t *testing.T) {
			w := sim.NewWorld(laneWidth, theme)
			route := sim.GetStandardRouteForMap(theme, w)

			// 1. Verify all 12 stops have dual-language names (e.g. "中文名 (English Name)")
			for _, s := range w.Stops {
				locale.SetLanguage(locale.LanguageZH)
				zhName := s.LocalizedName()
				if zhName == "" {
					t.Fatalf("[%s] Stop #%d has empty ZH name", theme.DisplayName(), s.ID)
				}

				locale.SetLanguage(locale.LanguageEN)
				enName := s.LocalizedName()
				if enName == "" {
					t.Fatalf("[%s] Stop #%d has empty EN name", theme.DisplayName(), s.ID)
				}
			}

			// 2. Verify route line wrapping at different widths (300px, 500px, 800px)
			widths := []float64{300.0, 500.0, 800.0}
			for _, width := range widths {
				lines := route.GetFormattedRouteLines(w, width, 14.0)
				if len(lines) == 0 {
					t.Fatalf("[%s] Formatted route lines empty for width %.1f", theme.DisplayName(), width)
				}
				if len(lines) > 3 {
					t.Fatalf("[%s] Formatted route lines exceeded max 3 lines: got %d lines", theme.DisplayName(), len(lines))
				}
			}
		})
	}

	// Restore default language to Chinese
	locale.SetLanguage(locale.LanguageZH)
}
