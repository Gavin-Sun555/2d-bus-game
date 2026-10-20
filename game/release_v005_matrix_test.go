package game

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRelease005SettingsMatrix(t *testing.T) {
	if testing.Short() {
		t.Skip("Release settings matrix")
	}
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		for width := 0; width < 4; width++ {
			for density := 1; density <= 4; density++ {
				for mode := 0; mode < 3; mode++ {
					t.Run(fmt.Sprintf("map%d/width%d/density%d/mode%d", theme, width, density, mode), func(t *testing.T) {
						sim.SeedSimulation(5)
						g := NewGame(1280, 720)
						g.Settings.LaneWidthLevel = width
						g.Settings.TrafficDensity = density
						g.Settings.AIBusCount = 8
						g.Settings.AutopilotEnabled = mode > 0
						g.Settings.AutopilotBerserk = mode == 2
						g.EnterCity(theme)
						for tick := 0; tick < 12; tick++ {
							if err := g.Update(); err != nil {
								t.Fatal(err)
							}
						}
						if got := len(g.TrafficMgr.GetActiveCars(density)); got != sim.TrafficCarCount(density) {
							t.Fatalf("Traffic density=%d want=%d", got, sim.TrafficCarCount(density))
						}
						if len(g.AIBusMgr.Buses) != 8 {
							t.Fatal("Fleet setting not applied")
						}
						if !finiteVehicle(g.Bus.Pos, g.Bus.Heading, g.Bus.Speed) {
							t.Fatal("Player state invalid")
						}
						for _, ab := range g.AIBusMgr.Buses {
							if !finiteVehicle(ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Speed) {
								t.Fatal("Fleet state invalid")
							}
						}
						// Settings and route planning must not advance city traffic or signals.
						pos := g.Bus.Pos
						timers := make([]float64, len(g.TrafficLights.Intersections))
						for i, inter := range g.TrafficLights.Intersections {
							timers[i] = inter.Timer
						}
						g.Settings.IsOpen = true
						if err := g.Update(); err != nil {
							t.Fatal(err)
						}
						g.Settings.IsOpen = false
						g.IsPlanMode = true
						if err := g.Update(); err != nil {
							t.Fatal(err)
						}
						if g.Bus.Pos != pos {
							t.Fatal("Paused bus moved")
						}
						for i, inter := range g.TrafficLights.Intersections {
							if inter.Timer != timers[i] {
								t.Fatal("Paused signal advanced")
							}
						}
					})
				}
			}
		}
	}
}
