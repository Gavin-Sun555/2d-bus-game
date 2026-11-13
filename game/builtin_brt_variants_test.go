package game

import (
	"fmt"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Exercise what selecting each card actually starts, not just the menu label.
func TestEveryBuiltinCityHasWorkingBRTVariant(t *testing.T) {
	for _, theme := range sim.BuiltinMapThemes() {
		for _, large := range []bool{false, true} {
			t.Run(fmt.Sprintf("map%d/large%v", theme, large), func(t *testing.T) {
				g := NewGame(1280, 720)
				g.Settings.SetLargeRoadNetwork(theme, large)
				g.Settings.SetCenterBRT(theme, false)
				g.EnterCity(theme)
				ordinaryRoads := append([]sim.RoadSegment(nil), g.World.Roads...)

				g.Settings.SetCenterBRT(theme, true)
				g.Bus.Speed = 120
				g.Route.CurrentIndex = 2
				g.EnterCity(theme)
				assertBRT := func() {
					t.Helper()
					if !g.World.CenterBRT || g.World.LargeGrid != large {
						t.Fatal("selected city did not start its BRT/grid variant")
					}
					roads, stops := 0, 0
					for _, r := range g.World.Roads {
						if r.HasMedianBusLane() {
							roads++
							if r.LaneCount() != 8 || !r.HasBusLane() {
								t.Fatal("BRT outside eight-lane bus road")
							}
						}
					}
					for _, s := range g.World.Stops {
						if s.CenterBRT {
							stops++
							if s.RoadLanes != 8 {
								t.Fatal("central station has wrong road width")
							}
						}
					}
					if roads == 0 || stops == 0 {
						t.Fatalf("button has no real BRT infrastructure: roads=%d stops=%d", roads, stops)
					}
				}
				assertBRT()
				if g.Bus.Speed != 0 || g.Route.CurrentIndex != 0 {
					t.Fatal("re-entry did not reset gameplay")
				}
				g.savePathOverride = filepath.Join(t.TempDir(), "builtin-brt.json")
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
				g.Settings.SetCenterBRT(theme, false)
				g.EnterCity(theme)
				if err := g.LoadGame(); err != nil {
					t.Fatal(err)
				}
				assertBRT()
				if !g.Settings.IsCenterBRT(theme) {
					t.Fatal("save lost the city BRT choice")
				}

				g.Settings.SetCenterBRT(theme, false)
				g.EnterCity(theme)
				if g.World.CenterBRT || len(g.World.Roads) != len(ordinaryRoads) {
					t.Fatal("ordinary variant not restored")
				}
				for i, r := range g.World.Roads {
					if r != ordinaryRoads[i] {
						t.Fatalf("ordinary road %d changed after disabling BRT", i)
					}
				}
			})
		}
	}
}
