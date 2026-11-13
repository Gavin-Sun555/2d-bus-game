package view

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestMetropolisLandmarksFollowMapVariants(t *testing.T) {
	const laneWidth = 52
	w := &sim.World{}
	expected := [...]metroStyle{
		metroTerminal, metroTower, metroPlaza, metroCampus, metroStadium, metroRail,
		metroCanal, metroCampus, metroHospital, metroGarden, metroCourt, metroExhibit,
	}

	for _, large := range []bool{false, true} {
		for _, brt := range []bool{false, true} {
			t.Run(fmt.Sprintf("large=%t/brt=%t", large, brt), func(t *testing.T) {
				// The map selector rebuilds the same World when either setting changes.
				w.LargeGrid, w.CenterBRT = large, brt
				w.BuildMap(sim.MapThemeMetropolis, laneWidth)
				scene := sceneryFor(w, laneWidth)
				seen := [len(expected)]bool{}
				conventionalStations := 0
				for _, lot := range scene.metroLots {
					if !lot.landmarkLabel {
						continue
					}
					if lot.style == metroConventionalRail {
						conventionalStations++
						if lot.footprint.Pos != w.Railways()[1].StationCenter {
							t.Fatal("conventional station landmark is detached from its railway")
						}
						continue
					}
					id := lot.variant
					if id >= len(expected) || seen[id] || lot.style != expected[id] {
						t.Fatalf("unexpected landmark for stop %d with style %d", id, lot.style)
					}
					seen[id] = true
					if distance := lot.footprint.Pos.Distance(w.Stops[id].Pos); distance > 740 {
						t.Errorf("landmark for stop %d is %.0f units away", id, distance)
					}
				}
				if conventionalStations != 1 {
					t.Fatalf("got %d conventional stations, want one", conventionalStations)
				}
				for id, placed := range seen {
					if !placed {
						t.Errorf("missing landmark for stop %d", id)
					}
				}
			})
		}
	}
}

func TestAllThemesLandmarks(t *testing.T) {
	const laneWidth = 52
	for _, theme := range sim.BuiltinMapThemes() {
		t.Run(theme.ShortName(), func(t *testing.T) {
			w := sim.NewWorld(laneWidth, theme)
			scene := sceneryFor(w, laneWidth)
			placed := 0
			for _, lot := range scene.metroLots {
				if lot.landmarkLabel {
					placed++
				}
			}
			t.Logf("Theme %s placed %d major landmarks out of %d stops, total lots %d, trees %d, chunks %d", theme.ShortName(), placed, len(w.Stops), len(scene.metroLots), len(scene.trees), len(scene.treeChunks))
			if placed < 10 {
				t.Errorf("Theme %s only placed %d major landmarks", theme.ShortName(), placed)
			}
		})
	}
}
