package sim

import (
	"fmt"
	"strings"
	"testing"
)

func TestDefaultTransitCircuitsHaveAtMostTwoUTurns(t *testing.T) {
	const lw = 52.
	check := func(t *testing.T, w *World) {
		t.Helper()
		for line, route := range DefaultTransitLines(w) {
			if turns := RouteUTurnCount(w, route.StopIDs, lw); turns > MaxRouteUTurns {
				t.Errorf("line %d exceeds the route planning limit with %d U-turns", line+1, turns)
			}
			turns := 0
			approachTurns := 0
			for i := 1; i < len(route.StopIDs); i++ {
				from, to := route.StopIDs[i-1], route.StopIDs[i]
				path := computeRoadPathBetweenStops(w, w.Stops[from], w.Stops[to], lw, &turns)
				computeRoadPathBetweenStops(w, w.Stops[from], w.Stops[to].ForwardDockStop(), lw, &approachTurns)
				if len(path) < 2 {
					t.Fatalf("line %d missing path from %d to %d", line+1, from, to)
				}
				for _, p := range path {
					if w.CheckBoundaries(p, 18, lw).Zone == ZoneHardBarrier {
						t.Fatalf("line %d leg %d -> %d crosses a hard boundary at %v", line+1, from, to, p)
					}
				}
			}
			if turns > 2 {
				t.Errorf("line %d has %d U-turns in a complete circuit: %v", line+1, turns, route.StopIDs)
			}
			if approachTurns > 2 {
				t.Errorf("line %d has %d U-turns using the front berths", line+1, approachTurns)
			}
		}
	}
	for _, theme := range BuiltinMapThemes() {
		for _, large := range []bool{false, true} {
			for _, brt := range []bool{false, true} {
				t.Run(fmt.Sprintf("map%d/large%v/brt%v", theme, large, brt), func(t *testing.T) {
					w := &World{LargeGrid: large, CenterBRT: brt}
					w.BuildMap(theme, lw)
					check(t, w)
				})
			}
		}
	}
	t.Run("starter custom map", func(t *testing.T) {
		w := &World{CustomMap: GetDefaultCustomMapData()}
		w.BuildMap(MapThemeCustom, lw)
		check(t, w)
	})
}

func TestGeneratedBlockNamesAreVariedAndStable(t *testing.T) {
	for _, theme := range BuiltinMapThemes() {
		w := NewWorld(52, theme)
		large := &World{LargeGrid: true}
		large.BuildMap(theme, 52)
		brt := &World{LargeGrid: true, CenterBRT: true}
		brt.BuildMap(theme, 52)
		seen := map[string]bool{}
		directions := 0
		for _, s := range w.Stops[48:] {
			if seen[s.Name] || !strings.Contains(s.Name, " (") {
				t.Fatalf("map %d: duplicate or untranslated name %q", theme, s.Name)
			}
			seen[s.Name] = true
			if strings.HasPrefix(s.Name, "街区") {
				directions++
			}
			if s.Name != large.Stops[s.ID].Name {
				t.Fatalf("map %d stop %d: name changed across large grid variant", theme, s.ID)
			}
			if !brt.Stops[s.ID].CenterBRT && s.Name != brt.Stops[s.ID].Name {
				t.Fatalf("map %d stop %d: non-BRT name changed in BRT variant", theme, s.ID)
			}
		}
		if directions == 0 || directions > len(seen)/4 {
			t.Fatalf("map %d: %d of %d labels are directions", theme, directions, len(seen))
		}
		s := w.Stops[48]
		want := s.Name
		s.Name = directionalBlockStopName(s.ID, s.Heading)
		w.Stops[49].Name = "已命名 (Renamed)"
		w.RefreshGeneratedStopNames()
		if s.Name != want || w.Stops[49].Name != "已命名 (Renamed)" {
			t.Fatalf("map %d: name migration changed authored labels or missed an old label", theme)
		}
	}
}
