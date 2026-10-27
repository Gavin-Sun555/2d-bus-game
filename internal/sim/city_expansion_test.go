package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestExpandedCitiesAndTransitNetwork(t *testing.T) {
	for _, theme := range BuiltinMapThemes() {
		t.Run(fmt.Sprint(theme), func(t *testing.T) {
			w := NewWorld(52, theme)
			if len(w.Stops) < 150 {
				t.Fatalf("stops=%d, want at least 150", len(w.Stops))
			}
			minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
			for _, r := range w.Roads {
				for _, p := range []Vec2{r.P1, r.P2} {
					minX = math.Min(minX, p.X)
					maxX = math.Max(maxX, p.X)
					minY = math.Min(minY, p.Y)
					maxY = math.Max(maxY, p.Y)
				}
			}
			if maxX-minX != 10400*MapGridScale || maxY-minY != 4800*MapGridScale {
				t.Fatalf("footprint=%v,%v", maxX-minX, maxY-minY)
			}
			for _, s := range w.Stops {
				if w.InsideJunction(s.Pos, 52, s.Length/2+100) {
					t.Errorf("stop %d too close to junction", s.ID)
				}
				if w.CheckBoundaries(s.Pos, 18, 52).Zone == ZoneHardBarrier {
					t.Errorf("stop %d inaccessible", s.ID)
				}
			}
			lines := DefaultTransitLines(w)
			if len(lines) != 6 {
				t.Fatal("need six editable templates")
			}
			seen := map[int]bool{}
			for i, r := range lines {
				if len(r.StopIDs) < 13 || len(r.StopIDs) > 17 || r.StopIDs[0] != r.StopIDs[len(r.StopIDs)-1] {
					t.Fatalf("line %d invalid %v", i, r.StopIDs)
				}
				for j, id := range r.StopIDs {
					if i < 4 {
						seen[id] = true
					}
					if j == 0 {
						continue
					}
					path := ComputeRoadPathBetweenStops(w, w.Stops[r.StopIDs[j-1]], w.Stops[id], 52)
					for _, p := range path {
						if w.CheckBoundaries(p, 18, 52).Zone == ZoneHardBarrier {
							t.Fatalf("line %d leg %d -> %d unsafe path at %v", i, r.StopIDs[j-1], id, p)
						}
					}
					if len(path) < 2 {
						t.Fatalf("line %d disconnected leg %d -> %d", i, r.StopIDs[j-1], id)
					}
				}
			}
			if len(seen) != 48 {
				t.Fatalf("first four lines cover %d stations", len(seen))
			}
			if theme == MapThemeRiverValley {
				for _, r := range w.Roads {
					if r.P1.Y < 0 && r.P2.Y > 0 && math.Abs(r.P1.X) != 1300*MapGridScale {
						t.Error("unbridged river crossing")
					}
				}
			}
		})
	}
}
