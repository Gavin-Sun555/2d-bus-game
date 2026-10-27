package sim

import (
	"math"
	"testing"
)

func TestDenseStopsOnBothSidesOfBlocks(t *testing.T) {
	for _, theme := range BuiltinMapThemes() {
		t.Run(theme.ShortName(), func(t *testing.T) {
			w := NewWorld(52, theme)
			blocks := w.EligibleBlockStops(52)
			covered := 0
			for _, block := range blocks {
				for side := 0; side < 2; side++ {
					h := NormalizeAngle(block.Heading + float64(side)*math.Pi)
					ew := math.Abs(math.Cos(h)) > .5
					found := false
					for _, s := range w.Stops {
						along, cross, coord := s.RoadCenter.Y, s.RoadCenter.X, block.Center.X
						if ew {
							along, cross, coord = s.RoadCenter.X, s.RoadCenter.Y, block.Center.Y
						}
						if math.Abs(cross-coord) < 1 && along > block.Start && along < block.End && math.Cos(s.Heading-h) > .99 {
							found = true
							break
						}
					}
					if found {
						covered++
					}
				}
			}
			t.Logf("%d stops, %d/%d block sides covered", len(w.Stops), covered, 2*len(blocks))
			if len(w.Stops) < 150 || float64(covered)/float64(len(blocks)*2) < .95 {
				t.Fatal("insufficient two-sided block coverage")
			}
			for _, s := range w.Stops {
				if s.ID >= 48 && (w.InsideJunction(s.Pos, 52, s.Length/2+160) || w.CheckBoundaries(s.Pos, 18, 52).Zone == ZoneHardBarrier) {
					t.Fatalf("unsafe generated stop%d", s.ID)
				}
			}
			if len(GetStandardRouteForMap(theme, w).StopIDs) != 13 {
				t.Fatal("default line should remain twelve stops")
			}
		})
	}
}
