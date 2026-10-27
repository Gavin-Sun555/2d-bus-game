package sim

import (
	"encoding/json"
	"fmt"
	"math/rand/v2"
	"testing"
)

func TestPreparedWorldGeometryMatchesOriginalQueries(t *testing.T) {
	for _, theme := range []MapTheme{MapThemeMetropolis, MapThemeCoastalBay, MapThemeRiverValley, MapThemeEmeraldLake, MapThemeTwinCities, MapThemeCustom} {
		for _, width := range []float64{20, 42, 52, 78, 100} {
			t.Run(fmt.Sprintf("map%d/width%.0f", theme, width), func(t *testing.T) {
				w := NewWorld(width, theme)
				raw, _ := json.Marshal(w)
				w.PrepareJunctionGeometry(width)
				cache := w.geometryCache
				after, _ := json.Marshal(w)
				if string(raw) != string(after) {
					t.Fatal("transient geometry changed save format")
				}
				rng := rand.New(rand.NewPCG(7, 19))
				for _, center := range w.Intersections {
					ew, ns := w.uncachedIntersectionRoadDimensions(center, width)
					if cache.entries[center].geometry != NewJunctionGeometry(ew, ns, width) {
						t.Fatal("cached road extent differs")
					}
					for i := 0; i < 10; i++ {
						p := center.Add(Vec2{X: rng.Float64()*2200 - 1100, Y: rng.Float64()*2200 - 1100})
						for _, margin := range []float64{-8, 0, 48, 500} {
							w.geometryCache = nil
							want := w.InsideJunction(p, width, margin)
							w.geometryCache = cache
							if w.InsideJunction(p, width, margin) != want {
								t.Fatalf("junction membership differs at %v margin%.0f", p, margin)
							}
						}
					}
				}
			})
		}
	}
}

func TestWorldGeometryInvalidatesEditsAndRebuild(t *testing.T) {
	w := NewWorld(52)
	w.PrepareJunctionGeometry(52)
	old := w.geometryCache
	w.PrepareJunctionGeometry(52)
	if w.geometryCache != old {
		t.Fatal("rebuilt unchanged geometry")
	}
	w.Roads[0].Lanes = 12
	w.PrepareJunctionGeometry(52)
	if w.geometryCache == old || w.GeometryRevision() <= old.revision {
		t.Fatal("in-place road edit not detected")
	}
	old = w.geometryCache
	w.Roundabouts = append(w.Roundabouts, Roundabout{Center: w.Intersections[0], Radius: 200})
	w.PrepareJunctionGeometry(52)
	if !w.IsAtRoundabout(w.Intersections[0]) || w.geometryCache == old {
		t.Fatal("roundabout edit not detected")
	}
	revision := w.GeometryRevision()
	w.RebuildIntersections()
	w.PrepareJunctionGeometry(78)
	if w.GeometryRevision() <= revision || w.geometryCache.width != 78 {
		t.Fatal("map/width rebuild reused a revision")
	}
	var loaded World
	raw, _ := json.Marshal(w)
	if err := json.Unmarshal(raw, &loaded); err != nil {
		t.Fatal(err)
	}
	if loaded.geometryCache != nil {
		t.Fatal("loaded stale cache")
	}
	loaded.PrepareJunctionGeometry(78)
	if loaded.JunctionAt(loaded.Intersections[0], 78) != w.JunctionAt(w.Intersections[0], 78) {
		t.Fatal("load did not rebuild geometry")
	}
}
