package view

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestStreetLampPlacementAndCacheFollowMapRebuild(t *testing.T) {
	for _, theme := range sim.BuiltinMapThemes() {
		w := &sim.World{}
		w.BuildMap(theme, 52)
		lamps := streetLamps(w, 52)
		if len(lamps) == 0 {
			t.Fatal("city has no street lamps", theme)
		}
		first := lamps[0]
		if first.base.Distance(first.head) < 1 || first.pool.Distance(first.head) < 1 {
			t.Fatal("lamp has no arm or road light pool")
		}
		w.LargeGrid = true
		w.BuildMap(theme, 52)
		expanded := streetLamps(w, 52)
		if len(expanded) <= len(lamps) {
			t.Fatal("lamp cache retained the old small grid", theme, len(lamps), len(expanded))
		}
		if len(streetLamps(w, 78)) == 0 {
			t.Fatal("lane-width update lost street lamps")
		}
	}
}
