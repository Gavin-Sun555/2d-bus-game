package traffic

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestEveryBuiltinBRTCivilianSpawnUsesGeneralLanes(t *testing.T) {
	for _, theme := range sim.BuiltinMapThemes() {
		for _, large := range []bool{false, true} {
			t.Run(fmt.Sprintf("map%d/large%v", theme, large), func(t *testing.T) {
				w := &sim.World{CenterBRT: true, LargeGrid: large}
				w.BuildMap(theme, 52)
				tm := &TrafficManager{}
				tm.BuildForWorld(w, MaxTrafficCars, 52)
				seen := 0
				for _, c := range tm.Cars {
					r, ok := w.RoadAt(c.RoadCenter, c.IsEW, c.Pos)
					if ok && r.HasMedianBusLane() {
						seen++
						if c.LaneIndex < 1 || c.LaneIndex > 3 {
							t.Fatalf("civilian spawned in reserved lane %d", c.LaneIndex)
						}
					}
				}
				if seen == 0 {
					t.Fatal("no BRT corridor traffic exercised")
				}
			})
		}
	}
}
