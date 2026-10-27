package sim

import (
	"fmt"
	"testing"
)

func TestUnifiedPlatformsHaveClearFrontAndRearBerths(t *testing.T) {
	for theme := MapThemeMetropolis; theme <= MapThemeCustom; theme++ {
		w := NewWorld(52, theme)
		fail := 0
		for _, st := range w.Stops {
			for _, p := range []Vec2{st.FrontBerthPosition(), st.RearBerthPosition()} {
				b := NewBus(p.X, p.Y, st.Heading)
				for _, corner := range BodyCorners(b.BodyParts()[0]) {
					r := w.CheckBoundaries(corner, .5, 52)
					if r.Zone == ZoneHardBarrier {
						if fail < 4 {
							t.Log(fmt.Sprint("map", theme, " stop", st.ID, " ", p, " ", r.ObstacleName))
						}
						fail++
						break
					}
				}
			}
		}
		if fail > 0 {
			t.Errorf("map%d blocked berth count%d", theme, fail)
		}
	}
}
