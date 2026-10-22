package sim

import (
	"fmt"
	"reflect"
	"testing"
)

func assertAllStopLoop(t *testing.T, route *Route, n int) {
	t.Helper()
	if n == 0 {
		if len(route.StopIDs) != 0 {
			t.Fatal("nonempty route without stations")
		}
		return
	}
	size := n + 1
	if n == 1 {
		size = 1
	}
	if len(route.StopIDs) != size || route.StopIDs[0] != 0 || route.StopIDs[len(route.StopIDs)-1] != 0 {
		t.Fatalf("not an all-stop loop: %v", route.StopIDs)
	}
	seen := make([]bool, n)
	for _, i := range route.StopIDs[:n] {
		if i < 0 || i >= n || seen[i] {
			t.Fatalf("invalid or repeated station: %v", route.StopIDs)
		}
		seen[i] = true
	}
}

func TestDefaultRoutesVisitEveryStation(t *testing.T) {
	for theme := MapThemeMetropolis; theme <= MapThemeCustom; theme++ {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("map%d/width%.0f", theme, lw), func(t *testing.T) {
				w := NewWorld(lw, theme)
				route := GetStandardRouteForMap(theme, w)
				assertAllStopLoop(t, route, len(w.Stops))
				if theme == MapThemeMetropolis && !reflect.DeepEqual(route.StopIDs, NewDefaultRoute().StopIDs) {
					t.Fatal("startup and map defaults differ")
				}
				for _, stop := range w.Stops {
					if w.InsideJunction(stop.Pos, lw, stop.Length/2+100) {
						t.Fatalf("station%d too close to junction", stop.ID)
					}
					if w.CheckBoundaries(stop.Pos, 18, lw).Zone == ZoneHardBarrier {
						t.Fatalf("station%d inaccessible", stop.ID)
					}
				}
				old := append([]int(nil), route.StopIDs...)
				route.StopIDs[1] = 0
				if !reflect.DeepEqual(GetStandardRouteForMap(theme, w).StopIDs, old) {
					t.Fatal("editing a route modified defaults")
				}
			})
		}
	}
}

func TestCustomDefaultRouteCoversAnyStationCount(t *testing.T) {
	for _, n := range []int{0, 1, 2, 3, 5, 8, 12, 13} {
		t.Run(fmt.Sprint(n), func(t *testing.T) {
			w := &World{Roads: []RoadSegment{{P1: Vec2{X: -10000}, P2: Vec2{X: 10000}, Lanes: 4}}}
			for i := 0; i < n; i++ {
				w.Stops = append(w.Stops, NewBusStop(100+i, "custom", Vec2{X: float64(i) * 500}, 0, 4, RGBA(0, 0, 0, 255), 52, 0))
			}
			assertAllStopLoop(t, GetStandardRouteForMap(MapThemeCustom, w), n)
		})
	}
	assertAllStopLoop(t, GetStandardRouteForMap(MapThemeMetropolis, nil), 0)
}
