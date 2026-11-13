package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestStationDispatchBalancesSharedStopsAndAvoidsJunctions(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	r := sim.GetStandardRouteForMap(w.Theme, w)
	var occupied []*sim.Bus
	counts := map[int]int{}
	for slot := 0; slot < 8; slot++ {
		bus := sim.NewBus(0, 0, 0)
		index, ok := PlaceInitialBusAtStation(bus, r, w, 52, occupied, nil, 0)
		if !ok {
			t.Fatal("safe station dispatch failed")
		}
		id := r.StopIDs[index]
		counts[id]++
		if !SafeInitialBusPosition(bus, w, 52) {
			t.Fatal("spawned bus in junction, ring or shoulder")
		}
		occupied = append(occupied, bus)
	}
	minCount, maxCount := 100, 0
	seen := map[int]bool{}
	for _, id := range r.StopIDs {
		if seen[id] {
			continue
		}
		seen[id] = true
		minCount = min(minCount, counts[id])
		maxCount = max(maxCount, counts[id])
	}
	if maxCount-minCount > 1 {
		t.Fatalf("unbalanced station occupancy: %v", counts)
	}
}

func TestInitialPlacementRejectsStationInJunctionOrRing(t *testing.T) {
	for _, ring := range []bool{false, true} {
		w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 4}}}
		w.Stops = []*sim.BusStop{sim.NewBusStop(0, "unsafe", sim.Vec2{}, 0, 4, sim.RGBA(0, 0, 0, 255), 52), sim.NewBusStop(1, "safe", sim.Vec2{X: 3000}, 0, 4, sim.RGBA(0, 0, 0, 255), 52)}
		if ring {
			w.Roundabouts = []sim.Roundabout{{Radius: 240, InnerRadius: 120}}
		} else {
			w.Intersections = []sim.Vec2{{}}
		}
		route := &sim.Route{StopIDs: []int{0, 1, 0}}
		bus := sim.NewBus(0, 0, 0)
		index, ok := PlaceInitialBusAtStation(bus, route, w, 52, nil, nil, 0)
		if !ok || index != 1 || !SafeInitialBusPosition(bus, w, 52) {
			t.Fatalf("ring%v unsafe station selected: index%d position%v", ring, index, bus.Pos)
		}
	}
}

func TestInitialSpawnStaysOffLevelRailCrossingApproach(t *testing.T) {
	w := sim.NewWorld(78, sim.MapTheme(2))
	var rail *sim.Railway
	var crossing sim.RailCrossing
	for _, r := range w.Railways() {
		for _, c := range r.Crossings {
			if c.Level {
				rail, crossing = r, c
				break
			}
		}
		if rail != nil {
			break
		}
	}
	if rail == nil {
		t.Skip("map has no level crossing")
	}
	bus := sim.NewBus(crossing.Center.X, crossing.Center.Y-rail.HalfWidth-150, 1.5707963267948966)
	bus.SetModel(sim.BusArticulated)
	bus.ResetArticulation()
	if safeInitialSpawn(bus, w, 78, nil, nil) {
		t.Fatalf("spawned just short of a level crossing at %v", bus.Pos)
	}
	bus.Pos.Y = crossing.Center.Y
	if safeInitialSpawn(bus, w, 78, nil, nil) {
		t.Fatal("spawned on the tracks")
	}
}
