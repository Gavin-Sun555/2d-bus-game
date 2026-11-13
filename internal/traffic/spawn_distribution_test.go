package traffic

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCivilianDispatchFavorsWideRoads(t *testing.T) {
	sim.SeedSimulation(1930)
	w := &sim.World{}
	for i, lanes := range []int{4, 6, 8} {
		y := float64(i * 1000)
		road := sim.RoadSegment{P1: sim.Vec2{X: -15000, Y: y}, P2: sim.Vec2{X: 15000, Y: y}, Lanes: lanes}
		if lanes == 8 {
			road.Type = sim.RoadBusBoulevard8Lane
		}
		w.Roads = append(w.Roads, road)
		w.Intersections = append(w.Intersections, sim.Vec2{X: -6000, Y: y}, sim.Vec2{X: 6000, Y: y})
	}

	const count = 600
	tm := &TrafficManager{}
	tm.BuildForWorld(w, count, 52)
	seen := make(map[sim.Vec2]bool)
	var initial [3]int
	for _, car := range tm.Cars {
		if seen[car.Pos] {
			t.Fatal("initial dispatch reused an occupied site")
		}
		seen[car.Pos] = true
		class := int(car.RoadCenter / 1000)
		initial[class]++
		if car.LaneIndex >= w.Roads[class].GeneralTrafficLanes() {
			t.Fatal("car spawned in the reserved bus lane")
		}
	}
	if len(tm.Cars) != count || initial[0] == 0 || initial[0] >= count/6 || initial[1] == 0 || initial[2] == 0 {
		t.Fatalf("unexpected initial distribution 4/6/8 = %v", initial)
	}

	tm.Cars = nil
	var replacement [3]int
	for i := 0; i < count; i++ {
		car := tm.spawnAtOtherIntersection(sim.Vec2{X: 6000, Y: 1000}, nil, 52, w)
		if car == nil {
			t.Fatal("no replacement on a clear map")
		}
		class := int(car.RoadCenter / 1000)
		replacement[class]++
		if car.LaneIndex >= w.Roads[class].GeneralTrafficLanes() {
			t.Fatal("replacement used the reserved bus lane")
		}
	}
	if replacement[0] == 0 || replacement[0] >= count/4 || replacement[1] == 0 || replacement[2] == 0 {
		t.Fatalf("unexpected replacement distribution 4/6/8 = %v", replacement)
	}
	t.Logf("initial 4/6/8=%v, replacements=%v", initial, replacement)
}
