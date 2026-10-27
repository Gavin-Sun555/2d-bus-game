package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

func TestJunctionBroadPhaseRetainsEveryPriorityVehicle(t *testing.T) {
	rng := rand.New(rand.NewSource(62841))
	for _, lw := range []float64{20, 42, 52, 78} {
		for _, lanes := range [][4]int{{}, {2, 2, 8, 8}, {8, 6, 2, 4}, {8, 8, 8, 8}} {
			inter := &sim.TrafficIntersection{Center: sim.Vec2{X: 123, Y: -456}, Phase: sim.PhaseEWGreen, ApproachLanes: lanes}
			for i, n := range lanes {
				inter.ApproachHasRoad[i] = n > 0
			}
			own := &sim.TrafficCar{ID: 1, Pos: inter.Center.Add(sim.Vec2{X: -800, Y: 26}), Heading: 0, Length: 48, Width: 22, Speed: 200}
			f := carFrame{car: own, tls: &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}, laneWidth: lw}
			reach := junctionApproachReach(inter, lw)
			for n := 0; n < 15000; n++ {
				other := &sim.TrafficCar{ID: 2, Pos: inter.Center.Add(sim.Vec2{X: rng.Float64()*8000 - 4000, Y: rng.Float64()*8000 - 4000}), Heading: rng.Float64() * 2 * math.Pi, Speed: rng.Float64() * 1000, Length: 46, Width: 22}
				if otherHasJunctionPriority(f, inter, other, 1, 100) && !carCanReachJunction(other, inter.Center, reach) {
					t.Fatalf("priority car omitted width%v lanes%v car%+v", lw, lanes, other)
				}
			}
			// A distant committed turn is never discarded by the approach filter.
			far := &sim.TrafficCar{Pos: inter.Center.Add(sim.Vec2{X: 10000}), JunctionTurn: &sim.CarJunctionTurn{Center: inter.Center}}
			if !carCanReachJunction(far, inter.Center, reach) {
				t.Fatal("lost committed turn")
			}
		}
	}
}
