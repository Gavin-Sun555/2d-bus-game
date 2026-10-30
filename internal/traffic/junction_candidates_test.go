package traffic

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestJunctionCandidatesRetainApproachesExitsAndCommittedTurns(t *testing.T) {
	rng := rand.New(rand.NewSource(84241))
	for _, width := range []float64{20, 42, 52, 78, 100} {
		inter := &sim.TrafficIntersection{Center: sim.Vec2{X: 321, Y: -654}, ApproachLanes: [4]int{2, 4, 8, 6}, ApproachHasRoad: [4]bool{true, true, true, true}}
		tm := &TrafficManager{spatialActive: true}
		for i := 0; i < 864; i++ {
			car := &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: rng.Float64()*30000 - 15000, Y: rng.Float64()*30000 - 15000}, Heading: rng.Float64() * math.Pi * 2, Length: 30 + rng.Float64()*200, Width: 22, Speed: rng.Float64() * 700}
			if i%17 == 0 {
				car.Speed = 0
			}
			if i%41 == 0 {
				car.JunctionTurn = &sim.CarJunctionTurn{Center: inter.Center, LaneChange: i%2 == 0}
			}
			tm.Cars = append(tm.Cars, car)
		}
		tm.spatial.build(tm.Cars)
		tm.prepareJunctionCandidates(len(tm.Cars))
		f := carFrame{activeCount: len(tm.Cars)}
		reach := junctionApproachReach(inter, width)
		exitReach := 2400.0
		assertCovered := func() {
			t.Helper()
			radius := math.Max(reach+tm.junctionMaxHorizon, exitReach)
			radius += 1e-7 * (1 + radius)
			want := tm.spatial.queryUnsorted(inter.Center, radius, nil)
			want = append(want, tm.junctionCommitted...)
			slices.Sort(want)
			want = slices.Compact(want)
			ids, indexed := tm.junctionScanCandidates(f, inter, reach, exitReach)
			if !slices.Equal(ids, want) {
				t.Fatalf("candidate set/order changed: got%v want%v", ids, want)
			}
			if !indexed {
				t.Fatal("unexpected full scan")
			}
			if !slices.IsSorted(ids) || len(slices.Compact(slices.Clone(ids))) != len(ids) {
				t.Fatal("candidate order/duplicates changed priority")
			}
			for i, car := range tm.Cars {
				if carCanReachJunction(car, inter.Center, reach) || (math.Abs(car.Speed) <= 5 && car.Pos.Distance(inter.Center) <= exitReach) {
					if !slices.Contains(ids, i) {
						t.Fatalf("width%g omitted car%d pos%v speed%g turn%v", width, i, car.Pos, car.Speed, car.JunctionTurn != nil)
					}
				}
			}
		}
		assertCovered()
		// Sequential movement across grid cells, a new commitment, and a
		// growing speed horizon must be visible to the next vehicle.
		for i := 0; i < 80; i++ {
			car := tm.Cars[i]
			car.Pos = inter.Center.Add(sim.Vec2{X: rng.Float64()*6000 - 3000, Y: rng.Float64()*6000 - 3000})
			car.Speed = 800
			if i%5 == 0 {
				car.Pos.X += 14000
				car.JunctionTurn = &sim.CarJunctionTurn{Center: inter.Center}
			}
			tm.spatial.move(i, car.Pos)
			tm.observeJunctionCandidate(i, car)
			assertCovered()
		}
		// Population shrinks and indexes are rebuilt at the next batch.
		tm.Cars = tm.Cars[:100]
		tm.spatial.build(tm.Cars)
		tm.prepareJunctionCandidates(len(tm.Cars))
		f.activeCount = len(tm.Cars)
		assertCovered()
		for _, id := range tm.junctionCommitted {
			if id >= len(tm.Cars) {
				t.Fatal("stale population ID")
			}
		}
	}
}

func TestJunctionCandidatesFallbackForStandaloneAndExtremeStates(t *testing.T) {
	tm := &TrafficManager{Cars: []*sim.TrafficCar{{Length: 48, Speed: 100}}}
	f := carFrame{activeCount: 1}
	inter := &sim.TrafficIntersection{}
	if _, ok := tm.junctionScanCandidates(f, inter, 500, 2000); ok {
		t.Fatal("standalone used unprepared index")
	}
	tm.spatialActive = true
	tm.spatial.build(tm.Cars)
	tm.prepareJunctionCandidates(1)
	if _, ok := tm.junctionScanCandidates(f, inter, 500, 2000); !ok {
		t.Fatal("prepared index not used")
	}
	for _, speed := range []float64{100000, math.Inf(1), math.NaN()} {
		tm.Cars[0].Speed = speed
		tm.prepareJunctionCandidates(1)
		if _, ok := tm.junctionScanCandidates(f, inter, 500, 2000); ok {
			t.Fatal("unbounded grid query")
		}
	}
}
