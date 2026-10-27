package sim

import (
	"math"
	"math/rand"
	"testing"
)

func TestRoadAxisBoundsMatchesOriginalScan(t *testing.T) {
	rng := rand.New(rand.NewSource(2610))
	check := func(w *World) {
		t.Helper()
		for _, ew := range []bool{false, true} {
			lo, hi := math.Inf(1), math.Inf(-1)
			for _, r := range w.Roads {
				a, b := r.P1.Y, r.P2.Y
				if ew {
					a, b = r.P1.X, r.P2.X
				}
				lo, hi = math.Min(lo, math.Min(a, b)), math.Max(hi, math.Max(a, b))
			}
			a, b := w.RoadAxisBounds(ew)
			if math.Float64bits(a) != math.Float64bits(lo) || math.Float64bits(b) != math.Float64bits(hi) {
				t.Fatalf("ew=%v got %v,%v want %v,%v", ew, a, b, lo, hi)
			}
		}
	}
	for n := 0; n < 100; n++ {
		w := &World{}
		for i := 0; i < n; i++ {
			w.Roads = append(w.Roads, RoadSegment{P1: Vec2{rng.NormFloat64() * 1e4, rng.NormFloat64() * 1e4}, P2: Vec2{rng.NormFloat64() * 1e4, rng.NormFloat64() * 1e4}})
		}
		check(w)
		w.BeginBoundaryQueries(52)
		if w.boundaryCache == nil {
			t.Fatal("finite fixture did not exercise cache")
		}
		check(w)
		w.BeginBoundaryQueries(52)
		check(w)
		w.EndBoundaryQueries()
		w.EndBoundaryQueries()
		if n > 0 {
			w.Roads[0].P1 = Vec2{-1e7, 1e7}
		}
		check(w)
		w.BeginBoundaryQueries(52)
		check(w)
		w.EndBoundaryQueries()
	}
	for _, v := range []float64{0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN()} {
		w := &World{Roads: []RoadSegment{{P1: Vec2{v, v}, P2: Vec2{v, v}}}}
		check(w)
		w.BeginBoundaryQueries(52)
		check(w)
		w.EndBoundaryQueries()
	}
	w := &World{Roads: []RoadSegment{{P1: Vec2{}, P2: Vec2{X: 20}}}}
	w.BeginBoundaryQueries(52)
	check(w)
	w.EndBoundaryQueries()
	w.Roads[0].P1.X = math.Copysign(0, -1)
	w.BeginBoundaryQueries(52)
	check(w)
	w.EndBoundaryQueries()

}
