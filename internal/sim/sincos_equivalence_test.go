package sim

import (
	"math"
	"math/rand"
	"testing"
)

func TestSincosMatchesSeparateCalls(t *testing.T) {
	check := func(h float64) {
		s, c := math.Sincos(h)
		if math.Float64bits(s) != math.Float64bits(math.Sin(h)) || math.Float64bits(c) != math.Float64bits(math.Cos(h)) {
			t.Fatalf("different result at %g", h)
		}
	}
	for _, h := range []float64{0, math.Copysign(0, -1), math.Pi, -math.Pi, math.Pi / 2, 1e9, -1e9, math.MaxFloat64, math.SmallestNonzeroFloat64} {
		check(h)
	}
	rng := rand.New(rand.NewSource(20261025))
	for i := 0; i < 2000000; i++ {
		check((rng.Float64() - .5) * 2e6)
	}
	for i := 0; i < 100000; i++ {
		h := math.Float64frombits(rng.Uint64())
		if !math.IsNaN(h) && !math.IsInf(h, 0) {
			check(h)
		}
	}
}
