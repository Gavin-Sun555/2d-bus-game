package sim

import (
	"math"
	"math/rand"
	"testing"
)

// The memo must return exactly what the math package returns, for first and
// repeated lookups, including colliding slots and special values.
func TestUnitFromAngleMemoIsBitIdentical(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	angles := []float64{0, math.Copysign(0, -1), math.Pi, -math.Pi, math.Pi / 2, -math.Pi / 2, 1e-300, 1e300, math.Inf(1), math.Inf(-1), math.NaN()}
	for i := 0; i < 200000; i++ {
		angles = append(angles, (rng.Float64()-.5)*4*math.Pi, math.Float64frombits(rng.Uint64()))
	}
	same := func(a, b float64) bool {
		return math.Float64bits(a) == math.Float64bits(b) || (math.IsNaN(a) && math.IsNaN(b))
	}
	for pass := 0; pass < 2; pass++ {
		for _, a := range angles {
			s, c := math.Sincos(a)
			u := UnitFromAngle(a)
			ms, mc := Sincos(a)
			if !same(u.X, c) || !same(u.Y, s) || !same(ms, s) || !same(mc, c) || !same(Cos(a), math.Cos(a)) {
				t.Fatalf("angle %v (%#x): memo (%v,%v) math (%v,%v) cos %v/%v", a, math.Float64bits(a), u.X, u.Y, c, s, Cos(a), math.Cos(a))
			}
		}
	}
}
