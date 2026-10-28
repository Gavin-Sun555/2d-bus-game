package sim

import (
	"math"
	"math/rand"
	"testing"
)

func assertCollisionSame(t *testing.T, a, b OBB) {
	t.Helper()
	wh, wn, wd := referenceCheckOBBCollision(a, b)
	gh, gn, gd := CheckOBBCollision(a, b)
	if gh != wh || !sameCollisionBits(gn.X, wn.X) || !sameCollisionBits(gn.Y, wn.Y) || !sameCollisionBits(gd, wd) {
		t.Fatalf("collision changed: %v %v %.17g want %v %v %.17g", gh, gn, gd, wh, wn, wd)
	}
}

func sameCollisionBits(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }
func TestCollisionGeometryMatchesOriginal(t *testing.T) {
	rng := rand.New(rand.NewSource(28104))
	for trial := 0; trial < 50000; trial++ {
		a := OBB{Center: Vec2{X: rng.Float64()*20000 - 10000, Y: rng.Float64()*20000 - 10000}, Heading: rng.Float64()*2*math.Pi - math.Pi, HalfLen: rng.Float64() * 200, HalfWid: rng.Float64() * 50}
		for j := 0; j < 4; j++ {
			b := OBB{Center: a.Center.Add(Vec2{X: rng.Float64()*800 - 400, Y: rng.Float64()*800 - 400}), Heading: rng.Float64()*2*math.Pi - math.Pi, HalfLen: rng.Float64() * 200, HalfWid: rng.Float64() * 50}
			assertCollisionSame(t, a, b)
		}
	}
}
func TestCollisionGeometryExceptionalAndContactCases(t *testing.T) {
	for _, v := range []float64{0, math.Copysign(0, -1), math.SmallestNonzeroFloat64, -1, 1e-200, 1e150, math.MaxFloat64, math.NaN(), math.Inf(1), math.Inf(-1)} {
		for field := 0; field < 5; field++ {
			a := OBB{HalfLen: 48, HalfWid: 18}
			b := OBB{Center: Vec2{X: 70}, HalfLen: 24, HalfWid: 11}
			switch field {
			case 0:
				a.Heading = v
			case 1:
				b.Heading = v
			case 2:
				a.Center.X = v
			case 3:
				a.HalfLen = v
			case 4:
				b.HalfWid = v
			}
			assertCollisionSame(t, a, b)
		}
	}
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, x := range []float64{72, math.Nextafter(72, 0), math.Nextafter(72, math.Inf(1))} {
			a := OBB{Heading: h, HalfLen: 48, HalfWid: 18}
			b := OBB{Center: UnitFromAngle(h).Mul(x), Heading: h, HalfLen: 24, HalfWid: 11}
			assertCollisionSame(t, a, b)
		}
	}
}
func TestCollisionPerpendicularAxisBitEquivalence(t *testing.T) {
	rng := rand.New(rand.NewSource(111))
	for i := 0; i < 250000; i++ {
		h := math.Float64frombits(rng.Uint64())
		if math.IsNaN(h) || math.IsInf(h, 0) {
			continue
		}
		u := UnitFromAngle(h)
		if !sameCollisionBits(-u.Y, -math.Sin(h)) || !sameCollisionBits(u.X, math.Cos(h)) {
			t.Fatalf("axis changed for %.17g", h)
		}
	}
}

func BenchmarkCollisionAxes(b *testing.B) {
	a := OBB{Center: Vec2{X: 30, Y: 40}, Heading: .37, HalfLen: 48, HalfWid: 18}
	other := OBB{Center: Vec2{X: 60, Y: 50}, Heading: -.23, HalfLen: 48, HalfWid: 18}
	b.Run("original", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			referenceCheckOBBCollision(a, other)
		}
	})
	b.Run("reuse", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			CheckOBBCollision(a, other)
		}
	})
}
