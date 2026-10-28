package sim

import (
	"math"
	"math/rand"
	"testing"
)

func TestGroupedCorridorsMatchOriginal(t *testing.T) {
	rng := rand.New(rand.NewSource(2831))
	for trial := 0; trial < 3000; trial++ {
		makeBodies := func(n int, shift float64) []OBB {
			out := make([]OBB, n)
			for i := range out {
				out[i] = OBB{Center: Vec2{X: float64(i)*16 + shift, Y: rng.Float64()*200 - 100}, Heading: rng.Float64()*math.Pi*2 - math.Pi, HalfLen: rng.Float64() * 100, HalfWid: rng.Float64() * 30}
			}
			return out
		}
		a, b := makeBodies(rng.Intn(100), 0), makeBodies(rng.Intn(100), rng.Float64()*1000-500)
		if trial%7 == 0 {
			for i := range b {
				b[i].Center.Y += 300
			}
		}
		if got, want := separatedBodies(a, b), referenceSeparatedBodies(a, b); got != want {
			t.Fatalf("trial%d separation%v want%v", trial, got, want)
		}
	}
}
func TestGroupedCorridorsFallbackAndTouching(t *testing.T) {
	base := make([]OBB, 40)
	for i := range base {
		base[i] = OBB{Center: Vec2{X: float64(i) * 16}, HalfLen: 24, HalfWid: 11}
	}
	for _, special := range []float64{math.NaN(), math.Inf(1), math.Inf(-1), -1, 1e10, math.Copysign(0, -1)} {
		for _, field := range []string{"x", "heading", "width"} {
			a := append([]OBB(nil), base...)
			b := append([]OBB(nil), base...)
			for i := range b {
				b[i].Center.Y = 22
			}
			switch field {
			case "x":
				b[24].Center.X = special
			case "heading":
				b[24].Heading = special
			case "width":
				b[24].HalfWid = special
			}
			if got, want := separatedBodies(a, b), referenceSeparatedBodies(a, b); got != want {
				t.Fatalf("%s %v changed separation", field, special)
			}
		}
	}
	for _, y := range []float64{22, math.Nextafter(22, 0), math.Nextafter(22, math.Inf(1))} {
		b := append([]OBB(nil), base...)
		for i := range b {
			b[i].Center.Y = y
		}
		if got, want := separatedBodies(base, b), referenceSeparatedBodies(base, b); got != want {
			t.Fatalf("touch %.17g changed", y)
		}
	}
	large := make([]OBB, 513)
	if separatedBodies(base, large) != referenceSeparatedBodies(base, large) {
		t.Fatal("large fallback changed")
	}
}
func BenchmarkCorridorGroupedSeparation(b *testing.B) {
	a := make([]OBB, 120)
	other := make([]OBB, 120)
	for i := range a {
		angle := float64(i) * math.Pi / 119
		a[i] = OBB{Center: Vec2{X: 400 * math.Cos(angle), Y: 400 * math.Sin(angle)}, Heading: angle + math.Pi/2, HalfLen: 24, HalfWid: 11}
		other[i] = a[i]
		other[i].Center = Vec2{X: 500 * math.Cos(angle), Y: 500 * math.Sin(angle)}
	}
	b.Run("original", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			referenceSeparatedBodies(a, other)
		}
	})
	b.Run("grouped", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			separatedBodies(a, other)
		}
	})
}
