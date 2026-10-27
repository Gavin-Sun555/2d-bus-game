package sim

import (
	"math"
	"math/rand"
	"reflect"
	"testing"
)

func sameRoundaboutPathBits(a, b [34]OBB) bool {
	for i, x := range a {
		y := b[i]
		for j, v := range []float64{x.Center.X, x.Center.Y, x.Heading, x.HalfLen, x.HalfWid} {
			w := []float64{y.Center.X, y.Center.Y, y.Heading, y.HalfLen, y.HalfWid}[j]
			if math.Float64bits(v) != math.Float64bits(w) {
				return false
			}
		}
	}
	return true
}
func TestRoundaboutEntryGeometryMatchesOriginal(t *testing.T) {
	rng := rand.New(rand.NewSource(20261027))
	var geometry RoundaboutEntryGeometry
	valid, cleared := 0, 0
	for epoch := 0; epoch < 48; epoch++ {
		lw := []float64{40, 52, 78}[epoch%3]
		rb := Roundabout{Center: Vec2{X: float64(epoch%5)*130 - 250, Y: float64(epoch%7)*70 - 200}, Radius: []float64{180, 240, 360}[epoch%3], InnerRadius: 80 + float64(epoch%4)*10}
		h := []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2, .731, math.Copysign(0, -1)}[epoch%6]
		f := Vec2{X: math.Cos(h), Y: math.Sin(h)}
		outer := (rb.InnerRadius + 16 + 3*rb.Radius) / 4
		for step := 0; step < 100; step++ {
			bus := step%2 == 0
			width := []float64{11, 18, 27, 45}[step%4]
			inner := (3*(rb.InnerRadius+16) + rb.Radius) / 4
			if bus {
				inner = math.Max(outer-lw, rb.InnerRadius+width+24)
			}
			lane := step % 2
			radius := inner
			if lane == 1 {
				radius = outer
			}
			curve := RoundaboutEntryJoin(rb, f, (float64(lane)+.5)*lw, radius, lw)
			p, tangent := RoundaboutEntryPoint(curve, rng.Float64())
			if step%5 == 0 {
				p = curve.position(0).Add(f.Mul(-rng.Float64() * 240))
			}
			if step%7 == 0 {
				p = p.Add(Vec2{X: 12, Y: -12})
			}
			body := OBB{Center: p, Heading: math.Atan2(tangent.Y, tangent.X) + (rng.Float64()-.5)*.1, HalfLen: 20 + rng.Float64()*100, HalfWid: width}
			gotLane, gotCurve, gotOK := geometry.corridor(rb, f, lw, body, bus)
			wantLane, wantCurve, wantOK := RoundaboutEntryCorridor(rb, f, lw, body, bus)
			if gotLane != wantLane || gotOK != wantOK || !reflect.DeepEqual(gotCurve, wantCurve) {
				t.Fatalf("epoch%d step%d corridor changed", epoch, step)
			}
			if gotOK {
				valid++
				if !sameRoundaboutPathBits(geometry.path(gotCurve, gotLane, body, bus), roundaboutSeparationPath(wantCurve, body)) {
					t.Fatal("sampled body bits changed")
				}
			}
			a := PrepareRoundaboutEntrySeparation(rb, f, lw, body, bus, &geometry)
			b := PrepareRoundaboutEntrySeparation(rb, f, lw, body, bus)
			if a.valid != b.valid || a.lane != b.lane || !sameRoundaboutPathBits(a.path, b.path) {
				t.Fatal("own sweep changed")
			}
			for j := 0; j < 6; j++ {
				ob := OBB{Center: rb.Center.Sub(f.Mul(310)).Add(Vec2{X: -f.Y, Y: f.X}.Mul((float64(j%2) + .5) * lw)), Heading: h, HalfLen: 24, HalfWid: 11}
				if j > 1 {
					ob.Center = ob.Center.Add(Vec2{X: rng.Float64()*600 - 300, Y: rng.Float64()*600 - 300})
					ob.Heading += rng.Float64() * 6.28
				}
				otherBus := j%3 == 0
				// Interleave a different map/width using the same cache. Existing scans
				// must rebuild their exact geometry rather than borrow stale sample arrays.
				geometry.corridor(Roundabout{Radius: 220, InnerRadius: 100}, Vec2{Y: 1}, 42, ob, false)
				got, want := a.Clears(ob, otherBus), b.Clears(ob, otherBus)
				if got != want {
					t.Fatalf("epoch%d step%d obstacle%d clearance changed", epoch, step, j)
				}
				if got {
					cleared++
				}
			}
		}
	}
	if valid < 1000 || cleared < 100 {
		t.Fatalf("insufficient valid sweeps: %d clear%d", valid, cleared)
	}
	t.Logf("4800 classifications, %d valid exact sweeps, 28800 clearance comparisons (%d clear)", valid, cleared)
}

func TestRoundaboutEntryGeometrySignedZeroKey(t *testing.T) {
	var cache RoundaboutEntryGeometry
	rb := Roundabout{Radius: 240, InnerRadius: 120}
	c := RoundaboutEntryJoin(rb, Vec2{X: 1}, 26, 150, 52)
	c.StartShift.X = 0
	cache.sample(c, 0, false)
	c.StartShift.X = math.Copysign(0, -1)
	got := cache.sample(c, 0, false)
	if got.key != roundaboutJoinBits(c) || got.key[10] != math.Float64bits(c.StartShift.X) {
		t.Fatal("signed zero key was collapsed")
	}
	for i := 0; i <= 32; i++ {
		p, tan := RoundaboutEntryPoint(c, float64(i)/32)
		if math.Float64bits(got.points[i].X) != math.Float64bits(p.X) || got.tangents[i] != tan {
			t.Fatal("sample changed")
		}
	}
}

func BenchmarkRoundaboutEntrySeparationGeometry(b *testing.B) {
	rb := Roundabout{Radius: 240, InnerRadius: 120}
	f := Vec2{X: 1}
	own := OBB{Center: Vec2{X: -310, Y: 26}, HalfLen: 48, HalfWid: 18}
	obstacles := make([]OBB, 864)
	for i := range obstacles {
		obstacles[i] = OBB{Center: Vec2{X: float64(i%36)*110 - 1900, Y: float64(i/36)*90 - 1000}, HalfLen: 24, HalfWid: 11}
	}
	obstacles[0] = OBB{Center: Vec2{X: -310, Y: 78}, HalfLen: 24, HalfWid: 11}
	for _, cached := range []bool{false, true} {
		name := "original"
		if cached {
			name = "cached"
		}
		b.Run(name, func(b *testing.B) {
			var g RoundaboutEntryGeometry
			var cache *RoundaboutEntryGeometry
			if cached {
				cache = &g
			}
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				test := PrepareRoundaboutEntrySeparation(rb, f, 52, own, true, cache)
				for _, ob := range obstacles {
					test.Clears(ob, false)
				}
			}
		})
	}
}

func TestRoundaboutEntryGeometryDecisionBoundaries(t *testing.T) {
	var cache RoundaboutEntryGeometry
	rb := Roundabout{Radius: 240, InnerRadius: 120}
	forward := Vec2{X: 1}
	curve := RoundaboutEntryJoin(rb, forward, 26, 166, 52)
	for _, u := range []float64{0, .25, .5, .75, 1} {
		point, tangent := RoundaboutEntryPoint(curve, u)
		h := math.Atan2(tangent.Y, tangent.X)
		normal := Vec2{X: -math.Sin(h), Y: math.Cos(h)}
		for _, distance := range []float64{math.Nextafter(12, 0), 12, math.Nextafter(12, math.Inf(1)), -12, 0} {
			for _, angle := range []float64{0, math.Acos(.93), math.Acos(.93) - 1e-12, math.Acos(.93) + 1e-12} {
				body := OBB{Center: point.Add(normal.Mul(distance)), Heading: h + angle, HalfLen: 48, HalfWid: 18}
				a, c, ok := cache.corridor(rb, forward, 52, body, true)
				b, d, want := RoundaboutEntryCorridor(rb, forward, 52, body, true)
				if a != b || c != d || ok != want {
					t.Fatal("classification threshold changed")
				}
			}
		}
	}
}
