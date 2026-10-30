package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

// Compare the new conservative prefilter against the unchanged exact predicate.
func filteredStopContainsBus(stop *sim.BusStop, bus *sim.Bus) bool {
	return !stopOutsideBusRange(stop, bus, busStopRangeReady(bus)) && stop.ContainsBus(bus)
}

func TestStopRangeFilterMatchesOriginal(t *testing.T) {
	rng := rand.New(rand.NewSource(20261027))
	accepted := 0
	check := func(stop *sim.BusStop, b *sim.Bus) {
		t.Helper()
		got, want := filteredStopContainsBus(stop, b), stop.ContainsBus(b)
		if got != want {
			t.Fatalf("got%v want%v stop%+v bus%+v", got, want, stop, b)
		}
		if want {
			accepted++
		}
	}
	for i := 0; i < 30000; i++ {
		h := rng.Float64()*2*math.Pi - math.Pi
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		stop := &sim.BusStop{Pos: sim.Vec2{X: rng.Float64()*60000 - 30000, Y: rng.Float64()*60000 - 30000}, Heading: h, Length: 150 + rng.Float64()*600, Width: 44 + rng.Float64()*40}
		p := stop.Pos.Add(u.Mul(rng.Float64()*800 - 400)).Add(r.Mul(rng.Float64()*300 - 150))
		if i%5 == 0 {
			p = stop.FrontBerthPosition()
		}
		if i%7 == 0 {
			p = p.Add(sim.Vec2{X: 20000, Y: -20000})
		}
		b := sim.NewBus(p.X, p.Y, h+(rng.Float64()-.5)*.7)
		b.SetModel(sim.BusModel(i % int(sim.BusModelCount)))
		b.TrailerHeading = b.Heading + (rng.Float64()-.5)*1.2
		check(stop, b)
		if i%17 == 0 {
			stop.Pos.X += 90000
			check(stop, b)
			stop.Pos = b.Pos
			stop.Heading = b.Heading
			check(stop, b)
		}
	}
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2, .37} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		for model := sim.BusStandard; model < sim.BusModelCount; model++ {
			stop := &sim.BusStop{Pos: sim.Vec2{X: 1e9, Y: -1e9}, Heading: h, Length: 336, Width: 44}
			for _, x := range []float64{-200, -122, 0, 96, 122, 200} {
				for _, y := range []float64{-26, 0, 26} {
					for _, toward := range []float64{math.Inf(-1), math.Inf(1)} {
						p := stop.Pos.Add(u.Mul(math.Nextafter(x, toward))).Add(r.Mul(math.Nextafter(y, toward)))
						b := sim.NewBus(p.X, p.Y, h)
						b.SetModel(model)
						check(stop, b)
					}
				}
			}
		}
	}
	if accepted < 1000 {
		t.Fatalf("insufficient positive cases: %d", accepted)
	}
}

func TestStopRangeFilterPreservesUnusualGeometryFallback(t *testing.T) {
	values := []float64{-1, 0, 1e10, math.Inf(1), math.NaN()}
	for field := 0; field < 10; field++ {
		for _, value := range values {
			// The original NormalizeAngle loops forever for infinite angles.
			// Preserve that implementation; only compare terminating cases here.
			if (field == 2 || field == 5) && math.Abs(value) > 100 {
				continue
			}
			stop := &sim.BusStop{Pos: sim.Vec2{X: 5000, Y: -5000}, Heading: 0, Length: 336, Width: 44}
			b := sim.NewBus(0, 0, 0)
			b.SetModel(sim.BusArticulated)
			switch field {
			case 0:
				b.Length = value
			case 1:
				b.Width = value
			case 2:
				b.Heading = value
			case 3:
				stop.Length = value
			case 4:
				stop.Width = value
			case 5:
				stop.Heading = value
			case 6:
				b.Pos.X = value
			case 7:
				stop.Pos.Y = value
			case 8:
				b.TrailerHeading = value
			case 9:
				b.GeometryVersion = 0
			}
			if got, want := filteredStopContainsBus(stop, b), stop.ContainsBus(b); got != want {
				t.Fatalf("field%d value%g got%v want%v", field, value, got, want)
			}
		}
	}
}

func BenchmarkStopRangeFilter(b *testing.B) {
	for _, far := range []bool{false, true} {
		name := "near"
		if far {
			name = "far"
		}
		stop := &sim.BusStop{Heading: .37, Length: 336, Width: 44}
		p := stop.FrontBerthPosition()
		bus := sim.NewBus(p.X, p.Y, .37)
		bus.SetModel(sim.BusArticulated)
		if far {
			bus.Pos = bus.Pos.Add(sim.Vec2{X: 5000, Y: -5000})
		}
		for _, old := range []bool{true, false} {
			method := "filtered"
			if old {
				method = "original"
			}
			b.Run(name+"/"+method, func(b *testing.B) {
				b.ReportAllocs()
				for b.Loop() {
					if old {
						_ = stop.ContainsBus(bus)
					} else {
						_ = filteredStopContainsBus(stop, bus)
					}
				}
			})
		}
	}
}
