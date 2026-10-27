package sim

import (
	"encoding/json"
	"math"
	"math/rand"
	"reflect"
	"testing"
)

func assertPosePrediction(t *testing.T, b *Bus, p Vec2, h float64) bool {
	t.Helper()
	want, rejected := originalPosePrediction(b, p, h)
	got := b.AtPose(p, h)
	got.predictionCache, want.predictionCache = nil, nil
	for _, pair := range [][2]float64{{got.Pos.X, want.Pos.X}, {got.Pos.Y, want.Pos.Y}, {got.Heading, want.Heading}, {got.TrailerHeading, want.TrailerHeading}, {got.Speed, want.Speed}} {
		if math.Float64bits(pair[0]) != math.Float64bits(pair[1]) {
			t.Fatalf("prediction changed float bits: got%016x want%016x", math.Float64bits(pair[0]), math.Float64bits(pair[1]))
		}
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("cached pose differs source%v heading%g tail%g target%v/%g rejected%v", b.Pos, b.Heading, b.TrailerHeading, p, h, rejected)
	}
	return rejected
}

// Frozen pre-C solver: compare against complete trig evaluation at every
// original substep, rather than against the optimized integrator itself.
func originalPosePrediction(b *Bus, pos Vec2, heading float64) (Bus, bool) {
	copy := *b
	if !copy.TrailerInitialized {
		copy.ResetArticulation()
	}
	copy.Pos, copy.Heading = pos, heading
	rejected := originalAdvanceTrailer(&copy, b.Pos, b.Heading)
	return copy, rejected
}
func originalAdvanceTrailer(b *Bus, oldPos Vec2, oldHeading float64) bool {
	if !b.IsArticulated() {
		return false
	}
	if !b.TrailerInitialized {
		b.ResetArticulation()
		return false
	}
	oldTail := b.TrailerHeading
	delta := b.Pos.Sub(oldPos)
	turn := NormalizeAngle(b.Heading - oldHeading)
	steps := max(1, int(math.Ceil(math.Max(delta.Length(), math.Abs(turn)*240))))
	steps = min(steps, 4096)
	oldHitch := oldPos.Sub(Vec2{X: math.Cos(oldHeading), Y: math.Sin(oldHeading)}.Mul(b.Length / 2))
	for i := 1; i <= steps; i++ {
		t := float64(i) / float64(steps)
		h := oldHeading + turn*t
		hitch := oldPos.Add(delta.Mul(t)).Sub(Vec2{X: math.Cos(h), Y: math.Sin(h)}.Mul(b.Length / 2))
		d := hitch.Sub(oldHitch)
		r := Vec2{X: -math.Sin(b.TrailerHeading), Y: math.Cos(b.TrailerHeading)}
		b.TrailerHeading = NormalizeAngle(b.TrailerHeading + d.Dot(r)/ArticulatedAxleDistance)
		angle := NormalizeAngle(h - b.TrailerHeading)
		if math.Abs(angle) > MaxArticulationAngle {
			b.Pos, b.Heading, b.TrailerHeading = oldPos, oldHeading, oldTail
			b.Speed, b.IsBraking = 0, true
			return true
		}
		oldHitch = hitch
	}
	return false
}

func TestPosePredictionStraightDirectionsMatchOriginalIntegrator(t *testing.T) {
	rng := rand.New(rand.NewSource(3755))
	for _, heading := range []float64{0, math.Copysign(0, -1), math.Pi / 2, -math.Pi / 2, math.Pi, -math.Pi, .23, -2.71} {
		for i := 0; i < 100; i++ {
			b := NewBus(rng.Float64()*20000-10000, rng.Float64()*20000-10000, heading)
			b.SetModel(BusArticulated)
			b.TrailerHeading = heading
			if i%2 == 0 {
				b.TrailerHeading += .1
			}
			b.PreparePosePredictions()
			for _, distance := range []float64{0, .01, 1, 20, 90, 400, -100, 5000} {
				p := b.Pos.Add(Vec2{X: math.Cos(heading), Y: math.Sin(heading)}.Mul(distance))
				assertPosePrediction(t, b, p, heading)
				assertPosePrediction(t, b, p, heading)
			}
		}
	}
}

func TestPosePredictionCacheExactInputsAndEviction(t *testing.T) {
	rng := rand.New(rand.NewSource(324100))
	b := NewBus(10, -20, .3)
	b.SetModel(BusArticulated)
	b.PreparePosePredictions()
	raw, _ := json.Marshal(b)
	rejectedCount := 0
	for i := 0; i < 3000; i++ {
		b.Pos = Vec2{rng.Float64()*1000 - 500, rng.Float64()*1000 - 500}
		b.Heading = rng.Float64()*2*math.Pi - math.Pi
		b.TrailerHeading = b.Heading + rng.Float64() - .5
		b.TrailerInitialized = i%9 != 0
		b.Length = 60 + rng.Float64()*80
		b.Speed = float64(i%200) - 80
		b.IsBraking = i%2 == 0
		p := b.Pos.Add(Vec2{rng.Float64()*800 - 400, rng.Float64()*800 - 400})
		h := b.Heading + rng.Float64()*3 - 1.5
		if assertPosePrediction(t, b, p, h) {
			rejectedCount++
		}
		hits, _ := b.PosePredictionStats()
		assertPosePrediction(t, b, p, h)
		nextHits, _ := b.PosePredictionStats()
		if nextHits != hits+1 {
			t.Fatal("identical geometry did not hit")
		}
		// A cache hit must preserve new speed, braking, dimensions, controls
		// and passenger state; only rejected geometry forces a stop.
		b.Speed += 31
		b.IsBraking = !b.IsBraking
		b.Width += .01
		b.SteeringAngle += .001
		b.PassengerCount = i % 85
		b.DoorsOpen = i%3 == 0
		b.DoorProgress = .4
		assertPosePrediction(t, b, p, h)
	}
	if rejectedCount == 0 {
		t.Fatal("missing jackknife rejection coverage")
	}
	// Original JSON contains no transient data, and a new load starts cold.
	var loaded Bus
	if err := json.Unmarshal(raw, &loaded); err != nil {
		t.Fatal(err)
	}
	if loaded.predictionCache != nil {
		t.Fatal("cache leaked into save")
	}
	loaded.PreparePosePredictions()
	assertPosePrediction(t, &loaded, Vec2{180, 40}, .7)
}

func TestPosePredictionCacheInvalidatesGeometryAndPreservesValueCopies(t *testing.T) {
	b := NewBus(20, -30, .1)
	b.SetModel(BusArticulated)
	b.PreparePosePredictions()
	p, h := Vec2{150, 5}, .4
	mutations := []func(){func() { b.Pos.X++ }, func() { b.Pos.Y++ }, func() { b.Heading += .1 }, func() { b.TrailerHeading += .2 }, func() { b.Length++ }, func() { b.TrailerInitialized = false }, func() { p.X++ }, func() { p.Y++ }, func() { h += .1 }}
	assertPosePrediction(t, b, p, h)
	for _, change := range mutations {
		_, misses := b.PosePredictionStats()
		change()
		assertPosePrediction(t, b, p, h)
		_, next := b.PosePredictionStats()
		if next != misses+1 {
			t.Fatal("geometry change reused old prediction")
		}
	}
	// Predictions made from predictions share storage, but have distinct keys.
	for i := 0; i < 40; i++ {
		copy := b.AtPose(p, h)
		assertPosePrediction(t, &copy, p.Add(Vec2{30, 15}), h+.05)
		assertPosePrediction(t, b, p, h)
	}
	for _, model := range []BusModel{BusStandard, BusDoubleDecker, BusArticulated} {
		b.SetModel(model)
		assertPosePrediction(t, b, p, h)
	}
	b.GeometryVersion = 0
	assertPosePrediction(t, b, p, h)
}

func TestPosePredictionCacheBitwiseZeroAndInitializedStop(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	b.PreparePosePredictions()
	for _, zero := range []float64{0, math.Copysign(0, -1)} {
		b.Pos.X, b.Heading, b.TrailerHeading = zero, zero, zero
		for _, initialized := range []bool{false, true} {
			b.TrailerInitialized = initialized
			for _, speed := range []float64{0, 50, -10} {
				b.Speed = speed
				assertPosePrediction(t, b, Vec2{zero, zero}, zero)
				assertPosePrediction(t, b, Vec2{zero, zero}, zero)
			}
		}
	}
}
