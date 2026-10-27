package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"reflect"
	"testing"
)

func TestBusCarIndexMatchesSequentialCollisionPasses(t *testing.T) {
	rng := rand.New(rand.NewSource(20261025))
	for scenario := 0; scenario < 120; scenario++ {
		tm := &TrafficManager{}
		for i := 0; i < 128; i++ {
			tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: rng.Float64()*3000 - 1500, Y: rng.Float64()*1000 - 500}, Heading: rng.Float64() * 6.28, Length: 30 + rng.Float64()*60, Width: 18 + rng.Float64()*15, Speed: float64(rng.Intn(100))})
		}
		buses := []*sim.Bus{}
		for i := 0; i < 10; i++ {
			b := sim.NewBus(rng.Float64()*1200-600, rng.Float64()*500-250, rng.Float64()*6.28)
			b.SetModel(sim.BusModel(i % 3))
			b.Speed = float64(rng.Intn(100))
			buses = append(buses, b)
		}
		oldCars := copyRecoveryCars(tm.Cars)
		oldBuses := make([]*sim.Bus, len(buses))
		for i, b := range buses {
			v := *b
			oldBuses[i] = &v
		}
		// Keep both passes and the intervening bus/bus corrections, as in the game.
		for pass := 0; pass < 2; pass++ {
			batch := tm.BeginBusCarSeparations(tm.Cars)
			for i, b := range buses {
				batch.Resolve(b)
				for _, c := range oldCars {
					sim.ResolveBusCarSeparation(oldBuses[i], c)
				}
				for j := i + 1; j < len(buses); j++ {
					sim.ResolveBusBusSeparation(b, buses[j])
					sim.ResolveBusBusSeparation(oldBuses[i], oldBuses[j])
				}
			}
			for i, b := range buses {
				a, o := b.BodyParts(), oldBuses[i].BodyParts()
				if a != o || b.Speed != oldBuses[i].Speed {
					t.Fatalf("scenario%d pass%d bus%d diverged", scenario, pass, i)
				}
			}
			for i, c := range tm.Cars {
				if !reflect.DeepEqual(c, oldCars[i]) {
					t.Fatalf("scenario%d pass%d car%d diverged", scenario, pass, i)
				}
			}
		}
	}
}

func TestBusCarIndexRequeriesAfterDeepSeparation(t *testing.T) {
	// An early overlap pushes the bus into another cell, bringing a later slot
	// into range. Re-query instead of trusting the original candidate list.
	tm := &TrafficManager{}
	for i := 0; i < 100; i++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: 5000 + float64(i)*1000}, Length: 46, Width: 22})
	}
	tm.Cars[0].Pos = sim.Vec2{X: 505}
	tm.Cars[0].Length = 900
	tm.Cars[50].Pos = sim.Vec2{X: 1000}
	b := sim.NewBus(510, 0, 0)
	b.Speed = 20
	old := *b
	cars := copyRecoveryCars(tm.Cars)
	tm.BeginBusCarSeparations(tm.Cars).Resolve(b)
	for _, c := range cars {
		sim.ResolveBusCarSeparation(&old, c)
	}
	if b.Pos != old.Pos || b.Speed != old.Speed {
		t.Fatal("moving bus query changed sequential result")
	}
	for i, c := range tm.Cars {
		if !reflect.DeepEqual(c, cars[i]) {
			t.Fatalf("car%d diverged", i)
		}
	}
}

func TestBusCarIndexFallbackForPublicStateEdits(t *testing.T) {
	for _, kind := range []string{"duplicate", "nil", "nonfinite", "large"} {
		tm := &TrafficManager{}
		for i := 0; i < 80; i++ {
			tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: sim.Vec2{X: float64(i) * 200}, Length: 46, Width: 22})
		}
		switch kind {
		case "duplicate":
			tm.Cars[4] = tm.Cars[3]
		case "nil":
			tm.Cars[3] = nil
		case "nonfinite":
			tm.Cars[3].Pos.X = math.NaN()
		case "large":
			tm.Cars[3].Length = 1e10
		}
		if tm.BeginBusCarSeparations(tm.Cars).indexed {
			t.Fatal("abnormal state used index:", kind)
		}
	}
}
