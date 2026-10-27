package traffic

import (
	"fmt"
	"math"
	"math/rand"
	"reflect"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLaneOrderFollowingMatchesSpatialOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(20261025))
	headings := []float64{0, math.Pi, -math.Pi, math.Pi / 2, -math.Pi / 2}
	queries, reduced := 0, 0
	for epoch := 0; epoch < 8; epoch++ {
		tm := &TrafficManager{}
		for i := 0; i < 864; i++ {
			c := &sim.TrafficCar{ID: 2000 - i, Pos: sim.Vec2{X: rng.Float64()*4000 - 2000, Y: float64(rng.Intn(9)-4)*52 + (rng.Float64()-.5)*12}, Heading: headings[rng.Intn(len(headings))], Length: 40 + rng.Float64()*30, Width: 18 + rng.Float64()*12, Speed: rng.Float64() * 400, TargetSpeed: 350}
			if i%7 == 0 {
				c.Heading = rng.Float64() * 2 * math.Pi
			}
			if i%13 == 0 {
				c.InRoundabout = true
			}
			if i%11 == 0 {
				c.JunctionTurn = &sim.CarJunctionTurn{LaneChange: true}
			}
			tm.Cars = append(tm.Cars, c)
		}
		tm.spatialActive = true
		tm.spatial.build(tm.Cars)
		tm.laneOrder.build(tm.Cars, 52)
		for step := 0; step < 240; step++ {
			// Already-updated cars may cross strips, move backwards, or change yaw.
			id := rng.Intn(len(tm.Cars))
			c := tm.Cars[id]
			c.Pos = c.Pos.Add(sim.Vec2{X: (rng.Float64() - .5) * 18, Y: (rng.Float64() - .5) * 18})
			c.Heading = rng.Float64() * 6.28
			tm.spatial.move(id, c.Pos)
			tm.laneOrder.observe(id, c)
			own := rng.Intn(len(tm.Cars))
			bus := tm.Cars[own]
			bus.Heading = headings[rng.Intn(len(headings))]
			bus.JunctionTurn = nil
			bus.InRoundabout = false
			bus.JunctionCorridor = nil
			if step%3 == 0 {
				bus.JunctionCorridor = &sim.JunctionCorridor{Remaining: 0}
			}
			f := carFrame{car: bus, index: own, activeCount: len(tm.Cars), laneWidth: 52, localCarRadius: localCarQueryRadius(bus)}
			original := *bus
			tm.disableLaneOrder = false
			tm.localCarsReady = false
			count := len(tm.followingCars(f))
			got := tm.carFollowingSpeed(f, 350)
			after := *bus
			*bus = original
			tm.disableLaneOrder = true
			tm.localCarsReady = false
			fullCount := len(tm.carsForFrame(f))
			want := tm.carFollowingSpeed(f, 350)
			if math.Float64bits(got) != math.Float64bits(want) || math.Float64bits(bus.Speed) != math.Float64bits(after.Speed) || bus.IsBraking != after.IsBraking {
				t.Fatalf("epoch%d step%d changed following: got%v want%v speed%v/%v", epoch, step, got, want, after.Speed, bus.Speed)
			}
			*bus = original
			queries++
			if count < fullCount {
				reduced++
			}
		}
	}
	if reduced < queries/2 {
		t.Fatalf("fast path insufficiently exercised: %d/%d", reduced, queries)
	}
	t.Logf("%d ordered comparisons, %d candidate reductions", queries, reduced)
}

func TestLaneOrderFullTrafficMatchesExistingQueries(t *testing.T) {
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities, sim.MapThemeCustom} {
		for _, density := range []int{1, 4} {
			t.Run(fmt.Sprintf("map%d/density%d", theme, density), func(t *testing.T) {
				sim.SeedSimulation(20261025)
				w := sim.NewWorld(52, theme)
				a := NewTrafficManager(52)
				a.BuildForWorld(w, MaxTrafficCars, 52)
				b := &TrafficManager{disableLaneOrder: true, CarSpeedRange: a.CarSpeedRange}
				for _, c := range a.Cars {
					copy := *c
					b.Cars = append(b.Cars, &copy)
				}
				b.RestorePopulation(a.PopulationSnapshot())
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				st := w.Stops[0]
				bus := sim.NewBus(st.Pos.X, st.Pos.Y, st.Heading)
				bus.SetModel(sim.BusArticulated)
				a.laneOrder.build(a.GetActiveCars(density), 52)
				if !a.laneOrder.ready {
					t.Fatal("real map population did not exercise the lane index")
				}
				for frame := 0; frame < 180; frame++ {
					tls.Update(1. / 60)
					sim.SeedSimulation(int64(9000 + frame))
					a.Update(1./60, bus, tls, density, 52, w)
					sim.SeedSimulation(int64(9000 + frame))
					b.Update(1./60, bus, tls, density, 52, w)
					if !reflect.DeepEqual(a.Cars, b.Cars) {
						t.Fatalf("frame%d changed live vehicle state", frame)
					}
					a.ResolveCarCollisions(density)
					b.ResolveCarCollisions(density)
					if !reflect.DeepEqual(a.Cars, b.Cars) {
						t.Fatalf("frame%d changed collision result", frame)
					}
				}
			})
		}
	}
}

func TestLaneOrderLargeMoveAndComplexStateFallback(t *testing.T) {
	tm := &TrafficManager{}
	for i := 0; i < 96; i++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: float64(i * 30), Y: 26}, Length: 48, Width: 22, TargetSpeed: 300})
	}
	tm.Cars[95].Pos.X = 12000
	tm.spatialActive = true
	tm.spatial.build(tm.Cars)
	tm.laneOrder.build(tm.Cars, 52)
	// A push/legacy relocation must not silently escape the frozen ordering.
	tm.Cars[95].Pos = sim.Vec2{X: 80, Y: 26}
	tm.spatial.move(95, tm.Cars[95].Pos)
	tm.laneOrder.observe(95, tm.Cars[95])
	f := carFrame{car: tm.Cars[0], activeCount: 96, localCarRadius: 1800, laneWidth: 52}
	if tm.laneOrder.movedX <= 4096 || !reflect.DeepEqual(tm.followingCars(f), tm.carsForFrame(f)) {
		t.Fatal("large move did not fall back")
	}
	got := tm.carFollowingSpeed(f, 300)
	tm.disableLaneOrder = true
	want := tm.carFollowingSpeed(f, 300)
	if got != want {
		t.Fatal("relocated leader missed")
	}
	tm.disableLaneOrder = false
	f.car.JunctionCorridor = &sim.JunctionCorridor{Remaining: 1}
	if !reflect.DeepEqual(tm.followingCars(f), tm.carsForFrame(f)) {
		t.Fatal("junction state did not fall back")
	}
	tm.Cars = tm.Cars[:32]
	tm.laneOrder.build(tm.Cars, 52)
	if len(tm.laneOrder.positions) != 32 {
		t.Fatal("population shrink retained slots")
	}
}

func TestLaneOrderRebuildHandlesReplacedAndReorderedSlots(t *testing.T) {
	rng := rand.New(rand.NewSource(1973))
	tm := &TrafficManager{}
	for round, n := range []int{864, 864, 300, 1152, 0, 864} {
		tm.Cars = nil
		for i := 0; i < n; i++ {
			tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: n - i, Pos: sim.Vec2{X: rng.Float64()*6000 - 3000, Y: float64(rng.Intn(12)-6) * 42}, Length: 48, Width: 22, TargetSpeed: 300})
		}
		tm.spatialActive = true
		tm.spatial.build(tm.Cars)
		tm.laneOrder.build(tm.Cars, 42)
		for _, order := range tm.laneOrder.order {
			if len(order) != n {
				t.Fatalf("round%d missing slots", round)
			}
			seen := make([]bool, n)
			for _, id := range order {
				if id < 0 || id >= n || seen[id] {
					t.Fatalf("round%d invalid order", round)
				}
				seen[id] = true
			}
		}
		if n == 0 {
			continue
		}
		for i := 0; i < 40; i++ {
			c := tm.Cars[i]
			c.Speed = 120
			f := carFrame{car: c, activeCount: n, localCarRadius: 1800, laneWidth: 42}
			tm.disableLaneOrder = false
			tm.localCarsReady = false
			got := tm.carFollowingSpeed(f, 300)
			speed, braking := c.Speed, c.IsBraking
			c.Speed = 120
			c.IsBraking = false
			tm.disableLaneOrder = true
			tm.localCarsReady = false
			want := tm.carFollowingSpeed(f, 300)
			if got != want || c.Speed != speed || c.IsBraking != braking {
				t.Fatalf("round%d slot%d changed result", round, i)
			}
		}
	}
}
