package traffic

import (
	"math/rand"
	"reflect"
	"slices"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Compare against a direct cell-membership scan, independent of both bucket
// representations. Exercise sequential moves, rebuilds, population changes,
// negative cells, directory growth and the sparse-map fallback.
func TestSpatialDirectoryMatchesCellScan(t *testing.T) {
	rng := rand.New(rand.NewSource(20261025))
	var index carSpatialIndex
	var dst []int
	for epoch := 0; epoch < 9; epoch++ {
		count := []int{864, 300, 1152, 0, 96, 864, 864, 1152, 300}[epoch]
		cars := make([]*sim.TrafficCar, count)
		spread := 12000.
		if epoch == 6 {
			spread = 1000000
		}
		for i := range cars {
			cars[i] = &sim.TrafficCar{Pos: sim.Vec2{X: (rng.Float64() - .5) * spread, Y: (rng.Float64() - .5) * spread}}
		}
		index.build(cars)
		for step := 0; step < 100; step++ {
			if count > 0 {
				i := rng.Intn(count)
				cars[i].Pos = cars[i].Pos.Add(sim.Vec2{X: (rng.Float64() - .5) * 20000, Y: (rng.Float64() - .5) * 20000})
				index.move(i, cars[i].Pos)
			}
			pos := sim.Vec2{X: (rng.Float64() - .5) * spread, Y: (rng.Float64() - .5) * spread}
			radius := []float64{0, 60, 512, 1800, 3200}[step%5]
			lo, hi := cellAt(pos.Sub(sim.Vec2{X: radius, Y: radius})), cellAt(pos.Add(sim.Vec2{X: radius, Y: radius}))
			want := []int{}
			for i, c := range cars {
				k := cellAt(c.Pos)
				if k.x >= lo.x && k.x <= hi.x && k.y >= lo.y && k.y <= hi.y {
					want = append(want, i)
				}
			}
			dst = index.query(pos, radius, dst)
			if !slices.Equal(dst, want) {
				t.Fatalf("epoch%d step%d query got%v want%v", epoch, step, dst, want)
			}
			dst = index.queryUnsorted(pos, radius, dst)
			slices.Sort(dst)
			if !slices.Equal(dst, want) {
				t.Fatalf("epoch%d step%d unsorted query differs", epoch, step)
			}
		}
	}
}

func TestLazyCarCandidatesKeepIndependentQueryBuffers(t *testing.T) {
	tm := &TrafficManager{}
	for i := 0; i < 96; i++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: float64(i * 60)}, TargetSpeed: 200})
	}
	tm.spatial.build(tm.Cars)
	tm.spatialActive = true
	f := carFrame{car: tm.Cars[0], activeCount: 96, localCarRadius: localCarQueryRadius(tm.Cars[0])}
	got := tm.carsForFrame(f)
	saved := append([]*sim.TrafficCar(nil), got...)
	tm.junctionCandidates = tm.spatial.queryUnsorted(sim.Vec2{X: 3000}, 500, tm.junctionCandidates)
	if !reflect.DeepEqual(got, saved) {
		t.Fatal("junction scratch overwrote perception")
	}
	tm.Cars[0].Speed = 0
	if !reflect.DeepEqual(tm.carsForFrame(f), saved) {
		t.Fatal("same-frame braking changed borrowed candidates")
	}
	f.localCarRadius = 0
	if len(tm.carsForFrame(f)) != 96 {
		t.Fatal("explicit full-population lookup was narrowed")
	}
	// Beginning the next car's call must refresh candidate slots and pointers.
	tm.localCarsReady = false
	f.car = tm.Cars[95]
	f.localCarRadius = localCarQueryRadius(f.car)
	if reflect.DeepEqual(tm.carsForFrame(f), saved) {
		t.Fatal("candidate list reused across different cars")
	}
}
