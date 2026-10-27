package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"reflect"
	"testing"
)

func TestSpatialTrafficMatchesFullPerception(t *testing.T) {
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeRiverValley} {
		for _, density := range []int{1, 4} {
			t.Run(fmt.Sprintf("map%d/density%d", theme, density), func(t *testing.T) {
				sim.SeedSimulation(907)
				w := sim.NewWorld(52, theme)
				a := NewTrafficManager(52)
				a.BuildForWorld(w, MaxTrafficCars, 52)
				b := &TrafficManager{disableSpatial: true, CarSpeedRange: a.CarSpeedRange}
				for _, car := range a.Cars {
					copy := *car
					b.Cars = append(b.Cars, &copy)
				}
				b.RestorePopulation(a.PopulationSnapshot())
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				st := w.Stops[0]
				player := sim.NewBus(st.Pos.X, st.Pos.Y, st.Heading)
				fleet := NewAIBusManager()
				fleet.BuildFleet(w, 8, 52, player)
				a.AIBusMgr = fleet
				b.AIBusMgr = fleet
				frames := 90
				if testing.Short() {
					frames = 15
				}
				for frame := 0; frame < frames; frame++ {
					tls.Update(1. / 60)
					sim.SeedSimulation(int64(1000 + frame))
					a.Update(1./60, player, tls, density, 52, w)
					sim.SeedSimulation(int64(1000 + frame))
					b.Update(1./60, player, tls, density, 52, w)
					if len(a.Cars) != len(b.Cars) {
						t.Fatalf("frame%d population differs", frame)
					}
					for i := range a.Cars {
						if !reflect.DeepEqual(a.Cars[i], b.Cars[i]) {
							t.Fatalf("frame%d car%d: indexed pos%v speed%.6f; full pos%v speed%.6f", frame, i, a.Cars[i].Pos, a.Cars[i].Speed, b.Cars[i].Pos, b.Cars[i].Speed)
						}
					}
				}
			})
		}
	}
}

func TestSpatialCollisionSeparationMatchesAllPairs(t *testing.T) {
	for seed := int64(0); seed < 12; seed++ {
		sim.SeedSimulation(seed)
		a := &TrafficManager{}
		b := &TrafficManager{disableSpatial: true}
		for i := 0; i < 300; i++ {
			c := &sim.TrafficCar{ID: i, Pos: sim.Vec2{X: float64(sim.RandomIntn(2000) - 1000), Y: float64(sim.RandomIntn(2000) - 1000)}, Length: 48, Width: 22, Heading: sim.RandomFloat64() * 6.28, Speed: sim.RandomFloat64() * 200}
			a.Cars = append(a.Cars, c)
			copy := *c
			b.Cars = append(b.Cars, &copy)
		}
		a.ResolveCarCollisions(4)
		b.ResolveCarCollisions(4)
		if !reflect.DeepEqual(a.Cars, b.Cars) {
			t.Fatalf("seed%d: spatial separation changed pair resolution", seed)
		}
	}
}
