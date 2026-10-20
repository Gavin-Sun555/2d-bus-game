package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

// Repeat the same live traffic frame so cars leaving or recovering cannot make
// later benchmark iterations artificially cheaper.
func BenchmarkTrafficFrame(b *testing.B) {
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeCoastalBay, sim.MapThemeRiverValley, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities, sim.MapThemeCustom} {
		for _, density := range []int{1, 4} {
			b.Run(fmt.Sprintf("map%d/cars%d", theme, sim.TrafficCarCount(density)), func(b *testing.B) {
				sim.SeedSimulation(5)
				w := sim.NewWorld(52, theme)
				tm := NewTrafficManager(52)
				tm.BuildForWorld(w, MaxTrafficCars, 52)
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				start := w.Stops[0]
				bus := sim.NewBus(start.Pos.X, start.Pos.Y, start.Heading)
				fleet := NewAIBusManager()
				fleet.BuildFleet(w, 8, 52, bus)
				tm.AIBusMgr = fleet
				active := tm.GetActiveCars(density)
				originals := append([]*sim.TrafficCar(nil), tm.Cars...)
				templates := make([]sim.TrafficCar, len(active))
				for i, c := range active {
					templates[i] = *c
				}
				b.ReportAllocs()
				b.ResetTimer()
				for b.Loop() {
					tm.Cars = originals
					for i := range templates {
						*tm.Cars[i] = templates[i]
					}
					tm.Update(1.0/60, bus, tls, density, 52, w)
				}
			})
		}
	}
}
