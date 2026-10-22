package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestLaneYieldApproachesReservedRearGap(t *testing.T) {
	bus := sim.NewBus(0, 26, 0)
	bus.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}, Heading: 0, Remaining: .3}
	for _, x := range []float64{-400, -110} {
		car := &sim.TrafficCar{Pos: sim.Vec2{X: x, Y: 78}, Heading: 0, Length: 48, Width: 22}
		f := carFrame{car: car, world: &sim.World{}, buses: []*sim.Bus{bus}, laneWidth: 52}
		speed := (&TrafficManager{}).busMergeYieldSpeed(f, 100)
		if x == -400 && speed < 90 {
			t.Fatalf("distant yielding car must close excess gap, target%.1f", speed)
		}
		if x == -110 && speed != 0 {
			t.Fatalf("near yielding car must hold safe gap, target%.1f", speed)
		}
		if car.YieldingToBus != bus {
			t.Fatal("missing merge priority")
		}
	}
}
