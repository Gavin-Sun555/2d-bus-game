package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestFleetRecoveryWaitsForFullCycleDeadline(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)
	route.CurrentIndex = 1
	pos := sim.Vec2{X: -400, Y: 26}
	ab := &AIBus{Bus: sim.NewBus(pos.X, pos.Y, 0), Route: route, Driver: autopilot.NewAutopilotDriver()}
	for i := 0; i < 111; i++ {
		ab.Bus.Pos = pos
		ab.Bus.Heading = 0
		ab.Bus.Speed = 0
		ab.updateDeadlock(.5, true, nil, w)
		if i < 110 && ab.Route.CurrentIndex == 0 {
			t.Fatalf("Fleet reset after only %.1fs", float64(i+1)*.5)
		}
	}
	if ab.Route.CurrentIndex != 0 {
		t.Fatalf("Fleet did not recover: clock=%.1f pos=%v", ab.DeadlockTimer, ab.Bus.Pos)
	}
}
