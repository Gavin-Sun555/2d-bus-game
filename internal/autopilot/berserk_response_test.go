package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBerserkDepartureAccelerationAndModeSwitch(t *testing.T) {
	const lw = 52.0
	w := busLaneTestWorld(lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	normal, fast := sim.NewBus(-1400, 3.5*lw, 0), sim.NewBus(-1400, 3.5*lw, 0)
	a, b := NewAutopilotDriver(), NewAutopilotDriver()
	b.BerserkMode = true
	step := func(ad *AutopilotDriver, bus *sim.Bus) {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, .1)
		bus.Update(.1, th, br, st, hb, doors)
	}
	step(a, normal)
	step(b, fast)
	if math.Abs(fast.Speed-1.2*normal.Speed) > 1e-8 {
		t.Fatalf("departure response normal%.2f berserk%.2f", normal.Speed, fast.Speed)
	}
	b.BerserkMode = false
	beforeNormal, beforeFast := normal.Speed, fast.Speed
	step(a, normal)
	step(b, fast)
	if math.Abs((normal.Speed-beforeNormal)-(fast.Speed-beforeFast)) > 1e-8 {
		t.Fatal("switching off berserk retained stronger engine response")
	}
}
