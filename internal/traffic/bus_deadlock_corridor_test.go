package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestBayQueueDoesNotCountSeparatedSideCarAsDeadlock(t *testing.T) {
	b := sim.NewBus(-658.9875354447261, 109.61308349312996, -.20124096137751346)
	c := &sim.TrafficCar{Pos: sim.Vec2{X: -615.3217200652875, Y: 37.38736460791132}, Heading: .33171960933253475, Length: 46, Width: 22}
	if busCarDeadlockCorridor(b, c) {
		t.Fatal("clear side car triggered a bus reset")
	}
	c.Pos = b.Pos
	if !busCarDeadlockCorridor(b, c) {
		t.Fatal("actual overlapping car was ignored")
	}
}
