package sim

import "testing"

func TestBusLaneChangeSignalKeepsSideWhileCountersteering(t *testing.T) {
	for _, side := range []int{-1, 1} {
		b := NewBus(0, 0, 0)
		direction := float64(side)
		b.Speed = 80
		b.SteeringAngle = direction * .2
		b.Heading = direction * .02
		b.updateTurnSignal(1./60, 0)
		if got := b.TurnSignalSide(); got != side {
			t.Fatalf("initial signal = %d, want %d", got, side)
		}
		b.Heading = direction * .16
		b.SteeringAngle = -direction * .16
		b.updateTurnSignal(.2, direction*.17)
		if got := b.TurnSignalSide(); got != side {
			t.Fatalf("countersteering changed side to %d", got)
		}
		b.updateTurnSignal(.25, b.Heading)
		if got := b.TurnSignalSide(); got != 0 {
			t.Fatalf("signal did not blink off: %d", got)
		}
		b.updateTurnSignal(.4, b.Heading)
		if got := b.TurnSignalSide(); got != side {
			t.Fatalf("blink restarted on wrong side: %d", got)
		}
		b.Heading, b.SteeringAngle = 0, 0
		b.updateTurnSignal(1./60, direction*.01)
		if got := b.TurnSignalSide(); got != 0 {
			t.Fatalf("signal did not cancel on alignment: %d", got)
		}
	}
}
