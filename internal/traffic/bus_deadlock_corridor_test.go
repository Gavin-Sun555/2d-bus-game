package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
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

func TestOpposingLaneBusDoesNotTriggerCarDeadlock(t *testing.T) {
	b := sim.NewBus(-7744.7, 4169.2, -1.568)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = -1.558
	c := &sim.TrafficCar{
		ID:        409,
		Pos:       sim.Vec2{X: -7826.0, Y: 4242.0},
		Heading:   math.Pi / 2,
		Length:    46,
		Width:     22,
		IsBraking: true,
	}
	tm := &TrafficManager{
		Cars:     []*sim.TrafficCar{c},
		AIBusMgr: &AIBusManager{Buses: []*AIBus{{ID: 93, Bus: b}}},
	}
	tm.ResolveDeadlocks(1.0, nil, 1, 52, &sim.World{}, nil)
	if c.DeadlockTimer != 0 {
		t.Fatalf("opposing-lane bus across centerline incremented car DeadlockTimer=%.2f", c.DeadlockTimer)
	}
}

func TestAdjacentLaneParallelBusDoesNotTriggerCarDeadlock(t *testing.T) {
	b := sim.NewBus(-39.85, -2726.61, 1.621)
	c1 := &sim.TrafficCar{ID: 3266, Pos: sim.Vec2{X: -77.6, Y: -2720.4}, Heading: 1.611, Length: 46, Width: 22}
	c2 := &sim.TrafficCar{ID: 3704, Pos: sim.Vec2{X: -78.0, Y: -2655.9}, Heading: 1.571, Length: 46, Width: 22}
	if carBusDeadlockCorridor(c1, b) {
		t.Fatal("adjacent-lane parallel bus triggered deadlock corridor for side car")
	}
	if carBusDeadlockCorridor(c2, b) {
		t.Fatal("adjacent-lane parallel bus triggered deadlock corridor for ahead car")
	}
}

func TestCurvedTurnExitQueueDoesNotTriggerCarDeadlock(t *testing.T) {
	c := &sim.TrafficCar{ID: 2818, Pos: sim.Vec2{X: -81.59, Y: -3509.03}, Heading: 1.276, Length: 46, Width: 22}
	leader := &sim.TrafficCar{ID: 2490, Pos: sim.Vec2{X: -78.0, Y: -3440.94}, Heading: 1.571, Length: 46, Width: 22}
	if carDeadlockCorridor(c, leader) {
		t.Fatal("curved turn exit car queued behind straight leader falsely triggered deadlock corridor")
	}
}

func TestCarQueuedBehindStoppedBusAheadDoesNotTriggerBusDeadlock(t *testing.T) {
	c := &sim.TrafficCar{ID: 2940, Pos: sim.Vec2{X: -2038.2, Y: -3574}, Heading: 0, Length: 46, Width: 22}
	bus := sim.NewBus(-1941.08, -3574, 0)
	if carBusDeadlockCorridor(c, bus) {
		t.Fatal("car queued directly behind stopped bus ahead falsely triggered bus deadlock corridor")
	}
}
