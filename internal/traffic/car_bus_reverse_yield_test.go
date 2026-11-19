package traffic

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCarReverseRelaysToRearBus(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 3000, Y: 7200}, P2: sim.Vec2{X: 10000, Y: 7200}, Lanes: 6}}}
	c := &sim.TrafficCar{ID: 4274, Pos: sim.Vec2{X: 6947.497905979907, Y: 7226}, Heading: 0, Length: 46, Width: 22, RoadCenter: 7200, IsEW: true, LaneIndex: 0, DeadlockTimer: 6, YieldReverseRequest: .6}
	front := sim.NewBus(7105.282815350631, 7234.7569604313685, -.2574287270161157)
	front.SetModel(sim.BusArticulated)
	front.TrailerHeading, front.TrailerInitialized = -.31850984074944716, true
	rear := sim.NewBus(6845.729748535933, 7229.603057324687, .022018850210006175)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading, rear.TrailerInitialized = .02604007550790658, true
	tm := NewTrafficManager(52)
	tm.Cars = []*sim.TrafficCar{c}
	f := carFrame{car: c, world: w, buses: []*sim.Bus{front, rear}, activeCount: 1, index: 0, laneWidth: 52}
	start := c.Pos
	tm.tryStartCarDeadlockReverse(f)
	if rear.ReverseYieldRequest <= 0 || c.ReverseRemaining != 0 || c.Pos != start {
		t.Fatal("boxed car did not ask the rear bus before moving")
	}
	// Once the rear fleet controller has opened the requested gap, the same
	// car controller can select its real reverse without ignoring any bus.
	rear.Pos.X -= 40
	tm.tryStartCarDeadlockReverse(f)
	if c.ReverseRemaining <= 0 {
		t.Fatal("car did not use the opened rear gap")
	}
}
