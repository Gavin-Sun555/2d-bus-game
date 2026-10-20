package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRoundaboutEntryCurveClearsWaitingNeighbor(t *testing.T) {
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -205, Y: 63}, Heading: 0, Speed: 70, Length: 48, Width: 22, InRoundabout: true, RoundaboutRadius: math.Hypot(205, 63), RoundaboutAngle: math.Atan2(63, -205), RoundaboutTargetR: 214, RoundaboutOuterRadius: 240}
	beginRoundaboutEntry(car)
	bus := sim.NewBus(-225, 21, .15)
	body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length/2 + 2, HalfWid: bus.Width/2 + 2}
	for i := 0; i < 60; i++ {
		other := sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 26, HalfWid: 13}
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			t.Fatalf("sweep collision frame%d pos%v heading%.3f radius%.1f bus length%.1f width%.1f", i, car.Pos, car.Heading, car.RoundaboutRadius, bus.Length, bus.Width)
		}
		moveCar(car, .04)
	}
}
