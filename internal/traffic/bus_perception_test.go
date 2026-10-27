package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusPerceptionRefreshesBetweenStandaloneSteps(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	stop := w.Stops[0]
	bus := sim.NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	car := &sim.TrafficCar{Pos: stop.RoadCenter, Heading: stop.Heading, Length: 46, Width: 22, IsEW: math.Abs(math.Cos(stop.Heading)) > .5}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	tm.StepCar(1./60, car, 0, bus, nil, 1, 52, w)
	if len(tm.observedBuses) != 1 || !tm.observedParked[0] {
		t.Fatal("parked bus not observed")
	}
	bus.Pos = bus.Pos.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(800))
	tm.StepCar(1./60, car, 0, bus, nil, 1, 52, w)
	if tm.observedParked[0] {
		t.Fatal("moved bus retained its old bay classification")
	}
	tm.StepCar(1./60, car, 0, nil, nil, 1, 52, w)
	if len(tm.observedBuses) != 0 || tm.busStorage[0] != nil {
		t.Fatal("removed bus retained in observation scratch")
	}
}
