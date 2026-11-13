package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCarStopsAtStationCrossingAndResumesOnGreen(t *testing.T) {
	const laneWidth, dt = 52., 1. / 60
	w := &sim.World{LargeGrid: true}
	w.BuildMap(sim.MapThemeMetropolis, laneWidth)
	lights := &sim.TrafficLightSystem{}
	for _, crossing := range w.StationCrossings() {
		if crossing.Kind != sim.StationSignalZebra {
			continue
		}
		axis := sim.UnitFromAngle(crossing.Heading)
		right := sim.Vec2{X: -axis.Y, Y: axis.X}
		start := crossing.Center.Sub(axis.Mul(1350)).Add(right.Mul(laneWidth / 2))
		car := &sim.TrafficCar{Pos: start, Heading: crossing.Heading, Speed: sim.SpeedKPH(120), TargetSpeed: sim.SpeedKPH(120), Length: 46, Width: 22}
		car.IsEW = math.Abs(axis.X) > .5
		car.RoadCenter = crossing.Center.X
		if car.IsEW {
			car.RoadCenter = crossing.Center.Y
		}
		tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
		for lights.StationSignalPhase(crossing) != sim.StationSignalRed {
			lights.Update(1)
		}
		for tick := 0; tick < 420; tick++ {
			tm.StepCar(dt, car, 0, nil, lights, 1, laneWidth, w)
			bumper := car.Pos.Add(axis.Mul(car.Length / 2))
			if crossing.Center.Sub(bumper).Dot(axis) < sim.StationCrossingStopOffset {
				t.Fatalf("crossed a red station signal at tick %d", tick)
			}
		}
		if car.Speed > .01 || crossing.Center.Sub(car.Pos).Dot(axis) > 90 {
			t.Fatalf("did not stop near the crossing: pos %v, speed %.1f", car.Pos, car.Speed)
		}
		for lights.StationSignalPhase(crossing) != sim.StationSignalGreen {
			lights.Update(1)
		}
		for tick := 0; tick < 180; tick++ {
			tm.StepCar(dt, car, 0, nil, lights, 1, laneWidth, w)
		}
		if car.Pos.Sub(crossing.Center).Dot(axis) <= 0 {
			t.Fatal("did not resume on green")
		}
		return
	}
	t.Fatal("no signal-controlled crossing")
}
