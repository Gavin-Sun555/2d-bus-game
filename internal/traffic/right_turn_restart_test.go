package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestMetroRightTurningBusAfterTwoThroughCars(t *testing.T) {
	const lw = 52.
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	c := sim.Vec2{Y: 3600}
	var it *sim.TrafficIntersection
	for _, p := range tls.Intersections {
		if p.Center.Distance(c) < 1 {
			it = p
			break
		}
	}
	line := it.GetStopLineDistForHeading(math.Pi, lw)
	b := sim.NewBus(line+230, c.Y-1.5*lw, math.Pi)
	ad := autopilot.NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	route := &sim.Route{StopIDs: []int{10, 5}, CurrentIndex: 1}
	tm := &TrafficManager{}
	for i := 0; i < 2; i++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i + 1, Pos: sim.Vec2{X: line + 40 + float64(i)*85, Y: b.Pos.Y}, Heading: math.Pi, Length: 48, Width: 22, IsEW: true, RoadCenter: c.Y, LaneIndex: 1, TargetSpeed: 150})
	}
	// Northbound turn must clear the southbound queue and opposing left-turn traffic.
	for lane := 0; lane < 2; lane++ {
		tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: lane + 3, Pos: sim.Vec2{X: -(float64(lane) + .5) * lw, Y: c.Y - it.GetStopLineDistForHeading(math.Pi/2, lw) - 40}, Heading: math.Pi / 2, Length: 48, Width: 22, RoadCenter: 0, LaneIndex: lane, TargetSpeed: 150})
	}
	for tick := 0; tick < 2400; tick++ {
		it.Phase = sim.PhaseNSGreen
		if tick >= 180 {
			it.Phase = sim.PhaseEWGreen
		}
		for i, car := range tm.Cars {
			tm.StepCar(1./60, car, i, b, tls, len(tm.Cars), lw, w)
		}
		var obs []autopilot.VehicleObstacle
		for _, car := range tm.Cars {
			obs = append(obs, autopilot.VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width, JunctionTurn: car.JunctionTurn, JunctionCorridor: car.JunctionCorridor})
		}
		th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
		b.Update(1./60, th, br, st, hb, d)
		if b.Pos.Y < c.Y-500 {
			return
		}
	}
	for _, car := range tm.Cars {
		t.Logf("car%d pos%v h%.2f spd%.2f turn%+v corridor%+v", car.ID, car.Pos, car.Heading, car.Speed, car.JunctionTurn, car.JunctionCorridor)
	}
	t.Fatalf("bus did not turn: pos%v h%.2f target%.2f pursuit%v", b.Pos, b.Heading, ad.LastTargetSpeed, ad.LastPursuitPt)
}
