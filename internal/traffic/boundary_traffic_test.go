package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func boundaryWorld() *sim.World {
	return &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 8}}, Intersections: []sim.Vec2{{X: -1000}, {X: 1000}}}
}
func TestBoundaryExitAndSafeReplacement(t *testing.T) {
	for _, heading := range []float64{0, math.Pi, math.Pi / 2, -math.Pi / 2} {
		t.Run(string(rune('A'+int(heading*10+40))), func(t *testing.T) {
			w := boundaryWorld()
			ew := math.Abs(math.Cos(heading)) > .5
			if !ew {
				w.Roads[0].P1 = sim.Vec2{Y: -2000}
				w.Roads[0].P2 = sim.Vec2{Y: 2000}
				w.Intersections = []sim.Vec2{{Y: -1000}, {Y: 1000}}
			}
			forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			car := &sim.TrafficCar{ID: 1, Pos: forward.Mul(2000), Heading: heading, Speed: 100, TargetSpeed: 100, Length: 46, Width: 22, IsEW: ew, LaneIndex: 3}
			car.Pos = car.Pos.Add(sim.Vec2{X: -forward.Y, Y: forward.X}.Mul(182))
			tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
			original := car.Pos
			tm.updateBoundaryTraffic(nil, 1, 52, w)
			if tm.Cars[0] != car {
				t.Fatal("Removed before rear crossed boundary")
			}
			for i := 0; i < 60 && tm.Cars[0] == car; i++ {
				tm.Update(1.0/60, nil, nil, 1, 52, w)
			}
			if tm.Cars[0] == car {
				t.Fatal("Car failed to leave map")
			}
			if car.Pos.Sub(original).Dot(forward) < 23 {
				t.Fatal("Car teleported before whole body left")
			}
			newborn := tm.Cars[0]
			if newborn.ID == car.ID || len(tm.Cars) != 1 {
				t.Fatal("Expected new car at unchanged density")
			}
			excluded := w.Intersections[1]
			other := w.Intersections[0]
			if heading == math.Pi || heading == -math.Pi/2 {
				excluded, other = other, excluded
			}
			if newborn.Pos.Distance(other) >= newborn.Pos.Distance(excluded) {
				t.Fatal("Spawned at exit intersection")
			}
			if newborn.LaneIndex >= 4 {
				t.Fatal("Invalid lane")
			}
		})
	}
}
func TestBoundaryRespawnWaitsForSafeSpace(t *testing.T) {
	w := boundaryWorld()
	tm := &TrafficManager{pendingExits: []sim.Vec2{{X: 2100}}}
	// Cover both approach regions of the only eligible intersection.
	for x := -1700.0; x < -300; x += 100 {
		for y := 26.0; y <= 182; y += 52 {
			for _, sign := range []float64{-1, 1} {
				tm.Cars = append(tm.Cars, &sim.TrafficCar{Pos: sim.Vec2{X: x, Y: y * sign}, Length: 46, Width: 22, IsEW: true})
			}
		}
	}
	before := len(tm.Cars)
	tm.updateBoundaryTraffic(nil, 4, 52, w)
	if len(tm.pendingExits) != 1 || len(tm.Cars) != before {
		t.Fatal("Spawned into occupied approach")
	}
	tm.Cars = nil
	tm.updateBoundaryTraffic(nil, 4, 52, w)
	if len(tm.pendingExits) != 0 || len(tm.Cars) != 1 {
		t.Fatal("Did not retry safe replacement")
	}
}
func TestIntersectionCarNeedsOwnFullTimeout(t *testing.T) {
	w := boundaryWorld()
	inter := &sim.TrafficIntersection{Center: w.Intersections[0], IsMajor: true, GreenDuration: 10, YellowDuration: 3, LeftGreenDuration: 7}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{inter}}
	car := &sim.TrafficCar{Pos: inter.Center, Length: 46, Width: 22}
	bus := sim.NewBus(inter.Center.X+50, inter.Center.Y, math.Pi)
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	timeout := tls.DeadlockTimeoutAt(car.Pos)
	for i := 0; i < int(timeout*2)-1; i++ {
		tm.ResolveDeadlocks(.5, bus, 1, 52, w, tls)
	}
	if len(tm.Cars) != 1 {
		t.Fatal("Removed before complete signal cycles elapsed")
	}
	tm.ResolveDeadlocks(.5, bus, 1, 52, w, tls)
	if len(tm.Cars) != 0 {
		t.Fatal("Did not remove after full stationary timeout")
	}
}
