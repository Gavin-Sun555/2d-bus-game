package traffic

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func capturedLeadingCarAndBus() (*sim.TrafficCar, *sim.Bus) {
	b := sim.NewBus(10733.4954044836, 7317.187417847488, -.22281911917635863)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = -.04296367125875803, true
	c := &sim.TrafficCar{ID: 12254, Pos: sim.Vec2{X: 10773.532310614637, Y: 7278}, Heading: 0, Length: 46, Width: 22, TargetSpeed: 166, RoadCenter: 7200, IsEW: true, LaneIndex: 1}
	return c, b
}

func TestLeadingCarClearsStoppedDiagonalBus(t *testing.T) {
	c, b := capturedLeadingCarAndBus()
	if !carPathClearsBus(c, b, 0) {
		t.Fatal("captured car path must physically clear the stopped bus")
	}
	if !carClearsParallelStoppedBus(c, b) {
		t.Fatal("leading car treated the stopped diagonal bus as a forward obstruction")
	}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 8000, Y: 7200}, P2: sim.Vec2{X: 16000, Y: 7200}, Lanes: 6}}}
	tm := NewTrafficManager(52)
	tm.Cars = []*sim.TrafficCar{c}
	start := c.Pos
	for tick := 0; tick < 60*5; tick++ {
		tm.StepCar(1./60, c, 0, b, nil, 1, 52, w)
		if hit, _, _ := b.CollidesBody(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}); hit {
			t.Fatal("leading car hit the stopped bus")
		}
	}
	if c.Pos.X < start.X+120 {
		t.Fatal("leading car remained stalled")
	}
}

func TestLeadingCarClearanceRetainsYieldAndCollisionGuards(t *testing.T) {
	for _, scenario := range []string{"following", "moving bus", "body conflict", "station merge", "yielding", "turning car"} {
		t.Run(scenario, func(t *testing.T) {
			c, b := capturedLeadingCarAndBus()
			switch scenario {
			case "following":
				c.Pos.X = b.Pos.X - 30
			case "moving bus":
				b.Speed = 1
			case "body conflict":
				c.Pos.Y += 2
			case "station merge":
				b.StationMergeRequest = &sim.BusStationMergeRequest{Remaining: .3}
			case "yielding":
				c.YieldingToBus = b
			case "turning car":
				c.JunctionTurn = &sim.CarJunctionTurn{}
			}
			if carClearsParallelStoppedBus(c, b) {
				t.Fatalf("unsafe permission for %s", scenario)
			}
		})
	}
}

func TestAlongsideCarClearsStoppedDiagonalBus(t *testing.T) {
	b := sim.NewBus(-69.87547183932826, -3006.4452583619773, 1.349)
	c := &sim.TrafficCar{
		ID:          2788,
		Pos:         sim.Vec2{X: -26, Y: -3007.1733968888225},
		Heading:     1.5707963267948966,
		Length:      46,
		Width:       22,
		TargetSpeed: 166,
		RoadCenter:  0,
		IsEW:        false,
		LaneIndex:   0,
	}
	if !carClearsParallelStoppedBus(c, b) {
		t.Fatal("car alongside stopped diagonal bus with physically clear forward path should clear")
	}
}
