package traffic

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPlayerTrailerWedgeStartsCheckedReverse(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 8}}}
	b := sim.NewBus(-5.395050710633, 0, 1.5392461143312675)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = .8477325672222826, true
	c := &sim.TrafficCar{ID: 6858, Pos: sim.Vec2{X: -26, Y: -133.6687158392}, Heading: math.Pi / 2, Length: 46, Width: 22, TargetSpeed: 166, IsBraking: true, LaneIndex: 0}
	tm := NewTrafficManager(52)
	tm.Cars = []*sim.TrafficCar{c}
	start := c.Pos
	for tick := 0; tick < 60*8; tick++ {
		tm.resolveDeadlocks(1./60, b, 1, 52, w, nil, false)
		tm.StepCar(1./60, c, 0, b, nil, 1, 52, w)
		if hit, _, _ := b.CollidesBody(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}); hit {
			t.Fatal("car retreat hit the player's trailer")
		}
	}
	if c.Pos.Y > start.Y-20 {
		t.Fatalf("player trailer wedge never enabled safe reverse: position%v timer%.2f", c.Pos, c.DeadlockTimer)
	}
}

func TestReverseRequestOnlyReleasesCompletedTurnExit(t *testing.T) {
	for _, scenario := range []string{"exit tail", "still bending", "near tangent", "inside junction"} {
		t.Run(scenario, func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{
				{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 8},
				{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8},
			}}
			w.RebuildIntersections()
			c := &sim.TrafficCar{Pos: sim.Vec2{X: -26, Y: 520}, Heading: math.Pi / 2, Length: 46, Width: 22}
			c.JunctionTurn = &sim.CarJunctionTurn{Points: []sim.Vec2{{X: -100}, {X: -26, Y: 450}, {X: -26, Y: 650}}, Segment: 1, ExitHeading: math.Pi / 2, ExitRoadCenter: 0, ExitLane: 0}
			switch scenario {
			case "still bending":
				c.JunctionTurn.Segment = 0
			case "near tangent":
				c.Pos.Y = 470
			case "inside junction":
				c.Pos.Y = 100
			}
			before := c.Pos
			if finishCarTurnExitBeforeReverse(carFrame{world: w, laneWidth: 52}, c) != (scenario == "exit tail") {
				t.Fatalf("incorrect release of %s", scenario)
			}
			if c.Pos != before {
				t.Fatal("finishing exit tail moved the vehicle")
			}
		})
	}
}
