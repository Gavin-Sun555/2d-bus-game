package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestCommittedThroughCarClearsStoppedOncomingTurn(t *testing.T) {
	b := sim.NewBus(3789.629006603333, 16.82424116720744, -.2694604281414428)
	b.JunctionCorridor = &sim.JunctionCorridor{Center: sim.Vec2{X: 3900}, Remaining: .25}
	c := &sim.TrafficCar{Pos: sim.Vec2{X: 3883.480041619549, Y: -26}, Heading: math.Pi, Length: 46, Width: 22}
	c.JunctionTurn = &sim.CarJunctionTurn{Straight: true, Center: sim.Vec2{X: 3900}, Points: []sim.Vec2{c.Pos, {X: 3500, Y: -26}}, ExitHeading: math.Pi, EntryHeading: math.Pi}
	if !committedThroughCarClearsStoppedTurn(c, b) {
		t.Fatal("clear committed car remains held by a following margin")
	}
	if got := (&TrafficManager{}).busFollowingSpeed(carFrame{car: c, buses: []*sim.Bus{b}}, 100); got != 100 {
		t.Fatalf("clear crossing held: %v", got)
	}
	c.Pos.Y = -15
	c.JunctionTurn.Points[0] = c.Pos
	c.JunctionTurn.Points[1].Y = -15
	if committedThroughCarClearsStoppedTurn(c, b) {
		t.Fatal("actual collision ignored")
	}
	c.Pos.Y = -26
	c.JunctionTurn.Points[0] = c.Pos
	c.JunctionTurn.Points[1].Y = -26
	b.Speed = 1
	if committedThroughCarClearsStoppedTurn(c, b) {
		t.Fatal("moving turn ignored")
	}
	b.Speed = 0
	c.JunctionTurn = nil
	if committedThroughCarClearsStoppedTurn(c, b) {
		t.Fatal("uncommitted car ignored entry rules")
	}
}

func TestCommittedLeadingCarClearsPendingBusMerge(t *testing.T) {
	b := sim.NewBus(-3857.836845360735, 563.2489391027148, -math.Pi/2)
	b.SetModel(sim.BusArticulated)
	b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{X: -3718}, Heading: -math.Pi / 2, Remaining: .3, RightTurn: true}
	c := &sim.TrafficCar{Pos: sim.Vec2{X: -3822, Y: 499.2615037334158}, Heading: -math.Pi / 2, Length: 46, Width: 22}
	c.JunctionTurn = &sim.CarJunctionTurn{Straight: true, Center: sim.Vec2{X: -3900}, EntryHeading: -math.Pi / 2, ExitHeading: -math.Pi / 2, Points: []sim.Vec2{c.Pos, {X: -3822, Y: -600}}}
	if !carClearsStoppedBusAhead(c, b) {
		t.Fatal("committed leading car waits on a merge behind it")
	}
	if got := (&TrafficManager{}).busFollowingSpeed(carFrame{car: c, buses: []*sim.Bus{b}}, 100); got != 100 {
		t.Fatalf("clear car blocked by bus margin: %.1f", got)
	}
	c.Pos.X = -3845
	c.JunctionTurn.Points[0] = c.Pos
	c.JunctionTurn.Points[1].X = -3845
	if carClearsStoppedBusAhead(c, b) {
		t.Fatal("overlapping committed sweep ignored")
	}
	c.Pos = sim.Vec2{X: -3822, Y: 700}
	c.JunctionTurn.Points[0] = c.Pos
	if carClearsStoppedBusAhead(c, b) {
		t.Fatal("rear car stole merge priority")
	}
}
