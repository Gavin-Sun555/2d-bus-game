package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestCenterBRTGeneralTrafficStaysOutsideMedian(t *testing.T) {
	const lw = 52.
	w := &sim.World{CenterBRT: true}
	w.BuildMap(sim.MapThemeMetropolis, lw)
	tm := &TrafficManager{}
	tm.BuildForWorld(w, MaxTrafficCars, lw)
	seen := 0
	for _, c := range tm.Cars {
		r, ok := w.RoadAt(c.RoadCenter, c.IsEW, c.Pos)
		if ok && r.HasMedianBusLane() {
			seen++
			if c.LaneIndex < 1 || c.LaneIndex > 3 {
				t.Fatal("civilian spawned in central bus lane", c.LaneIndex)
			}
		}
	}
	if seen == 0 {
		t.Fatal("no civilian cars tested on BRT road")
	}
	// The outgoing carriageway may reserve its innermost lane even when
	// the vehicle arrives from a narrower road using lane zero.
	road := sim.RoadSegment{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true}
	cross := sim.RoadSegment{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4}
	testWorld := &sim.World{Roads: []sim.RoadSegment{road, cross}}
	testWorld.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(testWorld)
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -400, Y: lw * .5}, Heading: 0, LaneIndex: 0, IsEW: true, Length: 46, Width: 22}
	path := planCarStraight(carFrame{car: car, world: testWorld, tls: tls, laneWidth: lw}, tls.Intersections[0])
	if path == nil || path.ExitLane != 1 {
		t.Fatal("straight junction exit occupied central BRT lane")
	}
	car.Pos.Y = 3.5 * lw
	car.LaneIndex = 3
	tm.prepareEightLaneApproach(carFrame{car: car, world: testWorld, tls: tls, laneWidth: lw})
	if car.JunctionTurn != nil {
		t.Fatal("outer general lane evacuated as if it were reserved")
	}
}

func TestCenterBRTLeftTurnUsesAdjacentGeneralLane(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
		{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
	}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	c := &sim.TrafficCar{Pos: sim.Vec2{X: -500, Y: lw * 1.5}, Heading: 0, LaneIndex: 1, IsEW: true, Length: 46, Width: 22}
	f := carFrame{car: c, world: w, tls: tls, laneWidth: lw}
	left, right := carIntersectionTurns(f, tls.Intersections[0])
	if !left || right {
		t.Fatal("adjacent general lane has no left turn")
	}
	path := buildCarJunctionTurn(f, tls.Intersections[0])
	if path == nil || path.Points[0].Y < lw {
		t.Fatal("left turn starts in reserved BRT lane")
	}
	rb := sim.Roundabout{Radius: 240}
	c.LaneIndex = 0
	c.RoundaboutTargetR = 214
	configureRoundaboutExit(c, w, &rb, lw, 0)
	if c.RoundaboutExitLane == 0 {
		t.Fatal("roundabout exit enters reserved central lane")
	}
}
