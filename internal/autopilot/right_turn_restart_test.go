package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestMetropolisRightTurnRestartsAfterQueue(t *testing.T) {
	const lw = 52.
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	center := sim.Vec2{Y: 3600}
	var inter *sim.TrafficIntersection
	for _, it := range tls.Intersections {
		if it.Center.Distance(center) < 1 {
			inter = it
			break
		}
	}
	if inter == nil {
		t.Fatal("missing junction")
	}
	inter.Phase = sim.PhaseEWGreen
	route := &sim.Route{StopIDs: []int{10, 5}, CurrentIndex: 1}
	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.PrepareLeg(w, w.Stops[10], w.Stops[5], lw)
	line := inter.GetStopLineDistForHeading(math.Pi, lw)
	// Ordinary fleet bus queues in the outer through/right-turn lane.
	b := sim.NewBus(line+210, center.Y-1.5*lw, math.Pi)
	obs := []VehicleObstacle{{Pos: sim.Vec2{X: line + 105, Y: b.Pos.Y}, Heading: math.Pi, Length: 48, Width: 22}, {Pos: sim.Vec2{X: line + 30, Y: b.Pos.Y}, Heading: math.Pi, Length: 48, Width: 22}}
	for tick := 0; tick < 1800; tick++ {
		obstacles := obs
		if tick >= 180 {
			obstacles = nil
		}
		th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, tls, obstacles, lw, 1./60)
		b.Update(1./60, th, br, st, hb, d)
		if b.Pos.Y < center.Y-500 {
			return
		}
	}
	f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: math.Abs(b.Speed), remaining: ad.LastRemaining, targetStop: w.Stops[5]}
	t.Fatalf("right turn stuck pos%v h%.2f target%.1f offset%.1f/%1.f hold%.1f signal%.1f admission%.1f safety%v path%v", b.Pos, b.Heading, ad.LastTargetSpeed, ad.LaneOffset, ad.TargetLaneOffset, ad.busTurnHoldDistance, ad.trafficLightSpeed(f, 150), ad.junctionAdmissionSpeed(f), ad.roadSafetyHold, ad.drivePath[:min(5, len(ad.drivePath))])
}

// A red queue's nose can pass the painted line slightly without entering the
// junction box. Its future crossing path must not reserve the entire box.
func TestRightTurnNotHeldByRedQueueAtEntryLine(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("width%.0f/berserk%v", lw, berserk), func(t *testing.T) { testRightTurnRedQueue(t, lw, berserk) })
		}
	}
}

func testRightTurnRedQueue(t *testing.T, lw float64, berserk bool) {
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	center := sim.Vec2{Y: 3600}
	var it *sim.TrafficIntersection
	for _, p := range tls.Intersections {
		if p.Center.Distance(center) < 1 {
			it = p
			break
		}
	}
	it.Phase = sim.PhaseEWGreen
	line := it.GetStopLineDistForHeading(math.Pi, lw)
	bus := sim.NewBus(line+62, center.Y-1.5*lw, math.Pi)
	ad := NewAutopilotDriver()
	ad.BerserkMode = berserk
	ad.DisallowOncomingOvertake = true
	ad.PrepareLeg(w, w.Stops[10], w.Stops[5], lw)
	ad.trackPath(bus.Pos)
	f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS}
	ad.publishJunctionCorridor(f)
	p := sim.Vec2{X: -line - 24 + 2, Y: center.Y + .5*lw}
	other := &sim.JunctionCorridor{Center: center, EntryHeading: 0, Entered: true, Remaining: .25}
	points := append([]sim.Vec2{p}, sim.QuarterTurnPath(center, sim.Vec2{X: 1}, sim.Vec2{Y: -1}, .5*lw, 1.5*lw, w.JunctionAt(center, lw).TurnRadius)...)
	points = append(points, sim.Vec2{X: 1.5 * lw, Y: center.Y - 500})
	other.Bodies = sim.AppendPathBodies(nil, points, 48, 22)
	other.RebuildBounds()
	f.obstacles = []VehicleObstacle{{Pos: p, Heading: 0, Length: 48, Width: 22, JunctionCorridor: other}}
	if got := ad.junctionAdmissionSpeed(f); !math.IsInf(got, 1) {
		t.Fatalf("right-turn bus held by waiting red queue: %.1f", got)
	}
	// The green-arrow phase of the opposing left turn still owns this merge.
	it.Phase = sim.PhaseEWLeftGreen
	if got := ad.junctionAdmissionSpeed(f); math.IsInf(got, 1) {
		t.Fatal("ignored lawful opposing left-turn reservation")
	}
	it.Phase = sim.PhaseEWGreen
	// A physically entered vehicle remains protected even if its light is red.
	inside := f.obstacles[0]
	inside.Pos.X = -w.JunctionAt(center, lw).HalfX + 24
	if waitingOutsideRedEntry(inside, it, w.JunctionAt(center, lw)) {
		t.Fatal("ignored actual vehicle inside box")
	}
	moving := f.obstacles[0]
	moving.Speed = 20
	if waitingOutsideRedEntry(moving, it, w.JunctionAt(center, lw)) {
		t.Fatal("ignored moving crossing vehicle")
	}
	route := &sim.Route{StopIDs: []int{10, 5}, CurrentIndex: 1}
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, f.obstacles, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, d)
		if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}, sim.OBB{Center: p, Heading: 0, HalfLen: 24, HalfWid: 11}); hit {
			t.Fatal("right turn contacted waiting queue")
		}
		if bus.Pos.Y < center.Y-500 {
			return
		}
	}
	t.Fatalf("bus did not clear right turn: %v target%.1f", bus.Pos, ad.LastTargetSpeed)
}
