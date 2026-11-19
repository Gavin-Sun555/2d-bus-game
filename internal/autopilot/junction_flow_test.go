package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestGreenBusFollowsTurningCarWithoutAdmissionPause(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4}}}
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{Y: -2000}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWLeftGreen
	line := it.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-60, lw/2, 0)
	bus.Speed = 85
	ad := NewAutopilotDriver()
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
	if bus.JunctionCorridor == nil {
		t.Fatal("missing corridor")
	}
	leader := *bus.JunctionCorridor
	leader.Bodies = append([]sim.OBB(nil), leader.Bodies...)
	leader.Entered = true
	body, ok := sim.PredictJunctionBody(&leader, 220)
	if !ok {
		t.Fatal("missing leader path")
	}
	// Publish the leader's remaining curve, beginning at its actual pose.
	bodies := []sim.OBB{body}
	for _, b := range leader.Bodies {
		if b.Center.Distance(bus.Pos) > 220 {
			bodies = append(bodies, b)
		}
	}
	leader.Bodies = bodies
	leader.RebuildBounds()
	obstacle := VehicleObstacle{Pos: body.Center, Heading: body.Heading, Speed: 85, Length: 48, Width: 22, JunctionCorridor: &leader}
	f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: 85, obstacles: []VehicleObstacle{obstacle}}
	if got := ad.junctionAdmissionSpeed(f); !math.IsInf(got, 1) {
		t.Fatalf("green bus stopped for same-lane turning leader: %.1f", got)
	}
	if got := ad.obstacleSpeed(f, 85); got < 60 {
		t.Fatalf("bus braked unnecessarily for moving leader: %.1f", got)
	}
}

// A green left-turning platoon keeps moving through the junction and each bus
// stays in its lane; covers ordinary and berserk followers.
func TestGreenTurningBusPlatoonKeepsMoving(t *testing.T) {
	for _, tc := range []struct {
		name       string
		crossLanes int
		berserk    bool
	}{{"cross4", 4, false}, {"berserk/cross2", 2, true}} {
		t.Run(tc.name, func(t *testing.T) {
			const lw = 52.
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: tc.crossLanes}}}
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{Y: -2000}, -math.Pi/2, tc.crossLanes, sim.RGBA(0, 0, 0, 255), lw)}
			w.RebuildIntersections()
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			it := tls.Intersections[0]
			it.Phase = sim.PhaseEWLeftGreen
			line := it.GetStopLineDistForHeading(0, lw)
			leader, follower := sim.NewBus(-line-60, lw/2, 0), sim.NewBus(-line-280, lw/2, 0)
			leader.Speed, follower.Speed = 85, 85
			a, b := NewAutopilotDriver(), NewAutopilotDriver()
			b.BerserkMode = tc.berserk
			ra, rb := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}, &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			for tick := 0; tick < 1800; tick++ {
				th, br, st, hb, d, _ := a.ComputeControls(leader, ra, w, tls, nil, lw, 1./60)
				leader.Update(1./60, th, br, st, hb, d)
				ob := VehicleObstacle{Pos: leader.Pos, Heading: leader.Heading, Speed: leader.Speed, Length: leader.Length, Width: leader.Width, JunctionCorridor: leader.JunctionCorridor}
				th, br, st, hb, d, _ = b.ComputeControls(follower, rb, w, tls, []VehicleObstacle{ob}, lw, 1./60)
				follower.Update(1./60, th, br, st, hb, d)
				if follower.Pos.X > -line-35 && follower.Pos.X < -line+35 && follower.Speed < 5 {
					t.Fatalf("bus paused at green line tick%d speed%.2f target%.2f", tick, follower.Speed, b.LastTargetSpeed)
				}
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: leader.Pos, Heading: leader.Heading, HalfLen: leader.Length / 2, HalfWid: leader.Width / 2}, sim.OBB{Center: follower.Pos, Heading: follower.Heading, HalfLen: follower.Length / 2, HalfWid: follower.Width / 2}); hit {
					t.Fatalf("bus platoon contact tick%d", tick)
				}
				if follower.Pos.Y < -550 {
					return
				}
			}
			t.Fatalf("bus platoon did not clear: %v speed%.2f", follower.Pos, follower.Speed)
		})
	}
}

func TestUTurnYieldsToOpposingLeftOnSharedGreen(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6},
		{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 6},
	}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWLeftGreen
	line := it.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-66, lw/2, 0)
	leftBus := sim.NewBus(line+66, -lw/2, math.Pi)
	turn := func(b *sim.Bus, exit float64) *sim.JunctionCorridor {
		c := &sim.JunctionCorridor{EntryHeading: b.Heading, Remaining: .25, Bodies: []sim.OBB{
			{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2},
			{Heading: exit, HalfLen: b.Length / 2, HalfWid: b.Width / 2},
		}}
		c.RebuildBounds()
		return c
	}
	bus.JunctionCorridor = turn(bus, math.Pi)
	leftBus.JunctionCorridor = turn(leftBus, math.Pi/2)
	ad := NewAutopilotDriver()
	f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, obstacles: []VehicleObstacle{ObserveBus(leftBus)}}
	if got := ad.junctionAdmissionSpeed(f); math.IsInf(got, 1) {
		t.Fatal("conflicting U-turn had no entry speed limit")
	}
	f.bus, f.obstacles[0] = leftBus, ObserveBus(bus)
	if got := ad.junctionAdmissionSpeed(f); !math.IsInf(got, 1) {
		t.Fatalf("regular left was also held: %.1f", got)
	}
	bus.JunctionCorridor.UTurnReady = true
	if got := ad.junctionAdmissionSpeed(f); math.IsInf(got, 1) {
		t.Fatal("regular left did not yield to the waiting U-turn")
	}
	f.bus, f.obstacles[0] = bus, ObserveBus(leftBus)
	if got := ad.junctionAdmissionSpeed(f); !math.IsInf(got, 1) {
		t.Fatalf("waiting U-turn was still held by the unentered left: %.1f", got)
	}
	leftBus.JunctionCorridor.Entered = true
	if got := ad.junctionAdmissionSpeed(f); math.IsInf(got, 1) {
		t.Fatal("U-turn ignored a vehicle that had already entered")
	}

}

func TestSameDirectionMovingLeaderWithPostJunctionTaper(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: 3900, Y: 0}, P2: sim.Vec2{X: 3900, Y: 3600}, Lanes: 4},
		{P1: sim.Vec2{X: 2000, Y: 1800}, P2: sim.Vec2{X: 5800, Y: 1800}, Lanes: 4},
	}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	bus := sim.NewBus(3808.2, 1257.8, 1.415)
	corridor := &sim.JunctionCorridor{
		Center:       sim.Vec2{X: 3900, Y: 1800},
		EntryHeading: math.Pi / 2,
		Remaining:    .25,
		Bodies: []sim.OBB{
			{Center: bus.Pos, Heading: bus.Heading, HalfLen: 52, HalfWid: 21},
			{Center: sim.Vec2{X: 3858.2, Y: 2420.6}, Heading: 1.3616, HalfLen: 52, HalfWid: 21},
		},
	}
	corridor.RebuildBounds()
	bus.JunctionCorridor = corridor
	carTurn := &sim.CarJunctionTurn{
		Center:       sim.Vec2{X: 3900, Y: 1800},
		EntryHeading: math.Pi / 2,
		ExitHeading:  math.Pi / 2,
		Straight:     true,
		Points:       []sim.Vec2{{X: 3763.8, Y: 1300.4}, {X: 3763.8, Y: 2200}},
	}
	ob := VehicleObstacle{
		Pos:          sim.Vec2{X: 3763.8, Y: 1300.4},
		Heading:      math.Pi / 2,
		Speed:        81,
		Length:       46,
		Width:        22,
		JunctionTurn: carTurn,
	}
	ad := NewAutopilotDriver()
	f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, obstacles: []VehicleObstacle{ob}}
	if !ad.sameDirectionMovingLeader(corridor, f, math.Pi/2, sim.Vec2{Y: 1}, ob, math.Pi/2, math.Pi/2) {
		t.Fatal("same-direction adjacent-lane moving car should be recognized as leader despite post-junction lane taper")
	}
}
