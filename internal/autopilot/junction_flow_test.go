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

func TestGreenTurningBusPlatoonKeepsMoving(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4}}}
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{Y: -2000}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWLeftGreen
	line := it.GetStopLineDistForHeading(0, lw)
	leader, follower := sim.NewBus(-line-60, lw/2, 0), sim.NewBus(-line-280, lw/2, 0)
	leader.Speed, follower.Speed = 85, 85
	a, b := NewAutopilotDriver(), NewAutopilotDriver()
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
}
