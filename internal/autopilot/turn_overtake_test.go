package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBerserkDoesNotStartPassingOnLeftTurn(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 4}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 2}}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{Y: -2000}, -math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw)}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	// Probe the latter half of the turn, where individual sampled angles are
	// small and the outgoing two-lane road permits the player to pass on straights.
	for i := 1; i < len(ad.drivePath)-1; i++ {
		p := ad.drivePath[i]
		u := ad.drivePath[i+1].Sub(p).Normalize()
		if !w.InsideJunction(p, lw, 48) || u.Y > -.4 {
			continue
		}
		b := sim.NewBus(p.X, p.Y, math.Atan2(u.Y, u.X))
		b.Speed = 40
		driver := NewAutopilotDriver()
		driver.BerserkMode = true
		driver.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		driver.trackPath(p)
		leader := VehicleObstacle{Pos: p.Add(u.Mul(145)), Heading: b.Heading, Speed: 20, Length: 96, Width: 36}
		// A preceding frame preserves the original approach direction.
		b.JunctionCorridor = &sim.JunctionCorridor{Center: sim.Vec2{}, EntryHeading: 0, Entered: true, Remaining: .25}
		driver.ComputeControls(b, route, w, tls, []VehicleObstacle{leader}, lw, 1./60)
		if math.Abs(driver.TargetLaneOffset) > .01 {
			t.Fatalf("tried to pass inside left turn at%v h%.2f offset%.1f", p, b.Heading, driver.TargetLaneOffset)
		}
	}
}

func TestBerserkTurningBusPlatoonStaysInLane(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 2}}}
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2000}, 0, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{Y: -2000}, -math.Pi/2, 2, sim.RGBA(0, 0, 0, 255), lw)}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWLeftGreen
	line := it.GetStopLineDistForHeading(0, lw)
	leader, follower := sim.NewBus(-line-60, lw/2, 0), sim.NewBus(-line-280, lw/2, 0)
	leader.Speed, follower.Speed = 85, 85
	a, b := NewAutopilotDriver(), NewAutopilotDriver()
	b.BerserkMode = true
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
