package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestGreenLeftPlatoonFollowsWithoutStopLinePause(t *testing.T) {
	for _, lanes := range []int{4, 6, 8} {
		t.Run(fmt.Sprint(lanes), func(t *testing.T) {
			w, tls := forcedLeftWorld(lanes, 0)
			inter := tls.Intersections[0]
			inter.Phase = sim.PhaseEWLeftGreen
			line := inter.GetStopLineDistForHeading(0, 52)
			tm := &TrafficManager{}
			for i := 0; i < 5; i++ {
				tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: i + 1, Pos: sim.Vec2{X: -line - 40 - float64(i)*110, Y: 26}, Heading: 0, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, IsEW: true})
			}
			for tick := 0; tick < 1300; tick++ {
				for i, c := range tm.Cars {
					tm.StepCar(1./60, c, i, nil, tls, len(tm.Cars), 52, w)
					if i > 0 && c.Pos.X > -line-30 && c.Pos.X < -line+25 && c.Speed < 10 {
						t.Fatalf("follower%d stopped at green line tick%d speed%.2f", i, tick, c.Speed)
					}
				}
				for i, a := range tm.Cars {
					for _, b := range tm.Cars[i+1:] {
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 24, HalfWid: 11}); hit {
							t.Fatalf("platoon collision tick%d", tick)
						}
					}
				}
			}
		})
	}
}

func TestForcedLeftQueuesMergeWithoutContact(t *testing.T) {
	for _, lanes := range []int{6, 8} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("lanes%d/h%.2f", lanes, h), func(t *testing.T) {
				w, tls := forcedLeftWorld(lanes, h)
				it := tls.Intersections[0]
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				it.Phase = sim.PhaseEWLeftGreen
				if math.Abs(u.Y) > .5 {
					it.Phase = sim.PhaseNSLeftGreen
				}
				line := it.GetStopLineDistForHeading(h, 52)
				tm := &TrafficManager{}
				for row := 0; row < 4; row++ {
					for lane := 0; lane < lanes/2; lane++ {
						p := u.Mul(-line - 300 - float64(row)*125 - float64(lane)*25).Add(r.Mul((float64(lane) + .5) * 52))
						tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: len(tm.Cars) + 1, CanOvertake: true, Pos: p, Heading: h, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, IsEW: math.Abs(u.X) > .5, LaneIndex: lane})
					}
				}
				for tick := 0; tick < 5400; tick++ {
					for i, c := range tm.Cars {
						tm.StepCar(1./60, c, i, nil, tls, len(tm.Cars), 52, w)
					}
					for i, a := range tm.Cars {
						for _, b := range tm.Cars[i+1:] {
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 24, HalfWid: 11}); hit {
								t.Fatalf("merge contact tick%d cars%d/%d pos%v/%v", tick, a.ID, b.ID, a.Pos, b.Pos)
							}
						}
					}
				}
				for _, c := range tm.Cars {
					if c.Pos.Dot(r.Mul(-1)) < 500 {
						for _, other := range tm.Cars {
							t.Logf("car%d lane%d pos%v speed%.2f merging%v", other.ID, other.LaneIndex, other.Pos, other.Speed, other.JunctionTurn != nil)
						}
						t.Fatalf("queue stuck: car%d lane%d pos%v", c.ID, c.LaneIndex, c.Pos)
					}
				}
			})
		}
	}
}

func TestTurningLeaderDoesNotReserveWholeJunctionForFollower(t *testing.T) {
	w, tls := forcedLeftWorld(6, 0)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWLeftGreen
	line := it.GetStopLineDistForHeading(0, 52)
	follower := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: -line - 30, Y: 26}, Heading: 0, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, IsEW: true}
	leader := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -line - 10, Y: 26}, Heading: 0, Speed: 85, TargetSpeed: 85, Length: 48, Width: 22, IsEW: true}
	f := carFrame{car: leader, world: w, tls: tls, laneWidth: 52, activeCount: 2}
	leader.JunctionTurn = planCarJunctionTurn(f, it)
	leader.Pos = leader.JunctionTurn.Points[0]
	moveCarThroughJunction(leader, 1.8)
	f.car = follower
	f.index = 1
	tm := &TrafficManager{Cars: []*sim.TrafficCar{leader, follower}}
	if got := tm.junctionQueueSpeed(f, 85); got < 85 {
		t.Fatalf("moving turning leader blocked green follower: %.1f heading%.2f", got, leader.Heading)
	}
}
