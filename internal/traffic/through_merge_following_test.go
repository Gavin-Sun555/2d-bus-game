package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestFollowingThroughMergeRequiresCompletedMatchingIntent(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, scenario := range []string{"completed through", "still merging", "turning right", "expired", "stopped bus", "different lane", "crossing", "different junction", "bus behind"} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, scenario), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				b := sim.NewBus(0, 0, h)
				b.Pos = u.Mul(-80).Add(r.Mul(78))
				b.Speed = 80
				b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: r.Mul(78), Heading: h, Remaining: .3, Joined: true}
				c := &sim.TrafficCar{Pos: u.Mul(-250).Add(r.Mul(78)), Heading: h, LaneIndex: 1, Length: 48, Width: 22}
				it := &sim.TrafficIntersection{}
				switch scenario {
				case "still merging":
					b.LaneMergeRequest.Joined = false
				case "turning right":
					b.LaneMergeRequest.RightTurn = true
				case "expired":
					b.LaneMergeRequest.Remaining = 0
				case "stopped bus":
					b.Speed = 0
				case "different lane":
					c.Pos = c.Pos.Add(r.Mul(52))
				case "crossing":
					c.Heading += math.Pi / 2
				case "different junction":
					b.LaneMergeRequest.Junction = sim.Vec2{X: 500}
				case "bus behind":
					b.Pos = u.Mul(-400).Add(r.Mul(78))
				}
				if got := canFollowCommittedThroughBus(carFrame{car: c, laneWidth: 52}, b, it); got != (scenario == "completed through") {
					t.Fatalf("follow=%v", got)
				}
			})
		}
	}
}

func TestFollowingCommittedBusStillStopsForRed(t *testing.T) {
	it := &sim.TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{6, 6, 4, 4}, ApproachIs4Lane: [4]bool{true, true, true, true}, Phase: sim.PhaseEWGreen}
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{it}}
	line := it.GetStopLineDistForHeading(0, 52)
	b := sim.NewBus(-line/2, 78, 0)
	b.Speed = 80
	b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}, Heading: 0, Remaining: .3, Joined: true}
	c := &sim.TrafficCar{Pos: sim.Vec2{X: -line - 34, Y: 78}, Heading: 0, LaneIndex: 1, Length: 48, Width: 22}
	f := carFrame{car: c, tls: tls, laneWidth: 52, buses: []*sim.Bus{b}}
	tm := &TrafficManager{}
	if got := tm.signalSpeed(f, 100); got != 100 {
		t.Fatalf("green follower blocked:%.1f", got)
	}
	it.Phase = sim.PhaseNSGreen
	if got := tm.signalSpeed(f, 100); got != 0 {
		t.Fatalf("red ignored:%.1f", got)
	}
}
