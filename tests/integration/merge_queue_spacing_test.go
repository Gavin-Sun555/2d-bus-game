package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestYieldingQueueClosesExcessGapWithFollowingBus(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", h), func(t *testing.T) {
			const lw = 52.
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 4}}}
			a := sim.NewBusStop(0, "from", u.Mul(-2200), h, 4, sim.RGBA(0, 0, 0, 255), lw)
			z := sim.NewBusStop(1, "to", u.Mul(2200), h, 4, sim.RGBA(0, 0, 0, 255), lw)
			w.Stops = []*sim.BusStop{a, z}
			merging := sim.NewBus(r.X*26, r.Y*26, h)
			merging.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: r.Mul(78), Heading: h, Remaining: 1}
			var cars []*sim.TrafficCar
			for i, x := range []float64{-400, -480, -560} {
				cars = append(cars, &sim.TrafficCar{ID: i, Pos: u.Mul(x).Add(r.Mul(78)), Heading: h, Length: 48, Width: 22, TargetSpeed: 100, LaneIndex: 1, IsEW: math.Abs(u.X) > .5})
			}
			tm := &traffic.TrafficManager{Cars: cars}
			follower := sim.NewBus(0, 0, h)
			follower.Pos = u.Mul(-720).Add(r.Mul(78))
			ad := autopilot.NewAutopilotDriver()
			ad.CanSkipTargetStop = true
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			for tick := 0; tick < 1200; tick++ {
				tm.Update(1.0/60, merging, nil, 1, lw, w)
				obs := []autopilot.VehicleObstacle{{Pos: merging.Pos, Heading: h, Length: 96, Width: 36}}
				for _, c := range cars {
					obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: 48, Width: 22, YieldingTo: c.YieldingToBus})
				}
				th, br, st, hb, doors, _ := ad.ComputeControls(follower, route, w, nil, obs, lw, 1.0/60)
				follower.Update(1.0/60, th, br, st, hb, doors)
				for i := 1; i < len(cars); i++ {
					if gap := cars[i-1].Pos.Sub(cars[i].Pos).Dot(u) - 48; gap < 4 {
						t.Fatalf("car contact gap%.1f", gap)
					}
				}
				if gap := cars[2].Pos.Sub(follower.Pos).Dot(u) - 72; gap < 8 {
					t.Fatalf("bus contact gap%.1f", gap)
				}
			}
			if gap := merging.Pos.Sub(cars[0].Pos).Dot(u) - 72; gap > 55 {
				t.Fatalf("first car left excess merge gap%.1f", gap)
			}
			for i := 1; i < len(cars); i++ {
				if gap := cars[i-1].Pos.Sub(cars[i].Pos).Dot(u) - 48; gap > 35 {
					t.Fatalf("trailing car left excess gap%.1f", gap)
				}
			}
			if gap := cars[2].Pos.Sub(follower.Pos).Dot(u) - 72; gap > 65 {
				t.Fatalf("following bus left excess gap%.1f", gap)
			}
		})
	}
}
