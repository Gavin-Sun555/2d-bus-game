package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestDepartureOnlyReleasesSafeStoppedRearTraffic(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8}}}
				from := sim.NewBusStop(0, "from", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), lw)
				to := sim.NewBusStop(1, "to", u.Mul(2000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{from, to}
				b := sim.NewBus(from.Pos.X, from.Pos.Y, h)
				ad := NewAutopilotDriver()
				ad.State = AutopilotStateDeparting
				ad.PrepareLeg(w, from, to, lw)
				ad.trackPath(b.Pos)
				for _, tc := range []struct {
					name         string
					along, speed float64
					clears       bool
				}{{"stopped rear bus", -150, 0, true}, {"moving rear bus", -150, 90, false}, {"alongside bus", 0, 0, false}, {"front bus", 100, 0, false}, {"rear nose alongside", -60, 0, false}} {
					t.Run(tc.name, func(t *testing.T) {
						p := from.Pos.Add(u.Mul(tc.along)).Sub(r.Mul(from.Pos.Sub(from.RoadCenter).Dot(r) - 3.5*lw))
						ob := VehicleObstacle{Pos: p, Heading: h, Speed: tc.speed, Length: 96, Width: 36}
						f := autopilotFrame{bus: b, world: w, targetStop: to, laneWidth: lw, s: ad.LastPathS, obstacles: []VehicleObstacle{ob}}
						if clear := ad.clearsStoppedDepartureFollower(f, ob); clear != tc.clears {
							t.Fatalf("departure clearance=%v want%v", clear, tc.clears)
						}
						if limit := ad.stationDepartureTrafficSpeed(f, 150); tc.clears && limit < 150 {
							t.Fatalf("safe stopped follower blocked departure: %.1f", limit)
						}
					})
				}
			})
		}
	}
}
