package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// A front bus stopped on/past the painted line must restart on through green
// even with a yawed articulated follower still finishing its bay departure.
func TestOverlineThroughGreenWithArticulatedFollower(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, over := range []float64{0, 5, 30} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/over%.0f", lw, h, over), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}, {P1: r.Mul(-3000), P2: r.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
					w.RebuildIntersections()
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-1500), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(1500), h, 8, sim.RGBA(0, 0, 0, 255), lw)}
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.Phase = sim.PhaseEWGreen
					if math.Abs(u.Y) > .5 {
						it.Phase = sim.PhaseNSGreen
					}
					p := u.Mul(-it.GetStopLineDistForHeading(h, lw) - 48 + over).Add(r.Mul(3.5 * lw))
					b := sim.NewBus(p.X, p.Y, h)
					rearPos := p.Sub(u.Mul(132)).Add(r.Mul(12))
					rear := sim.NewBus(rearPos.X, rearPos.Y, h-.16)
					rear.SetModel(sim.BusArticulated)
					rear.TrailerHeading = h - .05
					ad, following := NewAutopilotDriver(), NewAutopilotDriver()
					ad.DisallowOncomingOvertake = true
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					// Publish the follower's crossing intent without making it move into the leader.
					following.ComputeControls(rear, route, w, tls, []VehicleObstacle{ObserveBus(b)}, lw, 1./60)
					for step := 0; step < 240; step++ {
						obs := []VehicleObstacle{ObserveBus(rear)}
						th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
						b.Update(1./60, th, br, st, hb, d)
						if hit, _, _ := sim.CheckBusesCollision(b, rear); hit {
							t.Fatal("leader hit follower")
						}
						if b.Pos.Sub(p).Dot(u) > 100 {
							return
						}
					}
					t.Fatalf("missed through green: pos%v diagnosis%+v", b.Pos, ad.DiagnoseStop(b, route, w, tls, []VehicleObstacle{ObserveBus(rear)}, lw))
				})
			}
		}
	}
}
