package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusWaitsBeforeEntryUntilExitQueueClears(t *testing.T) {
	for _, movement := range []string{"straight", "left", "right"} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("%s/h%.2f", movement, h), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				out := u
				heading := h
				if movement == "left" {
					out = r.Mul(-1)
					heading = h - math.Pi/2
				}
				if movement == "right" {
					out = r
					heading = h + math.Pi/2
				}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-5000), P2: u.Mul(5000), Lanes: 6}, {P1: r.Mul(-5000), P2: r.Mul(5000), Lanes: 4}}}
				w.RebuildIntersections()
				a := sim.NewBusStop(0, "from", u.Mul(-4000), h, 6, sim.RGBA(0, 0, 0, 255), lw)
				lanes := 4
				if movement == "straight" {
					lanes = 6
				}
				z := sim.NewBusStop(1, "to", out.Mul(4000), heading, lanes, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{a, z}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				it := tls.Intersections[0]
				it.Phase = sim.PhaseEWGreen
				if movement == "left" {
					it.Phase = sim.PhaseEWLeftGreen
				}
				if math.Abs(u.Y) > .5 {
					if movement == "left" {
						it.Phase = sim.PhaseNSLeftGreen
					} else {
						it.Phase = sim.PhaseNSGreen
					}
				}
				line := it.GetStopLineDistForHeading(h, lw)
				side := 2.5 * lw
				if movement == "left" {
					side = .5 * lw
				}
				p := u.Mul(-line - 150).Add(r.Mul(side))
				bus := sim.NewBus(p.X, p.Y, h)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, a, z, lw)
				ad.trackPath(p)
				ad.publishJunctionCorridor(autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS})
				if bus.JunctionCorridor == nil {
					t.Fatal("missing route corridor")
				}
				bodies := bus.JunctionCorridor.Bodies
				last := bodies[len(bodies)-1]
				g := w.JunctionAt(sim.Vec2{}, lw)
				half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				q := last.Center.Add(out.Mul(half + 70 - last.Center.Dot(out)))
				obstacles := []VehicleObstacle{{Pos: q, Heading: heading, Length: 48, Width: 22}}
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for tick := 0; tick < 900; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.Pos.Dot(u)+48*math.Abs(math.Cos(bus.Heading-h))+18*math.Abs(math.Sin(bus.Heading-h)) > -line+.5 {
						t.Fatal("bus entered blocked junction")
					}
				}
				for tick := 0; tick < 2400; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.Pos.Dot(out) > half+250 {
						return
					}
				}
				t.Fatalf("bus did not resume after exit cleared: %v", bus.Pos)
			})
		}
	}
}
