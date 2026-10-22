package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRightTurnRequestsCurbLaneBeforeJunction(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("lanes%d/h%.2f", lanes, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				const lw = 52.
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2200), P2: u.Mul(2200), Lanes: lanes}, {P1: r.Mul(-2200), P2: r.Mul(2200), Lanes: 4}}}
				w.RebuildIntersections()
				a := sim.NewBusStop(0, "from", u.Mul(-1700), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
				z := sim.NewBusStop(1, "to", r.Mul(1700), h+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{a, z}
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, a, z, lw)
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
				lateral := .5 * lw
				if lanes == 2 {
					lateral = -.5 * lw
				}
				p := u.Mul(-line - 200).Add(r.Mul(lateral))
				b := sim.NewBus(p.X, p.Y, h)
				ad.trackPath(p)
				ad.LaneOffset = -(float64(lanes/2) - 1) * lw
				if lanes == 2 {
					ad.LaneOffset = -lw
				}
				ad.TargetLaneOffset = ad.LaneOffset
				ad.OvertakeState = OvertakePassing
				ad.updateOvertaking(b, w, tls, nil, lw, ad.LastPathS, 0, 2000, 1.0/60, z)
				if b.LaneMergeRequest == nil {
					t.Fatal("right turn did not request curb-lane merge")
				}
				want := (float64(lanes/2) - .5) * lw
				if got := b.LaneMergeRequest.Target.Dot(r); math.Abs(got-want) > .01 {
					t.Fatalf("target %.1f want%.1f", got, want)
				}
			})
		}
	}
}
