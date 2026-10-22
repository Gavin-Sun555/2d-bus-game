package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRightTurnReservesMergeRunwayBeforeQueue(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f", lw, lanes, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: lanes}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
					w.RebuildIntersections()
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					from := sim.NewBusStop(0, "from", u.Mul(-3000), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", r.Mul(3000), h+math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.PrepareLeg(w, from, to, lw)
					line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
					curb := (float64(lanes/2) - .5) * lw
					p := u.Mul(-line - 950).Add(r.Mul(curb - lw))
					b := sim.NewBus(p.X, p.Y, h)
					ad.trackPath(p)
					ad.LaneOffset = -lw
					ad.TargetLaneOffset = -lw
					ad.OvertakeState = OvertakePassing
					ad.updateOvertaking(b, w, tls, nil, lw, ad.LastPathS, 0, 4000, 1./60, to)
					if b.LaneMergeRequest == nil || !b.LaneMergeRequest.RightTurn {
						t.Fatal("right-turn lane return still begins too late")
					}
					if math.Abs(b.LaneMergeRequest.Target.Dot(r)-curb) > .01 {
						t.Fatal("incorrect curb target")
					}
				})
			}
		}
	}
}
