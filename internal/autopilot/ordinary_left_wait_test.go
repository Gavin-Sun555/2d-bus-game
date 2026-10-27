package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// The captured player was 1.9px off the inner left-lane center on the real
// 214 -> 282 leg. A following bus 134px behind blocked the passing-gap check
// forever, although neither the signal nor forward obstacle check required a stop.
func TestOrdinaryLeftQueueLeaderRestarts(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, fleet := range []bool{false, true} {
			for _, berserk := range []bool{false, true} {
				t.Run(fmt.Sprintf("lw%.0f/fleet%v/berserk%v", lw, fleet, berserk), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					var it *sim.TrafficIntersection
					for _, candidate := range tls.Intersections {
						if candidate.Center.Distance(sim.Vec2{X: 15600, Y: 5400}) < 1 {
							it = candidate
							break
						}
					}
					if it == nil {
						t.Fatal("missing captured junction")
					}
					ad := NewAutopilotDriver()
					ad.BerserkMode = berserk
					ad.DisallowOncomingOvertake = fleet
					ad.BerserkMaxSpeed = 667
					ad.PrepareLeg(w, w.Stops[213], w.Stops[281], lw)
					ad.ordinaryBusLeft = true
					ad.PrepareLeg(w, w.Stops[213], w.Stops[281], lw)
					ad.holdLaneOffset(-1.9)
					line := it.GetStopLineDistForHeading(0, lw)
					pos := sim.Vec2{X: it.Center.X - line - 62, Y: it.Center.Y + .5*lw - 1.9}
					bus := sim.NewBus(pos.X, pos.Y, .0074)
					route := &sim.Route{StopIDs: []int{213, 281}, CurrentIndex: 1}
					obs := []VehicleObstacle{{Pos: pos.Add(sim.Vec2{X: -134, Y: 7}), Heading: -.18, Length: 96, Width: 36}}
					it.Phase = sim.PhaseEWGreen
					for tick := 0; tick < 180; tick++ {
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						if bus.Pos.X+bus.Length/2 > it.Center.X-line+1 {
							t.Fatal("used through green for left turn")
						}
					}
					// A save made while stuck must recover without resetting the bus/route.
					restored, err := ad.Snapshot().Restore(w)
					if err != nil {
						t.Fatal(err)
					}
					ad = restored
					it.Phase = sim.PhaseEWLeftGreen
					it.Timer = 0
					moved := false
					for tick := 0; float64(tick)/60 < it.LeftGreenDuration; tick++ {
						it.Timer = float64(tick) / 60
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						if bus.Pos.X+bus.Length/2 > it.Center.X-line+2 {
							moved = true
							break
						}
					}
					if !moved {
						t.Fatalf("missed left arrow with rear queue: pos%v hold%.2f diagnosis%+v", bus.Pos, ad.busTurnHoldDistance, ad.DiagnoseStop(bus, route, w, tls, obs, lw))
					}
					if math.Abs(ad.LaneOffset) > 3 {
						t.Fatalf("did not recenter: %.2f", ad.LaneOffset)
					}
					cleared := false
					for tick := 0; tick < 1500; tick++ {
						tls.Update(1. / 60)
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						if bus.Pos.Y < it.Center.Y-w.JunctionAt(it.Center, lw).HalfY-bus.Length && math.Cos(bus.Heading+math.Pi/2) > .95 {
							cleared = true
							break
						}
					}
					if !cleared {
						t.Fatalf("left turn stalled after admission at %v speed%.1f", bus.Pos, bus.Speed)
					}
				})
			}
		}
	}
}
