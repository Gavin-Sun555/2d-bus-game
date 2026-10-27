package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// The 09:46 save's queue leader is +13.657px outside its curb route, with a
// following bus 132px back and the player beside that follower. This is the
// opposite sign from the earlier recentering cases and must not reserve a
// full passing gap from its own queue before it can release that queue.
func TestOuterCurbLeftLeaderRecentersWithFollowers(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, berserk := range []bool{false, true} {
				for _, initialOffset := range []float64{-.99, .25} {
					for _, followerYaw := range []float64{0, -.205, .205} {
						t.Run(fmt.Sprintf("w%.0f/h%.2f/berserk%v/offset%.2f/rearYaw%.3f", lw, h, berserk, initialOffset, followerYaw), func(t *testing.T) {
							w, in, _, _ := wideLeftExitWorld(lw, h, true)
							right := sim.Vec2{X: -in.Y, Y: in.X}
							tls := sim.NewTrafficLightSystem()
							tls.BuildForWorld(w)
							it := tls.Intersections[0]
							it.Phase = sim.PhaseEWGreen
							green := sim.PhaseEWLeftGreen
							if math.Abs(in.Y) > .5 {
								it.Phase = sim.PhaseNSGreen
								green = sim.PhaseNSLeftGreen
							}
							line := it.GetStopLineDistForHeading(h, lw)
							p := in.Mul(-line - 70).Add(right.Mul((3.5 + initialOffset) * lw))
							bus := sim.NewBus(p.X, p.Y, h-.01)
							ad := NewAutopilotDriver()
							ad.BerserkMode = berserk
							ad.DisallowOncomingOvertake = true
							ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
							ad.holdLaneOffset(initialOffset * lw)
							obs := []VehicleObstacle{
								{Pos: p.Sub(in.Mul(132)).Sub(right.Mul(initialOffset * lw)), Heading: h + followerYaw, Length: 96, Width: 36},
								{Pos: p.Sub(in.Mul(128)).Sub(right.Mul((1 + initialOffset) * lw)), Heading: h, Length: 96, Width: 36},
							}
							route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
							entered := false
							for tick := 0; float64(tick) < 180+it.LeftGreenDuration*60; tick++ {
								if tick == 180 {
									it.Phase = green
									it.Timer = 0
								}
								if tick >= 180 {
									it.Timer = float64(tick-180) / 60
								}
								for j := range obs {
									ob := &obs[j]
									follower := sim.NewBus(ob.Pos.X, ob.Pos.Y, ob.Heading)
									follower.Speed = ob.Speed
									if _, ok := sim.BusMergeYieldLimit(follower, bus, lw, ob.YieldingTo == bus); ok {
										ob.YieldingTo = bus
									} else {
										ob.YieldingTo = nil
									}
								}
								th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
								bus.Update(1./60, th, br, st, hb, doors)
								body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: 48, HalfWid: 18}
								for _, ob := range obs {
									if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
										t.Fatal("leader hit a follower while recentering")
									}
								}
								nose := bus.Pos.Dot(in) + bus.Length/2
								if tick < 180 && nose > -line+1 {
									t.Fatal("leader crossed before left arrow")
								}
								if tick >= 180 && nose > -line+2 {
									entered = true
									break
								}
							}
							if !entered {
								t.Fatalf("queue leader missed first arrow with free physical sweep: pos%v heading%.3f diagnosis%+v", bus.Pos, bus.Heading, ad.DiagnoseStop(bus, route, w, tls, obs, lw))
							}
						})
					}
				}
			}
		}
	}
}
