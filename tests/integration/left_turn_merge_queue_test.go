package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestLeftTurnMergeWaitsForQueuedVehicleInsteadOfPushing verifies that when an AI bus
// approaches an intersection and merges into the dedicated left-turn lane,
// it brakes and stops behind any vehicle already queued in the left-turn lane,
// waiting patiently at speed 0 without pushing or nudging the front vehicle.
func TestLeftTurnMergeWaitsForQueuedVehicleInsteadOfPushing(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("lw%.0f/berserk%t", lw, berserk), func(t *testing.T) {
				// 4-lane East-West road intersecting a 4-lane North-South road
				w := &sim.World{
					Roads: []sim.RoadSegment{
						{P1: sim.Vec2{X: -2000, Y: 0}, P2: sim.Vec2{X: 2000, Y: 0}, Lanes: 4},
						{P1: sim.Vec2{X: 0, Y: -2000}, P2: sim.Vec2{X: 0, Y: 2000}, Lanes: 4},
					},
				}
				w.RebuildIntersections()

				// Stop 0 on EW road heading East, Stop 1 on NS road heading North (requires left turn at X=0, Y=0)
				from := sim.NewBusStop(0, "WestStop", sim.Vec2{X: -1200, Y: 1.5 * lw}, 0.0, 4, sim.RGBA(0, 0, 0, 255), lw)
				to := sim.NewBusStop(1, "NorthStop", sim.Vec2{X: -0.5 * lw, Y: -1200}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{from, to}

				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				// Red light for EW traffic (NS is Green)
				if len(tls.Intersections) > 0 {
					tls.Intersections[0].Phase = sim.PhaseNSGreen
				}

				inter := tls.Intersections[0]
				stopLineDist := inter.GetStopLineDistForHeading(0.0, lw)

				// Lead bus (AI#1) waiting in dedicated left-turn lane (Lane 0: Y = 0.5 * lw)
				leadBusPos := sim.Vec2{X: -stopLineDist - 70.0, Y: 0.5 * lw}
				leadBus := sim.NewBus(leadBusPos.X, leadBusPos.Y, 0.0)
				leadBus.Speed = 0.0

				// Following bus (AI#3) approaching from behind in the merge zone
				// Placed angled towards the left turn lane, just behind leadBus
				followingBusPos := sim.Vec2{X: leadBusPos.X - 110.0, Y: 0.8 * lw}
				followingBus := sim.NewBus(followingBusPos.X, followingBusPos.Y, 0.15) // slight angle towards lane 0
				followingBus.Speed = 20.0

				driver := autopilot.NewAutopilotDriver()
				driver.BerserkMode = berserk
				driver.State = autopilot.AutopilotStateCruising
				driver.PrepareLeg(w, from, to, lw)

				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

				leadObstacle := autopilot.VehicleObstacle{
					Pos:     leadBus.Pos,
					Heading: leadBus.Heading,
					Speed:   leadBus.Speed,
					Length:  leadBus.Length,
					Width:   leadBus.Width,
				}

				// Step simulation: following bus must come to a stop behind lead bus
				stopped := false
				for step := 0; step < 300; step++ {
					obs := []autopilot.VehicleObstacle{leadObstacle}
					th, br, st, hb, doors, _ := driver.ComputeControls(followingBus, route, w, tls, obs, lw, 1.0/60)
					followingBus.Update(1.0/60, th, br, st, hb, doors)

					// Verify NO physical collision with lead bus
					b1 := sim.OBB{Center: followingBus.Pos, Heading: followingBus.Heading, HalfLen: followingBus.Length / 2, HalfWid: followingBus.Width / 2}
					b2 := sim.OBB{Center: leadBus.Pos, Heading: leadBus.Heading, HalfLen: leadBus.Length / 2, HalfWid: leadBus.Width / 2}
					if hit, _, _ := sim.CheckOBBCollision(b1, b2); hit {
						t.Fatalf("step %d: following bus collided with / pushed lead bus! Pos: %v, leadPos: %v", step, followingBus.Pos, leadBus.Pos)
					}

					if math.Abs(followingBus.Speed) < 0.1 && followingBus.Pos.Distance(leadBus.Pos) < 160.0 {
						stopped = true
						// When stopped, bus must NOT be applying throttle to push the lead vehicle
						if th > 0.01 {
							t.Fatalf("step %d: following bus is stopped behind lead vehicle but still applying throttle=%.2f to push!", step, th)
						}
					}
				}

				if !stopped {
					t.Fatalf("following bus never stopped behind queued lead bus; speed=%.2f, pos=%v", followingBus.Speed, followingBus.Pos)
				}

				// Phase 2: Signal turns Green for EW left turns, lead vehicle departs
				inter.Phase = sim.PhaseEWLeftGreen
				resumed := false
				for step := 0; step < 400; step++ {
					// Lead bus has cleared and moved far ahead through the intersection
					leadObstacle.Pos.X += 5.0
					obs := []autopilot.VehicleObstacle{leadObstacle}
					th, br, st, hb, doors, _ := driver.ComputeControls(followingBus, route, w, tls, obs, lw, 1.0/60)
					followingBus.Update(1.0/60, th, br, st, hb, doors)

					if followingBus.Speed > 10.0 && followingBus.Pos.X > leadBusPos.X-30.0 {
						resumed = true
						break
					}
				}

				if !resumed {
					t.Fatalf("following bus failed to resume driving after lead vehicle departed; speed=%.2f, pos=%v", followingBus.Speed, followingBus.Pos)
				}
			})
		}
	}
}
