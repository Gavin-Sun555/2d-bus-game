package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestRightTurnReturnsToCurbWithRearQueueYielding(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, lw := range []float64{42, 52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, near := range []bool{false, true} {
					for _, fleet := range []bool{false, true} {
						t.Run(fmt.Sprintf("lanes%d/lw%.0f/h%.2f/near%v/fleet%v", lanes, lw, h, near, fleet), func(t *testing.T) {
							u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
							r := sim.Vec2{X: -u.Y, Y: u.X}
							w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2500), P2: u.Mul(2500), Lanes: lanes}, {P1: r.Mul(-2500), P2: r.Mul(2500), Lanes: 4}}}
							w.RebuildIntersections()
							from := sim.NewBusStop(0, "from", u.Mul(-1700), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
							to := sim.NewBusStop(1, "to", r.Mul(1700), h+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
							w.Stops = []*sim.BusStop{from, to}
							tls := sim.NewTrafficLightSystem()
							tls.BuildForWorld(w)
							it := tls.Intersections[0]
							it.Phase = sim.PhaseEWGreen
							if math.Abs(u.Y) > .5 {
								it.Phase = sim.PhaseNSGreen
							}
							line := it.GetStopLineDistForHeading(h, lw)
							lateral := .5 * lw
							if lanes == 2 {
								lateral = -.5 * lw
							}
							if near && lanes >= 6 {
								lateral = (float64(lanes/2) - 1.5) * lw
							}
							distance := 500.
							if near {
								distance = 35
							}
							p := u.Mul(-line - distance).Add(r.Mul(lateral))
							b := sim.NewBus(p.X, p.Y, h)
							ad := autopilot.NewAutopilotDriver()
							ad.BerserkMode = true
							ad.BerserkMaxSpeed = 667
							ad.PrepareLeg(w, from, to, lw)
							ad.LaneOffset = -(float64(lanes/2) - 1) * lw
							if lanes == 2 {
								ad.LaneOffset = -lw
							}
							ad.TargetLaneOffset = ad.LaneOffset
							ad.OvertakeState = autopilot.OvertakePassing
							curb := (float64(lanes/2) - .5) * lw
							car := &sim.TrafficCar{ID: 1, Pos: u.Mul(-line - distance - 110).Add(r.Mul(curb)), Heading: h, Length: 48, Width: 22, TargetSpeed: 100, LaneIndex: lanes/2 - 1, IsEW: math.Abs(u.X) > .5}
							tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}
							route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
							yielded, merged := false, false
							var blocker *sim.Bus
							if near {
								bp := u.Mul(-line - distance + 70).Add(r.Mul(curb))
								blocker = sim.NewBus(bp.X, bp.Y, h)
								blocker.Speed = 80
							}
							var manager *traffic.AIBusManager
							if fleet {
								manager = &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 1, Bus: b, Driver: ad, Route: route, LastExchangedStopID: -1}}}
								tm.AIBusMgr = manager
							}
							for tick := 0; tick < 3600; tick++ {
								if blocker != nil {
									blocker.Pos = blocker.Pos.Add(u.Mul(80.0 / 60))
								}
								player := b
								if fleet {
									player = blocker
								}
								tm.Update(1.0/60, player, tls, 1, lw, w)
								yielded = yielded || car.YieldingToBus == b
								obs := []autopilot.VehicleObstacle{{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: 48, Width: 22, YieldingTo: car.YieldingToBus}}
								if blocker != nil {
									obs = append(obs, autopilot.VehicleObstacle{Pos: blocker.Pos, Heading: h, Speed: 80, Length: 96, Width: 36})
								}
								before := b.Pos
								if fleet {
									manager.Update(1.0/60, blocker, w, tls, tm, lw, 1)
								} else {
									th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1.0/60)
									b.Update(1.0/60, th, br, st, hb, doors)
								}
								if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 {
									t.Fatal("teleport")
								}
								fwd := sim.Vec2{X: math.Cos(b.Heading), Y: math.Sin(b.Heading)}
								normal := sim.Vec2{X: -fwd.Y, Y: fwd.X}
								for _, a := range []float64{-1, 1} {
									for _, z := range []float64{-1, 1} {
										corner := b.Pos.Add(fwd.Mul(a * 48)).Add(normal.Mul(z * 18))
										if w.CheckBoundaries(corner, 0, lw).Zone == sim.ZoneHardBarrier {
											t.Fatalf("turn hit road edge tick%d pos%v heading%.2f corner%v", tick, b.Pos, b.Heading, corner)
										}
									}
								}
								if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
									t.Fatalf("collision tick%d bus%v car%v", tick, b.Pos, car.Pos)
								}
								if blocker != nil {
									if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: blocker.Pos, Heading: h, HalfLen: 48, HalfWid: 18}); hit {
										t.Fatalf("hit clearing bus tick%d", tick)
									}
								}
								if math.Abs(b.Pos.Dot(r)-curb) < 6 && math.Abs(sim.NormalizeAngle(b.Heading-h)) < .15 {
									merged = true
								}
								if !merged && math.Abs(sim.NormalizeAngle(b.Heading-h)) > 1.0 {
									t.Fatalf("turned before reaching curb tick%d bus%v yaw%.2f", tick, b.Pos, sim.NormalizeAngle(b.Heading-h))
								}
								if b.Pos.Dot(r) > line+300 && math.Sin(b.Heading-h) > .9 {
									if b.Pos.Dot(u)+math.Abs(fwd.Dot(u))*48+math.Abs(normal.Dot(u))*18 > 0 {
										t.Fatal("right turn exited into opposing traffic")
									}
									if !merged || !yielded {
										t.Fatalf("missing merge/yield merged%v yielded%v", merged, yielded)
									}
									return
								}
							}
							t.Fatalf("right turn stuck merged%v yielded%v bus%v heading%.2f speed%.1f offset%.1f/%.1f", merged, yielded, b.Pos, b.Heading, b.Speed, ad.LaneOffset, ad.TargetLaneOffset)
						})
					}
				}
			}
		}
	}
}
