package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestArticulatedTurnsAndDocking(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, turn := range []float64{-math.Pi / 2, math.Pi / 2, math.Pi} {
					t.Run(fmt.Sprintf("w%.0f/lanes%d/h%.2f/turn%.2f", lw, lanes, h, turn), func(t *testing.T) {
						in := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -in.Y, Y: in.X}
						out := sim.Vec2{X: math.Cos(h + turn), Y: math.Sin(h + turn)}
						rotate := func(p sim.Vec2) sim.Vec2 { return in.Mul(p.X).Add(right.Mul(p.Y)) }
						typ := sim.RoadStreet2Lane
						if lanes == 8 {
							typ = sim.RoadBusBoulevard8Lane
						}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -2500}), P2: rotate(sim.Vec2{X: 2500}), Lanes: lanes, Type: typ}, {P1: rotate(sim.Vec2{Y: -2500}), P2: rotate(sim.Vec2{Y: 2500}), Lanes: lanes, Type: typ}}}
						w.RebuildIntersections()
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", in.Mul(-1500), h, lanes, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", out.Mul(1500), h+turn, lanes, sim.RGBA(0, 0, 0, 255), lw)}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						sim.EnsureLongBusBays(w, route)
						b := sim.NewBus(w.Stops[0].Pos.X, w.Stops[0].Pos.Y, h)
						b.SetModel(sim.BusArticulated)
						ad := NewAutopilotDriver()
						ad.DisallowOncomingOvertake = true
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						docked := false
						maxBend := 0.
						restored := false
						for tick := 0; tick < 6000; tick++ {
							tls.Update(1. / 60)
							th, br, steer, hb, doors, dock := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
							b.Update(1./60, th, br, steer, hb, doors)
							maxBend = math.Max(maxBend, math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())))
							if !restored && maxBend > .2 {
								data, err := json.Marshal(ad.Snapshot())
								if err != nil {
									t.Fatal(err)
								}
								var snap DriverSnapshot
								if err = json.Unmarshal(data, &snap); err != nil {
									t.Fatal(err)
								}
								ad, err = snap.Restore(w)
								if err != nil {
									t.Fatal(err)
								}
								if !ad.articulatedPath {
									t.Fatal("lost long vehicle path on load")
								}
								restored = true
							}
							if r := b.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
								t.Fatalf("body struck curb tick%d front%v heading%.2f tail%v tailH%.2f barrier%s", tick, b.Pos, b.Heading, b.TrailerBody().Center, b.TailHeading(), r.ObstacleName)
							}
							if dock {
								docked = true
								break
							}
						}
						if !docked {
							t.Fatalf("did not dock front%v h%.2f tail%v tailH%.2f bend%.2f diag%+v", b.Pos, b.Heading, b.TrailerBody().Center, b.TailHeading(), maxBend, ad.DiagnoseStop(b, route, w, tls, nil, lw))
						}
						if maxBend < .05 {
							t.Fatal("trailer never articulated")
						}
					})
				}
			}
		}
	}
}

func TestArticulatedLaneReturnKeepsTailClear(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, berserk := range []bool{false, true} {
				for _, blocked := range []bool{false, true} {
					t.Run(fmt.Sprintf("w%.0f/h%.2f/berserk%v/blocked%v", lw, h, berserk, blocked), func(t *testing.T) {
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
						w.RebuildIntersections()
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-1600), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2000), h, 8, sim.RGBA(0, 0, 0, 255), lw)}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						sim.EnsureLongBusBays(w, route)
						p := u.Mul(-700).Add(r.Mul(2.5 * lw))
						b := sim.NewBus(p.X, p.Y, h)
						b.SetModel(sim.BusArticulated)
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						ad.DisallowOncomingOvertake = true
						ad.CanSkipTargetStop = true
						ad.articulatedPath = true
						ad.drivePath = []sim.Vec2{u.Mul(-1600).Add(r.Mul(3.5 * lw)), u.Mul(2000).Add(r.Mul(3.5 * lw))}
						ad.cumLen = []float64{0, 3600}
						ad.pathWorld = w
						ad.pathFromID = 0
						ad.pathToID = 1
						ad.pathLane = lw
						ad.pathSkipTarget = true
						ad.pathRevision = currentDrivePathRevision
						ad.bypassStopID = 1
						ad.holdLaneOffset(-lw)
						var obs []VehicleObstacle
						if blocked {
							obs = []VehicleObstacle{{Pos: u.Mul(-700).Add(r.Mul(3.5 * lw)), Heading: h, Length: 46, Width: 22}}
						}
						settled := false
						for tick := 0; tick < 3600; tick++ {
							th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, obs, lw, 1./60)
							b.Update(1./60, th, br, st, hb, doors)
							for _, ob := range obs {
								if hit, _, _ := b.CollidesBody(sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
									t.Fatal("lane return struck car with tail", tick, b.Pos, b.TrailerBody())
								}
							}
							if b.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
								t.Fatalf("lane return hit road edge tick%d front%v h%.3f tail%+v error%+v", tick, b.Pos, b.Heading, b.TrailerBody(), b.CheckBoundaries(w, lw))
							}
							aligned := true
							for _, part := range b.BodyParts() {
								if math.Abs(part.Center.Dot(r)-3.5*lw) > 5 || math.Abs(sim.NormalizeAngle(part.Heading-h)) > .07 {
									aligned = false
								}
							}
							if aligned && b.Pos.Dot(u) > -100 {
								settled = true
								break
							}
						}
						if !settled {
							t.Fatalf("return stuck at%v h%.2f tail%v diagnosis%+v", b.Pos, b.Heading, b.TrailerBody(), ad.DiagnoseStop(b, route, w, nil, obs, lw))
						}
					})
				}
			}
		}
	}
}

func TestArticulatedRoundaboutTurns(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, turn := range []float64{-math.Pi / 2, 0, math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/turn%.2f", lw, heading, turn), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					rb := w.Roundabouts[0]
					forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					out := sim.Vec2{X: math.Cos(heading + turn), Y: math.Sin(heading + turn)}
					w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", rb.Center.Sub(forward.Mul(900)), heading, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", rb.Center.Add(out.Mul(900)), heading+turn, 4, sim.RGBA(0, 0, 0, 255), lw)}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					pos := rb.Center.Sub(forward.Mul(550)).Add(right.Mul(1.5 * lw))
					bus := sim.NewBus(pos.X, pos.Y, heading)
					bus.SetModel(sim.BusArticulated)
					sim.EnsureLongBusBays(w, route)
					bus.Speed = 90
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					entered := false
					for step := 0; step < 2400; step++ {
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1.0/60)
						bus.Update(1.0/60, th, br, st, hb, doors)
						if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
							t.Fatalf("tail hit roundabout boundary step%d pos%v tail%+v", step, bus.Pos, bus.TrailerBody())
						}
						if math.Abs(ad.LaneOffset) > .01 || math.Abs(ad.TargetLaneOffset) > .01 {
							t.Fatalf("empty approach changed lanes at step%d: %.2f/%.2f pos%v", step, ad.LaneOffset, ad.TargetLaneOffset, bus.Pos)
						}
						radius := bus.Pos.Distance(rb.Center)
						if radius <= rb.Radius+15 {
							entered = true
							if radius < (rb.InnerRadius+16+rb.Radius)/2 {
								t.Fatalf("moved into inner ring: radius%.1f", radius)
							}
						}
						if entered && bus.Pos.Sub(rb.Center).Dot(out) > rb.Radius+220 {
							return
						}
					}
					t.Fatalf("failed to pass roundabout pos%v entered%v", bus.Pos, entered)
				})
			}
		}
	}
}
