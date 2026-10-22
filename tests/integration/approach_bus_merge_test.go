package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestBusReturnsFromLeftOnlyLaneWithRearCarYielding(t *testing.T) {
	for _, fleet := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			for _, lanes := range []int{4, 6, 8} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("fleet%v/width%.0f/lanes%d/h%.2f", fleet, lw, lanes, h), func(t *testing.T) {
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -u.Y, Y: u.X}
						w := &sim.World{Intersections: []sim.Vec2{{}}, Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: lanes}, {P1: right.Mul(-2000), P2: right.Mul(2000), Lanes: 4}}}
						for i, x := range []float64{-1500, 1500} {
							w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, lanes, sim.RGBA(0, 0, 0, 255), lw))
						}
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						inter := tls.Intersections[0]
						red, green := sim.PhaseNSGreen, sim.PhaseEWGreen
						if math.Abs(u.Y) > .5 {
							red, green = sim.PhaseEWGreen, sim.PhaseNSGreen
						}
						inter.Phase = red
						line := inter.GetStopLineDistForHeading(h, lw)
						p := u.Mul(-line - 160).Add(right.Mul(.65 * lw))
						bus := sim.NewBus(p.X, p.Y, h+.14)
						ad := autopilot.NewAutopilotDriver()
						ad.BerserkMode = true
						ad.BerserkMaxSpeed = 667
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
						ad.LaneOffset = -(float64(lanes/2) - 1) * lw
						ad.TargetLaneOffset = ad.LaneOffset
						ad.OvertakeState = autopilot.OvertakePassing
						leadPos := p.Add(right.Mul(.85 * lw)).Add(u.Mul(100))
						rearPos := p.Add(right.Mul(.85 * lw)).Sub(u.Mul(110))
						lead := &sim.TrafficCar{ID: 1, Pos: leadPos, Heading: h, Speed: 80, TargetSpeed: 100, Length: 48, Width: 22, LaneIndex: 1, IsEW: math.Abs(u.X) > .5}
						rear := &sim.TrafficCar{ID: 2, Pos: rearPos, Heading: h, TargetSpeed: 100, Length: 48, Width: 22, LaneIndex: 1, IsEW: math.Abs(u.X) > .5}
						tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{lead, rear}}
						var manager *traffic.AIBusManager
						if fleet {
							manager = &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 1, Bus: bus, Driver: ad, Route: route, LastExchangedStopID: -1}}}
							tm.AIBusMgr = manager
						}
						yielded, merged, passed := false, false, false
						for tick := 0; tick < 1800; tick++ {
							if tick == 180 {
								inter.Phase = green
							}
							// Follow the same order as the actual game: cars observe the previous
							// tick's intent, then the bus observes their granted yield and speed.
							player := bus
							if fleet {
								player = nil
							}
							tm.Update(1.0/60, player, tls, 1, lw, w)
							yielded = yielded || rear.YieldingToBus == bus
							obs := []autopilot.VehicleObstacle{}
							for _, car := range tm.Cars {
								obs = append(obs, autopilot.VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width, YieldingTo: car.YieldingToBus})
							}
							before := bus.Pos
							if fleet {
								manager.Update(1.0/60, nil, w, tls, tm, lw, 1)
							} else {
								th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1.0/60)
								bus.Update(1.0/60, th, br, st, hb, doors)
							}
							expected := before.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(bus.Speed / 60))
							if bus.Pos.Distance(expected) > .01 {
								t.Fatal("merge relied on teleport/reset")
							}
							a := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
							for _, car := range tm.Cars {
								if hit, _, _ := sim.CheckOBBCollision(a, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
									t.Fatalf("merge collision tick%d bus%v car%v", tick, bus.Pos, car.Pos)
								}
							}
							nose := bus.Pos.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(bus.Length / 2))
							if nose.Dot(u) >= -line && before.Dot(u)+bus.Length/2 < -line && nose.Dot(right) < lw {
								t.Fatal("entered junction through left-only lane before completing required merge")
							}
							if tick < 180 && bus.Pos.Dot(u)+bus.Length/2 > -line {
								t.Fatal("merge ignored red signal")
							}
							if math.Abs(bus.Pos.Dot(right)-1.5*lw) < 5 && math.Cos(bus.Heading-h) > .99 {
								merged = true
							}
							if bus.Pos.Dot(u) > line+150 {
								passed = true
							}
							if passed && merged && rear.Pos.Dot(u) > line+50 && rear.YieldingToBus == nil {
								if !yielded {
									t.Fatal("rear car never granted priority")
								}
								return
							}
						}
						t.Fatalf("merge deadlock yielded%v merged%v passed%v bus%v v%.1f offset%.1f/%.1f rear%v v%.1f heading%.2f EW%v lane%d turn%+v", yielded, merged, passed, bus.Pos, bus.Speed, ad.LaneOffset, ad.TargetLaneOffset, rear.Pos, rear.Speed, rear.Heading, rear.IsEW, rear.LaneIndex, rear.JunctionTurn)
					})
				}
			}
		}
	}
}
