package integration_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestQueuedRightMergeWithActualYieldingTraffic(t *testing.T) {
	for _, fleet := range []bool{false, true} {
		for _, lw := range []float64{42, 52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("fleet%v/lw%.0f/h%.2f", fleet, lw, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2500), P2: u.Mul(2500), Lanes: 6}, {P1: r.Mul(-2500), P2: r.Mul(2500), Lanes: 4}}}
					w.RebuildIntersections()
					from := sim.NewBusStop(0, "from", u.Mul(-1700), h, 6, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", r.Mul(1700), h+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					red, green := sim.PhaseNSGreen, sim.PhaseEWGreen
					if math.Abs(u.Y) > .5 {
						red, green = sim.PhaseEWGreen, sim.PhaseNSGreen
					}
					it.Phase = red
					line := it.GetStopLineDistForHeading(h, lw)
					start := -line - 490
					p := u.Mul(start).Add(r.Mul(1.5*lw - 8))
					b := sim.NewBus(p.X, p.Y, h+.13)
					ad := autopilot.NewAutopilotDriver()
					ad.BerserkMode = true
					ad.BerserkMaxSpeed = 667
					ad.PrepareLeg(w, from, to, lw)
					ad.LaneOffset = -lw
					ad.TargetLaneOffset = -lw
					ad.OvertakeState = autopilot.OvertakeReturning
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					var cars []*sim.TrafficCar
					for i, spot := range []struct {
						along, lateral float64
						lane           int
					}{{368, 2.5, 2}, {255, 1.5, 1}, {-202, 2.5, 2}} {
						target := 0.
						if i == 2 {
							target = 100
						}
						cars = append(cars, &sim.TrafficCar{ID: i, Pos: u.Mul(start + spot.along).Add(r.Mul(spot.lateral * lw)), Heading: h, Length: 48, Width: 22, TargetSpeed: target, LaneIndex: spot.lane, IsEW: math.Abs(u.X) > .5})
					}
					tm := &traffic.TrafficManager{Cars: cars}
					var mgr *traffic.AIBusManager
					if fleet {
						mgr = &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 1, Bus: b, Driver: ad, Route: route, LastExchangedStopID: -1}}}
						tm.AIBusMgr = mgr
					}
					merged, yielded := false, false
					for tick := 0; tick < 3600; tick++ {
						if merged {
							it.Phase = green
							cars[0].TargetSpeed = 100
							cars[1].TargetSpeed = 100
						}
						player := b
						if fleet {
							player = nil
						}
						for i, c := range cars {
							tm.StepCar(1.0/60, c, i, player, tls, len(cars), lw, w)
						}
						yielded = yielded || cars[2].YieldingToBus == b
						before := b.Pos
						if fleet {
							mgr.Update(1.0/60, nil, w, tls, tm, lw, 1)
						} else {
							var obs []autopilot.VehicleObstacle
							for _, c := range cars {
								obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: 48, Width: 22, YieldingTo: c.YieldingToBus})
							}
							th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1.0/60)
							b.Update(1.0/60, th, br, st, hb, doors)
						}
						if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 {
							t.Fatal("teleport")
						}
						for _, c := range cars {
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 24, HalfWid: 11}); hit {
								t.Fatalf("contact tick%d bus%v car%v", tick, b.Pos, c.Pos)
							}
						}
						if math.Abs(b.Pos.Dot(r)-2.5*lw) < 6 && math.Abs(sim.NormalizeAngle(b.Heading-h)) < .15 {
							merged = true
						}
						if tick == 900 && !merged {
							t.Fatalf("stalled before forward queue was released: bus%v speed%.1f target%.1f", b.Pos, b.Speed, ad.LastTargetSpeed)
						}
						if b.Pos.Dot(r) > line+300 && math.Sin(b.Heading-h) > .9 {
							if !merged || !yielded {
								t.Fatal("missing merge or rear yield")
							}
							return
						}
					}
					t.Fatalf("did not resume after queue moved: bus%v speed%.1f", b.Pos, b.Speed)
				})
			}
		}
	}
}
