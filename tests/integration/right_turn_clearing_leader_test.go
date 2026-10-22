package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestBerserkRightMergeResumesAfterCurbBusTurns(t *testing.T) {
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
					it := tls.Intersections[0]
					it.Phase = sim.PhaseEWGreen
					if math.Abs(u.Y) > .5 {
						it.Phase = sim.PhaseNSGreen
					}
					from := sim.NewBusStop(0, "from", u.Mul(-3000), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", r.Mul(3000), h+math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					line := it.GetStopLineDistForHeading(h, lw)
					curb := (float64(lanes/2) - .5) * lw
					p := u.Mul(-line - 100).Add(r.Mul(curb - lw))
					b := sim.NewBus(p.X, p.Y, h)
					p = u.Mul(-line - 30).Add(r.Mul(curb))
					leader := sim.NewBus(p.X, p.Y, h)
					ad := autopilot.NewAutopilotDriver()
					ad.BerserkMode = true
					ad.PrepareLeg(w, from, to, lw)
					ad.LaneOffset = -lw
					ad.TargetLaneOffset = -lw
					ad.OvertakeState = autopilot.OvertakePassing
					ld := autopilot.NewAutopilotDriver()
					lr := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					rear := &sim.TrafficCar{ID: 1, Pos: u.Mul(-line - 260).Add(r.Mul(curb)), Heading: h, Length: 46, Width: 22, LaneIndex: lanes/2 - 1, TargetSpeed: 100, IsEW: math.Abs(u.X) > .5}
					tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{rear}}
					merged := false
					for tick := 0; tick < 3600; tick++ {
						tm.StepCar(1./60, rear, 0, b, tls, 1, lw, w)
						if tick >= 120 {
							obs := []autopilot.VehicleObstacle{{Pos: b.Pos, Heading: b.Heading, Length: 96, Width: 36, Speed: b.Speed, JunctionCorridor: b.JunctionCorridor}}
							th, br, st, hb, d, _ := ld.ComputeControls(leader, lr, w, tls, obs, lw, 1./60)
							leader.Update(1./60, th, br, st, hb, d)
						}
						obs := []autopilot.VehicleObstacle{{Pos: leader.Pos, Heading: leader.Heading, Speed: leader.Speed, Length: 96, Width: 36, JunctionCorridor: leader.JunctionCorridor}, {Pos: rear.Pos, Heading: rear.Heading, Speed: rear.Speed, Length: 46, Width: 22, YieldingTo: rear.YieldingToBus}}
						th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
						b.Update(1./60, th, br, st, hb, d)
						if math.Abs(b.Pos.Dot(r)-curb) < 6 && math.Abs(sim.NormalizeAngle(b.Heading-h)) < .15 {
							merged = true
						}
						for _, body := range []sim.OBB{{Center: leader.Pos, Heading: leader.Heading, HalfLen: 48, HalfWid: 18}, {Center: rear.Pos, Heading: rear.Heading, HalfLen: 23, HalfWid: 11}} {
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, body); hit {
								t.Fatalf("merge contact tick%d bus%v leader%v", tick, b.Pos, leader.Pos)
							}
						}
						if b.Pos.Dot(r) > line+300 {
							if !merged {
								t.Fatal("turned from incorrect lane")
							}
							return
						}
					}
					ownOb := []autopilot.VehicleObstacle{{Pos: leader.Pos, Heading: leader.Heading, Speed: leader.Speed, Length: 96, Width: 36, JunctionCorridor: leader.JunctionCorridor}, {Pos: rear.Pos, Heading: rear.Heading, Speed: rear.Speed, Length: 46, Width: 22, YieldingTo: rear.YieldingToBus}}
					leadOb := []autopilot.VehicleObstacle{{Pos: b.Pos, Heading: b.Heading, Speed: b.Speed, Length: 96, Width: 36, JunctionCorridor: b.JunctionCorridor}}
					od, ldg := ad.DiagnoseStop(b, route, w, tls, ownOb, lw), ld.DiagnoseStop(leader, lr, w, tls, leadOb, lw)
					t.Logf("own limits%v hold%.1f blocked%v; leader limits%v blocked%v", od.Limits, od.TurnHoldDistance, od.Blockers, ldg.Limits, ldg.Blockers)
					t.Fatalf("right merge stuck: bus%v h%.2f leader%v speed%.1f offset%.1f/%.1f", b.Pos, b.Heading, leader.Pos, ad.LastTargetSpeed, ad.LaneOffset, ad.TargetLaneOffset)
				})
			}
		}
	}
}
