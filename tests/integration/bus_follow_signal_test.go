package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusLaneSignalsFollowActualManeuver(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, turn := range []int{-1, 0, 1} {
				t.Run(fmt.Sprintf("width%.0f/heading%.2f/turn%d", lw, h, turn), func(t *testing.T) {
					w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
					w.RebuildIntersections()
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					n := sim.Vec2{X: -u.Y, Y: u.X}
					outH := h + float64(turn)*math.Pi/2
					out := sim.Vec2{X: math.Cos(outH), Y: math.Sin(outH)}
					from := sim.NewBusStop(0, "from", u.Mul(-2000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
					to := sim.NewBusStop(1, "to", out.Mul(2000), outH, 8, sim.RGBA(0, 0, 0, 255), lw)
					w.Stops = []*sim.BusStop{from, to}
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					ad := autopilot.NewAutopilotDriver()
					ad.BerserkMode = true
					ad.BerserkMaxSpeed = 667
					ad.CanSkipTargetStop = true
					b := sim.NewBus(0, 0, h)
					b.Pos = u.Mul(-1000).Add(n.Mul(3.5 * lw))
					b.Speed = 150
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					thru, left := sim.PhaseEWGreen, sim.PhaseEWLeftGreen
					if math.Abs(u.Y) > .5 {
						thru, left = sim.PhaseNSGreen, sim.PhaseNSLeftGreen
					}
					line := it.GetStopLineDistForHeading(h, lw)
					crossed := false
					for step := 0; step < 3600; step++ {
						it.Phase = left
						if turn == -1 {
							it.Phase = thru
						}
						if turn == 1 {
							it.Phase = sim.PhaseNSGreen
							if math.Abs(u.Y) > .5 {
								it.Phase = sim.PhaseEWGreen
							}
						}
						if step >= 600 {
							it.Phase = thru
							if turn == -1 {
								it.Phase = left
							}
						}
						th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, nil, lw, 1.0/60)
						b.Update(1.0/60, th, br, st, hb, doors)
						if b.Pos.Dot(u)+b.Length/2 > -line {
							crossed = true
							if turn != 1 && step < 600 {
								t.Fatalf("crossed red at step%d pos%v heading%.2f target%.1f", step, b.Pos, b.Heading, ad.LastTargetSpeed)
							}
						}
						if b.Pos.Dot(out) > 900 {
							if !crossed {
								t.Fatal("exit without crossing")
							}
							return
						}
					}
					t.Fatalf("did not clear legal signal: pos%v heading%.2f target%.1f", b.Pos, b.Heading, ad.LastTargetSpeed)
				})
			}
		}
	}
}

func TestBerserkBusFollowsBrakingAndRestartingBus(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", h), func(t *testing.T) {
			const lw = 52.0
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			n := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-10000), P2: u.Mul(10000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
			from := sim.NewBusStop(0, "from", u.Mul(-5000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
			to := sim.NewBusStop(1, "to", u.Mul(8000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
			w.Stops = []*sim.BusStop{from, to}
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			ad := autopilot.NewAutopilotDriver()
			ad.BerserkMode = true
			ad.BerserkMaxSpeed = 667
			ad.CanSkipTargetStop = true
			b := sim.NewBus(0, 0, h)
			b.Pos = u.Mul(-1100).Add(n.Mul(3.5 * lw))
			b.Speed = 445
			lead := sim.NewBus(0, 0, h)
			lead.Pos = u.Mul(-500).Add(n.Mul(3.5 * lw))
			lead.Speed = 120
			stopped := false
			minMoving := math.Inf(1)
			for step := 0; step < 1800; step++ {
				th, br := 0.0, 0.0
				if step >= 360 && step < 720 {
					br = 1
				} else if lead.Speed < 120 {
					th = .5
				} else {
					th = .15
				}
				lead.Update(1.0/60, th, br, 0, false, false)
				obs := []autopilot.VehicleObstacle{{Pos: lead.Pos, Heading: h, Speed: lead.Speed, Length: lead.Length, Width: lead.Width}}
				// Occupied adjacent lanes prevent overtaking; the player must follow.
				for lane := 0; lane < 3; lane++ {
					obs = append(obs, autopilot.VehicleObstacle{Pos: b.Pos.Add(n.Mul((float64(lane) - 3) * lw)), Heading: h, Speed: b.Speed, Length: 48, Width: 22})
				}
				th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, obs, lw, 1.0/60)
				b.Update(1.0/60, th, br, st, hb, doors)
				gap := lead.Pos.Sub(b.Pos).Dot(u) - (lead.Length+b.Length)/2
				if gap < 8 {
					t.Fatalf("rear-end at step%d gap%.1f speed%.1f lead%.1f", step, gap, b.Speed, lead.Speed)
				}
				if step > 540 && step < 720 && b.Speed < 1 {
					stopped = true
				}
				if step > 660 && step < 720 && b.Speed > 1 {
					t.Fatalf("restarted into stopped leader at step%d speed%.1f", step, b.Speed)
				}
				if step > 1500 {
					minMoving = math.Min(minMoving, b.Speed)
				}
			}
			if !stopped {
				t.Fatal("never settled behind stopped bus")
			}
			if minMoving < 90 {
				t.Fatalf("repeated braking or failure to resume: min speed %.1f", minMoving)
			}
		})
	}
}
