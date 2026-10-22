package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestGeneratedBusLaneStraightWaitsForThroughGreen(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("w%.0f/h%.2f", lw, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 4, Type: sim.RoadAvenue4Lane}}}
				w.RebuildIntersections()
				a := sim.NewBusStop(0, "from", u.Mul(-3000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
				z := sim.NewBusStop(1, "to", u.Mul(3000), h, 8, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{a, z}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				it := tls.Intersections[0]
				it.Phase = sim.PhaseEWLeftGreen
				if math.Abs(u.Y) > .5 {
					it.Phase = sim.PhaseNSLeftGreen
				}
				line := it.GetStopLineDistForHeading(h, lw)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.CanSkipTargetStop = true
				b := sim.NewBus(0, 0, h)
				b.Pos = u.Mul(-line - 900).Add(r.Mul(3.5 * lw))
				b.Speed = 300
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for step := 0; step < 1800; step++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if b.Pos.Dot(u)+b.Length/2*math.Cos(b.Heading-h) > -line+1 {
						t.Fatalf("crossed red: pos%v yaw%.2f target%.1f", b.Pos, b.Heading-h, ad.LastTargetSpeed)
					}
				}
			})
		}
	}
}

func TestStraightSignalIntentSurvivesLaneChangeYaw(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, yaw := range []float64{-.3, 0, .3} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/yaw%.1f", lw, h, yaw), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					tls := sim.NewTrafficLightSystem()
					w := busLaneTestWorld(lw)
					tls.BuildForWorld(w)
					it := tls.Intersections[0]
					it.ApproachLanes = [4]int{8, 8, 8, 8}
					it.Phase = sim.PhaseEWLeftGreen
					if math.Abs(u.Y) > .5 {
						it.Phase = sim.PhaseNSLeftGreen
					}
					line := it.GetStopLineDistForHeading(h, lw)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					setPath(ad, []sim.Vec2{u.Mul(-3000).Add(r.Mul(3.5 * lw)), u.Mul(3000).Add(r.Mul(3.5 * lw))})
					b := sim.NewBus(0, 0, h+yaw)
					b.Pos = u.Mul(-line - b.Length/2 - 8).Add(r.Mul(.7 * lw))
					ad.trackPath(b.Pos)
					f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS}
					if got := ad.trafficLightSpeed(f, 445); got != 0 {
						t.Fatalf("known straight movement used left green during merge: %.1f", got)
					}
				})
			}
		}
	}
}

func TestStraightSegmentExitBeforeLaterLeftTurn(t *testing.T) {
	ad := NewAutopilotDriver()
	setPath(ad, []sim.Vec2{{X: -500, Y: 182}, {X: 200, Y: 182}, {X: 1000, Y: -600}})
	turn, found := ad.intersectionExitTurn(sim.Vec2{}, 1, 0, 350, 218.4)
	if !found || turn != routeTurnStraight {
		t.Fatalf("near junction classified as later left turn: %v %v", turn, found)
	}
}

// Exercise steering and bus physics, starting in the passing/left-only lane.
// The entire bus must settle in a through lane well before the entry detector.
func TestRequiredThroughReturnSettlesBeforeStopLine(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, busLane := range []bool{false, true} {
			if busLane && lanes != 8 {
				continue
			}
			for _, lw := range []float64{42, 52, 78} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					for _, speed := range []float64{80, 445, 667} {
						t.Run(fmt.Sprintf("lanes%d/bus%v/w%.0f/h%.2f/speed%.0f", lanes, busLane, lw, h, speed), func(t *testing.T) {
							u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
							r := sim.Vec2{X: -u.Y, Y: u.X}
							road := sim.RoadSegment{P1: u.Mul(-5000), P2: u.Mul(5000), Lanes: lanes}
							if busLane {
								road.Type = sim.RoadBusBoulevard8Lane
							}
							w := &sim.World{Roads: []sim.RoadSegment{road, {P1: r.Mul(-5000), P2: r.Mul(5000), Lanes: 4}}}
							w.RebuildIntersections()
							a := sim.NewBusStop(0, "from", u.Mul(-4000), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
							z := sim.NewBusStop(1, "to", u.Mul(4000), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
							w.Stops = []*sim.BusStop{a, z}
							tls := sim.NewTrafficLightSystem()
							tls.BuildForWorld(w)
							it := tls.Intersections[0]
							it.Phase = sim.PhaseEWGreen
							if math.Abs(u.Y) > .5 {
								it.Phase = sim.PhaseNSGreen
							}
							line := it.GetStopLineDistForHeading(h, lw)
							ad := NewAutopilotDriver()
							ad.BerserkMode = true
							ad.BerserkMaxSpeed = 667
							ad.CanSkipTargetStop = false
							ad.PrepareLeg(w, a, z, lw)
							ad.State = AutopilotStateCruising
							b := sim.NewBus(0, 0, h)
							b.Pos = u.Mul(-line - 1000 - 250*float64(lanes/2-2)).Add(r.Mul(.5 * lw))
							if lanes == 2 {
								b.Pos = b.Pos.Sub(r.Mul(lw))
							}
							b.MaxSpeedFwd = 690
							b.Speed = speed
							if speed > 600 {
								b.Pos = b.Pos.Sub(u.Mul(1500))
							}
							ad.trackPath(b.Pos)
							ad.LaneOffset = b.Pos.Sub(ad.pointAt(ad.LastPathS)).Dot(r)
							ad.TargetLaneOffset = ad.LaneOffset
							ad.OvertakeState = OvertakePassing
							route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
							settled := false
							for step := 0; step < 2400; step++ {
								th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
								b.Update(1./60, th, br, st, hb, doors)
								yaw := sim.NormalizeAngle(b.Heading - h)
								legalSingle := lanes == 2 && math.Abs(b.Pos.Dot(r)-.5*lw)+b.Length/2*math.Abs(math.Sin(yaw))+b.Width/2*math.Abs(math.Cos(yaw)) < lw/2 && math.Abs(yaw) < .1
								lane := math.Round(b.Pos.Dot(r)/lw - .5)
								legalMulti := lanes >= 4 && lane >= 1 && lane < float64(lanes/2) && (lanes != 8 || busLane || lane < 3) && math.Abs(b.Pos.Dot(r)-(lane+.5)*lw)+b.Length/2*math.Abs(math.Sin(yaw))+b.Width/2*math.Abs(math.Cos(yaw)) < lw/2 && math.Abs(yaw) < .1
								if (ad.approachMergeJoined || legalSingle || legalMulti) && !settled {
									settled = true
									gap := -line - b.Pos.Dot(u) - b.Length/2
									if gap < 150 {
										t.Fatalf("returned too late: settled bumper gap %.1f", gap)
									}
								}
								if bad, msg := tls.CheckPlayerViolation(b, lw); bad {
									t.Fatalf("violation during return: %s", msg)
								}
								if b.Pos.Dot(u) > line+250 {
									if !settled {
										t.Fatalf("never settled before junction pos%v yaw%.2f offset%.1f/%.1f", b.Pos, yaw, ad.LaneOffset, ad.TargetLaneOffset)
									}
									return
								}
							}
							t.Fatalf("did not pass junction: pos%v target%.1f offset%.1f", b.Pos, ad.LastTargetSpeed, ad.LaneOffset)
						})
					}
				}
			}
		}
	}
}
