package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkApproachUsesRunwayAndKeepsRealLeaders(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, left := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/left%v", heading, left), func(t *testing.T) {
				const lw = 52.
				u := sim.UnitFromAngle(heading)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{
					{P1: u.Mul(-10000), P2: u.Mul(10000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
					{P1: r.Mul(-10000), P2: r.Mul(10000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
				}}
				w.RebuildIntersections()
				pos := u.Mul(-4000).Add(r.Mul(1.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				target := 3.5 * lw
				if left {
					target = .5 * lw
				}
				request := &sim.BusLaneMergeRequest{Target: r.Mul(target), Heading: heading, LeftTurn: left, RightTurn: !left}
				f := autopilotFrame{bus: bus, world: w, laneWidth: lw}
				far := ad.approachManeuverSpeed(f, request)
				if far < 230 {
					t.Fatalf("premature crawl with ample runway: %.1f", far)
				}
				// A blocked target-lane queue can be passed in the current lane;
				// the same-distance leader in our actual lane must still stop us.
				ad.busTurnHoldDistance = 3000
				ad.approachMerge = request
				if held := ad.approachManeuverSpeed(f, request); held < 600 {
					t.Fatal("blocked far merge slowed clear current lane", held)
				}
				f.obstacles = []VehicleObstacle{{Pos: pos.Add(u.Mul(90)).Add(r.Mul(target - 1.5*lw)), Heading: heading, Length: 48, Width: 22}}
				if limit := ad.requiredMergeTrafficSpeed(f); !math.IsInf(limit, 1) {
					t.Fatal("adjacent queue restricted held current lane", limit)
				}
				f.obstacles[0].Pos = pos.Add(u.Mul(90))
				if limit := ad.requiredMergeTrafficSpeed(f); limit > 40 {
					t.Fatal("ignored actual current-lane leader", limit)
				}
				bus.Pos = u.Mul(-350).Add(r.Mul(1.5 * lw))
				if near := ad.approachManeuverSpeed(f, request); near > 80 || near >= far {
					t.Fatal("did not slow for mandatory merge zone", near)
				}
				if ad.approachTrafficRequest(f, request) != request {
					t.Fatal("ignored target-lane traffic near stop line")
				}
			})
		}
	}
}

func TestBerserkAbortedPassReleasesCrawlInsideCommittedLane(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("model%d/h%.2f", model, heading), func(t *testing.T) {
				const lw = 52.
				u := sim.UnitFromAngle(heading)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-10000), P2: u.Mul(10000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				for id, along := range []float64{-9000, 9000} {
					w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", u.Mul(along), heading, 8, sim.RGBA(0, 0, 0, 255), lw))
				}
				pos := u.Mul(-2000).Add(r.Mul(2.5*lw + 5))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.SetModel(model)
				bus.Speed = 20
				ad := NewAutopilotDriver()
				ad.BerserkMode, ad.CanSkipTargetStop = true, true
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: r.Mul(2.5 * lw), Heading: heading}
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for tick := 0; tick < 120; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if tick == 0 && ad.midLaneRecovery != nil {
						t.Fatal("kept crawl after whole body regained its lane")
					}
					for _, part := range bus.BodyParts() {
						if part.HalfLen <= 0 {
							continue
						}
						for _, corner := range sim.BodyCorners(part) {
							if side := corner.Dot(r); side < 2*lw || side > 3*lw {
								t.Fatalf("left committed lane during acceleration: %.1f", side)
							}
						}
					}
				}
				if bus.Speed < 100 {
					t.Fatalf("failed to accelerate after aborted pass: %.1f", bus.Speed)
				}
			})
		}
	}
}

func TestBerserkEarlyCurbMergeKeepsBodyOnRoad(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("w%.0f/model%d/h%.2f", lw, model, heading), func(t *testing.T) {
					u := sim.UnitFromAngle(heading)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					w := medianRightTestWorld(lw, heading)
					pos := u.Mul(-2600).Add(r.Mul(1.5 * lw))
					bus := sim.NewBus(pos.X, pos.Y, heading)
					bus.SetModel(model)
					bus.Speed = 667
					ad := NewAutopilotDriver()
					ad.BerserkMode, ad.CanSkipTargetStop = true, true
					ad.BerserkMaxSpeed = 667
					ad.medianRightChoice = &sim.Vec2{}
					ad.curbBusRight = true
					ad.articulatedPath = bus.IsArticulated()
					ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
					ad.approachMerge = &sim.BusLaneMergeRequest{Target: r.Mul(3.5 * lw), Heading: heading, RightTurn: true}
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
					for tick := 0; tick < 1800; tick++ {
						th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
						bus.Update(1./60, th, br, st, hb, doors)
						for _, part := range bus.BodyParts() {
							if part.HalfLen <= 0 {
								continue
							}
							for _, corner := range sim.BodyCorners(part) {
								if side := corner.Dot(r); side < -.5 || side > 4*lw+.5 {
									t.Fatalf("merge left carriageway at tick%d side%.1f pos%v yaw%.3f tail%.3f wheel%.3f speed%.1f target%.1f", tick, side, bus.Pos, sim.NormalizeAngle(bus.Heading-heading), sim.NormalizeAngle(bus.TrailerHeading-heading), bus.SteeringAngle, bus.Speed, ad.LastTargetSpeed)
								}
							}
						}
						if ad.approachMergeJoined && bus.Speed > 160 {
							t.Logf("settled and accelerating in %.2fs", float64(tick+1)/60)
							return
						}
					}
					t.Fatalf("did not complete early curb merge: pos%v speed%.1f", bus.Pos, bus.Speed)
				})
			}
		}
	}
}

func TestBerserkEarlyLeftMergeKeepsBodyOnRoad(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
			t.Run(fmt.Sprintf("model%d/h%.2f", model, heading), func(t *testing.T) {
				const lw = 52.
				u := sim.UnitFromAngle(heading)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := medianRightTestWorld(lw, heading)
				w.CenterBRT = false
				for i := range w.Roads {
					w.Roads[i].MedianBusLane = false
				}
				w.Stops[1] = sim.NewBusStop(1, "left exit", r.Mul(-2200), heading-math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops[0].CenterBRT = false
				w.Stops[0].UpdateGeometry(lw)
				pos := u.Mul(-2600).Add(r.Mul(1.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.SetModel(model)
				bus.Speed = 667
				ad := NewAutopilotDriver()
				ad.BerserkMode, ad.CanSkipTargetStop = true, true
				ad.BerserkMaxSpeed = 667
				ad.articulatedPath = bus.IsArticulated()
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for tick := 0; tick < 1800; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					for _, part := range bus.BodyParts() {
						if part.HalfLen <= 0 {
							continue
						}
						for _, corner := range sim.BodyCorners(part) {
							if side := corner.Dot(r); side < -.5 || side > 4*lw+.5 {
								t.Fatalf("left merge left carriageway at tick%d side%.1f", tick, side)
							}
						}
					}
					if wholeBusInMergeLane(bus, &sim.BusLaneMergeRequest{Target: r.Mul(.5 * lw), Heading: heading}, lw) && ad.ordinaryBusLeft {
						return
					}
				}
				t.Fatalf("failed to join median left lane: pos%v speed%.1f", bus.Pos, bus.Speed)
			})
		}
	}
}
