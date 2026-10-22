package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func leftPathMatrixWorld(incoming, outgoing int, busLane bool, h float64) (*sim.World, *sim.TrafficLightSystem) {
	u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	end := u.Mul(4000)
	if incoming == 2 {
		end = sim.Vec2{}
	}
	kind := sim.RoadSegment{P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: outgoing}
	if busLane {
		kind.Type = sim.RoadBusBoulevard8Lane
	}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: end, Lanes: incoming}, kind}}
	if incoming == 2 {
		w.Roads = append(w.Roads, sim.RoadSegment{P1: u.Mul(5000).Add(r.Mul(5000)), P2: u.Mul(5500).Add(r.Mul(5000)), Lanes: 2})
	}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	return w, tls
}

func TestLeftTurnRetainsPathUntilRearClearsEveryRoadSize(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, inLanes := range []int{2, 4, 6, 8} {
			for _, dest := range []struct {
				lanes int
				bus   bool
			}{{2, false}, {4, false}, {6, false}, {8, false}, {8, true}} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("w%.0f/in%d/out%d/bus%v/h%.2f", lw, inLanes, dest.lanes, dest.bus, h), func(t *testing.T) {
						w, tls := leftPathMatrixWorld(inLanes, dest.lanes, dest.bus, h)
						it := tls.Intersections[0]
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						out := r.Mul(-1)
						car := &sim.TrafficCar{Pos: u.Mul(-it.GetStopLineDistForHeading(h, lw) - 30).Add(r.Mul(lw / 2)), Heading: h, Speed: 85, Length: 46, Width: 22, IsEW: math.Abs(u.X) > .5}
						f := carFrame{car: car, world: w, tls: tls, laneWidth: lw}
						car.JunctionTurn = planCarJunctionTurn(f, it)
						if car.JunctionTurn == nil || car.JunctionTurn.Straight {
							t.Fatal("left turn not planned")
						}
						car.Pos = car.JunctionTurn.Points[0]
						for tick := 0; tick < 1800 && car.JunctionTurn != nil; tick++ {
							moveCarThroughJunction(car, 1./60)
						}
						if car.JunctionTurn != nil {
							t.Fatal("turn did not finish")
						}
						g := w.JunctionAt(it.Center, lw)
						half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
						if rear := car.Pos.Sub(it.Center).Dot(out) - car.Length/2; rear < half {
							t.Fatalf("turn protection ended inside box: rear %.1f box edge %.1f", rear, half)
						}
						if car.LaneIndex != min(1, dest.lanes/2-1) || math.Abs(sim.NormalizeAngle(car.Heading-(h-math.Pi/2))) > .001 {
							t.Fatal("incorrect outgoing lane or direction")
						}
					})
				}
			}
		}
	}
}

func TestOpposingLeftCarsClearEveryRoadSize(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, inLanes := range []int{4, 6, 8} {
			for _, outLanes := range []int{2, 4, 6, 8} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("w%.0f/in%d/out%d/h%.2f", lw, inLanes, outLanes, h), func(t *testing.T) {
						w, tls := leftPathMatrixWorld(inLanes, outLanes, false, h)
						it := tls.Intersections[0]
						it.Phase = sim.PhaseEWLeftGreen
						if math.Abs(math.Sin(h)) > .5 {
							it.Phase = sim.PhaseNSLeftGreen
						}
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						p := u.Mul(-it.GetStopLineDistForHeading(h, lw) - 40).Add(r.Mul(lw / 2))
						cars := []*sim.TrafficCar{{ID: 1, Pos: p, Heading: h, Speed: 85, TargetSpeed: 85, Length: 46, Width: 22, IsEW: math.Abs(u.X) > .5}, {ID: 2, Pos: p.Mul(-1), Heading: sim.NormalizeAngle(h + math.Pi), Speed: 85, TargetSpeed: 85, Length: 46, Width: 22, IsEW: math.Abs(u.X) > .5}}
						tm := &TrafficManager{Cars: cars}
						for tick := 0; tick < 1800; tick++ {
							for i, c := range cars {
								tm.StepCar(1./60, c, i, nil, tls, 2, lw, w)
							}
							if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: cars[0].Pos, Heading: cars[0].Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: cars[1].Pos, Heading: cars[1].Heading, HalfLen: 23, HalfWid: 11}); hit {
								t.Fatalf("opposing left contact tick%d", tick)
							}
						}
						if cars[0].Pos.Dot(r.Mul(-1)) < 700 || cars[1].Pos.Dot(r) < 700 {
							t.Fatalf("opposing left stuck: %v %.1f / %v %.1f", cars[0].Pos, cars[0].Speed, cars[1].Pos, cars[1].Speed)
						}
					})
				}
			}
		}
	}
}

func TestLeftAndStraightPhaseHandoffEveryRoadSize(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, inLanes := range []int{2, 4, 6, 8} {
			for _, dest := range []struct {
				lanes int
				bus   bool
			}{{2, false}, {4, false}, {6, false}, {8, false}, {8, true}} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("w%.0f/in%d/out%d/bus%v/h%.2f", lw, inLanes, dest.lanes, dest.bus, h), func(t *testing.T) {
						w, tls := leftPathMatrixWorld(inLanes, dest.lanes, dest.bus, h)
						it := tls.Intersections[0]
						horizontal := math.Abs(math.Sin(h)) < .5
						leftPhase, straightPhase, crossPhase := sim.PhaseEWLeftGreen, sim.PhaseEWGreen, sim.PhaseNSGreen
						if !horizontal {
							leftPhase, straightPhase, crossPhase = sim.PhaseNSLeftGreen, sim.PhaseNSGreen, sim.PhaseEWGreen
						}
						it.Phase = leftPhase
						if inLanes == 2 {
							it.Phase = straightPhase
						}
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						out := r.Mul(-1)
						makeCar := func(id, lane int, dir sim.Vec2) *sim.TrafficCar {
							heading := math.Atan2(dir.Y, dir.X)
							right := sim.Vec2{X: -dir.Y, Y: dir.X}
							return &sim.TrafficCar{ID: id, Pos: dir.Mul(-it.GetStopLineDistForHeading(heading, lw) - 40).Add(right.Mul((float64(lane) + .5) * lw)), Heading: heading, LaneIndex: lane, TargetSpeed: 100, Speed: 60, Length: 46, Width: 22, IsEW: math.Abs(dir.X) > .5}
						}
						left := makeCar(1, 0, u)
						crossLane := 0
						if dest.lanes >= 4 {
							crossLane = 1
						}
						cross := makeCar(2, crossLane, out)
						cars := []*sim.TrafficCar{left, cross}
						directions := []sim.Vec2{out, out}
						if inLanes >= 4 {
							cars = append(cars, makeCar(3, 1, u))
							directions = append(directions, u)
						} else {
							cars = append(cars, makeCar(3, crossLane, out.Mul(-1)))
							directions = append(directions, out.Mul(-1))
						}
						tm := &TrafficManager{Cars: cars}
						for tick := 0; tick < 3000; tick++ {
							if tick == 120 {
								it.Phase = straightPhase
							}
							if tick == 900 {
								it.Phase = crossPhase
							}
							for i, c := range cars {
								tm.StepCar(1./60, c, i, nil, tls, 3, lw, w)
							}
							for i, a := range cars {
								for _, b := range cars[i+1:] {
									if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: 23, HalfWid: 11}, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 23, HalfWid: 11}); hit {
										t.Fatalf("left/through collision tick%d cars%d/%d", tick, a.ID, b.ID)
									}
								}
							}
						}
						for i, c := range cars {
							if c.Pos.Dot(directions[i]) < 700 {
								t.Fatalf("left/through deadlock car%d pos%v speed%.1f", c.ID, c.Pos, c.Speed)
							}
						}
					})
				}
			}
		}
	}
}

func TestStraightWidthChangesKeepLaneAndDirection(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, inLanes := range []int{2, 4, 6, 8} {
			for _, dest := range []struct {
				lanes int
				bus   bool
			}{{2, false}, {4, false}, {6, false}, {8, false}, {8, true}} {
				for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
					t.Run(fmt.Sprintf("w%.0f/in%d/out%d/bus%v/h%.2f", lw, inLanes, dest.lanes, dest.bus, h), func(t *testing.T) {
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						exitRoad := sim.RoadSegment{P1: sim.Vec2{}, P2: u.Mul(4000), Lanes: dest.lanes}
						if dest.bus {
							exitRoad.Type = sim.RoadBusBoulevard8Lane
						}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: sim.Vec2{}, Lanes: inLanes}, exitRoad, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 4}}}
						w.RebuildIntersections()
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						it := tls.Intersections[0]
						it.Phase = sim.PhaseEWGreen
						if math.Abs(u.Y) > .5 {
							it.Phase = sim.PhaseNSGreen
						}
						lane := min(2, inLanes/2-1)
						car := &sim.TrafficCar{Pos: u.Mul(-it.GetStopLineDistForHeading(h, lw) - 60).Add(r.Mul((float64(lane) + .5) * lw)), Heading: h, LaneIndex: lane, Length: 46, Width: 22, Speed: 100, TargetSpeed: 100, IsEW: math.Abs(u.X) > .5}
						tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
						for tick := 0; tick < 1500; tick++ {
							prev := car.Pos
							tm.StepCar(1./60, car, 0, nil, tls, 1, lw, w)
							if car.Pos.Distance(prev) > car.Speed/60+.001 {
								t.Fatal("lane transition slid sideways")
							}
						}
						expected := min(lane, exitRoad.GeneralTrafficLanes()-1)
						if car.Pos.Dot(u) < 700 || car.LaneIndex != expected || math.Abs(car.Pos.Dot(r)-(float64(expected)+.5)*lw) > .01 || math.Abs(sim.NormalizeAngle(car.Heading-h)) > .001 {
							t.Fatalf("straight width change failed: pos%v heading%.2f lane%d expected%d", car.Pos, car.Heading, car.LaneIndex, expected)
						}
					})
				}
			}
		}
	}
}
