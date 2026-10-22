package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestFastCarsPassAndReturnSmoothly(t *testing.T) {
	for _, lanes := range []int{4, 6, 8} {
		for _, busLane := range []bool{false, true} {
			if busLane && lanes != 8 {
				continue
			}
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("lanes%d/bus%v/h%.2f", lanes, busLane, h), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					road := sim.RoadSegment{P1: u.Mul(-20000), P2: u.Mul(20000), Lanes: lanes}
					if busLane {
						road.Type = sim.RoadBusBoulevard8Lane
					}
					w := &sim.World{Roads: []sim.RoadSegment{road}}
					lane := min(1, lanes/2-1)
					side := (float64(lane) + .5) * 52
					fast := &sim.TrafficCar{ID: 1, Pos: r.Mul(side), Heading: h, Length: 48, Width: 22, IsEW: math.Abs(u.X) > .5, LaneIndex: lane, Speed: sim.SpeedKPH(60), TargetSpeed: sim.SpeedKPH(60), MaxSpeedKPH: 60, CanOvertake: true}
					slow := &sim.TrafficCar{ID: 2, Pos: fast.Pos.Add(u.Mul(250)), Heading: h, Length: 48, Width: 22, IsEW: fast.IsEW, LaneIndex: lane, Speed: sim.SpeedKPH(30), TargetSpeed: sim.SpeedKPH(30), MaxSpeedKPH: 30}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{fast, slow}}
					passed, changed := false, false
					for tick := 0; tick < 1800; tick++ {
						before := fast.Pos
						tm.StepCar(1./60, slow, 1, nil, nil, 2, 52, w)
						tm.StepCar(1./60, fast, 0, nil, nil, 2, 52, w)
						if fast.JunctionTurn != nil && fast.JunctionTurn.LaneChange {
							changed = true
						}
						if fast.Pos.Distance(before) > fast.Speed/60+.001 {
							t.Fatal("lane change slid sideways")
						}
						if fast.Speed > fast.TargetSpeed+.001 {
							t.Fatal("exceeded fixed assigned speed")
						}
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: fast.Pos, Heading: fast.Heading, HalfLen: 24, HalfWid: 11}, sim.OBB{Center: slow.Pos, Heading: slow.Heading, HalfLen: 24, HalfWid: 11}); hit {
							t.Fatalf("passing contact at tick%d", tick)
						}
						if fast.Pos.Sub(slow.Pos).Dot(u) > 200 && fast.LaneIndex == lane && fast.JunctionTurn == nil && !fast.Passing {
							passed = true
							break
						}
					}
					if !passed || !changed {
						t.Fatalf("did not pass and return: fast%v slow%v lane%d passing%v", fast.Pos, slow.Pos, fast.LaneIndex, fast.Passing)
					}
				})
			}
		}
	}
}

func TestCarPassingRespectsMinimumTierAndOccupiedGap(t *testing.T) {
	for _, scenario := range []string{"minimum tier", "occupied gap", "near junction"} {
		t.Run(scenario, func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 4}, {P1: sim.Vec2{X: 1200, Y: -5000}, P2: sim.Vec2{X: 1200, Y: 5000}, Lanes: 4}}}
			w.RebuildIntersections()
			fast := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{Y: 78}, Heading: 0, Length: 48, Width: 22, IsEW: true, LaneIndex: 1, Speed: 250, TargetSpeed: 300, CanOvertake: scenario != "minimum tier"}
			lead := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: 200, Y: 78}, Heading: 0, Length: 48, Width: 22, Speed: 100}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{fast, lead}}
			f := carFrame{car: fast, world: w, index: 0, activeCount: 2, laneWidth: 52}
			if scenario == "occupied gap" {
				tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: 3, Pos: sim.Vec2{Y: 26}, Heading: 0, Length: 48, Width: 22, Speed: 300})
				f.activeCount = 3
			}
			if scenario == "near junction" {
				f.tls = sim.NewTrafficLightSystem()
				f.tls.BuildForWorld(w)
			}
			tm.prepareCarPassing(f)
			if fast.JunctionTurn != nil || fast.Passing {
				t.Fatal("unsafe or disabled passing started")
			}
		})
	}
}

func TestFastCarBrakesForStationaryLeader(t *testing.T) {
	for _, kph := range []int{60, 80, 100} {
		t.Run(fmt.Sprint(kph), func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 2}}}
			car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{Y: 26}, Heading: 0, Length: 48, Width: 22, IsEW: true, TargetSpeed: sim.SpeedKPH(kph), Speed: sim.SpeedKPH(kph)}
			lead := &sim.TrafficCar{ID: 2, Pos: sim.Vec2{X: 1200, Y: 26}, Heading: 0, Length: 48, Width: 22}
			tm := &TrafficManager{Cars: []*sim.TrafficCar{car, lead}}
			for tick := 0; tick < 1800; tick++ {
				tm.StepCar(1./60, car, 0, nil, nil, 2, 52, w)
				if car.Pos.X+48 > lead.Pos.X {
					t.Fatal("high speed car collided with stopped leader")
				}
			}
			if car.Speed > 1 {
				t.Fatalf("did not stop: %.1f", car.Speed)
			}
		})
	}
}

func TestTwoLaneCarsFollowWithoutBorrowing(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, occupied := range []bool{false, true} {
			t.Run(fmt.Sprintf("heading%.2f/oncoming%v", h, occupied), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				right := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-10000), P2: u.Mul(10000), Lanes: 2}}}
				fast := &sim.TrafficCar{ID: 1, Pos: right.Mul(26), Heading: h, Length: 48, Width: 22, IsEW: math.Abs(u.X) > .5, CanOvertake: true, Speed: 300, TargetSpeed: 333}
				lead := &sim.TrafficCar{ID: 2, Pos: fast.Pos.Add(u.Mul(250)), Heading: h, Length: 48, Width: 22, IsEW: fast.IsEW, Speed: 166, TargetSpeed: 166}
				tm := &TrafficManager{Cars: []*sim.TrafficCar{fast, lead}}
				if occupied {
					tm.Cars = append(tm.Cars, &sim.TrafficCar{ID: 3, Pos: u.Mul(1200).Sub(right.Mul(26)), Heading: h + math.Pi, Length: 48, Width: 22, Speed: 300})
				}
				for tick := 0; tick < 1200; tick++ {
					tm.StepCar(1./60, lead, 1, nil, nil, len(tm.Cars), 52, w)
					tm.StepCar(1./60, fast, 0, nil, nil, len(tm.Cars), 52, w)
					if fast.Passing || fast.JunctionTurn != nil || fast.LaneIndex != 0 || math.Abs(fast.Pos.Dot(right)-26) > .01 {
						t.Fatalf("borrowed opposing lane at tick %d: %+v", tick, fast)
					}
					if lead.Pos.Sub(fast.Pos).Dot(u) < 48 {
						t.Fatal("failed to follow safely")
					}
				}
			})
		}
	}
}
