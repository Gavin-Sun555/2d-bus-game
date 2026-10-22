package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestEightLaneRightMergeFollowsHeading(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, speed := range []float64{90, sim.SpeedKPH(100)} {
				t.Run(fmt.Sprintf("width%.0f/h%.2f/speed%.0f", lw, heading, speed), func(t *testing.T) {
					w, tls := eightLaneWorld(sim.RoadBusBoulevard8Lane)
					inter := tls.Intersections[0]
					inter.Phase = sim.PhaseEWGreen
					if math.Abs(math.Sin(heading)) > .5 {
						inter.Phase = sim.PhaseNSGreen
					}
					u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					car := &sim.TrafficCar{ID: 3, Heading: heading, Speed: speed, TargetSpeed: speed, Length: 46, Width: 22, LaneIndex: 2, IsEW: math.Abs(u.X) > .5}
					stop := inter.GetStopLineDistForHeading(heading, lw)
					car.Pos = u.Mul(-stop - carLaneChangeLength(car, lw) - 150).Add(r.Mul(2.5 * lw))
					tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
					merging, rotated, joined, turned := false, false, false, false
					for tick := 0; tick < 2400; tick++ {
						before := car.Pos
						oldHeading := car.Heading
						tm.StepCar(1./60, car, 0, nil, tls, 1, lw, w)
						delta := car.Pos.Sub(before)
						if delta.Length() > car.Speed/60+.001 {
							t.Fatalf("sideways translation tick%d: moved%.3f allowed%.3f", tick, delta.Length(), car.Speed/60)
						}
						if car.JunctionTurn != nil && car.JunctionTurn.LaneChange {
							merging = true
							if car.LaneIndex != 2 {
								t.Fatal("lane identity changed before merge completed")
							}
							yaw := sim.NormalizeAngle(car.Heading - heading)
							if yaw > .08 {
								rotated = true
							}
							if delta.Length() > .01 && delta.Normalize().Dot(sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}) < .995 {
								t.Fatal("motion disagrees with heading")
							}
							if math.Abs(sim.NormalizeAngle(car.Heading-oldHeading)) > .08 {
								t.Fatal("abrupt heading change")
							}
						}
						if merging && !joined && car.JunctionTurn == nil && car.LaneIndex == 3 {
							joined = true
							if -car.Pos.Dot(u) < stop+car.Length/2+20 {
								t.Fatal("merge completed too close to line")
							}
							if math.Abs(car.Pos.Dot(r)-3.5*lw) > .01 || math.Abs(sim.NormalizeAngle(car.Heading-heading)) > .01 {
								t.Fatal("merge did not settle in target lane")
							}
						}
						if joined && car.Pos.Dot(r) > 700 {
							turned = true
							break
						}
					}
					if !merging || !rotated || !joined || !turned {
						t.Fatalf("incomplete: merge%v rotated%v joined%v turned%v pos%v", merging, rotated, joined, turned, car.Pos)
					}
				})
			}
		}
	}
}

func TestEightLaneRightMergeNeedsRunway(t *testing.T) {
	w, tls := eightLaneWorld(sim.RoadBusBoulevard8Lane)
	stop := tls.Intersections[0].GetStopLineDistForHeading(0, 52)
	car := &sim.TrafficCar{ID: 3, Pos: sim.Vec2{X: -stop - 100, Y: 130}, Heading: 0, Speed: 90, TargetSpeed: 90, Length: 46, Width: 22, LaneIndex: 2, IsEW: true}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	before := car.Pos
	tm.prepareEightLaneApproach(carFrame{car: car, world: w, tls: tls, laneWidth: 52})
	if car.JunctionTurn != nil || car.LaneIndex != 2 || car.Pos != before {
		t.Fatal("late right merge forced into the junction")
	}
}

func TestEightLaneReservedLaneReturnIsCurved(t *testing.T) {
	w, _ := eightLaneWorld(sim.RoadBusBoulevard8Lane)
	car := &sim.TrafficCar{ID: 1, Pos: sim.Vec2{X: -1300, Y: 182}, Heading: 0, Speed: 90, Length: 46, Width: 22, LaneIndex: 3, IsEW: true}
	tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
	f := carFrame{car: car, world: w, laneWidth: 52}
	blocker := sim.NewBus(-1300, 130, 0)
	f.buses = []*sim.Bus{blocker}
	tm.prepareEightLaneApproach(f)
	if car.JunctionTurn != nil {
		t.Fatal("return ignored occupied lane")
	}
	f.buses = nil
	tm.prepareEightLaneApproach(f)
	if car.JunctionTurn == nil || !car.JunctionTurn.LaneChange || car.JunctionTurn.ExitLane != 2 || car.LaneIndex != 3 {
		t.Fatal("return did not use curve follower")
	}
	rotated := false
	for tick := 0; tick < 600 && car.JunctionTurn != nil; tick++ {
		before := car.Pos
		moveCar(car, 1./60)
		tm.handleCarNavigation(car, 52, w)
		if car.Pos.Distance(before) > car.Speed/60+.001 {
			t.Fatal("return slid sideways")
		}
		rotated = rotated || car.Heading < -.08
	}
	if !rotated || car.LaneIndex != 2 || math.Abs(car.Pos.Y-130) > .01 || math.Abs(car.Heading) > .01 {
		t.Fatalf("return failed: %+v", car)
	}
}
