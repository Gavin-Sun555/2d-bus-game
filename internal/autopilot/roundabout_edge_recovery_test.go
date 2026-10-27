package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func edgeRecoveryFixture(angle float64) (autopilotFrame, *AutopilotDriver) {
	rotate := func(v sim.Vec2) sim.Vec2 {
		return sim.Vec2{X: v.X*math.Cos(angle) - v.Y*math.Sin(angle), Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle)}
	}
	w := &sim.World{Roundabouts: []sim.Roundabout{{Radius: 220, InnerRadius: 105}}, Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -4000}, P2: sim.Vec2{X: 4000}, Lanes: 4}, {P1: sim.Vec2{Y: -4000}, P2: sim.Vec2{Y: 4000}, Lanes: 4}}}
	p := rotate(sim.Vec2{X: 92.97266528722096, Y: 189.79780309488922})
	b := sim.NewBus(p.X, p.Y, -.2422724161869267+angle)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = .5192670464630142 + angle
	b.SteeringAngle = -b.MaxSteerAngle
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	return autopilotFrame{bus: b, world: w, laneWidth: 52}, ad
}

func TestRoundaboutEdgeRecoveryClearsCurbWithoutReset(t *testing.T) {
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(angle), func(t *testing.T) {
			f, ad := edgeRecoveryFixture(angle)
			start := f.bus.Pos
			if !ad.tryRoundaboutEdgeRecovery(f, -1) {
				t.Fatal("safe short adjustment not planned")
			}
			if ad.roundaboutEdgeRecovery.Distance > 10 {
				t.Fatal("unbounded reverse")
			}
			t.Logf("planned reverse %.1f", ad.roundaboutEdgeRecovery.Distance)
			reverseSteps := 0
			for tick := 0; tick < 600; tick++ {
				if tick == 80 {
					data, err := json.Marshal(ad.Snapshot())
					if err != nil {
						t.Fatal(err)
					}
					var snapshot DriverSnapshot
					if err = json.Unmarshal(data, &snapshot); err != nil {
						t.Fatal(err)
					}
					ad, err = snapshot.Restore(f.world)
					if err != nil {
						t.Fatal(err)
					}
				}
				before := f.bus.Pos
				th, br, st := ad.roundaboutEdgeControls(f)
				f.bus.Update(1./60, th, br, st, false, false)
				if f.bus.Speed < 0 {
					reverseSteps++
				}
				if f.bus.Pos.Distance(before) > math.Abs(f.bus.Speed)/60+.01 {
					t.Fatal("pose reset")
				}
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatal("recovery crossed curb")
				}
				if ad.roundaboutEdgeRecovery == nil {
					break
				}
			}
			if ad.roundaboutEdgeRecovery != nil || reverseSteps == 0 || start.Distance(f.bus.Pos) > 10.1 || f.bus.Gear != sim.GearDrive {
				t.Fatalf("adjustment did not finish: position%v gear%v", f.bus.Pos, f.bus.Gear)
			}
			forward := 0.
			for tick := 0; tick < 600 && forward < 55; tick++ {
				before := f.bus.Pos
				th, br := ad.longitudinalControls(f.bus.Speed, 16)
				f.bus.Update(1./60, th, br, -1, false, false)
				forward += f.bus.Pos.Distance(before)
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatalf("forward turn remained blocked after%.2f reverse%.2f at%v", forward, start.Distance(f.bus.Pos), f.bus.Pos)
				}
			}
			if forward < 55 {
				t.Fatal("forward turn failed to resume")
			}
		})
	}
}

func TestRoundaboutEdgeRecoveryRejectsOccupiedRear(t *testing.T) {
	f, ad := edgeRecoveryFixture(0)
	tail := f.bus.TrailerBody()
	u := sim.Vec2{X: math.Cos(tail.Heading), Y: math.Sin(tail.Heading)}
	f.obstacles = []VehicleObstacle{{Pos: tail.Center.Sub(u.Mul(tail.HalfLen + 23 + 2)), Heading: tail.Heading, Length: 46, Width: 22}}
	if ad.tryRoundaboutEdgeRecovery(f, -1) {
		t.Fatal("reversed into stationary rear car")
	}
}

func TestRightTurnInLaneCorrectionDoesNotWaitForFollower(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
		b := sim.NewBus(0, 1.5*52+1.04, 0)
		b.SetModel(model)
		ad := NewAutopilotDriver()
		ad.LaneOffset = -1.04
		ad.TargetLaneOffset = -1.04
		ad.drivePath = []sim.Vec2{{X: -500, Y: 78}, {X: 500, Y: 78}}
		ad.cumLen = []float64{0, 1000}
		f := autopilotFrame{bus: b, laneWidth: 52, s: 500, obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: -140, Y: 78}, Heading: 0, Length: 46, Width: 22}}}
		ad.trackRightTurnWithoutPassingOffset(f)
		if ad.rightTurnWaiting || ad.LaneOffset != 0 {
			t.Fatalf("model%d in-lane correction reserved following gap", model)
		}
	}
}

func TestArticulatedTurnKeepsJunctionClearanceUntilTailExits(t *testing.T) {
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(angle), func(t *testing.T) {
			rotate := func(v sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: v.X*math.Cos(angle) - v.Y*math.Sin(angle), Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle)}
			}
			w := &sim.World{Intersections: []sim.Vec2{{}}, Roads: []sim.RoadSegment{{P1: rotate(sim.Vec2{X: -4000}), P2: rotate(sim.Vec2{X: 4000}), Lanes: 4}, {P1: rotate(sim.Vec2{Y: -4000}), P2: rotate(sim.Vec2{Y: 4000}), Lanes: 4}}}
			p := rotate(sim.Vec2{X: 7, Y: 275})
			b := sim.NewBus(p.X, p.Y, 1.9277549984362579+angle)
			b.SetModel(sim.BusArticulated)
			b.TrailerHeading = 1.052669196259934 + angle
			ad := NewAutopilotDriver()
			center := sim.Vec2{}
			ad.rightTurnJunction = &center
			ad.drivePath = []sim.Vec2{rotate(sim.Vec2{X: -78, Y: -2000}), rotate(sim.Vec2{X: -78, Y: 2000})}
			ad.cumLen = []float64{0, 4000}
			f := autopilotFrame{bus: b, world: w, laneWidth: 52, s: 2275}
			if actual := ad.roadLaneSafeSteering(f, .65); ad.roadSafetyHold || actual != .65 {
				t.Fatal("tail inside junction was treated as a straight-road median violation")
			}
			b.Pos = rotate(sim.Vec2{X: 7, Y: 775})
			f.s = 2775
			ad.roadLaneSafeSteering(f, .65)
			if !ad.roadSafetyHold {
				t.Fatal("median protection did not resume after full tail exited")
			}
		})
	}
}
