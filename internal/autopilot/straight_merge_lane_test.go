package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBlockedStraightMergeSettlesInLaneBeforeRetryingClearCurb(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			const lw = 52.
			forward := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			junction := forward.Mul(2000)
			w := &sim.World{Roads: []sim.RoadSegment{
				{P1: forward.Mul(-5000), P2: forward.Mul(6000), Lanes: 6, Type: sim.RoadExpressway6Lane},
				{P1: junction.Sub(right.Mul(3000)), P2: junction.Add(right.Mul(3000)), Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
			}}
			w.RebuildIntersections()
			w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", forward.Mul(-4000), heading, 6, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", forward.Mul(5000), heading, 6, sim.RGBA(0, 0, 0, 255), lw)}
			lights := sim.NewTrafficLightSystem()
			lights.BuildForWorld(w)
			for _, inter := range lights.Intersections {
				inter.Phase = sim.PhaseEWGreen
				if math.Abs(forward.Y) > .5 {
					inter.Phase = sim.PhaseNSGreen
				}
			}
			bus := sim.NewBus(0, 0, heading-.0374623878)
			bus.SetModel(sim.BusArticulated)
			bus.Pos = right.Mul(47.8138920606)
			bus.TrailerHeading = heading - .0716516056
			bus.SteeringAngle = .0611579639
			bus.Speed = 17.01643777
			ad := NewAutopilotDriver()
			ad.articulatedPath = true
			ad.BerserkMode = true
			ad.BerserkMaxSpeed = 1000
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			ad.holdLaneOffset(-82.1754858178)
			ad.approachMerge = &sim.BusLaneMergeRequest{Target: right.Mul(78), Heading: heading, Junction: junction, Remaining: .3}
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			var obstacles []VehicleObstacle
			for _, along := range []float64{1.5, 65.6, 129.6} {
				obstacles = append(obstacles, VehicleObstacle{Pos: forward.Mul(along).Add(right.Mul(78)), Heading: heading, Length: 46, Width: 22})
			}
			settled := false
			for tick := 0; tick < 3600; tick++ {
				th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, lights, obstacles, lw, 1./60)
				bus.Update(1./60, th, br, st, hb, d)
				for _, ob := range obstacles {
					if hit, _, _ := bus.CollidesBody(sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: 23, HalfWid: 11}); hit {
						t.Fatalf("hit adjacent car tick%d", tick)
					}
				}
				for _, part := range bus.BodyParts() {
					for _, corner := range sim.BodyCorners(part) {
						if corner.Dot(right) < -.1 {
							t.Fatalf("crossed median tick%d", tick)
						}
					}
				}
				request := &sim.BusLaneMergeRequest{Target: right.Mul(26), Heading: heading}
				if wholeBusInMergeLane(bus, request, lw) && math.Abs(bus.Pos.Dot(right)-26) < 4 {
					settled = true
					break
				}
			}
			if !settled {
				t.Fatalf("kept straddling: pos%v h%.3f target%.1f mid%v", bus.Pos, bus.Heading, ad.TargetLaneOffset, ad.midLaneRecovery)
			}
			passed := false
			for tick := 0; tick < 1200; tick++ {
				th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, lights, nil, lw, 1./60)
				bus.Update(1./60, th, br, st, hb, d)
				target := &sim.BusLaneMergeRequest{Target: right.Mul(130), Heading: heading}
				if wholeBusInMergeLane(bus, target, lw) && math.Abs(bus.Pos.Dot(right)-130) < 4 && bus.Pos.Dot(forward) > 600 {
					passed = true
					break
				}
			}
			if !passed {
				t.Fatalf("did not use clear curb lane pos%v speed%.1f approach%v mid%v", bus.Pos, bus.Speed, ad.approachMerge, ad.midLaneRecovery)
			}
		})
	}
}

func TestLaneRetreatReverseWaitsForRearTraffic(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}}}
	bus := sim.NewBus(0, 47.8, -.037)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -.071
	ad := NewAutopilotDriver()
	ad.midLaneRecovery = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 26}}
	ad.approachMerge = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 130}, Junction: sim.Vec2{X: 2000}}
	f := autopilotFrame{bus: bus, world: w, laneWidth: 52}
	if !ad.headonRearClear(f, 136) {
		t.Fatal("empty rear space was rejected")
	}
	for _, ob := range []VehicleObstacle{
		{Pos: sim.Vec2{X: -210, Y: 47.8}, Heading: 0, Length: 46, Width: 22},
		{Pos: sim.Vec2{X: -600, Y: 47.8}, Heading: 0, Speed: 200, Length: 46, Width: 22},
	} {
		f.obstacles = []VehicleObstacle{ob}
		if ad.headonRearClear(f, 136) {
			t.Fatal("lane retreat reversed into rear traffic")
		}
	}
}

func TestStraightMergeChoosesOnlyReceivingThroughLanes(t *testing.T) {
	for _, exitLanes := range []int{4, 6} {
		t.Run(fmt.Sprint(exitLanes), func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{
				{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{}, Lanes: 6},
				{P1: sim.Vec2{}, P2: sim.Vec2{X: 5000}, Lanes: exitLanes},
				{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4},
			}}
			w.RebuildIntersections()
			bus := sim.NewBus(-1500, 26, 0)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			request := &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}}
			f := autopilotFrame{bus: bus, world: w, laneWidth: 52, obstacles: []VehicleObstacle{
				{Pos: sim.Vec2{X: -1100, Y: 78}, Heading: 0, Length: 46, Width: 22},
			}}
			ad.chooseStraightMergeLane(f, request)
			expected := 78.
			if exitLanes == 6 {
				expected = 130
			}
			if request.Target.Y != expected {
				t.Fatalf("selected lane %.0f for %d-lane exit", request.Target.Y, exitLanes)
			}
			// The other lane becoming faster cannot reverse a pending merge.
			f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1100, Y: 130}, Heading: 0, Length: 46, Width: 22}}
			ad.chooseStraightMergeLane(f, request)
			if request.Target.Y != expected {
				t.Fatal("changed target while waiting for the chosen lane")
			}
		})
	}
}
