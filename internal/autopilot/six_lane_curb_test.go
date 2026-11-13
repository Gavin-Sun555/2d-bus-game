package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkSixLaneUsesClearCurbInsteadOfFollowingMiddle(t *testing.T) {
	const lw = 52.
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, state := range []int{OvertakeNone, OvertakePassing} {
			t.Run(fmt.Sprintf("heading%.2f/state%d", heading, state), func(t *testing.T) {
				forward := sim.UnitFromAngle(heading)
				right := sim.Vec2{X: -forward.Y, Y: forward.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-6000), P2: forward.Mul(6000), Lanes: 6, Type: sim.RoadExpressway6Lane}}}
				w.Stops = []*sim.BusStop{
					sim.NewBusStop(0, "start", forward.Mul(-4500), heading, 6, sim.RGBA(0, 0, 0, 255), lw),
					sim.NewBusStop(1, "end", forward.Mul(4500), heading, 6, sim.RGBA(0, 0, 0, 255), lw),
				}
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				pos := forward.Mul(-1200).Add(right.Mul(1.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.Speed = 140
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.BerserkMaxSpeed = 445
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				ad.holdLaneOffset(-lw)
				ad.OvertakeState = state
				lead := VehicleObstacle{Pos: bus.Pos.Add(forward.Mul(132)), Heading: heading, Speed: 140, Length: 46, Width: 22}
				// A fast rear car initially owns the right-side gap. Once it leaves,
				// the bus can move right without crowding its middle-lane leader.
				rear := VehicleObstacle{Pos: bus.Pos.Sub(forward.Mul(84)).Add(right.Mul(lw)), Heading: heading, Speed: 300, Length: 46, Width: 22}
				ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{lead, rear}, lw, 1./60)
				if ad.TargetLaneOffset == 0 {
					t.Fatal("planned a merge in front of fast rear traffic")
				}
				ad.holdLaneOffset(-lw)
				ad.OvertakeState = state
				for tick := 0; tick < 420; tick++ {
					lead.Pos = lead.Pos.Add(forward.Mul(lead.Speed / 60))
					th, br, st, hb, door, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{lead}, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, door)
					if hit, _, _ := bus.CollidesBody(sim.OBB{Center: lead.Pos, Heading: heading, HalfLen: 23, HalfWid: 11}); hit {
						t.Fatalf("contact with lead at tick %d", tick)
					}
					if math.Abs(bus.Pos.Dot(right)-2.5*lw) < 4 && bus.Pos.Sub(lead.Pos).Dot(forward) > 105 {
						return
					}
				}
				t.Fatalf("did not pass from clear curb lane: lateral %.1f, offset %.1f -> %.1f", bus.Pos.Dot(right), ad.LaneOffset, ad.TargetLaneOffset)
			})
		}
	}
}

func TestBerserkSixLaneReturnsToCurbBeforeStraightRed(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -6000}, P2: sim.Vec2{X: 6000}, Lanes: 6, Type: sim.RoadExpressway6Lane},
		{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4, Type: sim.RoadAvenue4Lane},
	}}
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "start", sim.Vec2{X: -4500}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "end", sim.Vec2{X: 4500}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
	}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	inter := tls.Intersections[0]
	inter.Phase = sim.PhaseNSGreen
	stopX := -inter.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(stopX-520, 1.5*lw, 0)
	bus.Speed = 130
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	ad.holdLaneOffset(-lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	middle := VehicleObstacle{Pos: sim.Vec2{X: stopX - 215, Y: 1.5 * lw}, Heading: 0, Speed: 25, Length: 46, Width: 22}
	for tick := 0; tick < 480; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{middle}, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > stopX+.1 {
			t.Fatalf("curb preference crossed a red light at tick %d", tick)
		}
	}
	if math.Abs(bus.Pos.Y-2.5*lw) > 4 || math.Abs(bus.Speed) > 1 {
		t.Fatalf("bus did not queue in the clear right lane: pos %v speed %.1f", bus.Pos, bus.Speed)
	}
}

func TestCompletedStraightMergeAllowsSixLaneCurbPass(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -6000}, P2: sim.Vec2{X: 6000}, Lanes: 6, Type: sim.RoadExpressway6Lane},
		{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4, Type: sim.RoadAvenue4Lane},
	}}
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "start", sim.Vec2{X: -4500}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "end", sim.Vec2{X: 4500}, 0, 6, sim.RGBA(0, 0, 0, 255), lw),
	}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for _, tc := range []struct {
		name string
		x    float64
	}{
		{"runway", -2600},
		{"near stop line", -490},
	} {
		t.Run(tc.name, func(t *testing.T) {
			bus := sim.NewBus(tc.x, 1.5*lw, 0)
			bus.Speed = 80
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			ad.holdLaneOffset(-lw)
			ad.OvertakeState = OvertakeNone
			ad.approachMerge = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 1.5 * lw}, Junction: tls.Intersections[0].Center}
			lead := VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{X: 170}), Speed: 70, Length: 46, Width: 22}
			ad.ComputeControls(bus, route, w, tls, []VehicleObstacle{lead}, lw, 1./60)
			if tc.name == "near stop line" {
				if ad.approachMerge == nil || math.Abs(ad.TargetLaneOffset+lw) > .1 {
					t.Fatal("abandoned a settled approach close to the junction")
				}
			} else if ad.approachMerge != nil || ad.TargetLaneOffset != 0 {
				t.Fatalf("did not choose the clear curb lane after joining a through lane: merge %+v, offset %.1f", ad.approachMerge, ad.TargetLaneOffset)
			}
		})
	}
}
