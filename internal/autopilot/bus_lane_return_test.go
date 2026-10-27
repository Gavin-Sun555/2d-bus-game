package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusLaneRecentersFractionalOffset(t *testing.T) {
	const lw = 52.0
	w := busLaneTestWorld(lw)
	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	bus := sim.NewBus(-1400, 3.5*lw-15, 0)
	ad.holdLaneOffset(-15)
	s, _ := ad.trackPath(bus.Pos)
	ad.updateBusLane(autopilotFrame{bus: bus, world: w, laneWidth: lw, s: s, remaining: 3000, targetStop: w.Stops[1]}, .016)
	if ad.TargetLaneOffset != 0 {
		t.Fatalf("fractional lane offset retained: %v", ad.TargetLaneOffset)
	}
}

func TestBusLaneLeftReturnAcceptsSafeQueueGap(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		const lw = 52.0
		w := busLaneTestWorld(lw)
		w.Stops[1] = sim.NewBusStop(1, "left", sim.Vec2{Y: -2200}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)
		ad := NewAutopilotDriver()
		ad.BerserkMode = berserk
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		bus := sim.NewBus(-700, 3*lw, 0)
		ad.holdLaneOffset(-.5 * lw)
		s, _ := ad.trackPath(bus.Pos)
		f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: s, remaining: 3000, targetStop: w.Stops[1], obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: -540, Y: 3.5 * lw}, Length: 96, Width: 36}}}
		ad.updateBusLane(f, .016)
		ad.holdBlockedRightMerge(f)
		if ad.TargetLaneOffset != 0 {
			t.Fatalf("berserk%v: safe queue gap rejected, offset %.1f", berserk, ad.TargetLaneOffset)
		}
	}
}

// Replay a bus left halfway across the marking by a blocked station departure.
// A stopped leader must not prevent recentering, and the first left arrow must
// release both the player's driver and the fleet's restricted driver.
func TestBusLaneLeftReturnRestartsOnFirstArrow(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, crossingLanes := range []int{4, 8} {
				for _, fleet := range []bool{false, true} {
					for _, berserk := range []bool{false, true} {
						t.Run(fmt.Sprintf("lw%.0f/h%.2f/cross%d/fleet%v/berserk%v", lw, heading, crossingLanes, fleet, berserk), func(t *testing.T) {
							checkBusLaneLeftArrowRestart(t, lw, heading, crossingLanes, fleet, berserk)
						})
					}
				}
			}
		}
	}
}

func checkBusLaneLeftArrowRestart(t *testing.T, lw, heading float64, crossingLanes int, fleet, berserk bool) {
	t.Helper()
	forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	rotate := func(p sim.Vec2) sim.Vec2 { return forward.Mul(p.X).Add(right.Mul(p.Y)) }
	w := busLaneTestWorld(lw)
	w.Roads[1].Lanes = crossingLanes
	if crossingLanes == 8 {
		w.Roads[1].Type = sim.RoadBusBoulevard8Lane
	}
	for i := range w.Roads {
		w.Roads[i].P1 = rotate(w.Roads[i].P1)
		w.Roads[i].P2 = rotate(w.Roads[i].P2)
	}
	w.RebuildIntersections()
	w.Stops[0] = sim.NewBusStop(0, "from", forward.Mul(-2200), heading, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops[1] = sim.NewBusStop(1, "left", right.Mul(-2200), heading-math.Pi/2, crossingLanes, sim.RGBA(0, 0, 0, 255), lw)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseEWGreen
	green := sim.PhaseEWLeftGreen
	if math.Abs(forward.Y) > .5 {
		it.Phase = sim.PhaseNSGreen
		green = sim.PhaseNSLeftGreen
	}
	line := it.GetStopLineDistForHeading(heading, lw)
	pos := forward.Mul(-line - 300).Add(right.Mul(3 * lw))
	bus := sim.NewBus(pos.X, pos.Y, heading)
	ad := NewAutopilotDriver()
	ad.BerserkMode = berserk
	ad.DisallowOncomingOvertake = fleet
	ad.holdLaneOffset(-.5 * lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	obstacles := []VehicleObstacle{{Pos: bus.Pos.Add(forward.Mul(160)).Add(right.Mul(.5 * lw)), Heading: heading, Length: 96, Width: 36}}
	for step := 0; step < 240; step++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, .016)
		bus.Update(.016, th, br, st, hb, doors)
		if bus.Pos.Dot(forward)+bus.Length/2 > -line+1 {
			t.Fatal("crossed stop line before left arrow")
		}
		body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
		other := sim.OBB{Center: obstacles[0].Pos, Heading: heading, HalfLen: 48, HalfWid: 18}
		if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
			t.Fatal("return hit queued leader")
		}
	}
	it.Phase = green
	it.Timer = 0
	for step := 0; float64(step)*.016 < it.LeftGreenDuration; step++ {
		it.Timer = float64(step) * .016
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, .016)
		bus.Update(.016, th, br, st, hb, doors)
		if bus.Pos.Dot(forward)+bus.Length/2 > -line+1 {
			return
		}
	}
	t.Fatalf("missed arrow at %v speed%.1f target%.1f offset%.1f", bus.Pos, bus.Speed, ad.LastTargetSpeed, ad.LaneOffset)
}

func TestBusLaneLeftReturnStillYieldsToAlongsideTraffic(t *testing.T) {
	const lw = 52.0
	w := busLaneTestWorld(lw)
	w.Stops[1] = sim.NewBusStop(1, "left", sim.Vec2{Y: -2200}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	bus := sim.NewBus(-700, 2.5*lw, 0)
	ad.holdLaneOffset(-lw)
	s, _ := ad.trackPath(bus.Pos)
	ad.updateBusLane(autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: s, remaining: 3000, targetStop: w.Stops[1], obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: -700, Y: 3.5 * lw}, Length: 96, Width: 36}}}, .016)
	if ad.TargetLaneOffset != -lw {
		t.Fatal("merged into alongside bus")
	}
	if ad.busTurnHoldDistance <= 0 {
		t.Fatal("missing stop-line hold while lane return blocked")
	}
}
