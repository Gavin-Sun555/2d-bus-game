package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRequiredMergeCompletesBehindStoppedForwardQueue(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("lw%.0f/h%.2f", lw, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2500), P2: u.Mul(2500), Lanes: 6}, {P1: r.Mul(-2500), P2: r.Mul(2500), Lanes: 4}}}
				w.RebuildIntersections()
				from := sim.NewBusStop(0, "from", u.Mul(-1700), h, 6, sim.RGBA(0, 0, 0, 255), lw)
				to := sim.NewBusStop(1, "to", r.Mul(1700), h+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{from, to}
				checkQueuedRightMerge(t, w, from, to, sim.Vec2{}, h, lw)
			})
		}
	}
}

func TestMetropolisNorthRingMergeBehindQueue(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, from := range []int{4, 5} {
			t.Run(fmt.Sprintf("lw%.0f/from%d", lw, from), func(t *testing.T) {
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				checkQueuedRightMerge(t, w, w.Stops[from], w.Stops[11], sim.Vec2{X: 2600 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}, 0, lw)
			})
		}
	}
}

func checkQueuedRightMerge(t *testing.T, w *sim.World, from, to *sim.BusStop, center sim.Vec2, h, lw float64) {
	t.Helper()
	u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	var it *sim.TrafficIntersection
	for _, candidate := range tls.Intersections {
		if candidate.Center.Distance(center) < 1 {
			it = candidate
			break
		}
	}
	if it == nil {
		t.Fatal("missing test junction")
	}
	it.Phase = sim.PhaseNSGreen
	if math.Abs(u.Y) > .5 {
		it.Phase = sim.PhaseEWGreen
	}
	line := it.GetStopLineDistForHeading(h, lw)
	p := center.Add(u.Mul(-line - 490)).Add(r.Mul(1.5*lw - 8))
	b := sim.NewBus(p.X, p.Y, h+.13)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 667
	ad.PrepareLeg(w, from, to, lw)
	ad.LaneOffset = -lw
	ad.TargetLaneOffset = -lw
	ad.OvertakeState = OvertakeReturning
	// A queued car is well ahead of the intended lane, another queues in the
	// current lane, and a yielding car waits safely behind the bus.
	obs := []VehicleObstacle{{Pos: u.Mul(-line - 490 + 368).Add(r.Mul(2.5 * lw)), Heading: h, Length: 48, Width: 22}, {Pos: u.Mul(-line - 490 + 255).Add(r.Mul(1.5 * lw)), Heading: h, Length: 48, Width: 22}, {Pos: u.Mul(-line - 490 - 202).Add(r.Mul(2.5 * lw)), Heading: h, Length: 48, Width: 22, YieldingTo: b}}
	for i := range obs {
		obs[i].Pos = obs[i].Pos.Add(center)
	}
	route := &sim.Route{StopIDs: []int{from.ID, to.ID}, CurrentIndex: 1}
	merged := false
	for tick := 0; tick < 900; tick++ {
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1.0/60)
		b.Update(1.0/60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 {
			t.Fatal("teleport")
		}
		for _, ob := range obs {
			if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: 24, HalfWid: 11}); hit {
				t.Fatalf("contact at tick%d bus%v obstacle%v", tick, b.Pos, ob.Pos)
			}
		}
		if math.Abs(b.Pos.Sub(center).Dot(r)-2.5*lw) < 6 && math.Abs(sim.NormalizeAngle(b.Heading-h)) < .15 {
			merged = true
			break
		}
	}
	if !merged {
		t.Fatalf("merge stalled despite safe forward gap: bus%v yaw%.2f speed%.1f hold%.2f offset%.1f/%.1f", b.Pos, sim.NormalizeAngle(b.Heading-h), b.Speed, ad.busTurnHoldDistance, ad.LaneOffset, ad.TargetLaneOffset)
	}
}

func TestRequiredMergeRetainsCollisionClearance(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, scenario := range []string{"safe forward slot", "close forward car", "alongside car", "rear car", "yielding rear car", "fast yielding rear car", "high speed", "oncoming", "intermediate lane"} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, scenario), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				b := sim.NewBus(0, 0, h)
				ad := NewAutopilotDriver()
				ad.approachMerge = &sim.BusLaneMergeRequest{Target: r.Mul(52), Heading: h, Remaining: .3}
				ad.drivePath = []sim.Vec2{u.Mul(-1000), u.Mul(2000)}
				ad.cumLen = []float64{0, 3000}
				ob := VehicleObstacle{Pos: u.Mul(200).Add(r.Mul(52)), Heading: h, Length: 48, Width: 22}
				speed := 80.
				target := 52.
				want := false
				switch scenario {
				case "safe forward slot":
					want = true
				case "close forward car":
					ob.Pos = u.Mul(100).Add(r.Mul(52))
				case "alongside car":
					ob.Pos = r.Mul(52)
				case "rear car":
					ob.Pos = u.Mul(-110).Add(r.Mul(52))
				case "yielding rear car":
					ob.Pos = u.Mul(-110).Add(r.Mul(52))
					ob.YieldingTo = b
					want = true
				case "fast yielding rear car":
					ob.Pos = u.Mul(-150).Add(r.Mul(52))
					ob.Speed = 150
					ob.YieldingTo = b
				case "high speed":
					speed = 445
					b.Speed = 445
				case "oncoming":
					ob.Heading = h + math.Pi
				case "intermediate lane":
					ob.Pos = r.Mul(52)
					target = 156
					ad.approachMerge.Target = r.Mul(target)
				}
				f := autopilotFrame{bus: b, laneWidth: 52, speedAbs: speed, obstacles: []VehicleObstacle{ob}}
				if got := requiredMergeLaneClear(f, h, 0, target); got != want {
					t.Fatalf("clear%v want%v", got, want)
				}
				if scenario == "close forward car" && ad.requiredMergeTrafficSpeed(f) != 0 {
					t.Fatal("close queue lost stop cap")
				}
			})
		}
	}
}

func TestThroughMergeLockRequiresParallelFarExit(t *testing.T) {
	for _, scenario := range []struct {
		name string
		path []sim.Vec2
		want bool
	}{
		{"straight", []sim.Vec2{{X: -1000, Y: 78}, {X: 1000, Y: 78}}, true},
		{"left turn", []sim.Vec2{{X: -1000, Y: 78}, {X: -50, Y: 78}, {X: -50, Y: -1000}}, false},
		{"right turn", []sim.Vec2{{X: -1000, Y: 78}, {X: -50, Y: 78}, {X: -50, Y: 1000}}, false},
		{"wide curved approach", []sim.Vec2{{X: -1000, Y: 78}, {X: 40, Y: 78}, {X: 100, Y: 150}, {X: 100, Y: 1000}}, false},
		{"parallel different road", []sim.Vec2{{X: -1000, Y: 78}, {X: -50, Y: 78}, {X: -50, Y: 900}, {X: 1000, Y: 900}}, false},
	} {
		t.Run(scenario.name, func(t *testing.T) {
			ad := NewAutopilotDriver()
			ad.drivePath = scenario.path
			bus := sim.NewBus(-500, 26, 0)
			f := autopilotFrame{bus: bus, laneWidth: 52}
			it := &sim.TrafficIntersection{IsMajor: true, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachLanes: [4]int{6, 6, 4, 4}, ApproachIs4Lane: [4]bool{true, true, true, true}}
			if got := ad.hasStraightMergeExit(f, it); got != scenario.want {
				t.Fatalf("straight exit%v want%v", got, scenario.want)
			}
		})
	}
}
