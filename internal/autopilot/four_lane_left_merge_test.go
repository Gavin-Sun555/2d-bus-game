package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestOrdinaryFourLaneLeftApproachPublishesMerge(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading %.2f", h), func(t *testing.T) {
			const lw = 52.
			ewLanes, nsLanes := 4, 8
			if math.Abs(math.Sin(h)) > .5 {
				ewLanes, nsLanes = 8, 4
			}
			w, _ := turnTestWorld(ewLanes, nsLanes, h, lw)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			in := sim.UnitFromAngle(h)
			r := sim.Vec2{X: -in.Y, Y: in.X}
			g := w.JunctionAt(sim.Vec2{}, lw)
			half := math.Abs(in.X)*g.HalfX + math.Abs(in.Y)*g.HalfY
			p := in.Mul(-half - 170).Add(r.Mul(78))
			b := sim.NewBus(p.X, p.Y, h)
			b.SetModel(sim.BusArticulated)
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			alongside := VehicleObstacle{Pos: b.Pos.Add(in.Mul(20)).Sub(r.Mul(52)), Heading: h, Length: 46, Width: 22}
			ad.ComputeControls(b, route, w, tls, []VehicleObstacle{alongside}, lw, 1./60)
			request := b.LaneMergeRequest
			if request == nil || !request.LeftTurn || request.Junction.Length() > 1 {
				t.Fatal("normal left merge was not published")
			}
			if math.Abs(request.Target.Dot(r)-26) > 1e-6 {
				t.Fatal("wrong median lane")
			}
			f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, obstacles: []VehicleObstacle{alongside}}
			if requiredMergeLaneClear(f, h, 0, request.Target.Sub(b.Pos).Dot(r)) {
				t.Fatal("accepted occupied adjacent slot")
			}
			ad.ComputeControls(b, route, w, tls, []VehicleObstacle{alongside}, lw, 1./60)
			if b.LaneMergeRequest == nil || !b.LaneMergeRequest.LeftTurn {
				t.Fatal("normal-mode policy discarded its merge next tick")
			}
		})
	}
}

// On wide roads a normal-mode bus requests the inner left lane like on four
// lanes; berserk buses keep their own required-merge handling.
func TestFourLaneMergeDoesNotOverrideWideCurbPermission(t *testing.T) {
	for _, lanes := range []int{6, 8} {
		for _, berserk := range []bool{false, true} {
			w, _ := turnTestWorld(lanes, 8, 0, 52)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			b := sim.NewBus(-900, (float64(lanes)/2-.5)*52, 0)
			ad := NewAutopilotDriver()
			ad.BerserkMode = berserk
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
			f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: 52, targetStop: w.Stops[1], remaining: 4000}
			ad.requestFourLaneLeftMerge(f)
			if berserk && ad.approachMerge != nil {
				t.Fatalf("forced berserk %d-lane bus toward median", lanes)
			}
			if !berserk && (ad.approachMerge == nil || math.Abs(ad.approachMerge.Target.Y-26) > 1) {
				t.Fatalf("%d-lane fleet bus did not request the inner left lane: %+v", lanes, ad.approachMerge)
			}
		}
	}
}

// A stationary origin-lane follower can be physically clear even inside the
// ordinary passing buffer. Use both sections when checking the rear gap.
func TestLeftMergeReleasesSeparatedOriginFollower(t *testing.T) {
	leader := sim.NewBus(0, 62, 0)
	leader.SetModel(sim.BusArticulated)
	follower := sim.NewBus(-252, 70, 0)
	follower.SetModel(sim.BusArticulated)
	f := autopilotFrame{bus: leader, laneWidth: 52}
	if !stoppedOriginFollowerClearsMerge(f, ObserveBus(follower), 0, 36, 0) {
		t.Fatal("separated follower held the leader's merge")
	}
	follower.Pos.X = -192
	if stoppedOriginFollowerClearsMerge(f, ObserveBus(follower), 0, 36, 0) {
		t.Fatal("unsafe rear gap accepted")
	}
	follower.Pos = sim.Vec2{X: -252, Y: 26}
	if stoppedOriginFollowerClearsMerge(f, ObserveBus(follower), 0, 36, 0) {
		t.Fatal("destination lane follower exempted")
	}
}

func TestCommittedFourLaneLeftMergeHandsOffInsideJunction(t *testing.T) {
	const lw = 52.
	w, _ := turnTestWorld(4, 8, 0, lw)
	// The nose has entered the left lane, but its center is still offset
	// farther than the old handoff tolerance.
	b := sim.NewBus(-200, 46.22, -.113)
	b.SetModel(sim.BusArticulated)
	ad := NewAutopilotDriver()
	req := &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 26}, LeftTurn: true, Remaining: .3}
	ad.approachMerge, b.LaneMergeRequest = req, req
	f := autopilotFrame{bus: b, world: w, laneWidth: lw}
	if ad.advanceApproachMerge(f, 1./60) || ad.approachMerge != nil || b.LaneMergeRequest != nil {
		t.Fatal("incoming lane request still controls an admitted left turn")
	}
}

func TestBentRearBusEnvelopeDoesNotBlockArticulatedSweep(t *testing.T) {
	leader := sim.NewBus(61853.10956039541, 14459.478062657561, -.04269217165016759)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading = .0026743240786119915
	leader.SteeringAngle = -.45416444897504704
	follower := sim.NewBus(61723.03895314148, 14423.162824335242, -.008881415999957534)
	follower.SetModel(sim.BusArticulated)
	follower.TrailerHeading = -.08981340445689587
	f := autopilotFrame{bus: leader}
	own := leader.BodyParts()[1]
	padded := follower.BodyParts()[0]
	padded.HalfLen++
	padded.HalfWid++
	if !isRearFollowerContact(f, own, ObserveBus(follower), padded) {
		t.Fatal("empty space in a bent follower's envelope blocked the leader")
	}
	follower.Pos.Y += 5
	if isRearFollowerContact(f, own, ObserveBus(follower), padded) {
		t.Fatal("physical contact was treated as padding")
	}
}

func TestStoppedAI46CanMergeAheadOfBentOriginFollower(t *testing.T) {
	leader := sim.NewBus(61837.72592298135, 14459.474826615859, -.0010144132235138667)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading = -.043439795169689245
	follower := sim.NewBus(61626.5992126317, 14464.858012810964, -.04704451014083207)
	follower.SetModel(sim.BusArticulated)
	follower.TrailerHeading = -.13308204955146727
	ob := ObserveBus(follower)
	if math.Abs(ob.Pos.Y-leader.Pos.Y) <= 52*.2 {
		t.Fatal("captured trailer no longer shifts the envelope beyond the lane tolerance")
	}
	f := autopilotFrame{bus: leader, laneWidth: 52, obstacles: []VehicleObstacle{ob}}
	shift := 14426 - leader.Pos.Y
	if !requiredMergeLaneClear(f, 0, 0, shift) {
		t.Fatal("stopped follower behind the physical rear blocked AI46")
	}
	follower.Pos.X += 20
	f.obstacles[0] = ObserveBus(follower)
	if requiredMergeLaneClear(f, 0, 0, shift) {
		t.Fatal("follower with insufficient physical rear clearance was exempted")
	}
}

func TestBlockedFourLaneLeftUsesCurbAndRespectsArrow(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		t.Run(fmt.Sprintf("berserk=%t", berserk), func(t *testing.T) {
			const lw = 52.
			w, _ := turnTestWorld(4, 8, 0, lw)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			inter := tls.Intersections[0]
			half := w.JunctionAt(sim.Vec2{}, lw).HalfX
			b := sim.NewBus(-half-170, 1.5*lw, 0)
			b.SetModel(sim.BusArticulated)
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			blocker := VehicleObstacle{Pos: b.Pos.Add(sim.Vec2{X: 20, Y: -lw}), Length: 46, Width: 22}
			for _, blocked := range []bool{false, true} {
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				var obs []VehicleObstacle
				if blocked {
					obs = []VehicleObstacle{blocker}
				}
				for i := 0; i < 60; i++ {
					ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
				}
				if chosen := ad.busLaneJunction != nil; chosen != blocked {
					t.Fatalf("curb fallback chosen=%v with blocked median=%v", chosen, blocked)
				}
				if !blocked {
					continue
				}
				if b.LaneMergeRequest != nil {
					t.Fatal("continued to reserve median after choosing curb")
				}
				b.Pos.X = -inter.GetStopLineDistForHeading(0, lw) - b.Length/2 - 10
				s, _ := ad.trackPath(b.Pos)
				f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: s}
				inter.Phase = sim.PhaseEWGreen
				if got := ad.trafficLightSpeed(f, 100); got != 0 {
					t.Fatalf("curb left obeyed through green, speed %v", got)
				}
				inter.Phase = sim.PhaseEWLeftGreen
				if got := ad.trafficLightSpeed(f, 100); got != 100 {
					t.Fatalf("left arrow did not release the curb turn, speed %v", got)
				}
			}
		})
	}
}

// The curb-lane left turn is only a fallback after serving a stop just before
// the junction, and only where a protected left arrow holds through traffic.
func TestBlockedLeftCurbFallbackNeedsStopBeforeJunctionAndArrow(t *testing.T) {
	for _, tc := range []struct {
		name         string
		midJunction  bool
		minor        bool
		wantFallback bool
	}{{"stop-before-junction", false, false, true}, {"junction-between", true, false, false}, {"minor-junction", false, true, false}} {
		t.Run(tc.name, func(t *testing.T) {
			const lw = 52.
			w, _ := turnTestWorld(4, 8, 0, lw)
			if tc.midJunction {
				w.Roads = append(w.Roads, sim.RoadSegment{P1: sim.Vec2{X: -800, Y: -2200}, P2: sim.Vec2{X: -800, Y: 2200}, Lanes: 2})
				w.RebuildIntersections()
			}
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			for _, inter := range tls.Intersections {
				if tc.minor {
					inter.IsMajor = false
				}
			}
			half := w.JunctionAt(sim.Vec2{}, lw).HalfX
			b := sim.NewBus(-half-170, 1.5*lw, 0)
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			blocker := VehicleObstacle{Pos: b.Pos.Add(sim.Vec2{X: 20, Y: -lw}), Length: 46, Width: 22}
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			for i := 0; i < 60; i++ {
				ad.ComputeControls(b, route, w, tls, []VehicleObstacle{blocker}, lw, 1./60)
			}
			if chosen := ad.busLaneJunction != nil; chosen != tc.wantFallback {
				t.Fatalf("curb fallback chosen=%v, want %v", chosen, tc.wantFallback)
			}
		})
	}
}
