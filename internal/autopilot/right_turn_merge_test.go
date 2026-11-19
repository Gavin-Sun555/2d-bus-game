package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestRightTurnRequestsCurbLaneBeforeJunction(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("lanes%d/h%.2f", lanes, h), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				const lw = 52.
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2200), P2: u.Mul(2200), Lanes: lanes}, {P1: r.Mul(-2200), P2: r.Mul(2200), Lanes: 4}}}
				w.RebuildIntersections()
				a := sim.NewBusStop(0, "from", u.Mul(-1700), h, lanes, sim.RGBA(0, 0, 0, 255), lw)
				z := sim.NewBusStop(1, "to", r.Mul(1700), h+math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{a, z}
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, a, z, lw)
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
				lateral := .5 * lw
				if lanes == 2 {
					lateral = -.5 * lw
				}
				p := u.Mul(-line - 200).Add(r.Mul(lateral))
				b := sim.NewBus(p.X, p.Y, h)
				ad.trackPath(p)
				ad.LaneOffset = -(float64(lanes/2) - 1) * lw
				if lanes == 2 {
					ad.LaneOffset = -lw
				}
				ad.TargetLaneOffset = ad.LaneOffset
				ad.OvertakeState = OvertakePassing
				ad.updateOvertaking(b, w, tls, nil, lw, ad.LastPathS, 0, 2000, 1.0/60, z)
				if b.LaneMergeRequest == nil {
					t.Fatal("right turn did not request curb-lane merge")
				}
				want := (float64(lanes/2) - .5) * lw
				if got := b.LaneMergeRequest.Target.Dot(r); math.Abs(got-want) > .01 {
					t.Fatalf("target %.1f want%.1f", got, want)
				}
			})
		}
	}
}

func TestRightTurnWaitingFallbackAndCorridor(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: 7800, Y: 0}, P2: sim.Vec2{X: 7800, Y: 3600}, Lanes: 4},
		{P1: sim.Vec2{X: 5000, Y: 1800}, P2: sim.Vec2{X: 9600, Y: 1800}, Lanes: 4},
	}}
	w.RebuildIntersections()
	a := sim.NewBusStop(0, "from", sim.Vec2{X: 7800, Y: 600}, math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
	z := sim.NewBusStop(1, "to", sim.Vec2{X: 6000, Y: 1800}, math.Pi, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{a, z}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, a, z, lw)

	b := sim.NewBus(7774.84, 1630.91, 1.582)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = 1.571
	ad.LaneOffset = -65.5
	ad.TargetLaneOffset = -65.5
	ad.OvertakeState = OvertakePassing
	obs := []VehicleObstacle{{
		Pos:     sim.Vec2{X: 7722, Y: 1508.04},
		Heading: math.Pi / 2,
		Length:  48,
		Width:   22,
	}}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for tick := 0; tick < 150; tick++ {
		ad.ComputeControls(b, route, w, tls, obs, lw, 1.0/60)
		if tick == 0 && b.JunctionCorridor != nil && b.JunctionCorridor.Entered {
			t.Fatal("held right-turn-waiting bus should not mark junction corridor entered across adjacent lane")
		}
	}
	if ad.fallbackJunction == nil || ad.rightTurnWaiting || ad.busTurnHoldDistance > 0 {
		t.Fatalf("expected current-lane fallback to splice right-turn path, got fallback=%v rightWait=%v hold=%.2f",
			ad.fallbackJunction, ad.rightTurnWaiting, ad.busTurnHoldDistance)
	}
}

func TestOncomingPassFitsBeforeRoundaboutAndNarrowExit(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -6000, Y: 1800}, P2: sim.Vec2{X: 0, Y: 1800}, Lanes: 2},
		},
		Roundabouts: []sim.Roundabout{
			{Center: sim.Vec2{X: -3900, Y: 1800}, Radius: 300},
		},
	}
	b := sim.NewBus(-3336, 1774, math.Pi)
	b.SetModel(sim.BusArticulated)
	tls := sim.NewTrafficLightSystem()
	f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, speedAbs: 90}
	if oncomingPassFitsBeforeJunction(f, sim.Vec2{X: -1, Y: 0}, 150) {
		t.Fatal("oncoming pass should not start right in front of a roundabout mouth")
	}

	ad := NewAutopilotDriver()
	ad.drivePath = []sim.Vec2{{X: -1626, Y: 3678}, {X: 0, Y: 3626}, {X: 5140, Y: 3628}}
	ad.WaypointIdx = 1
	inter := &sim.TrafficIntersection{Center: sim.Vec2{X: 0, Y: 3600}, ApproachLanes: [4]int{4, 4, 2, 4}}
	b2 := sim.NewBus(-500, 3648, 0)
	if ad.hasStraightMergeExit(autopilotFrame{bus: b2, world: w, laneWidth: lw}, inter) {
		t.Fatal("narrowing 2-lane exit at 0.5*lw should not trigger 1.5*lw outer straight merge")
	}
}
