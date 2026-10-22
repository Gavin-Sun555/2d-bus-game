package autopilot

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// twoLaneGridWorld is a 2-lane street crossed by two 2-lane streets 650px apart,
// matching the spacing of minor streets in the shipped maps.
func twoLaneGridWorld() *sim.World {
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 2, Type: sim.RoadStreet2Lane},
		{P1: sim.Vec2{X: 0, Y: -3000}, P2: sim.Vec2{X: 0, Y: 3000}, Lanes: 2, Type: sim.RoadStreet2Lane},
		{P1: sim.Vec2{X: 650, Y: -3000}, P2: sim.Vec2{X: 650, Y: 3000}, Lanes: 2, Type: sim.RoadStreet2Lane},
	}}
	w.RebuildIntersections()
	return w
}

func setPath(ad *AutopilotDriver, pts []sim.Vec2) {
	ad.drivePath = pts
	ad.cumLen = make([]float64, len(pts))
	for i := 1; i < len(pts); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + pts[i].Distance(pts[i-1])
	}
}

// A right turn planned at the next-but-one junction must not grant right-on-red
// to the straight movement through the nearer red junction.
func TestBerserkStraightThroughMinorRedIgnoresLaterRightTurn(t *testing.T) {
	const lw = 52.0
	for _, farFirst := range []bool{true, false} {
		t.Run(fmt.Sprintf("farFirst=%v", farFirst), func(t *testing.T) {
			w := twoLaneGridWorld()
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			var near, far *sim.TrafficIntersection
			for _, it := range tls.Intersections {
				switch {
				case it.Center.X < 1 && it.Center.X > -1:
					near = it
				case it.Center.X > 649 && it.Center.X < 651:
					far = it
				}
			}
			if near == nil || far == nil {
				t.Fatal("fixture intersections missing")
			}
			if farFirst {
				tls.Intersections = []*sim.TrafficIntersection{far, near}
			} else {
				tls.Intersections = []*sim.TrafficIntersection{near, far}
			}
			near.Phase, near.Timer = sim.PhaseNSGreen, 0
			far.Phase, far.Timer = sim.PhaseNSGreen, 0

			y := lw / 2
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			// Straight through x=0, then right turn (south, +Y) at x=650.
			setPath(ad, []sim.Vec2{{X: -2000, Y: y}, {X: -200, Y: y}, {X: 0, Y: y}, {X: 200, Y: y}, {X: 650 - lw/2, Y: y}, {X: 650 - lw/2, Y: 300}, {X: 650 - lw/2, Y: 2000}})
			line := near.GetStopLineDistForHeading(0, lw)
			b := sim.NewBus(0, 0, 0)
			b.Pos = sim.Vec2{X: -line - b.Length/2 - 100, Y: y}
			b.Speed = 445
			ad.trackPath(b.Pos)
			f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: 445}
			if got := ad.trafficLightSpeed(f, 445); got >= 445 {
				t.Fatalf("berserk bus did not brake for red at the nearer minor junction: target %.1f", got)
			}
		})
	}
}

// Berserk mode may take a yellow only if it reaches the stop line before red.
func TestBerserkYellowDilemmaZone(t *testing.T) {
	const lw = 52.0
	for _, tc := range []struct {
		name      string
		remaining float64
		wantStop  bool
	}{
		{"red before arrival", 0.4, true},
		{"clears on yellow", 2.0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w := twoLaneGridWorld()
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			for _, it := range tls.Intersections {
				it.Phase, it.Timer = sim.PhaseEWYellow, it.YellowDuration-tc.remaining
			}
			y := lw / 2
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			setPath(ad, []sim.Vec2{{X: -2000, Y: y}, {X: 2000, Y: y}})
			var near *sim.TrafficIntersection
			for _, it := range tls.Intersections {
				if it.Center.X < 1 && it.Center.X > -1 {
					near = it
				}
			}
			line := near.GetStopLineDistForHeading(0, lw)
			b := sim.NewBus(0, 0, 0)
			b.Pos = sim.Vec2{X: -line - b.Length/2 - 400, Y: y}
			b.Speed = 445
			ad.trackPath(b.Pos)
			f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: 445}
			got := ad.trafficLightSpeed(f, 445)
			if stopped := got < 445; stopped != tc.wantStop {
				t.Fatalf("stop=%v want %v (target %.1f)", stopped, tc.wantStop, got)
			}
		})
	}
}
