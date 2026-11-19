package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestThroughBusKeepsLaneInsideCrossing(t *testing.T) {
	const lw = 52.
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		world := &sim.World{CenterBRT: true, Intersections: []sim.Vec2{{}}, Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, MedianBusLane: true}, {P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 6}}}
		b := sim.NewBus(0, 0, h)
		b.SetModel(sim.BusArticulated)
		b.Pos = u.Mul(-200).Add(r.Mul(26))
		b.Speed = 220
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		ad.CanSkipTargetStop = true
		ad.drivePath = []sim.Vec2{u.Mul(-3000).Add(r.Mul(26)), u.Mul(4000).Add(r.Mul(26))}
		ad.cumLen = []float64{0, 7000}
		s, _ := ad.trackPath(b.Pos)
		f := autopilotFrame{bus: b, world: world, laneWidth: lw, s: s, speedAbs: b.Speed, remaining: 4200}
		// A stopped leader makes another lane faster, but the through movement
		// already inside the crossing retains its physical lane.
		f.obstacles = []VehicleObstacle{{Pos: b.Pos.Add(u.Mul(220)), Heading: h, Length: 46, Width: 22}}
		if !ad.updateApproachLanePolicies(f, 1./60) || math.Abs(ad.TargetLaneOffset) > .01 {
			t.Fatalf("h%.2f started optional pass %.1f", h, ad.TargetLaneOffset)
		}
		// An existing approach commitment must not be discarded by this guard.
		ad.OvertakeState = OvertakeChangingLane
		ad.TargetLaneOffset = 52
		if !ad.holdStraightJunctionLane(f, 1./60) || ad.TargetLaneOffset != 52 {
			t.Fatal("discarded ongoing lane change")
		}
	}
}
