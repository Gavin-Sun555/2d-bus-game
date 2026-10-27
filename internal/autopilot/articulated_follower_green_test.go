package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestJunctionLeaderClearsMovingArticulatedFollower(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("h%.2f/w%.0f", h, lw), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}, {P1: r.Mul(-3000), P2: r.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				w.RebuildIntersections()
				g := w.JunctionAt(sim.Vec2{}, lw)
				p := u.Mul(-math.Abs(u.X)*g.HalfX - math.Abs(u.Y)*g.HalfY - 24).Add(r.Mul(3.5 * lw))
				leader := sim.NewBus(p.X, p.Y, h)
				rearPos := p.Sub(u.Mul(133)).Sub(r.Mul(28))
				rear := sim.NewBus(rearPos.X, rearPos.Y, h+.175)
				rear.SetModel(sim.BusArticulated)
				rear.TrailerHeading = h + .10
				rear.Speed = 2
				// The corridor is published for the powered front section. ObserveBus
				// carries an envelope centered roughly half a section farther back.
				rear.JunctionCorridor = &sim.JunctionCorridor{Remaining: 1, Bodies: []sim.OBB{{Center: rear.Pos, Heading: rear.Heading}, {Center: rear.Pos.Add(u.Mul(500)), Heading: h}}}
				ad := NewAutopilotDriver()
				ad.drivePath = []sim.Vec2{p, p.Add(u.Mul(1500))}
				ad.cumLen = []float64{0, 1500}
				f := autopilotFrame{bus: leader, world: w, laneWidth: lw, speedAbs: 3.8, obstacles: []VehicleObstacle{ObserveBus(rear)}}
				if limit := ad.junctionSweptSpeed(f); limit == 0 {
					t.Fatalf("clear leader blocked by following long bus: %v", limit)
				}
			})
		}
	}
}

// Correcting the long bus's reference point must preserve its rear section as
// a physical blocker. A vehicle across that rear section still has to stop.
func TestSweepsRetainArticulatedRearSection(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", h), func(t *testing.T) {
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			long := sim.NewBus(0, 0, h)
			long.SetModel(sim.BusArticulated)
			long.TrailerHeading = h + .4
			tail := long.TrailerBody()
			crossing := sim.NewBus(tail.Center.X, tail.Center.Y, h+math.Pi/2)
			front := crossing.BodyParts()[0]
			ob := ObserveBus(long)
			if !obstacleSweepCollision(crossing, front, ob, 0, 0, 0) {
				t.Fatal("rear section no longer blocks a crossing vehicle")
			}
			// A real obstacle directly in front retains braking, with or without a
			// corridor. This is independent of whether a signal allows movement.
			short := sim.NewBus(0, 0, h)
			ahead := sim.NewBus(u.X*90, u.Y*90, h)
			if !obstacleSweepCollision(short, short.BodyParts()[0], ObserveBus(ahead), 0, 1, 1) {
				t.Fatal("lost front collision containment")
			}
		})
	}
}
