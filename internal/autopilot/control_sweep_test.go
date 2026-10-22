package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestControlSweepProtectsYawedBodyOutsideJunction(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, oncoming := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/oncoming%v", h, oncoming), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				bus := sim.NewBus(0, 0, h+.23)
				bus.Speed = 150
				bus.SteeringAngle = .03
				obs := VehicleObstacle{Pos: u.Mul(25).Add(r.Mul(40)), Heading: h, Length: 46, Width: 22}
				if oncoming {
					obs.Heading += math.Pi
					obs.Speed = 30
				}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: 4}}}
				ad := NewAutopilotDriver()
				f := autopilotFrame{bus: bus, world: w, laneWidth: 52, speedAbs: 150, obstacles: []VehicleObstacle{obs}}
				if limit := ad.controlSweptSpeed(f, 0); math.IsInf(limit, 1) || limit >= bus.Speed {
					t.Fatalf("yawed nose needs braking: limit=%v", limit)
				}
				f.obstacles[0].Pos = u.Mul(25).Add(r.Mul(100))
				if limit := ad.controlSweptSpeed(f, 0); !math.IsInf(limit, 1) {
					t.Fatalf("clear neighboring lane blocked: %v", limit)
				}
				// A yielding vehicle wholly behind the bus must not prevent departure.
				f.obstacles[0].Pos = u.Mul(-130)
				f.obstacles[0].Speed = 0
				if limit := ad.controlSweptSpeed(f, 0); !math.IsInf(limit, 1) {
					t.Fatalf("rear follower blocked: %v", limit)
				}
			})
		}
	}
}
