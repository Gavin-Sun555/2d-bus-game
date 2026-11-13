package autopilot

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
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

func TestControlSweepLazyTrailerMatchesFullCollision(t *testing.T) {
	rng := rand.New(rand.NewSource(720))
	bus := sim.NewBus(1000, -900, .25)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -.22
	for i := range 100 {
		front := sim.OBB{
			Center:  bus.Pos.Add(sim.Vec2{X: rng.Float64()*450 - 100, Y: rng.Float64()*160 - 80}),
			Heading: rng.Float64()*2 - 1,
			HalfLen: bus.Length/2 + 1, HalfWid: bus.Width/2 + 1,
		}
		sample := sweepSample{front: front}
		bodies := sweepBodies(bus, front)
		for j := range 12 {
			// Sample near the source, the target, and the intervening lane.
			pos := bus.Pos.Add(front.Center.Sub(bus.Pos).Mul(float64(j%3) / 2))
			obstacle := VehicleObstacle{
				Pos:     pos.Add(sim.Vec2{X: rng.Float64()*450 - 225, Y: rng.Float64()*240 - 120}),
				Heading: rng.Float64() * math.Pi, Length: 46, Width: 24,
			}
			distance := rng.Float64() * 60
			if got, want := sample.collides(bus, obstacle, distance, 1, 1), sweepBodiesCollision(bodies, obstacle, distance, 1, 1); got != want {
				t.Fatalf("sample %d obstacle %d: collision = %t, full sweep = %t", i, j, got, want)
			}
		}
	}
}
