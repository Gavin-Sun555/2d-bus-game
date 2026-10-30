package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

// Pre-trial search recomputes BodyParts for each obstacle.
func (ad *AutopilotDriver) referenceSidecarSweepClear(f autopilotFrame, steer, heading, center float64, ew bool, yawGain ...float64) bool {
	// This narrow recovery predicts forward traffic only. Defer to normal
	// avoidance while a nearby vehicle is reversing.
	for _, ob := range f.obstacles {
		if ob.Speed < 0 {
			return false
		}
	}
	b := *f.bus
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	origin := sim.Vec2{Y: center}
	if !ew {
		origin = sim.Vec2{X: center}
	}
	for tick := 0; tick <= 120; tick++ {
		if tick > 0 {
			if len(yawGain) > 0 {
				steer = sim.Clamp(-sim.NormalizeAngle(b.Heading-heading)*yawGain[0], -1, 1)
			}
			th, br := ad.longitudinalControls(b.Speed, 18)
			b.SetNextAccelerationScale(1)
			b.Update(1./60, th, br, steer, false, false)
		}
		if b.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier || math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle-.025 {
			return false
		}
		for _, part := range b.BodyParts() {
			if part.HalfLen > 0 {
				yaw := part.Heading - heading
				if part.Center.Sub(origin).Dot(r)-part.HalfLen*math.Abs(math.Sin(yaw))-part.HalfWid*math.Abs(math.Cos(yaw)) < .5 {
					return false
				}
			}
		}
		for _, ob := range f.obstacles {
			padding := 2.
			if stoppedSidecar(f, ob, heading) {
				padding = .05
			}
			for _, other := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*float64(tick)/60) {
				if other.HalfLen > 0 {
					other.HalfLen += padding
					other.HalfWid += padding
					if hit, _, _ := b.CollidesBody(other); hit {
						return false
					}
				}
			}
		}
	}
	return b.Pos.Sub(f.bus.Pos).Dot(u) > 10
}

func TestSidecarPoseReuseMatchesOriginalSearch(t *testing.T) {
	rng := rand.New(rand.NewSource(73584))
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -10000}, P2: sim.Vec2{X: 10000}, Lanes: 8}}}
	for scene := 0; scene < 250; scene++ {
		bus := sim.NewBus(0, 150, rng.Float64()*.3-.15)
		bus.SetModel(sim.BusArticulated)
		bus.TrailerHeading = bus.Heading + rng.Float64()*.4 - .2
		bus.Speed = rng.Float64() * 25
		f := autopilotFrame{bus: bus, world: w, laneWidth: 52, speedAbs: bus.Speed}
		if scene%5 != 0 {
			for i := 0; i < 24; i++ {
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: rng.Float64()*900 - 450, Y: rng.Float64() * 400}, Heading: rng.Float64()*.3 - .15, Length: 46, Width: 22, Speed: float64(i%3) * 20})
			}
		}
		ad := NewAutopilotDriver()
		for _, steer := range []float64{-1, -.3, 0, .3, 1} {
			for _, gain := range []float64{0, 3, 8} {
				var g []float64
				if gain != 0 {
					g = []float64{gain}
				}
				got := ad.sidecarSweepClear(f, steer, 0, 0, true, g...)
				want := ad.referenceSidecarSweepClear(f, steer, 0, 0, true, g...)
				if got != want {
					t.Fatalf("scene%d steer%g gain%g got%v want%v", scene, steer, gain, got, want)
				}
			}
		}
	}
}
