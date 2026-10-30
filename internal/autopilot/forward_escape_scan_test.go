package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

func TestForwardEscapeScanMatchesEveryOriginalPoseCheck(t *testing.T) {
	rng := rand.New(rand.NewSource(58931))
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -10000}, P2: sim.Vec2{X: 10000}, Lanes: 8}}}
	for scene := 0; scene < 80; scene++ {
		bus := sim.NewBus(0, 150, rng.Float64()*.3-.15)
		bus.SetModel(sim.BusArticulated)
		bus.Speed = rng.Float64() * 25
		bus.TrailerHeading = bus.Heading + rng.Float64()*.3 - .15
		f := autopilotFrame{bus: bus, world: w, laneWidth: 52}
		for i := 0; i < 200; i++ {
			pos := bus.Pos.Add(sim.Vec2{X: rng.Float64()*3000 - 1500, Y: rng.Float64()*900 - 450})
			speed := []float64{0, 0, .5, 30, 200, -10}[i%6]
			if i%4 == 0 {
				other := sim.NewBus(pos.X, pos.Y, rng.Float64()*math.Pi*2)
				other.SetModel(sim.BusArticulated)
				other.Speed = speed
				other.TrailerHeading = other.Heading + .3
				f.obstacles = append(f.obstacles, ObserveBus(other))
			} else {
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: pos, Heading: rng.Float64() * math.Pi * 2, Speed: speed, Length: 46, Width: 22})
			}
		}
		var scan forwardEscapeScan
		scan.prepare(f, 18)
		for _, steer := range []float64{-1, -.3, 0, .3, 1} {
			b := *bus
			ad := NewAutopilotDriver()
			for tick := 0; tick <= 120; tick++ {
				elapsed := float64(tick) / 60
				if got, want := scan.poseClear(f, &b, elapsed), departureEscapePoseClear(f, &b, elapsed); got != want {
					t.Fatalf("scene%d steer%v tick%d optimized%v full%v", scene, steer, tick, got, want)
				}
				th, br := ad.longitudinalControls(b.Speed, 18)
				b.SetNextAccelerationScale(1)
				b.Update(1./60, th, br, steer, false, false)
			}
		}
	}
}

func TestForwardEscapeScanRetainsFullFallbacks(t *testing.T) {
	bus := sim.NewBus(0, 100, 0)
	bus.SetModel(sim.BusArticulated)
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -10000}, P2: sim.Vec2{X: 10000}, Lanes: 8}}}
	f := autopilotFrame{bus: bus, world: w, laneWidth: 52}
	for i := 0; i < 129; i++ {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: 200, Y: 100}, Length: 46, Width: 22})
	}
	var scan forwardEscapeScan
	scan.prepare(f, 18)
	if scan.valid {
		t.Fatal("dense search did not fall back")
	}
	if scan.poseClear(f, bus, 0) != departureEscapePoseClear(f, bus, 0) {
		t.Fatal("dense fallback changed result")
	}
	f.obstacles = f.obstacles[:1]
	scan.prepare(f, 18)
	if !scan.valid {
		t.Fatal("ordinary search fell back")
	}
	b := *bus
	b.Pos.X = 200
	if scan.poseClear(f, &b, 1) != departureEscapePoseClear(f, &b, 1) {
		t.Fatal("out-of-bound motion did not use the original scan")
	}
	f.obstacles[0].Speed = math.NaN()
	scan.prepare(f, 18)
	if scan.valid {
		t.Fatal("abnormal input did not fall back")
	}
}
