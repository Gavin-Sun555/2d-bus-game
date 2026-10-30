package autopilot

import (
	"math"
	"math/rand"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func highSpeedSweepFrame(kph int) (*AutopilotDriver, autopilotFrame) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -20000}, P2: sim.Vec2{X: 20000}, Lanes: 8}, {P1: sim.Vec2{Y: -20000}, P2: sim.Vec2{Y: 20000}, Lanes: 8}}, Intersections: []sim.Vec2{{}}}
	b := sim.NewBus(0, 182, 0)
	b.SetModel(sim.BusArticulated)
	b.Speed = sim.SpeedKPH(kph)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.drivePath = []sim.Vec2{{Y: 182}, {X: 5000, Y: 182}}
	ad.cumLen = []float64{0, 5000}
	return ad, autopilotFrame{bus: b, world: w, speedAbs: b.Speed, laneWidth: 52}
}
func TestJunctionPredictionMatchesFrozenOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(20261029))
	finite := 0
	for scenario := 0; scenario < 120; scenario++ {
		ad, f := highSpeedSweepFrame([]int{30, 80, 100, 110, 120, 160}[scenario%6])
		if scenario%4 == 0 {
			f.bus.SetModel(sim.BusStandard)
		}
		if scenario%3 == 0 {
			ad.drivePath = []sim.Vec2{{Y: 182}, {X: 100, Y: 182}, {X: 300, Y: -100}, {X: 300, Y: -5000}}
			ad.cumLen = []float64{0, 100, 100 + math.Hypot(200, 282), 100 + math.Hypot(200, 282) + 4900}
		}
		f.bus.Heading = (rng.Float64() - .5) * .2
		f.bus.TrailerHeading = f.bus.Heading + (rng.Float64()-.5)*.9
		f.bus.Pos.X = rng.Float64() * 20
		ad.LaneOffset = (rng.Float64() - .5) * 15
		for i := 0; i < 96; i++ {
			ob := VehicleObstacle{Pos: sim.Vec2{X: rng.Float64()*4000 - 1000, Y: rng.Float64()*1500 - 750}, Heading: rng.Float64() * 6.28, Speed: rng.Float64()*650 - 50, Length: 46, Width: 22}
			if i%13 == 0 {
				b := sim.NewBus(ob.Pos.X, ob.Pos.Y, ob.Heading)
				b.SetModel(sim.BusArticulated)
				b.Speed = ob.Speed
				b.TrailerHeading = b.Heading + .4
				ob = ObserveBus(b)
			}
			f.obstacles = append(f.obstacles, ob)
		}
		if scenario%4 == 1 {
			f.obstacles[0] = VehicleObstacle{Pos: sim.Vec2{X: 180, Y: 182}, Length: 46, Width: 22}
		}
		for repeat := 0; repeat < 2; repeat++ {
			f.bus.PreparePosePredictions()
			got := ad.junctionSweptSpeed(f)
			want := ad.frozenJunctionSweptSpeed(f)
			if math.Float64bits(got) != math.Float64bits(want) {
				t.Fatalf("scenario%d repeat%d got%v want%v", scenario, repeat, got, want)
			}
			if !math.IsInf(want, 1) {
				finite++
			}
			f.bus.Pos.X += .3
		}
	}
	if finite == 0 {
		t.Fatal("no blocked sweeps exercised")
	}
}
func TestJunctionPoseStorageReuseAndReset(t *testing.T) {
	var storage [][2]sim.OBB
	c := sweepPoseCache{overflow: junctionPoseStorage(&storage, 1275, 4)}
	for i := 0; i < 319; i++ {
		b := [2]sim.OBB{{Center: sim.Vec2{X: float64(i)}}}
		c.store(i, b)
		if v, ok := c.load(i); !ok || v != b {
			t.Fatal(i)
		}
	}
	first := &storage[0]
	c = sweepPoseCache{overflow: junctionPoseStorage(&storage, 1000, 4)}
	if first != &storage[0] {
		t.Fatal("reallocated reusable storage")
	}
	for i := 0; i < 251; i++ {
		if _, ok := c.load(i); ok {
			t.Fatal("old pose reused across ticks")
		}
	}
	if x := junctionPoseStorage(&storage, math.Inf(1), 4); x != nil {
		t.Fatal("unbounded allocation")
	}
}

func TestJunctionActualCacheAndWarmAllocations(t *testing.T) {
	ad, f := highSpeedSweepFrame(120)
	for i := 0; i < 12; i++ {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: float64(i) * 2, Y: 213}, Length: 46, Width: 22})
	}
	want := ad.frozenJunctionSweptSpeed(f)
	if !math.IsInf(want, 1) {
		t.Fatal("fixture lacks actual-clearance passage")
	}
	if got := ad.junctionSweptSpeed(f); got != want {
		t.Fatal(got, want)
	}
	if ad.temporaryGeometry == nil || len(ad.temporaryGeometry.junctionActual) < 191 || len(ad.temporaryGeometry.junctionMargin) < 85 {
		t.Fatal("high-speed actual branch not cached")
	}
	if n := testing.AllocsPerRun(5, func() { ad.junctionSweptSpeed(f) }); n != 0 {
		t.Fatalf("warm junction scan allocates %g times", n)
	}
}

func TestJunctionPoseStorageGrowsGeometrically(t *testing.T) {
	var storage [][2]sim.OBB
	moves := 0
	var previous *[2]sim.OBB
	for samples := 129; samples <= 319; samples++ {
		junctionPoseStorage(&storage, float64(samples-1)*4, 4)
		if &storage[0] != previous {
			moves++
			previous = &storage[0]
		}
	}
	if moves > 2 {
		t.Fatalf("accelerating scan reallocated %d times", moves)
	}
}
