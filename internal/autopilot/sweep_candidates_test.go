package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

func TestArticulatedSweepCandidatesMatchFrozenOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(20261028))
	world := sim.NewWorld(52, sim.MapThemeMetropolis)
	reduced, limited := 0, 0
	for scenario := 0; scenario < 180; scenario++ {
		bus := sim.NewBus(rng.Float64()*10000-5000, rng.Float64()*10000-5000, rng.Float64()*6.28)
		bus.SetModel(sim.BusArticulated)
		if scenario%15 == 0 {
			bus.Pos.X = 1e9
			bus.Pos.Y = -1e9
			bus.Heading = 1e6
		}
		bus.Speed = rng.Float64() * 445
		bus.SteeringAngle = (rng.Float64() - .5) * .6
		bus.TrailerHeading = bus.Heading + (rng.Float64()-.5)*1.8
		bus.PreparePosePredictions()
		f := autopilotFrame{bus: bus, world: world, laneWidth: 52, speedAbs: bus.Speed}
		for i := 0; i < 864; i++ {
			f.obstacles = append(f.obstacles, VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{X: rng.Float64()*8000 - 4000, Y: rng.Float64()*8000 - 4000}), Heading: rng.Float64() * 6.28, Speed: rng.Float64() * 400, Length: 20 + rng.Float64()*180, Width: 20 + rng.Float64()*20})
		}
		if scenario%3 != 0 {
			f.obstacles[0].Pos = bus.Pos.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(float64(scenario%4)*80 - 96))
		}
		steer := (rng.Float64() - .5) * 2
		speed := math.Max(16, f.speedAbs)
		var storage [128]int
		ids, filtered := articulatedSweepCandidates(f, speed, steer, storage[:0])
		if filtered && len(ids) < len(f.obstacles) {
			reduced++
		}
		chosen := make(map[int]bool)
		for i, id := range ids {
			chosen[id] = true
			if i > 0 && ids[i-1] >= id {
				t.Fatal("order changed")
			}
		}
		// Independently enumerate every original broad-phase pass at all 16 poses,
		// including poses after a hitch rejection. None may disappear from candidates.
		copy := *bus
		for tick := 1; tick <= 16; tick++ {
			wheel := copy.SteeringAngle + sim.Clamp(steer*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
			angle := copy.Heading + speed/copy.WheelBase*math.Tan(wheel)*.05
			pos := copy.Pos.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(speed * .05))
			copy = copy.AtPose(pos, angle)
			copy.SteeringAngle = wheel
			for id, ob := range f.obstacles {
				d := ob.Pos.Sub(copy.Pos)
				r := copy.OverallLength() + ob.Length + 100
				if filtered && !(d.Dot(d) > r*r) && !chosen[id] {
					t.Fatalf("scenario%d tick%d missed obstacle%d", scenario, tick, id)
				}
			}
		}
		ad := NewAutopilotDriver()
		for _, straight := range []bool{false, true} {
			got := ad.articulatedSweptSpeed(f, steer, straight)
			want := frozenArticulatedSweep(f, steer, straight)
			if math.Float64bits(got) != math.Float64bits(want) {
				t.Fatalf("scenario%d limits changed %v/%v", scenario, got, want)
			}
			if !math.IsInf(got, 1) {
				limited++
			}
		}
	}
	if reduced < 170 || limited == 0 {
		t.Fatalf("insufficient exercise reduced%d limited%d", reduced, limited)
	}
	t.Logf("180 scenarios x 16 pose inclusion checks; 360 exact limit comparisons; %d reductions, %d finite limits", reduced, limited)
}

func TestArticulatedSweepCandidateFallbackAndEdges(t *testing.T) {
	bus := sim.NewBus(0, 0, 0)
	bus.SetModel(sim.BusArticulated)
	f := autopilotFrame{bus: bus, speedAbs: 200}
	for i := 0; i < 864; i++ {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: float64(i)*1000 + 1000}, Length: 48, Width: 22})
	}
	var buf [128]int
	reach := bus.OverallLength() + 48 + 100 + 200*.8
	f.obstacles[0].Pos.X = reach
	f.obstacles[1].Pos.X = math.Nextafter(reach, 0)
	f.obstacles[2].Length = math.Inf(1)
	f.obstacles[3].Length = -10000
	ids, ok := articulatedSweepCandidates(f, 200, 0, buf[:0])
	if !ok {
		t.Fatal("ordinary fast path unavailable")
	}
	seen := map[int]bool{}
	for _, i := range ids {
		seen[i] = true
	}
	for i := 0; i < 4; i++ {
		if !seen[i] {
			t.Fatalf("boundary/abnormal obstacle%d lost", i)
		}
	}
	for i := range f.obstacles {
		f.obstacles[i].Pos = bus.Pos
	}
	if _, ok := articulatedSweepCandidates(f, 200, 0, buf[:0]); ok {
		t.Fatal("dense overflow must fall back")
	}
	original := *bus
	for _, mutate := range []func(){func() { bus.WheelBase = 0 }, func() { bus.SteeringAngle = math.Pi / 2 }, func() { bus.Pos.X = math.NaN() }, func() { bus.SteerTurnSpeed = -1 }} {
		*bus = original
		mutate()
		if _, ok := articulatedSweepCandidates(f, 200, 0, buf[:0]); ok {
			t.Fatal("abnormal kinematics must fall back")
		}
	}
	*bus = original
	if _, ok := articulatedSweepCandidates(f, math.NaN(), 0, buf[:0]); ok {
		t.Fatal("NaN speed must fall back")
	}
}

func TestArticulatedSweepCandidateNilDriver(t *testing.T) {
	bus := sim.NewBus(0, 0, 0)
	bus.SetModel(sim.BusArticulated)
	f := autopilotFrame{bus: bus, world: &sim.World{}, laneWidth: 52, speedAbs: 40}
	for i := 0; i < 100; i++ {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: float64(i)*1000 + 1000}, Length: 48, Width: 22})
	}
	var driver *AutopilotDriver
	if got, want := driver.articulatedSweptSpeed(f, 0), frozenArticulatedSweep(f, 0); math.Float64bits(got) != math.Float64bits(want) {
		t.Fatal("nil receiver behavior changed")
	}
}

func frozenArticulatedSweep(f autopilotFrame, steer float64, checkStraight ...bool) float64 {
	if !f.bus.IsArticulated() || f.world == nil {
		return math.Inf(1)
	}
	copy := *f.bus
	speed := math.Max(16, f.speedAbs)
	for tick := 1; tick <= 16; tick++ {
		elapsed := float64(tick) * .05
		wheel := copy.SteeringAngle + sim.Clamp(steer*copy.MaxSteerAngle-copy.SteeringAngle, -copy.SteerTurnSpeed*.05, copy.SteerTurnSpeed*.05)
		angle := copy.Heading + speed/copy.WheelBase*math.Tan(wheel)*.05
		pos := copy.Pos.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(speed * .05))
		copy = copy.AtPose(pos, angle)
		copy.SteeringAngle = wheel
		var ownBodies [2]sim.OBB
		ownReady := false
		collision := false
		if tick%4 == 0 && ((len(checkStraight) > 0 && checkStraight[0]) || math.Abs(wheel) > .02 || math.Abs(sim.NormalizeAngle(copy.Heading-copy.TailHeading())) > .03) {
			if copy.CheckBoundaries(f.world, f.laneWidth).Zone == sim.ZoneHardBarrier {
				collision = true
			}
		}
		for _, ob := range f.obstacles {
			delta := ob.Pos.Sub(copy.Pos)
			reach := copy.OverallLength() + ob.Length + 100
			if delta.Dot(delta) > reach*reach {
				continue
			}
			hit := false
			for _, body := range predictedObstacleBodies(ob, math.Max(0, ob.Speed)*elapsed) {
				if body.HalfLen <= 0 {
					continue
				}
				body.HalfLen += 1
				body.HalfWid += 1
				if !ownReady {
					ownBodies, ownReady = copy.BodyParts(), true
				}
				for _, own := range ownBodies {
					if own.HalfLen <= 0 {
						continue
					}
					if touching, _, depth := sim.CheckOBBCollision(own, body); touching && depth > 0 {
						hit = true
						break
					}
				}
				if hit {
					break
				}
			}
			if hit {
				collision = true
				break
			}
		}
		if collision {
			return math.Sqrt(2 * 180 * math.Max(0, speed*elapsed-20))
		}
	}
	return math.Inf(1)
}
