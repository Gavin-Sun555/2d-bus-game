package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"math/rand"
	"testing"
)

func TestRoundaboutStaticRangeRetainsEveryContact(t *testing.T) {
	rng := rand.New(rand.NewSource(748326))
	excluded := 0
	for epoch := 0; epoch < 1200; epoch++ {
		h := rng.Float64() * 6.28
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		points := []sim.Vec2{{X: float64(epoch%7) * 200, Y: float64(epoch%5) * 100}}
		for i := 1; i < 24; i++ {
			points = append(points, points[i-1].Add(u.Mul(20)).Add(r.Mul(rng.Float64()*30-15)))
		}
		s := rng.Float64()*900 - 250
		base := entryPointAt(points, u, s)
		pos := base.Add(sim.Vec2{X: rng.Float64()*120 - 60, Y: rng.Float64()*120 - 60})
		bus := sim.NewBus(pos.X, pos.Y, h+.4)
		bus.SetModel(sim.BusModel(epoch % 3))
		f := autopilotFrame{bus: bus}
		bounds := roundaboutStaticEntryRange(f, points, u, s)
		if !bounds.valid {
			t.Fatal("valid entry did not produce bounds")
		}
		shift := bus.Pos.Sub(base)
		for j := 0; j < 12; j++ {
			center := entryPointAt(points, u, s+float64(j)*40)
			own := sim.OBB{Center: center, Heading: rng.Float64() * 6.28, HalfLen: bus.Length/2 + 10, HalfWid: bus.Width/2 + 8}
			lag := own
			lag.Center = lag.Center.Add(shift)
			lag.Heading += .6
			other := sim.OBB{Center: center.Add(sim.Vec2{X: rng.Float64()*4000 - 2000, Y: rng.Float64()*4000 - 2000}), Heading: rng.Float64() * 6.28, HalfLen: 20 + rng.Float64()*200, HalfWid: 10 + rng.Float64()*50}
			if j%4 == 0 {
				other.Center = lag.Center
			}
			if bounds.excludes(other) {
				excluded++
				if hit, _, _ := sim.CheckOBBCollision(own, other); hit {
					t.Fatal("dropped nominal contact")
				}
				if hit, _, _ := sim.CheckOBBCollision(lag, other); hit {
					t.Fatal("dropped lagged contact")
				}
			}
		}
	}
	if excluded < 5000 {
		t.Fatalf("insufficient far candidates: %d", excluded)
	}
}

func TestRoundaboutCandidateFallbackAndPriority(t *testing.T) {
	rb := &sim.Roundabout{Radius: 240, InnerRadius: 120}
	f := autopilotFrame{bus: sim.NewBus(-330, 0, 0)}
	f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: 0, Y: 200}, Heading: 0, Length: 48, Width: 22, Speed: 2000})
	f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: -330}, Heading: 0, Length: 48, Width: 22})
	var store [128]int
	ids, ok := roundaboutMovingCandidates(f, rb, store[:0])
	if !ok || len(ids) != 1 || ids[0] != 0 {
		t.Fatal("fast circulating vehicle omitted or approach admitted", ids, ok)
	}
	empty, ok := roundaboutMovingCandidates(f, rb, store[:0], []int{})
	if !ok || len(empty) != 0 {
		t.Fatal("empty explicit selection changed")
	}
	for len(f.obstacles) < 140 {
		f.obstacles = append(f.obstacles, f.obstacles[0])
	}
	if _, ok := roundaboutMovingCandidates(f, rb, store[:0]); ok {
		t.Fatal("dense ring did not fall back")
	}
	points := []sim.Vec2{{X: -300}, {X: -200}, {Y: 200}}
	bounds := roundaboutStaticEntryRange(f, points, sim.Vec2{X: 1}, -20)
	if !bounds.excludes(sim.OBB{Center: sim.Vec2{X: 10000, Y: 10000}, HalfLen: 24, HalfWid: 11}) {
		t.Fatal("far static body retained")
	}
	if bounds.excludes(sim.OBB{Center: sim.Vec2{X: math.NaN()}, HalfLen: 24, HalfWid: 11}) {
		t.Fatal("non-finite body did not fall back")
	}
	f.bus.Length = math.NaN()
	if roundaboutStaticEntryRange(f, points, sim.Vec2{X: 1}, 0).valid {
		t.Fatal("invalid vehicle geometry produced a filter")
	}
}

func TestRoundaboutCandidatesKeepOriginalSpeed(t *testing.T) {
	rng := rand.New(rand.NewSource(792371))
	finite, free := 0, 0
	for epoch := 0; epoch < 240; epoch++ {
		lw := []float64{42, 52, 78}[epoch%3]
		h := float64(epoch%4) * math.Pi / 2
		u := sim.UnitFromAngle(h)
		right := sim.Vec2{X: -u.Y, Y: u.X}
		rb := sim.Roundabout{Center: sim.Vec2{X: float64(epoch%5) * 500, Y: float64(epoch%7) * 300}, Radius: 240, InnerRadius: 120}
		rot := func(p sim.Vec2) sim.Vec2 { return rb.Center.Add(u.Mul(p.X)).Add(right.Mul(p.Y)) }
		outer := roundaboutOuterLaneRadius(&rb)
		angle := math.Pi - math.Asin(math.Min(1.5*lw, outer*.8)/outer)
		entry := sim.Vec2{X: math.Cos(angle) * outer, Y: math.Sin(angle) * outer}
		a := NewAutopilotDriver()
		a.drivePath = []sim.Vec2{rot(entry.Sub(sim.Vec2{X: 600}))}
		for i := 0; i <= 32; i++ {
			theta := angle - math.Pi*float64(i)/32
			a.drivePath = append(a.drivePath, rot(sim.UnitFromAngle(theta).Mul(outer)))
		}
		theta := angle - math.Pi
		a.drivePath = append(a.drivePath, a.drivePath[len(a.drivePath)-1].Add(u.Mul(math.Sin(theta)*800)).Add(right.Mul(-math.Cos(theta)*800)))
		a.cumLen = make([]float64, len(a.drivePath))
		for i := 1; i < len(a.drivePath); i++ {
			a.cumLen[i] = a.cumLen[i-1] + a.drivePath[i].Distance(a.drivePath[i-1])
		}
		a.LaneOffset = -lw
		a.parallelEntryCenter = &rb.Center
		p := rot(sim.Vec2{X: -330 + rng.Float64()*40, Y: lw*.5 + rng.Float64()*40 - 20})
		bus := sim.NewBus(p.X, p.Y, h+(rng.Float64()-.5)*.5)
		bus.SetModel(sim.BusModel(epoch % 3))
		bus.TrailerHeading = bus.Heading + .3
		bus.Speed = float64(epoch%5) * 30
		f := autopilotFrame{bus: bus, world: &sim.World{Roundabouts: []sim.Roundabout{rb}}, laneWidth: lw, s: 470, speedAbs: bus.Speed}
		count := []int{0, 1, 8, 96, 864}[epoch%5]
		for i := 0; i < count; i++ {
			q := rot(sim.Vec2{X: rng.Float64()*12000 - 6000, Y: rng.Float64()*12000 - 6000})
			heading := rng.Float64() * 6.28
			if i%17 == 0 {
				theta := rng.Float64() * 6.28
				q = rb.Center.Add(sim.UnitFromAngle(theta).Mul(200))
				heading = theta - math.Pi/2
			}
			f.obstacles = append(f.obstacles, VehicleObstacle{Pos: q, Heading: heading, Speed: []float64{0, 30, 90, 400, 1800, -90}[i%6], Length: 48 + float64(i%4)*48, Width: 22})
		}
		original := *a
		got := a.roundaboutEntryObstacleSpeed(f)
		want := original.originalTemporaryRoundabout(f)
		if math.Float64bits(got) != math.Float64bits(want) {
			t.Fatalf("epoch%d speed changed %g/%g", epoch, got, want)
		}
		if math.IsInf(want, 1) {
			free++
		} else {
			finite++
		}
	}
	if finite < 10 || free < 10 {
		t.Fatalf("insufficient blocked/free coverage: %d/%d", finite, free)
	}
}

// Frozen original moving scan; do not share the optimized candidate filter.
func originalEntryMovingConflict(f autopilotFrame, rb *sim.Roundabout, points []sim.Vec2, forward sim.Vec2, s float64, selection ...[]int) bool {
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += points[i].Distance(points[i-1])
	}
	for distance := 0.0; s+distance < total-2; distance += 8 {
		position := entryPointAt(points, forward, s+distance)
		tangent := entryPointAt(points, forward, s+distance+2).Sub(position)
		body := sim.OBB{Center: position, Heading: math.Atan2(tangent.Y, tangent.X), HalfLen: f.bus.Length/2 + 12, HalfWid: f.bus.Width/2 + 10}
		arrival := roundaboutEntryArrival(distance, f.speedAbs)
		count := len(f.obstacles)
		if len(selection) > 0 {
			count = len(selection[0])
		}
		for i := 0; i < count; i++ {
			index := i
			if len(selection) > 0 {
				index = selection[0][i]
			}
			obstacle := f.obstacles[index]
			// Only circulating traffic has priority here. A following vehicle
			// on the approach must not make the bus stop at the yield line.
			obstacleBody := sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: obstacle.Length / 2, HalfWid: obstacle.Width / 2}
			if !sim.HasRoundaboutPriority(*rb, obstacleBody) {
				continue
			}
			for time := math.Max(0, arrival-0.6); time <= arrival+0.8; time += 0.08 {
				other := roundaboutObstacleAt(rb, obstacle, time)
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					return true
				}
			}
		}
	}
	return false
}
