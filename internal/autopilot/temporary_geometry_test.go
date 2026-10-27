package autopilot

import (
	"math"
	"math/rand"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestTemporaryRoundaboutMatchesOriginal(t *testing.T) {
	rng := rand.New(rand.NewSource(20261027))
	var a AutopilotDriver
	for epoch := 0; epoch < 120; epoch++ {
		lw := []float64{42, 52, 78}[epoch%3]
		rb := sim.Roundabout{Center: sim.Vec2{X: float64(epoch%4) * 250, Y: float64(epoch%5) * 100}, Radius: 240, InnerRadius: 120}
		outer := roundaboutOuterLaneRadius(&rb)
		angle := math.Pi - math.Asin(math.Min(1.5*lw, outer*.8)/outer)
		entry := rb.Center.Add(sim.Vec2{X: math.Cos(angle), Y: math.Sin(angle)}.Mul(outer))
		a.drivePath = []sim.Vec2{entry.Sub(sim.Vec2{X: 600})}
		for i := 0; i <= 32; i++ {
			h := angle - math.Pi*float64(i)/32
			a.drivePath = append(a.drivePath, rb.Center.Add(sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}.Mul(outer)))
		}
		last := a.drivePath[len(a.drivePath)-1]
		h := angle - math.Pi
		a.drivePath = append(a.drivePath, last.Add(sim.Vec2{X: math.Sin(h), Y: -math.Cos(h)}.Mul(800)))
		a.cumLen = make([]float64, len(a.drivePath))
		for i := 1; i < len(a.drivePath); i++ {
			a.cumLen[i] = a.cumLen[i-1] + a.drivePath[i].Distance(a.drivePath[i-1])
		}
		a.LaneOffset = -lw
		a.parallelEntryCenter = &rb.Center
		bus := sim.NewBus(rb.Center.X-330, rb.Center.Y+lw*.5, 0)
		bus.SetModel(sim.BusModel(epoch % 3))
		bus.Speed = float64(epoch%5) * 30
		bus.Pos.Y += float64(epoch%7) - 3
		bus.Heading = (rng.Float64() - .5) * .05
		bus.TrailerHeading = bus.Heading + .15
		f := autopilotFrame{bus: bus, world: &sim.World{Roundabouts: []sim.Roundabout{rb}}, laneWidth: lw, s: 470, speedAbs: bus.Speed}
		for i := 0; i < 8+32*(epoch%4); i++ {
			f.obstacles = append(f.obstacles, VehicleObstacle{Pos: rb.Center.Add(sim.Vec2{X: rng.Float64()*1000 - 500, Y: rng.Float64()*800 - 400}), Heading: rng.Float64() * 6.28, Speed: float64(i%4) * 30, Length: 48, Width: 22})
		}
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: rb.Center.Add(sim.Vec2{X: -290, Y: lw * 1.5}), Length: 48, Width: 22})
		if !a.parallelRoundaboutJoin(f, &rb) {
			t.Fatal("parallel path not exercised")
		}
		original := a
		f.obstacles[0].Bus = bus
		got := a.roundaboutEntryObstacleSpeed(f)
		want := original.originalTemporaryRoundabout(f)
		if a.temporaryGeometry != nil && len(a.temporaryGeometry.ringIndices) != 0 {
			t.Fatal("retained selected indices")
		}
		if math.Float64bits(got) != math.Float64bits(want) {
			t.Fatalf("epoch%d limit changed %v/%v", epoch, got, want)
		}
	}
}

func TestTemporaryAdmissionMatchesOriginal(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -5000}, P2: sim.Vec2{X: 5000}, Lanes: 8},
		{P1: sim.Vec2{Y: -5000}, P2: sim.Vec2{Y: 5000}, Lanes: 8},
	}}
	w.RebuildIntersections()
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	rng := rand.New(rand.NewSource(26002))
	ad := NewAutopilotDriver()
	for epoch := 0; epoch < 180; epoch++ {
		bus := sim.NewBus(-600, 26, 0)
		bus.JunctionCorridor = &sim.JunctionCorridor{Remaining: 1, EntryHeading: 0,
			Bodies: sim.AppendPathBodies(nil, []sim.Vec2{{X: -600, Y: 26}, {X: 600, Y: 26}}, 96, 36)}
		bus.JunctionCorridor.RebuildBounds()
		f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: 52}
		tls.Intersections[0].Phase = sim.LightPhase(epoch % 4)
		for i := 0; i < 1+epoch%24; i++ {
			x, y := rng.Float64()*1000-500, rng.Float64()*800-400
			ob := VehicleObstacle{Pos: sim.Vec2{X: x, Y: y}, Heading: float64(i%4) * math.Pi / 2,
				Speed: float64(i%3) * 20, Length: 48, Width: 22}
			ob.JunctionTurn = &sim.CarJunctionTurn{Points: []sim.Vec2{{X: x, Y: y}, {X: x + 300, Y: y}, {X: x + 600, Y: y + 150}}, Segment: i % 2, EntryHeading: ob.Heading, ExitHeading: 0}
			if i%5 == 0 {
				ob.JunctionCorridor = &sim.JunctionCorridor{Remaining: 1, Entered: true,
					Bodies: sim.AppendPathBodies(nil, ob.JunctionTurn.Points, 48, 22), EntryHeading: ob.Heading}
				ob.JunctionCorridor.RebuildBounds()
			}
			f.obstacles = append(f.obstacles, ob)
		}
		got, want := ad.junctionAdmissionSpeed(f), ad.originalTemporaryAdmission(f)
		if math.Float64bits(got) != math.Float64bits(want) {
			t.Fatalf("epoch%d limit changed %v/%v", epoch, got, want)
		}
	}
	ad.Reset()
	if ad.temporaryGeometry != nil {
		t.Fatal("reset retained scratch")
	}
	var empty *AutopilotDriver
	f := autopilotFrame{world: &sim.World{}, obstacles: []VehicleObstacle{{Length: 48}}}
	if !math.IsInf(empty.roundaboutEntryObstacleSpeed(f), 1) {
		t.Fatal("nil receiver empty world changed")
	}
	ad.temporaryGeometry = &temporaryGeometry{
		ringIndices: make([]int, maxTemporaryObstacles+1),
		ringBodies:  make([]sim.OBB, maxTemporaryObstacles+1),
	}
	ad.roundaboutEntryObstacleSpeed(f)
	if ad.temporaryGeometry.ringIndices != nil || ad.temporaryGeometry.ringBodies != nil {
		t.Fatal("retained oversized temporary arrays")
	}
}
