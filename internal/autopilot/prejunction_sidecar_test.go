package autopilot

import (
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/sidecar-prejunction-2026-11-08.json
var preJunctionSidecarFixture []byte

//go:embed testdata/sidecar-mouth-2026-11-08.json
var mouthSidecarFixture []byte

func TestSavedMouthSidecarClearsWithoutReset(t *testing.T) {
	replaySidecarFixture(t, mouthSidecarFixture, true)
}

func TestSavedPreJunctionSidecarClearsWithoutReset(t *testing.T) {
	replaySidecarFixture(t, preJunctionSidecarFixture, false)
}

func replaySidecarFixture(t *testing.T, raw []byte, mouth bool) {
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if err := json.Unmarshal(raw, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{world: x.World, bus: x.Bus, laneWidth: 52}
	if !mouth && (!x.World.InsideJunction(x.Bus.Pos, 52, x.Bus.OverallLength()/2) || sidecarBodyInJunction(f)) {
		t.Fatal("fixture does not reproduce overly expanded junction guard")
	}
	if mouth && (!sidecarBodyInJunction(f) || !sidecarAtIncomingThroat(f)) {
		t.Fatal("fixture must straddle the incoming mouth")
	}
	ad.BeforeRecovery = func() { t.Fatal("repositioned instead of driving out") }
	var obs []VehicleObstacle
	for _, car := range x.Cars {
		obs = append(obs, VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width})
	}
	for _, bus := range x.OtherBuses {
		obs = append(obs, ObserveBus(bus))
	}
	start, passengers := x.Bus.Pos, x.Bus.PassengerCount
	for tick := 0; tick < 7200; tick++ {
		x.Lights.Update(1. / 60)
		before := x.Bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(x.Bus, x.Route, x.World, x.Lights, obs, 52, 1./60)
		x.Bus.Update(1./60, th, br, st, hb, doors)
		if x.Bus.Pos.Distance(before) > math.Abs(x.Bus.Speed)/60+.05 || x.Bus.PassengerCount != passengers {
			t.Fatal("relocated or lost passengers")
		}
		for _, ob := range obs {
			if sweepBodiesCollision(x.Bus.BodyParts(), ob, 0, 0, 0) {
				t.Fatalf("touched sidecar or follower tick%d", tick)
			}
		}
		if x.Bus.Pos.Distance(start) > 1000 {
			t.Logf("cleared frozen sidecar in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("pre-junction sidecar remained locked at %v", x.Bus.Pos)
}

func TestSidecarJunctionGuardIncludesPhysicalTrailer(t *testing.T) {
	w, _ := turnTestWorld(8, 8, 0, 52)
	g := w.JunctionAt(sim.Vec2{}, 52)
	bus := sim.NewBus(g.HalfX+100, 26, 0)
	bus.SetModel(sim.BusArticulated)
	f := autopilotFrame{world: w, bus: bus, laneWidth: 52}
	if w.InsideJunction(bus.Pos, 52, bus.Length/2) || !sidecarBodyInJunction(f) {
		t.Fatal("rear section crossing junction was ignored")
	}
	bus.Pos.X += bus.OverallLength()
	bus.ResetArticulation()
	if sidecarBodyInJunction(f) {
		t.Fatal("fully cleared bus classified as inside junction")
	}
}

func TestLocalRecoveryObserverRunsBeforeReposition(t *testing.T) {
	ad, f, _ := loadStraddledLeft(t)
	start := f.bus.Pos
	ad.strandedMedianStart, ad.strandedMedianWait = start, 10
	called := false
	ad.BeforeRecovery = func() {
		called = true
		if f.bus.Pos != start {
			t.Fatal("observer received post-recovery pose")
		}
	}
	if !ad.RecoverStrandedMedianApproach(f.bus, f.world, f.obstacles, 52, .1) || !called {
		t.Fatal("local jam recovery did not notify observer")
	}
	if ad.Snapshot().Public.BeforeRecovery != nil {
		t.Fatal("snapshot retained a live observer")
	}
}

func TestSidecarEntryThroatDoesNotIncludeTurnOrExit(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, _ := turnTestWorld(8, 8, 0, 52)
		g := w.JunctionAt(sim.Vec2{}, 52)
		u := sim.UnitFromAngle(heading)
		right := sim.Vec2{X: -u.Y, Y: u.X}
		half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
		bus := sim.NewBus(0, 0, heading)
		bus.SetModel(sim.BusArticulated)
		f := autopilotFrame{world: w, bus: bus, laneWidth: 52}
		bus.Pos = u.Mul(-half + 30).Add(right.Mul(78))
		bus.ResetArticulation()
		if !sidecarAtIncomingThroat(f) {
			t.Fatal("entry straightening excluded")
		}
		bus.Heading = heading + .6
		if sidecarAtIncomingThroat(f) {
			t.Fatal("turning bus admitted to straight crawl")
		}
		bus.Heading = heading
		for _, along := range []float64{0, half + 50, -half - bus.OverallLength()} {
			bus.Pos = u.Mul(along).Add(right.Mul(78))
			bus.ResetArticulation()
			if sidecarAtIncomingThroat(f) {
				t.Fatalf("non-entry position %.0f admitted", along)
			}
		}
	}
}
