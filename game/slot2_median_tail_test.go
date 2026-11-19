package game

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Keep the actual player, five bus controllers, neighboring cars and road
// geometry from slot 2. Unrelated fleets and operational accounting are omitted.
//
//go:embed testdata/slot2-median-tail-20261115.json.gz
var slot2MedianTailScene []byte

//go:embed testdata/slot2-corner-crossing-20261115.json.gz
var slot2CornerCrossingScene []byte

//go:embed testdata/slot2-median-straddle-20261116.json.gz
var slot2MedianStraddleScene []byte

//go:embed testdata/slot2-player-trailer-20261116.json.gz
var slot2PlayerTrailerScene []byte

//go:embed testdata/slot2-leading-car-20261116.json.gz
var slot2LeadingCarScene []byte

//go:embed testdata/slot2-aligned-wheel-sweep-20261116.json.gz
var slot2AlignedWheelScene []byte

//go:embed testdata/slot2-close-recenter-20261116.json.gz
var slot2CloseRecenterScene []byte

//go:embed testdata/slot2-car-bus-retreat-20261116.json.gz
var slot2CarBusRetreatScene []byte

//go:embed testdata/slot2-parallel-tail-overlap-20261116.json.gz
var slot2ParallelTrailerOverlapScene []byte

func TestSavedSlot2ParallelTrailerOverlapReleases(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2ParallelTrailerOverlapScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	b := fleetBus(t, g, 123)
	start := b.Bus.Pos
	health := map[int]float64{}
	for _, a := range g.AIBusMgr.Buses {
		health[a.ID] = a.Bus.Durability
		a.Driver.BeforeRecovery = func() { t.Fatal("bus relocated instead of opening rear space") }
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset instead of relaying retreat", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset instead of relaying retreat", e) }
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < health[a.ID] {
				t.Fatal("bus damaged", a.ID)
			}
		}
		if b.Bus.Pos.Distance(start) > 500 {
			return
		}
	}
	t.Fatalf("parallel trailer overlap stayed blocked: %v", b.Bus.Pos)
}

func TestSavedSlot2CarBusRetreatChainReleases(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2CarBusRetreatScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	b := fleetBus(t, g, 122)
	start := b.Bus.Pos
	health := map[int]float64{}
	for _, a := range g.AIBusMgr.Buses {
		health[a.ID] = a.Bus.Durability
		a.Driver.BeforeRecovery = func() { t.Fatal("bus relocated instead of opening rear space") }
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset instead of relaying retreat", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset instead of relaying retreat", e) }
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < health[a.ID] {
				t.Fatal("bus damaged", a.ID)
			}
		}
		if b.Bus.Pos.Distance(start) > 500 {
			return
		}
	}
	t.Fatalf("car-to-bus retreat chain stayed blocked: %v", b.Bus.Pos)
}

func TestSavedSlot2CloseRecenterClearsJunction(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2CloseRecenterScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	starts, health := map[int]sim.Vec2{}, map[int]float64{}
	for _, b := range g.AIBusMgr.Buses {
		starts[b.ID], health[b.ID] = b.Bus.Pos, b.Bus.Durability
		b.Driver.BeforeRecovery = func() { t.Fatal("bus relocated instead of recentering") }
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset during recenter", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset during recenter", e) }
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		cleared := true
		for _, b := range g.AIBusMgr.Buses {
			if b.Bus.Durability < health[b.ID] {
				t.Fatal("bus damaged during recenter", b.ID)
			}
			if b.Bus.Pos.Distance(starts[b.ID]) < 500 {
				cleared = false
			}
		}
		if cleared {
			return
		}
	}
	t.Fatal("close follower kept the junction blocked")
}

func TestSavedSlot2AlignedWheelSweepQueueReleases(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2AlignedWheelScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	starts, health := map[int]sim.Vec2{}, map[int]float64{}
	for _, b := range g.AIBusMgr.Buses {
		starts[b.ID], health[b.ID] = b.Bus.Pos, b.Bus.Durability
		b.Driver.BeforeRecovery = func() { t.Fatal("bus relocated instead of releasing wheel sweep") }
	}
	// This fixture retains the west block only. Ambient cars spawned on
	// omitted roads are covered by the full-save soak, not this local test.
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.Pos.Distance(starts[173]) < 2000 {
			t.Fatal("local car reset instead of clearing sweep", e)
		}
	}
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset instead of clearing sweep", e) }
	for tick := 0; tick < 60*90; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		cleared := true
		for _, b := range g.AIBusMgr.Buses {
			if b.Bus.Durability < health[b.ID] {
				t.Fatal("bus damaged", b.ID)
			}
			if (b.ID == 84 || b.ID == 173) && b.Bus.Pos.Distance(starts[b.ID]) < 500 {
				cleared = false
			}

		}
		if cleared {
			return
		}
	}
	for _, b := range g.AIBusMgr.Buses {
		if (b.ID == 84 || b.ID == 173) && b.Bus.Pos.Distance(starts[b.ID]) < 500 {
			t.Fatalf("bus%d still blocked by aligned wheel sweep: %v", b.ID, b.Bus.Pos)
		}
	}
}

func TestSavedSlot2LeadingCarReleasesStationBus(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2LeadingCarScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	b := fleetBus(t, g, 173)
	start, health := b.Bus.Pos, b.Bus.Durability
	b.Driver.BeforeRecovery = func() { t.Fatal("station bus relocated instead of letting car clear") }
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("leading car reset", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("station bus reset", e) }
	for tick := 0; tick < 60*30; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if b.Bus.Durability < health {
			t.Fatal("station bus damaged while car cleared")
		}
		if b.Bus.Pos.Distance(start) > 500 {
			return
		}
	}
	t.Fatalf("station bus remained wedged with leading car: %v", b.Bus.Pos)
}

func TestSavedSlot2PlayerTrailerQueueRetreats(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2PlayerTrailerScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	start, health := g.Bus.Pos, g.Bus.Durability
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player relocated instead of clearing trailer queue") }
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset instead of retreating", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset instead of clearing queue", e) }
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatal("player damaged during trailer recovery")
		}
		if g.Bus.Pos.Distance(start) > 500 {
			return
		}
	}
	t.Fatalf("player trailer queue remained blocked: %v", g.Bus.Pos)
}

func TestSavedSlot2PlayerStraddlingMedianReturnsToLane(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2MedianStraddleScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	start, health := g.Bus.Pos, g.Bus.Durability
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player relocated during median return") }
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset during median return", e) }
	for tick := 0; tick < 60*20; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health || g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatal("player damaged or relocated during median return")
		}
	}
	if g.Bus.Pos.Distance(start) < 1000 {
		t.Fatalf("saved player remained stuck: %v", g.Bus.Pos)
	}
}

func TestSavedSlot2CornerBusYieldsToCommittedCar(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2CornerCrossingScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	bus := fleetBus(t, g, 155)
	var car *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == 8433 {
			car = c
		}
	}
	if car == nil {
		t.Fatal("missing crossing car")
	}
	startBus, startCar := bus.Bus.Pos, car.Pos
	health := bus.Bus.Durability
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset", e) }
	for tick := 0; tick < 60*60; tick++ {
		before := bus.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Durability < health || bus.Bus.Pos.Distance(before) > math.Abs(bus.Bus.Speed)/60+.05 {
			t.Fatal("bus hit or relocated during crossing retreat")
		}
	}
	if car.Pos.Distance(startCar) < 500 || bus.Bus.Pos.Distance(startBus) < 500 {
		t.Fatalf("corner remained blocked: car%v bus%v", car.Pos, bus.Bus.Pos)
	}
}

func TestSavedSlot2MedianAndTrailerQueuesRelease(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot2MedianTailScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	starts := map[int]sim.Vec2{0: g.Bus.Pos}
	health := map[int]float64{0: g.Bus.Durability}
	for _, a := range g.AIBusMgr.Buses {
		starts[a.ID], health[a.ID] = a.Bus.Pos, a.Bus.Durability
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("car reset instead of clearing queue", e) }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("bus reset instead of clearing queue", e) }
	for _, a := range g.AIBusMgr.Buses {
		a.Driver.BeforeRecovery = func() { t.Fatal("bus relocated instead of clearing queue") }
	}
	for tick := 0; tick < 60*105; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health[0] || g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 {
			t.Fatal("player damaged or relocated")
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < health[a.ID] {
				t.Fatal("bus damaged", a.ID)
			}
		}
	}
	if g.Bus.Pos.Distance(starts[0]) < 1000 {
		t.Fatal("player remained pinned by median protection")
	}
	for _, a := range g.AIBusMgr.Buses {
		if a.Bus.Pos.Distance(starts[a.ID]) < 500 {
			t.Fatalf("bus%d queue remained pinned: %v", a.ID, a.Bus.Pos)
		}
	}
}
