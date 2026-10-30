package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func parkInitialFleetForTest(m *AIBusManager, st *sim.BusStop, lw float64) {
	var others []*sim.Bus
	for _, ab := range m.Buses {
		ab.Route.CurrentIndex = 0
		ab.Driver.Reset()
		sim.PlaceBusInStationQueue(ab.Bus, st, others, lw)
		ab.Driver.QueueAtStation(st.ID)
		others = append(others, ab.Bus)
	}
}

func TestInitialFleetDistributedByMileageWithCorrectTargets(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
		w := sim.NewWorld(52, sim.MapThemeMetropolis)
		r := sim.GetStandardRouteForMap(w.Theme, w)
		st := w.Stops[r.StopIDs[0]]
		player := sim.NewBus(st.FrontBerthPosition().X, st.FrontBerthPosition().Y, st.Heading)
		settings := LineFleetSettings{BusCount: 8, Model: model}
		m := NewAIBusManager()
		m.ConfigureLineFleets([]LineFleetPlan{{r, settings}}, w, 52, player, true)
		if len(m.Buses) != 8 {
			t.Fatalf("model%v count %d", model, len(m.Buses))
		}
		runs, total := initialRouteRuns(r, w, 52)
		others := []*sim.Bus{player}
		for i, ab := range m.Buses {
			if ab.Bus.Model != model {
				t.Fatalf("wrong model %v, want %v", ab.Bus.Model, model)
			}
			if !safeInitialSpawn(ab.Bus, w, 52, others, nil) {
				t.Fatalf("unsafe initial bus%d model%v pos%v", i, model, ab.Bus.Pos)
			}
			best := math.Inf(1)
			ideal := (float64(i) + .5) * total / 8
			for _, run := range runs {
				delta := ab.Bus.Pos.Sub(run.start)
				along := delta.Dot(run.direction)
				if run.target != ab.Route.CurrentIndex || along < 0 || along > run.length || delta.Sub(run.direction.Mul(along)).Length() > 1e-5 {
					continue
				}
				gap := math.Abs(run.at + along - ideal)
				best = math.Min(best, math.Min(gap, total-gap))
			}
			if best > math.Max(1024, total/8*.5) {
				t.Fatalf("bus%d not spread near %.0f: error %.0f target%d", i, ideal, best, ab.Route.CurrentIndex)
			}
			others = append(others, ab.Bus)
		}
		// Editing speed or count must not relocate vehicles already in service.
		first, pos := m.Buses[0], m.Buses[0].Bus.Pos
		settings.SpeedRange = sim.SpeedRange{MinKPH: 30, MaxKPH: 50}
		m.ConfigureLineFleets([]LineFleetPlan{{r, settings}}, w, 52, player, false)
		if m.Buses[0] != first || first.Bus.Pos != pos {
			t.Fatal("live fleet was repositioned")
		}
	}
}

func TestInitialSpawnAllowsRearCarsTheirBrakingDistance(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -6000}, P2: sim.Vec2{X: 6000}, Lanes: 8}}}
	bus := sim.NewBus(0, 182, 0)
	bus.SetModel(sim.BusArticulated)
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -240, Y: 182}, Speed: 300, Length: 46, Width: 22}
	if safeInitialSpawn(bus, w, 52, nil, []*sim.TrafficCar{car}) {
		t.Fatal("spawn cut into a moving car's braking distance")
	}
	car.Pos.X = -900
	if !safeInitialSpawn(bus, w, 52, nil, []*sim.TrafficCar{car}) {
		t.Fatal("clear road rejected")
	}
}

func TestDistributedArticulatedFleetMovesOnItsAssignedLegs(t *testing.T) {
	sim.SeedSimulation(119)
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	r := sim.GetStandardRouteForMap(w.Theme, w)
	m := NewAIBusManager()
	settings := LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 40}}
	m.ConfigureLineFleets([]LineFleetPlan{{r, settings}}, w, 52, nil, true)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	starts := make([]sim.Vec2, len(m.Buses))
	moved := make([]bool, len(m.Buses))
	for i, b := range m.Buses {
		starts[i] = b.Bus.Pos
	}
	for tick := 0; tick < 3600; tick++ {
		// Isolate route localization and driving from long passenger dwell.
		for _, st := range w.Stops {
			st.WaitingPassengers = 0
		}
		tls.Update(1. / 60)
		m.Update(1./60, nil, w, tls, nil, 52)
		for i, b := range m.Buses {
			if b.Bus.Pos.Distance(starts[i]) > 300 {
				moved[i] = true
			}
			if b.Bus.Durability < 100 {
				t.Fatalf("initial fleet collision bus%d tick%d health%v", i, tick, b.Bus.Durability)
			}
		}
	}
	for i, ok := range moved {
		if !ok {
			t.Errorf("bus%d never progressed: start%v pos%v target%d state%v", i, starts[i], m.Buses[i].Bus.Pos, m.Buses[i].Route.CurrentIndex, m.Buses[i].Driver.State)
		}
	}
}

func TestMaximumDistributedArticulatedFleetAcrossMapsWithTraffic(t *testing.T) {
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		for _, lw := range []float64{42, 52, 78} {
			sim.SeedSimulation(879)
			w := sim.NewWorld(lw, theme)
			routes := sim.DefaultTransitLines(w)
			tm := NewTrafficManager(lw)
			tm.BuildForWorld(w, MaxTrafficCars, lw)
			cars := tm.GetActiveCars(4)
			var plans []LineFleetPlan
			for _, r := range routes {
				plans = append(plans, LineFleetPlan{r, LineFleetSettings{BusCount: 8, Model: sim.BusArticulated}})
			}
			st := w.Stops[routes[0].StopIDs[0]]
			player := sim.NewBus(st.FrontBerthPosition().X, st.FrontBerthPosition().Y, st.Heading)
			m := NewAIBusManager()
			m.ConfigureLineFleets(plans, w, lw, player, true, cars)
			if len(m.Buses) != len(routes)*8 {
				t.Fatal("lost vehicles")
			}
			others := []*sim.Bus{player}
			for _, ab := range m.Buses {
				if ab.Route.CurrentIndex != 0 && !safeInitialSpawn(ab.Bus, w, lw, others, cars) {
					t.Fatalf("unsafe map%v width%.0f bus%d position%v target%d", theme, lw, ab.ID, ab.Bus.Pos, ab.Route.CurrentIndex)
				}
				// A short route can fall back to its physical starting queue;
				// it must still avoid both full bus bodies and live traffic.
				for _, other := range others {
					if hit, _, _ := sim.CheckBusesCollision(ab.Bus, other); hit {
						t.Fatalf("overlap map%v width%.0f buses", theme, lw)
					}
				}
				for _, car := range cars {
					if hit, _, _ := ab.Bus.CollidesBody(sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}); hit {
						t.Fatalf("car overlap map%v width%.0f bus%d", theme, lw, ab.ID)
					}
				}
				others = append(others, ab.Bus)
			}
		}
	}
}
