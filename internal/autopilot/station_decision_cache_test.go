package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math/rand"
	"slices"
	"testing"
)

func TestStationDecisionQueriesMatchLiveGeometry(t *testing.T) {
	w := sim.NewWorld(52)
	stop := w.Stops[0]
	rng := rand.New(rand.NewSource(3597))
	for scene := 0; scene < 200; scene++ {
		bus := sim.NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
		bus.SetModel(sim.BusModel(scene % 3))
		bus.Pos = bus.Pos.Add(sim.Vec2{X: rng.Float64()*300 - 150, Y: rng.Float64()*100 - 50})
		bus.TrailerHeading = bus.Heading + .2
		var obs []VehicleObstacle
		for i := 0; i < 8; i++ {
			other := sim.NewBus(stop.Pos.X+rng.Float64()*600-300, stop.Pos.Y+rng.Float64()*300-150, stop.Heading)
			other.SetModel(sim.BusModel(i % 3))
			other.DoorsOpen = i%2 == 0
			obs = append(obs, ObserveBus(other))
		}
		ad := NewAutopilotDriver()
		ad.stationQueriesActive = true
		for repeat := 0; repeat < 4; repeat++ {
			got := ad.availableStationBerths(bus, stop, obs)
			want := availableStationBerths(bus, stop, obs)
			if !slices.Equal(got, want) {
				t.Fatalf("scene%d repeat%d got%v want%v", scene, repeat, got, want)
			}
			if ad.stationBoardingCapacity(bus, stop, obs) != stationBoardingCapacity(bus, stop, obs) {
				t.Fatal("capacity changed")
			}
			g, ok := ad.stationQueueTarget(bus, stop, obs)
			v, yes := stationQueueTarget(bus, stop, obs)
			if g != v || ok != yes {
				t.Fatal("queue changed")
			}
			// Appending the outer-lane alternatives must not overwrite cached berths.
			_ = append(got, sim.Vec2{X: 999999})
		}
		// Temporary source poses and different views must never reuse the old key.
		bus.Pos.X += 120
		if got, want := ad.availableStationBerths(bus, stop, obs), availableStationBerths(bus, stop, obs); !slices.Equal(got, want) {
			t.Fatal("source pose invalidation failed")
		}
		if got, want := ad.availableStationBerths(bus, stop, obs[1:]), availableStationBerths(bus, stop, obs[1:]); !slices.Equal(got, want) {
			t.Fatal("obstacle view invalidation failed")
		}
	}
}

func TestStationDecisionCacheExpiresAtControlBoundary(t *testing.T) {
	w := sim.NewWorld(52)
	stop := w.Stops[0]
	r := &sim.Route{StopIDs: []int{0, 2, 0}}
	bus := sim.NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	ad := NewAutopilotDriver()
	ad.QueueAtStation(stop.ID)
	ad.ComputeControls(bus, r, w, nil, nil, 52, 1./60)
	if ad.stationQueriesActive || ad.stationQueries.bus != nil || ad.stationQueries.first != nil {
		t.Fatal("query cache survived the decision")
	}
	other := sim.NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	obs := []VehicleObstacle{ObserveBus(other)}
	if got, want := ad.availableStationBerths(bus, stop, obs), availableStationBerths(bus, stop, obs); !slices.Equal(got, want) {
		t.Fatal("post-decision query used old occupancy")
	}
}
