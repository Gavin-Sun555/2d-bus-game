package sim

import "testing"

func TestPassengerDestinationsFollowRepeatedStopsAndInPlaceRouteEdits(t *testing.T) {
	stop := NewBusStop(0, "Station", Vec2{}, 0, 2, RGBA(1, 2, 3, 255), 52, 0)
	world := &World{Stops: []*BusStop{stop}}
	route := &Route{StopIDs: []int{0, 1, 0, 2, 3}}

	waitingFor := func(destination int) {
		stop.WaitingPassengers = 1
		stop.WaitingDestinations = []int{destination}
	}
	waitingFor(1)
	world.Update(0, route)
	if stop.WaitingDestinations[0] != 1 {
		t.Fatal("a reachable passenger was reassigned")
	}

	route.CurrentIndex = 2
	world.Update(0, route)
	if stop.WaitingDestinations[0] != 1 {
		t.Fatal("advancing to another occurrence reassigned a reachable passenger")
	}

	route.StopIDs[3] = 4
	waitingFor(2)
	world.ReconcileWaitingPassengers(route)
	if stop.WaitingPassengers != 0 || len(stop.WaitingDestinations) != 0 {
		t.Fatal("edited route kept or reassigned an unreachable passenger")
	}

	waitingFor(1)
	route.StopIDs[2], route.StopIDs[0] = 1, 5
	world.Update(0, route)
	if stop.WaitingPassengers != 0 || len(stop.WaitingDestinations) != 0 {
		t.Fatal("stop removed from all active routes kept its waiting queue")
	}
}
