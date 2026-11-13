package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestOperationsSaveCompletesUnfinishedLap(t *testing.T) {
	o := simpleOperations()
	b := NewBus(0, 0, 0)
	r := &Route{StopIDs: []int{0, 1, 2, 0}}
	o.Track(0, b, r, 0)
	b.Speed = SpeedKPH(36)
	for i := 0; i < 50; i++ {
		b.Pos.X += b.Speed
		o.Track(0, b, r, 1)
	}
	raw, err := json.Marshal(o.Snapshot())
	if err != nil {
		t.Fatal(err)
	}
	var snapshot OperationsSnapshot
	if err = json.Unmarshal(raw, &snapshot); err != nil {
		t.Fatal(err)
	}
	restored := RestoreOperations(snapshot, nil, r, 52, DefaultFarePolicy(), map[int]*Bus{0: b}, map[int]*Route{0: r})
	// The simple fixture uses a supplied distance table instead of a world.
	restored.Distances = o.Distances
	for i := 0; i < 50; i++ {
		b.Pos.X += b.Speed
		restored.Track(0, b, r, 1)
	}
	b.Speed = 0
	for i := 0; i < 100; i++ {
		restored.Track(0, b, r, 1)
	}
	r.TotalTrips++
	restored.Track(0, b, r, .001)
	speed, ok := restored.Vehicle(0).AverageKPH()
	if !ok || math.Abs(speed-18) > .01 || restored.Vehicle(0).CompletedLaps != 1 {
		t.Fatalf("unfinished lap lost at save: speed %.3f, valid %v", speed, ok)
	}
}

func TestLineWaitTimeUsesBoardedPassengersAndSurvivesSave(t *testing.T) {
	stop := NewBusStop(0, "Shared stop", Vec2{}, 0, 2, RGBA(0, 0, 0, 255), 52, 0)
	stop.SpawnInterval = 1000
	stop.WaitingPassengers = 1
	stop.WaitingDestinations = []int{1} // A legacy passenger without a timestamp.
	stop.SeedWaitingPassengers(2)
	stop.WaitingDestinations = append(stop.WaitingDestinations, 2)
	stop.Update(30)
	stop.SeedWaitingPassengers(3)
	stop.WaitingDestinations = append(stop.WaitingDestinations, 1)
	stop.Update(10)

	lineA := &Route{Name: "A", StopIDs: []int{0, 1, 0}}
	lineB := &Route{Name: "B", StopIDs: []int{0, 2, 0}}
	if stop.WaitingForLine(lineA) != 2 || stop.WaitingForLine(lineB) != 1 {
		t.Fatal("shared queue was not filtered by destination")
	}
	operations := simpleOperations()
	bus := NewBus(0, 0, 0)
	board := func(route *Route, count int) {
		bus.PassengerService = PassengerService{Active: true, BoardingTarget: count, Candidates: route.GetUpcomingCandidateStopIDs(0)}
		_, boarded, _ := bus.StepStationService(0, stop, route, operations)
		if boarded != count {
			t.Fatalf("line %s boarded %d passengers, want %d", route.Name, boarded, count)
		}
	}
	board(lineA, 2)
	// Reload both the queue and the operations ledger before the other line boards.
	queue, err := json.Marshal(stop)
	if err != nil {
		t.Fatal(err)
	}
	stop = &BusStop{}
	if err := json.Unmarshal(queue, stop); err != nil {
		t.Fatal(err)
	}
	raw, err := json.Marshal(operations.Snapshot())
	if err != nil {
		t.Fatal(err)
	}
	var snapshot OperationsSnapshot
	if err := json.Unmarshal(raw, &snapshot); err != nil {
		t.Fatal(err)
	}
	operations = RestoreOperations(snapshot, nil, lineA, 52, DefaultFarePolicy(), nil, nil)
	stop.Update(5)
	board(lineB, 1)
	a, b := operations.LineStations["A"][0], operations.LineStations["B"][0]
	if a.Boarded != 2 || a.WaitSamples != 1 || a.WaitSeconds != 10 ||
		b.Boarded != 1 || b.WaitSamples != 1 || b.WaitSeconds != 45 {
		t.Fatalf("unexpected wait totals, line A %+v, line B %+v", a, b)
	}
}
