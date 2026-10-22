package sim

import (
	"math"
	"testing"
)

func simpleOperations() *TransitOperations {
	return &TransitOperations{Policy: FarePolicy{Mode: FareByStops, Base: 2, IncludedStops: 1, StopsPerYuan: 1, Cap: 8}, Distances: RouteDistances{StopIDs: []int{0, 1, 2}, Legs: []RouteLegDistance{{KM: .1}, {KM: .1}, {KM: .1}}, TotalKM: .3}, Vehicles: map[int]*VehicleOperations{}, Stations: map[int]*StationActivity{}}
}
func TestTransitBoardingAccountsForDestinationAndVehicle(t *testing.T) {
	o := simpleOperations()
	b := NewBus(0, 0, 0)
	r := &Route{StopIDs: []int{0, 1, 2, 0}}
	n, cash := o.Board(0, b, r, 0, 4, []int{2})
	if n != 4 || cash != 12 || o.Revenue != 12 || o.Vehicles[0].Revenue != 12 || o.Stations[0].Boarded != 4 {
		t.Fatal("boarding/revenue ledger mismatch")
	}
	o.Policy.Fixed = 10
	o.Policy.Mode = FareFixed
	if b.Passengers[0].PaidFare != 3 {
		t.Fatal("tariff edit changed onboard fare")
	}
	_, cash = o.Board(1, NewBus(0, 0, 0), r, 0, 2, []int{1})
	if cash != 20 || o.Revenue != 32 || o.Vehicles[1].Revenue != 20 || o.Vehicles[0].Revenue != 12 {
		t.Fatal("vehicle revenues mixed")
	}
	o.Alight(0, 2, b.RemovePassengersAlightingAt(2))
	o.Alight(0, 2, 0)
	if o.Stations[2].Alighted != 4 || o.Vehicles[0].Alighted != 4 {
		t.Fatal("alighting not recorded exactly once")
	}
	o.Policy = FarePolicy{Mode: FareFixed, Fixed: 0}
	n, cash = o.Board(0, b, r, 0, 100, []int{1})
	if n != b.Capacity || cash != 0 || o.Vehicles[0].Boarded != 4+b.Capacity {
		t.Fatal("free fares or capacity accounting incorrect")
	}
}
func TestOperationAverageNeedsFullLapAndIncludesStops(t *testing.T) {
	o := simpleOperations()
	b := NewBus(0, 0, 0)
	r := &Route{StopIDs: []int{0, 1, 2, 0}}
	o.Track(0, b, r, 0)
	// 100 simulated seconds moving at 36 km/h, then 100 seconds waiting.
	for i := 0; i < 100; i++ {
		b.Speed = SpeedKPH(36)
		b.Pos.X += b.Speed
		o.Track(0, b, r, 1)
	}
	b.Speed = 0
	for i := 0; i < 100; i++ {
		o.Track(0, b, r, 1)
	}
	if _, ok := o.Vehicle(0).AverageKPH(); ok {
		t.Fatal("average visible before first full lap")
	}
	r.TotalTrips++
	o.Track(0, b, r, .001)
	average, ok := o.Vehicle(0).AverageKPH()
	if !ok || math.Abs(average-18) > .01 || o.Vehicle(0).CompletedLaps != 1 {
		t.Fatalf("mean must include waiting: %.3f", average)
	}
	b.Pos.X += 1000
	o.Track(0, b, r, 1)
	r.TotalTrips++
	o.Track(0, b, r, 1)
	if o.Vehicle(0).CompletedLaps != 1 {
		t.Fatal("teleported lap was counted")
	}
	ai := NewBus(0, 0, 0)
	aiRoute := &Route{StopIDs: []int{0, 1, 2, 0}, CurrentIndex: 2}
	o.Track(1, ai, aiRoute, 0)
	ai.Speed = SpeedKPH(36)
	for i := 0; i < 30; i++ {
		ai.Pos.X += ai.Speed
		o.Track(1, ai, aiRoute, 1)
	}
	aiRoute.TotalTrips++
	o.Track(1, ai, aiRoute, 1)
	if o.Vehicle(1).CompletedLaps != 0 {
		t.Fatal("partial initial dispatch counted as full lap")
	}
}
