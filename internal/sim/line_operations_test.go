package sim

import "testing"

func TestLineSummaryWeightsCompletedTimeAndKeepsServicesSeparate(t *testing.T) {
	r := &Route{Name: "Line 1", StopIDs: []int{0, 1, 2, 0}}
	clone := *r
	o := &TransitOperations{Distances: RouteDistances{TotalKM: 6, Legs: []RouteLegDistance{{0, 1, 1}, {1, 2, 3}, {2, 0, 2}}}, Vehicles: map[int]*VehicleOperations{
		0: {Route: r, LineName: r.Name, CompletedLaps: 1, CompletedKM: 12, CompletedSeconds: 600, KM: 100, Seconds: 10000},
		1: {Route: &clone, LineName: r.Name, CompletedLaps: 1, CompletedKM: 12, CompletedSeconds: 1200},
		2: {Route: r, LineName: "Line 2", CompletedLaps: 1, CompletedKM: 500, CompletedSeconds: 1},
		3: {Route: &Route{Name: r.Name, StopIDs: []int{0, 2, 0}}, LineName: r.Name, CompletedLaps: 1, CompletedKM: 500, CompletedSeconds: 1},
	}}
	l := o.LineSummary(r)
	speed, ready := l.AverageKPH()
	if !ready || speed != 48 || l.AverageSpacingKM != 2 || l.TotalKM != 6 || l.Legs != 3 || l.Vehicles != 2 || l.CompletedLaps != 2 {
		t.Fatalf("wrong line summary: %+v, speed=%v", l, speed)
	}
	o.Vehicles = nil
	if _, ready := o.LineSummary(r).AverageKPH(); ready {
		t.Fatal("unfinished service reported speed")
	}
}
