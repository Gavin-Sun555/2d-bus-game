package sim

import (
	"testing"
)

func TestBusStopPassengerArrivalsAndContainment(t *testing.T) {
	stop := NewBusStop(1, "TestStop", Vec2{X: 200, Y: 100}, 0, 4, RGBA(0, 0, 0, 0), 52.0, 5)
	stop.SpawnInterval = 0.5 // Fast arrival cycle
	stop.SpawnTimer = 0.0
	stop.WaitingPassengers = 3
	stop.MaxWaiting = 15

	// Advance less than spawn interval
	stop.Update(0.2)
	if stop.WaitingPassengers != 3 {
		t.Errorf("Passengers should not spawn before interval elapses")
	}

	// Cross spawn interval
	stop.Update(0.4)
	if stop.WaitingPassengers <= 3 {
		t.Errorf("Passengers should have arrived after spawn interval elapsed, got %d", stop.WaitingPassengers)
	}
	if stop.SpawnTimer != 0 {
		t.Errorf("Spawn timer should reset to 0 after spawning, got %.2f", stop.SpawnTimer)
	}

	// Bus Containment
	bus := NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	if !stop.ContainsBus(bus) {
		t.Errorf("Bus aligned at stop position should be contained inside bay")
	}

	farBus := NewBus(stop.Pos.X+400.0, stop.Pos.Y, stop.Heading)
	if stop.ContainsBus(farBus) {
		t.Errorf("Distant bus should not be contained inside bay")
	}
}

func TestPassengerDestinationAssignment(t *testing.T) {
	// 1. Metropolis closed loop: [0, 2, 10, 8, 0]
	route := NewDefaultRoute()

	// Stop 0 (Terminal at start of loop): candidates must strictly be [2, 10, 8], NOT 0
	c0 := route.GetUpcomingCandidateStopIDs(0)
	if len(c0) != 3 {
		t.Fatalf("Expected 3 candidates for Stop 0, got %v", c0)
	}
	for _, id := range c0 {
		if id == 0 {
			t.Fatalf("Stop 0 must NEVER be a candidate destination for passengers boarding at Stop 0! Got %v", c0)
		}
	}

	// Stop 2 (index 1): candidates must strictly be [10, 8, 0], NOT 2
	c1 := route.GetUpcomingCandidateStopIDs(1)
	if len(c1) != 3 {
		t.Fatalf("Expected 3 candidates for Stop 2, got %v", c1)
	}
	for _, id := range c1 {
		if id == 2 {
			t.Fatalf("Stop 2 must NEVER be a candidate destination for passengers boarding at Stop 2! Got %v", c1)
		}
	}

	// Stop 10 (index 2): candidates must be [8, 0]
	c2 := route.GetUpcomingCandidateStopIDs(2)
	if len(c2) != 2 || c2[0] != 8 || c2[1] != 0 {
		t.Fatalf("Expected candidates [8, 0] for Stop 10, got %v", c2)
	}

	// Stop 8 (index 3): candidates must be [0]
	c3 := route.GetUpcomingCandidateStopIDs(3)
	if len(c3) != 1 || c3[0] != 0 {
		t.Fatalf("Expected candidates [0] for Stop 8, got %v", c3)
	}

	// Stop 0 at terminus (index 4): candidates for next run must be [2, 10, 8], NOT 0
	c4 := route.GetUpcomingCandidateStopIDs(4)
	if len(c4) != 3 {
		t.Fatalf("Expected 3 candidates for terminal boarding, got %v", c4)
	}
	for _, id := range c4 {
		if id == 0 {
			t.Fatalf("Stop 0 must NEVER be candidate destination for terminal boarding, got %v", c4)
		}
	}

	// Also check GetUpcomingCandidateStopIDsForStop
	cFor0 := route.GetUpcomingCandidateStopIDsForStop(0)
	for _, id := range cFor0 {
		if id == 0 {
			t.Fatalf("GetUpcomingCandidateStopIDsForStop(0) must not contain 0, got %v", cFor0)
		}
	}
}

func TestPassengerBoardingAndAlighting(t *testing.T) {
	bus := NewBus(0, 0, 0)
	if bus.PassengerCount != 0 || len(bus.Passengers) != 0 {
		t.Fatalf("New bus must have 0 passengers")
	}

	// Board 10 passengers with destinations in [2, 10, 8]
	candidates := []int{2, 10, 8}
	boarded := bus.BoardPassengers(10, candidates)
	if boarded != 10 || bus.PassengerCount != 10 || len(bus.Passengers) != 10 {
		t.Fatalf("Expected 10 boarded passengers, got %d (count=%d)", boarded, bus.PassengerCount)
	}

	// Verify all passengers have a valid destination in candidates
	for _, p := range bus.Passengers {
		if p.TargetStopID != 2 && p.TargetStopID != 10 && p.TargetStopID != 8 {
			t.Fatalf("Invalid passenger destination: %d", p.TargetStopID)
		}
	}

	// Alight passengers destined for Stop 2
	expectedAlighting2 := bus.CountPassengersAlightingAt(2)
	removed2 := bus.RemovePassengersAlightingAt(2)
	if removed2 != expectedAlighting2 {
		t.Fatalf("Removed count %d != counted alighting %d", removed2, expectedAlighting2)
	}
	if bus.PassengerCount != 10-removed2 || len(bus.Passengers) != 10-removed2 {
		t.Fatalf("PassengerCount mismatch after alighting: %d vs %d", bus.PassengerCount, 10-removed2)
	}
	if bus.CountPassengersAlightingAt(2) != 0 {
		t.Fatalf("Expected 0 passengers remaining for Stop 2")
	}

	// Clear passengers
	remaining := bus.PassengerCount
	cleared := bus.ClearPassengers()
	if cleared != remaining || bus.PassengerCount != 0 || len(bus.Passengers) != 0 {
		t.Fatalf("ClearPassengers failed: cleared %d, count %d", cleared, bus.PassengerCount)
	}
}
