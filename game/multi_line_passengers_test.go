package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestMultiLinePassengerDestinationAndBoardingFiltering(t *testing.T) {
	// Setup world with 6 stops: 0, 1, 2, 3, 4, 5
	world := sim.NewWorld(52.0)
	for i := 0; i < 6; i++ {
		st := sim.NewBusStop(i, "Stop", sim.Vec2{X: float64(i * 100), Y: 0}, 0, 2, sim.RGBA(0, 0, 0, 0), 52.0, 0)
		world.Stops = append(world.Stops, st)
	}

	// Line 1: Stop 0 -> Stop 1 -> Stop 2
	line1 := sim.NewRoute()
	line1.StopIDs = []int{0, 1, 2}
	// Line 2: Stop 0 -> Stop 1 -> Stop 4 -> Stop 5
	line2 := sim.NewRoute()
	line2.StopIDs = []int{0, 1, 4, 5}

	// At Stop 1:
	// Line 1 upcoming candidates from Stop 1: {2}
	// Line 2 upcoming candidates from Stop 1: {4, 5}
	// Union of destinations from Stop 1: {2, 4, 5}

	stop1 := world.Stops[1]
	stop1.WaitingPassengers = 0
	stop1.WaitingDestinations = nil
	stop1.SpawnInterval = 1.0

	// Advance world update so passengers spawn at Stop 1
	for i := 0; i < 20; i++ {
		world.Update(1.0, line1, line2)
	}

	if stop1.WaitingPassengers == 0 || len(stop1.WaitingDestinations) == 0 {
		t.Fatalf("expected passengers to spawn at Stop 1, got %d", stop1.WaitingPassengers)
	}

	// Verify all destinations belong to {2, 4, 5}
	allowed := map[int]bool{2: true, 4: true, 5: true}
	for _, d := range stop1.WaitingDestinations {
		if !allowed[d] {
			t.Errorf("passenger destination %d not in allowed union {2, 4, 5}", d)
		}
	}

	// Now manually configure Stop 1 with known passengers:
	// 2 passengers wanting Stop 2 (Line 1 only)
	// 3 passengers wanting Stop 5 (Line 2 only)
	stop1.WaitingDestinations = []int{2, 5, 2, 5, 5}
	stop1.WaitingPassengers = len(stop1.WaitingDestinations)

	// Arrive Bus A on Line 1 at Stop 1
	busLine1 := sim.NewBus(stop1.Pos.X, stop1.Pos.Y, stop1.Heading)
	busLine1.Capacity = 10
	candidatesLine1 := line1.GetUpcomingCandidateStopIDsForStop(1) // {2}
	busLine1.BeginStationService(1, 1, 0, false, candidatesLine1)

	// Rebalance boarding
	sim.RebalanceStationBoarding(stop1, []*sim.Bus{busLine1}, true)

	// Bus A should only target 2 passengers (the ones wanting Stop 2)
	matchingForBus1 := stop1.CountMatchingWaiting(candidatesLine1)
	if matchingForBus1 != 2 {
		t.Fatalf("expected 2 matching passengers for Line 1, got %d", matchingForBus1)
	}
	if busLine1.PassengerService.BoardingTarget != 2 {
		t.Fatalf("expected Line 1 bus boarding target to be 2, got %d", busLine1.PassengerService.BoardingTarget)
	}

	// Step station service to board
	busLine1.PassengerService.BoardingRemaining = 0
	alighted, boarded, _ := busLine1.StepStationService(1, stop1, line1, nil)
	if alighted != 0 || boarded != 2 {
		t.Fatalf("expected 2 boarded passengers for Line 1, got alighted=%d boarded=%d", alighted, boarded)
	}

	// Check onboard passengers on Bus A
	for _, p := range busLine1.Passengers {
		if p.TargetStopID != 2 {
			t.Errorf("busLine1 passenger has target %d, want 2", p.TargetStopID)
		}
	}

	// Verify remaining passengers at Stop 1: only the 3 passengers wanting Stop 5 should remain!
	if stop1.WaitingPassengers != 3 {
		t.Fatalf("expected 3 passengers remaining at Stop 1, got %d", stop1.WaitingPassengers)
	}
	for _, d := range stop1.WaitingDestinations {
		if d != 5 {
			t.Errorf("remaining passenger at Stop 1 has target %d, want 5", d)
		}
	}

	// Now arrive Bus B on Line 2 at Stop 1
	busLine2 := sim.NewBus(stop1.Pos.X, stop1.Pos.Y, stop1.Heading)
	busLine2.Capacity = 10
	candidatesLine2 := line2.GetUpcomingCandidateStopIDsForStop(1) // {4, 5}
	busLine2.BeginStationService(1, 1, 0, false, candidatesLine2)

	sim.RebalanceStationBoarding(stop1, []*sim.Bus{busLine2}, true)
	if busLine2.PassengerService.BoardingTarget != 3 {
		t.Fatalf("expected Line 2 bus boarding target to be 3, got %d", busLine2.PassengerService.BoardingTarget)
	}

	busLine2.PassengerService.BoardingRemaining = 0
	alighted2, boarded2, _ := busLine2.StepStationService(2, stop1, line2, nil)
	if alighted2 != 0 || boarded2 != 3 {
		t.Fatalf("expected 3 boarded passengers for Line 2, got alighted=%d boarded=%d", alighted2, boarded2)
	}

	for _, p := range busLine2.Passengers {
		if p.TargetStopID != 5 {
			t.Errorf("busLine2 passenger has target %d, want 5", p.TargetStopID)
		}
	}

	// Platform should now be completely clear
	if stop1.WaitingPassengers != 0 || len(stop1.WaitingDestinations) != 0 {
		t.Fatalf("expected platform to be empty, got %d passengers", stop1.WaitingPassengers)
	}
}
