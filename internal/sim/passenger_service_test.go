package sim

import (
	"math"
	"testing"
)

func TestPassengerServiceMinimumDwellAndDoorInterlock(t *testing.T) {
	// 1. No passengers boarding or alighting -> 0s minimum dwell time, doors can close immediately.
	b := NewBus(0, 0, 0)
	b.DoorsOpen = true
	b.DoorProgress = 1.0
	b.AddPassengerService(0, 0)
	if b.PassengerServiceRemaining() != 0 {
		t.Fatalf("expected 0s dwell when nobody boards or alights, got %.2f", b.PassengerServiceRemaining())
	}
	b.Update(1./60, 0, 0, 0, false, true)
	if b.DoorsOpen {
		t.Fatal("expected doors to close immediately when 0 passengers boarded or alighted")
	}

	// 2. Front door boarding (2s/person) > rear door alighting (1s/person): max(2*2s, 3*1s) = 4s.
	b = NewBus(0, 0, 0)
	b.DoorsOpen = true
	b.DoorProgress = 1.0
	b.AddPassengerService(0, 3)
	b.AddPassengerService(2, 0)
	if math.Abs(b.PassengerServiceRemaining()-4.0) > 1e-9 {
		t.Fatalf("expected max(2*2s, 3*1s) = 4s dwell, got %.2f", b.PassengerServiceRemaining())
	}

	// Attempt to close doors and accelerate before 4s finishes: must remain open and stationary.
	for i := 0; i < 60*4-1; i++ {
		b.Update(1./60, 1.0, 0, 0, false, true)
		if !b.DoorsOpen || b.Speed != 0 {
			t.Fatalf("tick %d: doors closed or bus moved before 4s dwell finished", i)
		}
	}
	// Final tick completes the 4s dwell; next toggle closes doors.
	b.Update(1./60, 0, 0, 0, false, false)
	if b.PassengerServiceRemaining() != 0 {
		t.Fatalf("expected 0s remaining after 4s, got %.6f", b.PassengerServiceRemaining())
	}
	b.Update(1./60, 0, 0, 0, false, true)
	if b.DoorsOpen {
		t.Fatal("expected doors to close after 4s dwell completed")
	}

	// 3. Rear door alighting (1s/person) > front door boarding (2s/person): max(1*2s, 5*1s) = 5s.
	b = NewBus(0, 0, 0)
	b.DoorsOpen = true
	b.DoorProgress = 1.0
	b.AddPassengerService(1, 5)
	if math.Abs(b.PassengerServiceRemaining()-5.0) > 1e-9 {
		t.Fatalf("expected max(1*2s, 5*1s) = 5s dwell, got %.2f", b.PassengerServiceRemaining())
	}
	for i := 0; i < 60*5-1; i++ {
		b.Update(1./60, 1.0, 0, 0, false, true)
		if !b.DoorsOpen || b.Speed != 0 {
			t.Fatalf("tick %d: doors closed or bus moved before 5s dwell finished", i)
		}
	}
	b.Update(1./60, 0, 0, 0, false, false)
	if b.PassengerServiceRemaining() != 0 {
		t.Fatalf("expected 0s remaining after 5s, got %.6f", b.PassengerServiceRemaining())
	}
	b.Update(1./60, 0, 0, 0, false, true)
	if b.DoorsOpen {
		t.Fatal("expected doors to close after 5s dwell completed")
	}
}

func TestGradualStepByStepBoardingAndAlighting(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.DoorsOpen = true
	b.DoorProgress = 1.0
	b.Passengers = []Passenger{{TargetStopID: 1}, {TargetStopID: 1}, {TargetStopID: 2}}
	b.PassengerCount = len(b.Passengers)

	stop := &BusStop{ID: 1, WaitingPassengers: 3}
	route := &Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	b.BeginStationService(stop.ID, route.CurrentIndex, 2, false, []int{2})
	RebalanceStationBoarding(stop, []*Bus{b}, true)

	// Immediately upon opening doors (t = 0), no one has finished boarding or alighting yet.
	b.StepStationService(0, stop, route, nil)
	if b.PassengerCount != 3 || stop.WaitingPassengers != 3 {
		t.Fatalf("t=0s: expected onboard=3 waiting=3, got onboard=%d waiting=%d", b.PassengerCount, stop.WaitingPassengers)
	}

	// t = 1s: 1 alighted (3 -> 2), 0 boarded (waiting still 3).
	b.Update(1.0, 0, 0, 0, false, false)
	b.StepStationService(0, stop, route, nil)
	if b.PassengerCount != 2 || stop.WaitingPassengers != 3 {
		t.Fatalf("t=1s: expected onboard=2 waiting=3, got onboard=%d waiting=%d", b.PassengerCount, stop.WaitingPassengers)
	}

	// t = 2s: 2nd alighted and 1st boarded (2 - 1 + 1 = 2 onboard, waiting 3 -> 2).
	b.Update(1.0, 0, 0, 0, false, false)
	b.StepStationService(0, stop, route, nil)
	if b.PassengerCount != 2 || stop.WaitingPassengers != 2 {
		t.Fatalf("t=2s: expected onboard=2 waiting=2, got onboard=%d waiting=%d", b.PassengerCount, stop.WaitingPassengers)
	}

	// t = 4s: 2nd boarded (onboard 2 -> 3, waiting 2 -> 1).
	b.Update(2.0, 0, 0, 0, false, false)
	b.StepStationService(0, stop, route, nil)
	if b.PassengerCount != 3 || stop.WaitingPassengers != 1 {
		t.Fatalf("t=4s: expected onboard=3 waiting=1, got onboard=%d waiting=%d", b.PassengerCount, stop.WaitingPassengers)
	}

	// t = 6s: 3rd boarded (onboard 3 -> 4, waiting 1 -> 0).
	b.Update(2.0, 0, 0, 0, false, false)
	b.StepStationService(0, stop, route, nil)
	if b.PassengerCount != 4 || stop.WaitingPassengers != 0 || b.PassengerServiceRemaining() != 0 {
		t.Fatalf("t=6s: expected onboard=4 waiting=0 remaining=0, got onboard=%d waiting=%d remaining=%.2f", b.PassengerCount, stop.WaitingPassengers, b.PassengerServiceRemaining())
	}
}

func TestMultiBusEvenPassengerSharing(t *testing.T) {
	b1 := NewBus(0, 0, 0)
	b2 := NewBus(0, 0, 0)
	b1.DoorsOpen, b1.DoorProgress = true, 1.0
	b2.DoorsOpen, b2.DoorProgress = true, 1.0

	stop := &BusStop{ID: 1, WaitingPassengers: 6}
	route := &Route{StopIDs: []int{1, 2}, CurrentIndex: 0}
	b1.BeginStationService(stop.ID, 0, 0, false, []int{2})
	b2.BeginStationService(stop.ID, 0, 0, false, []int{2})
	RebalanceStationBoarding(stop, []*Bus{b1, b2}, true)

	if b1.PassengerService.BoardingTarget != 6 || b2.PassengerService.BoardingTarget != 6 {
		t.Fatalf("expected both doors to work from the shared queue of 6, got %d and %d", b1.PassengerService.BoardingTarget, b2.PassengerService.BoardingTarget)
	}

	for step := 1; step <= 3; step++ {
		b1.Update(2.0, 0, 0, 0, false, false)
		b2.Update(2.0, 0, 0, 0, false, false)
		b1.StepStationService(1, stop, route, nil)
		b2.StepStationService(2, stop, route, nil)
		if b1.PassengerCount != step || b2.PassengerCount != step || stop.WaitingPassengers != 6-2*step {
			t.Fatalf("step %d: expected b1=%d b2=%d waiting=%d, got b1=%d b2=%d waiting=%d",
				step, step, step, 6-2*step, b1.PassengerCount, b2.PassengerCount, stop.WaitingPassengers)
		}
	}
}
