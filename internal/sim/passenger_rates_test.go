package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestModelPassengerRateDefaultsAndChoices(t *testing.T) {
	for model := BusStandard; model < BusModelCount; model++ {
		for _, seconds := range PassengerTimeOptions {
			b := NewBus(0, 0, 0)
			b.SetModel(model)
			b.DoorsOpen = true
			r := PassengerRates{seconds, seconds}
			b.Passengers = []Passenger{{TargetStopID: 1}, {TargetStopID: 1}}
			b.PassengerCount = 2
			b.BeginStationService(1, 0, 2, false, []int{2}, r)
			b.AddPassengerService(3, 0)
			if math.Abs(b.PassengerServiceRemaining()-3*seconds) > 1e-9 {
				t.Fatalf("model %v rate %v wrong dwell", model, seconds)
			}
			// Boarding and alighting advance concurrently; door interlock is retained.
			b.Update(seconds, 1, 0, 0, false, true)
			if !b.DoorsOpen || b.Speed != 0 {
				t.Fatal("service interlock bypassed")
			}
			stop := &BusStop{ID: 1, WaitingPassengers: 3}
			b.StepStationService(0, stop, &Route{StopIDs: []int{1, 2}}, nil)
			if b.PassengerService.BoardedSoFar != 1 || b.PassengerService.AlightedSoFar != 1 {
				t.Fatalf("rate %v not used for per-person completion: %+v", seconds, b.PassengerService)
			}
		}
		b := NewBus(0, 0, 0)
		b.SetModel(model)
		b.BeginStationService(1, 0, 2, false, nil)
		want := 2.0
		if model == BusArticulated {
			want = 1
		}
		if b.PassengerServiceRemaining() != want {
			t.Fatalf("model %v default dwell %v", model, b.PassengerServiceRemaining())
		}
	}
}

func TestMixedRateQueueWorksAtEachBusFullSpeed(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		stop := &BusStop{ID: 1, WaitingPassengers: 18}
		route := &Route{StopIDs: []int{1, 2}}
		normal, art := NewBus(0, 0, 0), NewBus(0, 0, 0)
		art.SetModel(BusArticulated)
		buses := []*Bus{normal, art}
		if reverse {
			buses = []*Bus{art, normal}
		}
		for _, b := range buses {
			b.DoorsOpen = true
			b.BeginStationService(1, 0, 0, false, []int{2})
		}
		RebalanceStationBoarding(stop, buses, true)
		if normal.PassengerService.BoardingTarget != 18 || art.PassengerService.BoardingTarget != 18 {
			t.Fatal("shared queue was reserved into quotas")
		}
		for tick := 1; tick <= 12*60; tick++ {
			for _, b := range buses {
				b.Update(1./60, 0, 0, 0, false, false)
			}
			for id, b := range buses {
				b.StepStationService(id, stop, route, nil)
			}
			RebalanceStationBoarding(stop, buses, false)
			if tick == 120 && (normal.PassengerCount != 1 || art.PassengerCount != 2) {
				t.Fatalf("boarding throttled: %d/%d", normal.PassengerCount, art.PassengerCount)
			}
		}
		if stop.WaitingPassengers != 0 || normal.PassengerCount != 6 || art.PassengerCount != 12 {
			t.Fatal("queue not drained at combined throughput")
		}
		if normal.PassengerServiceRemaining() != 0 || art.PassengerServiceRemaining() != 0 {
			t.Fatal("empty queue retained dwell")
		}
	}
}

func TestPassengerRateSaveRetainsPartialProgressAndLegacyTiming(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	b.DoorsOpen = true
	b.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{.75, .25})
	b.AddPassengerService(2, 0)
	b.Update(.25, 0, 0, 0, false, false)
	data, _ := json.Marshal(b)
	var restored Bus
	if err := json.Unmarshal(data, &restored); err != nil {
		t.Fatal(err)
	}
	if restored.PassengerServiceRemaining() != 1.25 || !restored.PassengerService.Valid() {
		t.Fatal("saved partial service lost")
	}
	legacy := PassengerService{Active: true, BoardingTarget: 2, BoardingRemaining: 3}
	if legacy.rates() != (PassengerRates{2, 1}) || !legacy.Valid() {
		t.Fatal("legacy active service changed rate")
	}
}

func TestCapacityAndLateJoiningBusRedistributeSharedQueue(t *testing.T) {
	stop := &BusStop{ID: 1, WaitingPassengers: 8}
	route := &Route{StopIDs: []int{1, 2}}
	fast, slow := NewBus(0, 0, 0), NewBus(0, 0, 0)
	fast.Capacity = 2
	fast.DoorsOpen = true
	slow.DoorsOpen = true
	fast.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{.25, 1})
	slow.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{.5, 1})
	buses := []*Bus{fast, slow}
	RebalanceStationBoarding(stop, buses, true)
	for tick := 0; tick < 3*60; tick++ {
		for _, b := range buses {
			b.Update(1./60, 0, 0, 0, false, false)
			b.StepStationService(0, stop, route, nil)
		}
		RebalanceStationBoarding(stop, buses, false)
	}
	if fast.PassengerCount != 2 || slow.PassengerCount != 6 || stop.WaitingPassengers != 0 {
		t.Fatalf("capacity/queue error %d/%d waiting %d", fast.PassengerCount, slow.PassengerCount, stop.WaitingPassengers)
	}
	// A newly arriving bus may take work, without resetting the first bus's partial passenger.
	stop.WaitingPassengers = 10
	fast.Capacity = 45
	fast.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{2, 1})
	RebalanceStationBoarding(stop, []*Bus{fast}, true)
	fast.Update(.5, 0, 0, 0, false, false)
	slow.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{1, 1})
	RebalanceStationBoarding(stop, buses, true)
	first := fast.PassengerService.BoardingRemaining - float64(fast.PassengerService.BoardingTarget-1)*2
	if math.Abs(first-1.5) > 1e-9 {
		t.Fatal("joining bus reset partial boarding progress")
	}
}

func TestFullBusAlightingDoesNotReserveWaitingPassengers(t *testing.T) {
	stop := &BusStop{ID: 1, WaitingPassengers: 2}
	route := &Route{StopIDs: []int{1, 2}}
	full, empty := NewBus(0, 0, 0), NewBus(0, 0, 0)
	full.Capacity = 1
	full.Passengers = []Passenger{{TargetStopID: 1}}
	full.PassengerCount = 1
	full.DoorsOpen, empty.DoorsOpen = true, true
	full.BeginStationService(1, 0, 1, false, []int{2}, PassengerRates{.25, 3})
	empty.BeginStationService(1, 0, 0, false, []int{2}, PassengerRates{1, 1})
	buses := []*Bus{full, empty}
	RebalanceStationBoarding(stop, buses, true)
	for tick := 0; tick < 120; tick++ {
		for _, b := range buses {
			b.Update(1./60, 0, 0, 0, false, false)
			b.StepStationService(0, stop, route, nil)
		}
		RebalanceStationBoarding(stop, buses, false)
	}
	if stop.WaitingPassengers != 0 || empty.PassengerCount != 2 || full.PassengerService.BoardedSoFar != 0 {
		t.Fatal("full bus's faster doors reserved passengers instead of letting empty bus board")
	}
}
