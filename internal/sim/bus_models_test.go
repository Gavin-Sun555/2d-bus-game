package sim

import "testing"

func TestBusModelCapacityAndAcceleration(t *testing.T) {
	var speeds [BusModelCount]float64
	for model := BusStandard; model < BusModelCount; model++ {
		b := NewBus(10, 20, 0)
		if !b.SetModel(model) {
			t.Fatal("model rejected")
		}
		want := []int{45, 80, 85}[model]
		if b.Capacity != want || b.BoardPassengers(100, []int{1}) != want || b.BoardPassengers(1, []int{1}) != 0 {
			t.Fatalf("model%d capacity%d riders%d", model, b.Capacity, b.PassengerCount)
		}
		for i := 0; i < 60; i++ {
			b.Update(1./60, 1, 0, 0, false, false)
		}
		speeds[model] = b.Speed
	}
	if speeds[BusStandard] <= speeds[BusDoubleDecker] || speeds[BusDoubleDecker] <= speeds[BusArticulated] {
		t.Fatalf("unexpected acceleration: %v", speeds)
	}
}

func TestInServiceBusReplacementOnlyAfterTerminalAlighting(t *testing.T) {
	for from := BusStandard; from < BusModelCount; from++ {
		for to := BusStandard; to < BusModelCount; to++ {
			if from == to {
				continue
			}
			b := NewBus(10, 20, 0)
			b.SetModel(from)
			b.Durability = 61
			b.MaxSpeedFwd = 300
			b.RequestModel(to)
			b.BoardPassengers(100, []int{1})
			if b.Model != from || b.PassengerCount != from.Capacity() {
				t.Fatal("request changed current boarding capacity")
			}
			for i := 0; i < 60; i++ {
				b.Update(1./60, 0, 1, 0, false, false)
			}
			b.RemovePassengersAlightingAt(1)
			b.BoardPassengers(1, []int{2})
			if b.Model != from || b.PendingModel == nil {
				t.Fatal("empty mid-route bus changed models")
			}
			r := &Route{StopIDs: []int{0, 1, 0}, CurrentIndex: 1}
			b.DoorsOpen = true
			if b.ReplaceModelAtTerminus(r, 1) {
				t.Fatal("replaced at intermediate stop")
			}
			r.CurrentIndex = 2
			if b.ReplaceModelAtTerminus(r, 0) {
				t.Fatal("replaced before all riders alighted")
			}
			b.ClearPassengers()
			b.Speed = 20
			if b.ReplaceModelAtTerminus(r, 0) {
				t.Fatal("replaced while moving")
			}
			b.Speed = 0
			pos := b.Pos
			if !b.ReplaceModelAtTerminus(r, 0) || b.Model != to || b.Capacity != to.Capacity() || b.PendingModel != nil || b.Durability != 100 || b.Pos != pos || b.MaxSpeedFwd != 300 || !b.DoorsOpen {
				t.Fatal("terminal did not supply fresh vehicle")
			}
		}
	}
}

func TestRequestCurrentModelCancelsReplacement(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.RequestModel(BusDoubleDecker)
	b.RequestModel(BusArticulated)
	if b.PendingModel == nil || *b.PendingModel != BusArticulated {
		t.Fatal("cannot revise request")
	}
	b.RequestModel(BusStandard)
	if b.PendingModel != nil || b.Model != BusStandard {
		t.Fatal("cannot cancel request")
	}
}
