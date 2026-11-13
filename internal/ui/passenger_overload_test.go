package ui

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestPassengerOverloadStepsAndLimits(t *testing.T) {
	s := &Settings{}
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		for step := 0; step <= 6; step++ {
			want := min(step*10, 50)
			if s.ModelPassengerOverload(m) != want || s.ModelPassengerCapacity(m) != m.Capacity()*(100+want)/100 {
				t.Fatalf("model%d step%d wrong overload or capacity", m, step)
			}
			s.AdjustPassengerOverload(m, 1)
		}
		for step := 5; step >= -1; step-- {
			if got := s.ModelPassengerOverload(m); got != max(0, step*10) {
				t.Fatalf("model%d decrease step%d got%d", m, step, got)
			}
			s.AdjustPassengerOverload(m, -1)
		}
	}
}

func TestLegacyCapacityConvertsToOverloadPercent(t *testing.T) {
	s := &Settings{PassengerCapacities: &[sim.BusModelCount]int{56, 100, 106}}
	s.NormalizePassengerOverloads()
	if s.PassengerCapacities != nil {
		t.Fatal("legacy absolute counts retained")
	}
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		if s.ModelPassengerOverload(m) != 30 {
			t.Fatalf("model%d old 25%% allowance not mapped to 30%% step", m)
		}
	}
	raw, err := json.Marshal(s)
	if err != nil {
		t.Fatal(err)
	}
	var restored Settings
	if err = json.Unmarshal(raw, &restored); err != nil {
		t.Fatal(err)
	}
	restored.AdjustPassengerOverload(sim.BusStandard, 1)
	if restored.ModelPassengerOverload(sim.BusStandard) != 40 || restored.ModelPassengerOverload(sim.BusArticulated) != 30 {
		t.Fatal("model overload settings did not stay independent after reload")
	}
}
