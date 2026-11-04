package ui

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPassengerSpawnRateSettingCycling(t *testing.T) {
	s := NewSettings()

	// Default must be Moderate
	if s.PassengerSpawnRate != sim.PassengerSpawnModerate {
		t.Fatalf("expected default spawn rate to be Moderate, got %v", s.PassengerSpawnRate)
	}

	// Cycle: Moderate -> Few
	r1 := s.CyclePassengerSpawnRate()
	if r1 != sim.PassengerSpawnFew || s.PassengerSpawnRate != sim.PassengerSpawnFew {
		t.Fatalf("expected Few after first cycle, got %v", s.PassengerSpawnRate)
	}

	// Cycle: Few -> Many
	r2 := s.CyclePassengerSpawnRate()
	if r2 != sim.PassengerSpawnMany || s.PassengerSpawnRate != sim.PassengerSpawnMany {
		t.Fatalf("expected Many after second cycle, got %v", s.PassengerSpawnRate)
	}

	// Cycle: Many -> Moderate
	r3 := s.CyclePassengerSpawnRate()
	if r3 != sim.PassengerSpawnModerate || s.PassengerSpawnRate != sim.PassengerSpawnModerate {
		t.Fatalf("expected Moderate after third cycle, got %v", s.PassengerSpawnRate)
	}
}

