package ui

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// TestUnifiedMaxSpeedLimitManualAndBerserk verifies that the configurable top speed limit
// (60, 80, 100, 120 km/h) applies consistently to BOTH manual driving and AI Berserk mode.
func TestUnifiedMaxSpeedLimitManualAndBerserk(t *testing.T) {
	s := NewSettings()

	// 1. Verify default is 80 km/h
	if s.AutopilotBerserkMaxSpeed != 80 {
		t.Fatalf("Default max speed should be 80 km/h, got %d", s.AutopilotBerserkMaxSpeed)
	}
	if math.Abs(s.GetBusMaxSpeed()-445.0) > 1.0 {
		t.Fatalf("Default bus max speed should be 445.0 px/s (80 km/h), got %.2f", s.GetBusMaxSpeed())
	}
	if math.Abs(s.GetBerserkCruiseSpeed()-445.0) > 1.0 {
		t.Fatalf("Default berserk cruise speed should be 445.0 px/s (80 km/h), got %.2f", s.GetBerserkCruiseSpeed())
	}

	testCases := []struct {
		kmh          int
		manualMaxSpd float64
		cruiseSpd    float64
	}{
		{60, 334.0, 333.0},
		{80, 445.0, 445.0},
		{100, 556.0, 555.0},
		{120, 667.0, 667.0},
	}

	for _, tc := range testCases {
		s.SetMaxSpeed(tc.kmh)
		if s.AutopilotBerserkMaxSpeed != tc.kmh {
			t.Fatalf("Expected speed %d, got %d", tc.kmh, s.AutopilotBerserkMaxSpeed)
		}
		if math.Abs(s.GetBusMaxSpeed()-tc.manualMaxSpd) > 0.1 {
			t.Fatalf("[%d km/h] Expected bus manual max speed %.1f, got %.2f", tc.kmh, tc.manualMaxSpd, s.GetBusMaxSpeed())
		}
		if math.Abs(s.GetBerserkCruiseSpeed()-tc.cruiseSpd) > 0.1 {
			t.Fatalf("[%d km/h] Expected berserk cruise speed %.1f, got %.2f", tc.kmh, tc.cruiseSpd, s.GetBerserkCruiseSpeed())
		}

		// 2. Simulate manual driving kinematics
		bus := sim.NewBus(100, 100, 0)
		bus.MaxSpeedFwd = s.GetBusMaxSpeed()

		// Full throttle forward for 5 seconds
		dt := 0.016
		for step := 0; step < 350; step++ {
			bus.Update(dt, 1.0, 0.0, 0.0, false, false)
		}

		// Verify speed capped at MaxSpeedFwd
		if bus.Speed > bus.MaxSpeedFwd+0.01 {
			t.Fatalf("[%d km/h] Manual bus exceeded speed limit! Speed=%.2f, Max=%.2f", tc.kmh, bus.Speed, bus.MaxSpeedFwd)
		}

		// Verify converted KM/H on HUD matches target speed limit
		hudKmh := int(math.Abs(bus.Speed) * 0.18)
		if hudKmh != tc.kmh {
			t.Fatalf("[%d km/h] Expected HUD speed %d km/h, got %d km/h (raw speed: %.2f)", tc.kmh, tc.kmh, hudKmh, bus.Speed)
		}
	}

	// 3. Test Cycling Sequence: 60 -> 80 -> 100 -> 120 -> 60
	s.SetMaxSpeed(60)
	if next := s.CycleMaxSpeed(); next != 80 {
		t.Fatalf("Expected cycle 60 -> 80, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 100 {
		t.Fatalf("Expected cycle 80 -> 100, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 120 {
		t.Fatalf("Expected cycle 100 -> 120, got %d", next)
	}
	if next := s.CycleMaxSpeed(); next != 60 {
		t.Fatalf("Expected cycle 120 -> 60, got %d", next)
	}
}
