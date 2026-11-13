package ui

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkPresetsAndManualCeiling(t *testing.T) {
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

	for _, kmh := range []int{80, 120, 160, 180} {
		s.SetMaxSpeed(kmh)
		s.AutopilotEnabled, s.AutopilotBerserk = true, false
		want := sim.SpeedKPH(kmh)
		if s.GetBusMaxSpeed() != want {
			t.Fatalf("[%d km/h] Standard autopilot ignored the player top speed: %.2f", kmh, s.GetBusMaxSpeed())
		}
		s.AutopilotBerserk = true
		if s.GetBusMaxSpeed() != want || s.GetBerserkCruiseSpeed() != want {
			t.Fatalf("[%d km/h] Incorrect Berserk ceiling", kmh)
		}
		s.AutopilotEnabled = false
		if s.GetBusMaxSpeed() != want {
			t.Fatalf("[%d km/h] Switching to manual changed the ceiling to %.2f", kmh, s.GetBusMaxSpeed())
		}
		s.AutopilotBerserk = false
		if s.GetBusMaxSpeed() != want {
			t.Fatalf("[%d km/h] Clearing Berserk changed the manual ceiling to %.2f", kmh, s.GetBusMaxSpeed())
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
		if math.Abs(bus.Speed-want) > 0.01 {
			t.Fatalf("[%d km/h] Manual bus speed %.2f, want %.2f", kmh, bus.Speed, want)
		}

		// Verify converted KM/H on HUD matches target speed limit
		hudKmh := int(math.Abs(bus.Speed) * 0.18)
		if hudKmh != kmh {
			t.Fatalf("[%d km/h] Expected HUD speed %d km/h, got %d km/h (raw speed: %.2f)", kmh, kmh, hudKmh, bus.Speed)
		}
	}

	s.SetMaxSpeed(80)
	for _, want := range []int{120, 160, 180, 80} {
		if next := s.CycleMaxSpeed(); next != want {
			t.Fatalf("cycled to %d km/h, want %d", next, want)
		}
	}
	for _, legacy := range []int{60, 100} {
		s.AutopilotBerserkMaxSpeed = legacy
		if got := s.BerserkSpeedKPH(); got != 80 {
			t.Fatalf("legacy preset %d resolved to %d", legacy, got)
		}
	}
}
