package sim

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
)

func TestPassengerSpawnRatePropertiesAndMultipliers(t *testing.T) {
	rates := []struct {
		rate       PassengerSpawnRate
		multiplier float64
		valid      bool
	}{
		{PassengerSpawnModerate, 1.0, true},
		{PassengerSpawnFew, 0.5, true},
		{PassengerSpawnMany, 2.0, true},
		{PassengerSpawnRate(-1), 1.0, false},
		{PassengerSpawnRate(99), 1.0, false},
	}

	for _, r := range rates {
		if r.rate.Valid() != r.valid {
			t.Errorf("rate %v valid got %v, want %v", r.rate, r.rate.Valid(), r.valid)
		}
		if r.rate.Multiplier() != r.multiplier {
			t.Errorf("rate %v multiplier got %v, want %v", r.rate, r.rate.Multiplier(), r.multiplier)
		}
	}
}

func TestPassengerSpawnRateLabels(t *testing.T) {
	locale.SetLanguage(locale.LanguageZH)
	if PassengerSpawnModerate.Label() != "【适中 (默认)】" {
		t.Errorf("moderate ZH label got %q", PassengerSpawnModerate.Label())
	}
	if PassengerSpawnFew.Label() != "【偏少】" {
		t.Errorf("few ZH label got %q", PassengerSpawnFew.Label())
	}
	if PassengerSpawnMany.Label() != "【偏多】" {
		t.Errorf("many ZH label got %q", PassengerSpawnMany.Label())
	}

	locale.SetLanguage(locale.LanguageEN)
	if PassengerSpawnModerate.Label() != "【Moderate (Default)】" {
		t.Errorf("moderate EN label got %q", PassengerSpawnModerate.Label())
	}
	if PassengerSpawnFew.Label() != "【Few】" {
		t.Errorf("few EN label got %q", PassengerSpawnFew.Label())
	}
	if PassengerSpawnMany.Label() != "【Many】" {
		t.Errorf("many EN label got %q", PassengerSpawnMany.Label())
	}
	locale.SetLanguage(locale.LanguageZH)
}

func TestStopPassengerSpawnRateAffectsArrivalSpeed(t *testing.T) {
	stopMod := NewBusStop(1, "StopMod", Vec2{X: 100, Y: 100}, 0, 2, RGBA(0, 0, 0, 0), 52, 0)
	stopFew := NewBusStop(2, "StopFew", Vec2{X: 100, Y: 100}, 0, 2, RGBA(0, 0, 0, 0), 52, 0)
	stopMany := NewBusStop(3, "StopMany", Vec2{X: 100, Y: 100}, 0, 2, RGBA(0, 0, 0, 0), 52, 0)

	stopMod.SpawnInterval = 10.0
	stopFew.SpawnInterval = 10.0
	stopMany.SpawnInterval = 10.0

	// Advance 5 seconds
	dt := 5.0
	stopMod.UpdateWithDestinations(dt, PassengerSpawnModerate.Multiplier(), []int{10})
	stopFew.UpdateWithDestinations(dt, PassengerSpawnFew.Multiplier(), []int{10})
	stopMany.UpdateWithDestinations(dt, PassengerSpawnMany.Multiplier(), []int{10})

	// Moderate: timer = 5.0 < 10 -> 0 passengers
	if stopMod.WaitingPassengers != 0 {
		t.Errorf("moderate at 5s should have 0 pax, got %d", stopMod.WaitingPassengers)
	}
	// Few: timer = 2.5 < 10 -> 0 passengers
	if stopFew.WaitingPassengers != 0 {
		t.Errorf("few at 5s should have 0 pax, got %d", stopFew.WaitingPassengers)
	}
	// Many: timer = 5 * 2.0 = 10.0 >= 10 -> triggered spawn!
	if stopMany.WaitingPassengers <= 0 {
		t.Errorf("many at 5s should have spawned passengers, got %d", stopMany.WaitingPassengers)
	}
}
