package ui

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
	"testing"
)

func TestPassengerTimingSettingsAndPersistence(t *testing.T) {
	s := NewSettings()
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		initial := s.ModelPassengerRates(model)
		if initial != sim.DefaultPassengerRates(model) {
			t.Fatal("wrong model default")
		}
		for door := 0; door < 2; door++ {
			seen := map[float64]bool{}
			for i := 0; i < len(sim.PassengerTimeOptions); i++ {
				r := s.ModelPassengerRates(model)
				v := r.Boarding
				if door == 1 {
					v = r.Alighting
				}
				seen[v] = true
				s.CyclePassengerRate(model, door)
			}
			if len(seen) != 7 || s.ModelPassengerRates(model) != initial {
				t.Fatal("missing time option or cross-door change")
			}
		}
	}
	s.CyclePassengerRate(sim.BusArticulated, 0)
	data, _ := json.Marshal(s)
	var restored Settings
	if err := json.Unmarshal(data, &restored); err != nil {
		t.Fatal(err)
	}
	if restored.ModelPassengerRates(sim.BusArticulated) != (sim.PassengerRates{Boarding: 1.5, Alighting: .5}) {
		t.Fatal("custom timing lost")
	}
	var legacy Settings
	if err := json.Unmarshal([]byte(`{}`), &legacy); err != nil {
		t.Fatal(err)
	}
	if legacy.ModelPassengerRates(sim.BusArticulated) != (sim.PassengerRates{Boarding: 1, Alighting: .5}) {
		t.Fatal("old settings did not adopt new defaults")
	}
	s.IsOpen = true
	s.PassengerTimingOpen = true
	s.Draw(ebiten.NewImage(800, 720))
	for _, row := range s.passengerRateBounds {
		for _, b := range row {
			if b.X < 0 || b.Y < 0 || b.X+b.W > 800 || b.Y+b.H > s.passengerTimingClose.Y {
				t.Fatal("timing controls clipped")
			}
		}
	}
}
