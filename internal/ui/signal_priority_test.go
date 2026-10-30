package ui

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/hajimehoshi/ebiten/v2"
	"testing"
)

func TestBusSignalPrioritySettingAndLayout(t *testing.T) {
	var old Settings
	if err := json.Unmarshal([]byte(`{"TrafficDensity":2}`), &old); err != nil {
		t.Fatal(err)
	}
	if old.BusSignalPriority {
		t.Fatal("old save unexpectedly enabled priority")
	}
	s := NewSettings()
	s.BusSignalPriority = true
	raw, _ := json.Marshal(s)
	if err := json.Unmarshal(raw, &old); err != nil || !old.BusSignalPriority {
		t.Fatal("priority setting did not persist")
	}
	previous := locale.CurrentLanguage
	defer func() { locale.CurrentLanguage = previous }()
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.CurrentLanguage = lang
		s.IsOpen = true
		s.Draw(ebiten.NewImage(1280, 720))
		b, p := s.busSignalPriorityBounds, s.passengerTimingBounds
		if b.X < p.X+p.W || b.Y+b.H > s.closeButtonBounds.Y || b.Y < s.operationsBounds.Y+s.operationsBounds.H {
			t.Fatal("priority option overlaps existing controls")
		}
	}
}
