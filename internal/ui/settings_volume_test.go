package ui

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
	"github.com/hajimehoshi/ebiten/v2"
	"testing"
)

func TestVolumeSettingsCompatibilityAndLayout(t *testing.T) {
	var s Settings
	if err := json.Unmarshal([]byte(`{"TrafficDensity":2}`), &s); err != nil {
		t.Fatal(err)
	}
	if s.Volumes() != sound.DefaultVolumes() {
		t.Fatal("old save became silent")
	}
	s.AdjustVolume(0, -100)
	s.AdjustVolume(1, -70)
	s.AdjustVolume(2, -40)
	data, _ := json.Marshal(s)
	var loaded Settings
	if err := json.Unmarshal(data, &loaded); err != nil {
		t.Fatal(err)
	}
	if loaded.Volumes() != (sound.Volumes{Engine: 0, Announcements: 30, Horn: 60}) {
		t.Fatalf("volumes lost: %+v", loaded.Volumes())
	}
	loaded.AdjustVolume(0, -10)
	loaded.AdjustVolume(2, 100)
	if loaded.Volumes().Engine != 0 || loaded.Volumes().Horn != 100 {
		t.Fatal("out of range volume")
	}
	loaded.IsOpen = true
	loaded.Draw(ebiten.NewImage(1280, 720))
	for _, row := range loaded.volumeBounds {
		for _, b := range row {
			if b.Y < 0 || b.Y+b.H > loaded.saveBounds.Y {
				t.Fatalf("overlapping volume layout: %+v", b)
			}
		}
	}
}
