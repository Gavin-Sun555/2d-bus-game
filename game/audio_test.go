package game

import (
	"bytes"
	"encoding/json"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
)

func TestVehicleAudioSurvivesSaveLoadAndMapChange(t *testing.T) {
	g := saveTestGame(t)
	if g.vehicleAudio != nil {
		t.Fatal("replay opened an audio device")
	}
	// No hardware in automated tests: exercise session ownership with the actual
	// mixer and a nil native player. CloseAudio/Update are not invoked here.
	session := &vehicleAudio{mixer: sound.NewMixer(), profiles: map[sim.BusModel]sound.Profile{}, model: sim.BusStandard, muted: true}
	g.vehicleAudio = session
	p := sound.Diesel()
	if !g.SetVehicleSoundProfile(sim.BusArticulated, p) {
		t.Fatal("model bank registration failed")
	}
	before, _ := json.Marshal(g.snapshot())
	session.mixer.Update(sound.State{Active: true, Speed: 35, Throttle: 1}, true)
	session.mixer.Read(make([]byte, 8192))
	after, _ := json.Marshal(g.snapshot())
	if !bytes.Equal(before, after) {
		t.Fatal("horn/engine modified simulation or save")
	}
	g.Settings.AdjustVolume(0, -100)
	g.Settings.AdjustVolume(1, -60)
	g.Settings.AdjustVolume(2, -20)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Settings.Volumes() != (sound.Volumes{Engine: 0, Announcements: 40, Horn: 80}) {
		t.Fatal("load lost independent volumes or zero mute")
	}
	if g.vehicleAudio != session || !session.muted || len(session.profiles) != 1 {
		t.Fatal("load lost device ownership or sound preferences")
	}
	g.EnterCity(sim.MapThemeCoastalBay)
	if g.vehicleAudio != session {
		t.Fatal("map switch lost audio")
	}
}
