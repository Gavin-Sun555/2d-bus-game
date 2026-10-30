package game

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/audio"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

var audioContextOnce sync.Once
var vehicleAudioContext *audio.Context

type vehicleAudio struct {
	mixer    *sound.Mixer
	player   *audio.Player
	profiles map[sim.BusModel]sound.Profile
	model    sim.BusModel
	muted    bool
	throttle float64
}

// EnableAudio is opt-in for the native game. Replays/tests constructing NewGame
// never open an audio device or change the simulation's timing/random sequence.
func (g *Game) EnableAudio() error {
	if g.vehicleAudio != nil {
		return nil
	}
	audioContextOnce.Do(func() { vehicleAudioContext = audio.NewContext(sound.SampleRate) })
	mixer := sound.NewMixer()
	player, err := vehicleAudioContext.NewPlayerF32(mixer)
	if err != nil {
		return fmt.Errorf("vehicle audio: %w", err)
	}
	// At 24 kHz stereo float32 (8 bytes/frame), each macOS AudioQueue buffer is
	// 12288 bytes (64 ms). A 60 ms (11520-byte) player buffer underruns on every
	// hardware callback, chopping 4 ms zero gaps (15.6 Hz popping/crackling) into
	// the stream. Keep >2.5 hardware buffers queued to prevent underrun pops.
	player.SetBufferSize(160 * time.Millisecond)
	g.vehicleAudio = &vehicleAudio{mixer: mixer, player: player, model: sim.BusStandard, profiles: make(map[sim.BusModel]sound.Profile)}
	return nil
}

func (g *Game) CloseAudio() error {
	if g.vehicleAudio == nil {
		return nil
	}
	err := g.vehicleAudio.player.Close()
	g.vehicleAudio = nil
	return err
}

// SetVehicleSoundProfile assigns a sound bank to a bus model. All unassigned
// models use diesel. This is a runtime presentation setting, independent of saves.
func (g *Game) SetVehicleSoundProfile(model sim.BusModel, profile sound.Profile) bool {
	if g.vehicleAudio == nil || !model.Valid() || !profile.Valid() {
		return false
	}
	a := g.vehicleAudio
	a.profiles[model] = profile
	if model == a.model {
		a.mixer.SetProfile(profile)
	}
	return true
}

func (g *Game) updateVehicleAudio(driving bool) {
	a := g.vehicleAudio
	if a == nil {
		return
	}
	active := driving && g.State == StatePlaying && !g.IsPlanMode && !g.Settings.IsOpen && !g.NetworkOpen && !g.OperationsPanel.IsOpen && !g.savePicker.open && g.Bus != nil && ebiten.IsFocused()
	if active && inpututil.IsKeyJustPressed(ebiten.KeyF8) {
		a.muted = !a.muted
		if a.muted {
			g.HUD.AddToast(locale.T("音效已静音 [F8]", "Sound muted [F8]"), true)
		} else {
			g.HUD.AddToast(locale.T("音效已开启 [F8]", "Sound enabled [F8]"), true)
		}
	}
	if active && a.model != g.Bus.Model {
		profile, ok := a.profiles[g.Bus.Model]
		if !ok {
			profile = sound.Diesel()
		}
		a.mixer.SetProfile(profile)
		a.model = g.Bus.Model
	}
	// Running audio uses the softened low-frequency diesel/road bed.
	state := sound.State{Active: active && !a.muted}
	if active {
		state.Speed = math.Abs(g.Bus.Speed) * .18
		state.Throttle = a.throttle
	}
	a.mixer.SetVolumes(g.Settings.Volumes())
	a.mixer.Update(state, active && !a.muted && ebiten.IsKeyPressed(ebiten.KeyH))
	if state.Active && !a.player.IsPlaying() {
		a.player.Play()
	}
}
