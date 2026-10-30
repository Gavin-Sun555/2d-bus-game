package sound

import (
	"math"
	"testing"
)

func TestSeparateEngineAndHornVolumes(t *testing.T) {
	for _, horn := range []bool{false, true} {
		full, quiet := NewMixer(), NewMixer()
		v := DefaultVolumes()
		if horn {
			v.Engine, v.Horn = 0, 25
		} else {
			v.Engine, v.Horn = 25, 0
		}
		quiet.SetVolumes(v)
		v.Engine, v.Horn = 100, 0
		if horn {
			v.Engine, v.Horn = 0, 100
		}
		full.SetVolumes(v)
		state := State{Active: true, Speed: 40, Throttle: .8}
		full.Update(state, horn)
		quiet.Update(state, horn)
		a, b := rms(render(t, full, SampleRate/2)), rms(render(t, quiet, SampleRate/2))
		if a <= 0 || math.Abs(b/a-.25) > .00001 {
			t.Fatalf("horn=%v gain ratio=%f", horn, b/a)
		}
	}
	m := NewMixer()
	m.SetVolumes(Volumes{})
	m.Update(State{Active: true, Speed: 60, Throttle: 1}, true)
	if rms(render(t, m, SampleRate)) != 0 {
		t.Fatal("zero volume must mute both channels")
	}
}

func TestRapidHornTapsRemainSeparateBeforeAudioRead(t *testing.T) {
	for _, taps := range []int{1, 2, 3} {
		m := NewMixer()
		s := State{Active: true, HornOnly: true}
		m.Update(s, false)
		render(t, m, SampleRate/5)
		// All events arrive within one native audio callback interval.
		for i := 0; i < taps; i++ {
			m.Update(s, true)
			m.Update(s, false)
		}
		bursts, sounding := 0, false
		for i := 0; i < 160; i++ {
			on := rms(render(t, m, SampleRate/200)) > .005
			if on && !sounding {
				bursts++
			}
			sounding = on
		}
		if bursts != taps {
			t.Fatalf("%d taps produced %d audible bursts", taps, bursts)
		}
	}
}

func TestSpeechVolumeCommand(t *testing.T) {
	v := .25
	u := Utterance{Text: "下一站 [[volm 1]] Central", Volume: &v}
	if got := speechText(u); got != "[[volm 0.250]]下一站 ［［volm 1］］ Central" {
		t.Fatalf("unsafe/incorrect speech: %s", got)
	}
}
