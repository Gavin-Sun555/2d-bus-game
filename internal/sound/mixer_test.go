package sound

import (
	"bytes"
	"encoding/binary"
	"io"
	"math"
	"sync"
	"testing"
)

func render(t *testing.T, m *Mixer, frames int) []byte {
	t.Helper()
	p := make([]byte, frames*8)
	if n, err := io.ReadFull(m, p); err != nil || n != len(p) {
		t.Fatalf("read %d: %v", n, err)
	}
	return p
}
func rms(p []byte) float64 {
	sum := 0.
	for i := 0; i < len(p); i += 8 {
		v := float64(math.Float32frombits(binary.LittleEndian.Uint32(p[i:])))
		sum += v * v
	}
	return math.Sqrt(sum / float64(len(p)/8))
}
func TestPCMStreamChunkingAndLevels(t *testing.T) {
	a, b := NewMixer(), NewMixer()
	state := State{Active: true, Speed: 45, Throttle: .8}
	a.Update(state, true)
	b.Update(state, true)
	expected := render(t, a, SampleRate)
	var actual bytes.Buffer
	for actual.Len() < len(expected) {
		n := min(137, len(expected)-actual.Len())
		p := make([]byte, n)
		if _, err := io.ReadFull(b, p); err != nil {
			t.Fatal(err)
		}
		actual.Write(p)
	}
	if !bytes.Equal(actual.Bytes(), expected) {
		t.Fatal("PCM depends on audio read chunk size")
	}
	peak := 0.
	for i := 0; i < len(expected); i += 8 {
		left := binary.LittleEndian.Uint32(expected[i:])
		right := binary.LittleEndian.Uint32(expected[i+4:])
		v := float64(math.Float32frombits(left))
		if left != right || math.IsNaN(v) || math.IsInf(v, 0) || math.Abs(v) > .95 {
			t.Fatalf("invalid stereo sample %v", v)
		}
		peak = max(peak, math.Abs(v))
	}
	if peak < .25 || rms(expected) < .03 {
		t.Fatal("horn or engine inaudible")
	}
}
func TestHornHoldReleaseAndPause(t *testing.T) {
	m := NewMixer()
	s := State{Active: true}
	m.Update(s, false)
	render(t, m, SampleRate/2)
	idle := rms(render(t, m, SampleRate/4))
	m.Update(s, true)
	// Sustain beyond several loop boundaries. No automatic timeout or gaps.
	for i := 0; i < 24; i++ {
		horn := rms(render(t, m, SampleRate/8))
		if horn < idle*2 {
			t.Fatalf("held horn dropped out at block %d: %f idle %f", i, horn, idle)
		}
	}
	m.Update(s, false)
	render(t, m, SampleRate/20) // 40ms release, then engine only.
	if len(m.hornBank) != 0 {
		t.Fatal("released horn continues")
	}
	if got := rms(render(t, m, SampleRate/4)); math.Abs(got-idle) > .001 {
		t.Fatalf("release did not restore idle: %f want %f", got, idle)
	}
	// Short taps and quickly pressing again must work as well.
	m.Update(s, true)
	render(t, m, SampleRate/100)
	m.Update(s, false)
	render(t, m, SampleRate/100)
	m.Update(s, true)
	if rms(render(t, m, SampleRate/4)) < idle*2 {
		t.Fatal("rapid re-press failed")
	}
	// Pause/mute cancels the horn even if the key remains held.
	m.Update(State{}, true)
	render(t, m, SampleRate/2)
	if len(m.hornBank) != 0 || rms(render(t, m, SampleRate/4)) != 0 {
		t.Fatal("paused horn not cancelled")
	}
	m.Update(s, false)
	if got := rms(render(t, m, SampleRate/2)); got < .01 || got > idle*1.1 {
		t.Fatal("resume retained a stuck horn or lost engine")
	}
}
func TestHeldHornDoesNotRestartPerGameTick(t *testing.T) {
	a, b := NewMixer(), NewMixer()
	s := State{Active: true}
	a.Update(s, true)
	b.Update(s, true)
	for tick := 0; tick < 180; tick++ {
		a.Update(s, true)
		if !bytes.Equal(render(t, a, SampleRate/60), render(t, b, SampleRate/60)) {
			t.Fatalf("held-key updates restart/stack horn at tick%d", tick)
		}
	}
}
func TestThrottleAndProfiles(t *testing.T) {
	m := NewMixer()
	m.Update(State{Active: true}, false)
	render(t, m, SampleRate)
	idle := m.rpm
	m.Update(State{Active: true, Speed: 15, Throttle: 1}, false)
	render(t, m, SampleRate)
	if m.rpm <= idle+300 {
		t.Fatal("acceleration does not raise engine pitch")
	}
	original := m.profile
	for _, p := range []Profile{{}, {EngineLoop: []float32{0, 0}, Horn: []float32{0, 0}, IdleRPM: 600, MaxRPM: math.NaN()}, {EngineLoop: []float32{0, 2}, Horn: []float32{0, 0}, IdleRPM: 600, MaxRPM: 1000}} {
		if m.SetProfile(p) {
			t.Fatal("invalid profile accepted")
		}
	}
	if &m.profile.EngineLoop[0] != &original.EngineLoop[0] {
		t.Fatal("invalid bank changed current sound")
	}
	replacement := Profile{EngineLoop: []float32{.1, -.1}, Horn: []float32{.2, -.2}, IdleRPM: 800, MaxRPM: 2000}
	if !m.SetProfile(replacement) {
		t.Fatal("future vehicle sound bank rejected")
	}
	render(t, m, 100)
	if m.profile.IdleRPM != 800 {
		t.Fatal("profile not applied")
	}
}
func TestConcurrentAudioAndGameThread(t *testing.T) {
	m := NewMixer()
	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		defer wg.Done()
		p := make([]byte, 512)
		for i := 0; i < 1000; i++ {
			m.Read(p)
		}
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 1000; i++ {
			m.Update(State{Active: i%3 != 0, Speed: float64(i % 120), Throttle: .8}, i%10 == 0)
			if i%20 == 0 {
				m.SetProfile(Diesel())
			}
		}
	}()
	wg.Wait()
}
func TestStreamingDoesNotAllocate(t *testing.T) {
	m := NewMixer()
	m.Update(State{Active: true, Speed: 40, Throttle: 1}, true)
	p := make([]byte, 1024)
	if allocs := testing.AllocsPerRun(100, func() { m.Read(p) }); allocs != 0 {
		t.Fatalf("%g allocations per read", allocs)
	}
}
func BenchmarkVehicleAudio(b *testing.B) {
	m := NewMixer()
	m.Update(State{Active: true, Speed: 60, Throttle: .8}, true)
	p := make([]byte, SampleRate/60*8)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		m.Read(p)
	}
}

func TestHornOnlyDrivingIsSilentUntilHAndAfterRelease(t *testing.T) {
	m := NewMixer()
	for _, speed := range []float64{0, 15, 45, 80, 120} {
		state := State{Active: true, HornOnly: true, Speed: speed, Throttle: 1}
		m.Update(state, false)
		if rms(render(t, m, SampleRate/4)) != 0 {
			t.Fatalf("running noise audible at %v km/h", speed)
		}
		m.Update(state, true)
		if rms(render(t, m, SampleRate/4)) < .1 {
			t.Fatal("explicit H horn was lost")
		}
		m.Update(state, false)
		render(t, m, SampleRate/20)
		if rms(render(t, m, SampleRate/4)) != 0 {
			t.Fatal("horn/running sound remained after H release")
		}
	}
	m.Update(State{}, true)
	render(t, m, SampleRate/4)
	if rms(render(t, m, SampleRate/4)) != 0 {
		t.Fatal("paused audio not silent")
	}
}

func TestSoftRunningAudioRetainsEngineAndLeavesHornUnfiltered(t *testing.T) {
	for _, speed := range []float64{0, 40, 80, 120} {
		m := NewMixer()
		m.Update(State{Active: true, Speed: speed, Throttle: .8}, false)
		render(t, m, SampleRate)
		p := render(t, m, SampleRate)
		level := rms(p)
		var difference, maxStep float64
		var previous float64
		for i := 0; i < len(p); i += 8 {
			sample := float64(math.Float32frombits(binary.LittleEndian.Uint32(p[i:])))
			if i > 0 {
				step := math.Abs(sample - previous)
				difference += step * step
				maxStep = max(maxStep, step)
			}
			previous = sample
		}
		roughness := math.Sqrt(difference/float64(len(p)/8-1)) / level
		if level < .01 || level > .09 || roughness > .04 || maxStep > .004 {
			t.Fatalf("speed=%v level=%g roughness=%g maxStep=%g", speed, level, roughness, maxStep)
		}
		t.Logf("%v km/h: RMS=%.5f, normalized sample difference=%.5f, maxStep=%.5f", speed, level, roughness, maxStep)
	}
	// Removing the running bed from the full mix must recover the original H
	// horn: the engine filters must never dull or alter the deliberate horn.
	full, bed, horn := NewMixer(), NewMixer(), NewMixer()
	state := State{Active: true, Speed: 60, Throttle: .8}
	full.Update(state, true)
	bed.Update(state, false)
	state.HornOnly = true
	horn.Update(state, true)
	a, b, c := render(t, full, SampleRate), render(t, bed, SampleRate), render(t, horn, SampleRate)
	for i := 0; i < len(a); i += 8 {
		read := func(p []byte) float64 { return float64(math.Float32frombits(binary.LittleEndian.Uint32(p[i:]))) }
		if math.Abs((read(a)-read(b))-read(c)) > 1e-6 {
			t.Fatalf("horn timbre changed at frame %d", i/8)
		}
	}
}
