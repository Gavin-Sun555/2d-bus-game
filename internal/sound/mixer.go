package sound

import (
	"encoding/binary"
	"math"
	"sync"
)

// State is a copy of audible driver inputs, never a reference to simulation data.
// Speed is absolute speed in km/h; Throttle is in [0,1].
type State struct {
	Active          bool
	HornOnly        bool // Suppress synthesized running noise; manual horn remains audible.
	Speed, Throttle float64
}

// Mixer is an endless stereo float32 PCM reader. One bounded voice each for the
// engine, rolling noise and horn avoids a player/goroutine per simulated vehicle.
// Update runs on the game thread; Read runs on the audio thread.
type Mixer struct {
	mu                                             sync.Mutex
	profile                                        Profile
	state                                          State
	phase, rpm, load, speed, gain                  float64
	engineLow, engineMid, engineSmooth             float64
	rolling, rollingLow, rollingMid, rollingSmooth float64
	noise                                          uint32
	horn                                           int
	hornBank                                       []float32
	hornHeld                                       bool
	hornGain                                       float64
	hornPresses, hornPulseLeft, hornGapLeft        int
	volumes                                        Volumes
	pending                                        [8]byte
	pendingN                                       int
}

func NewMixer() *Mixer {
	return &Mixer{profile: Diesel(), rpm: 600, noise: 0x782fa91, volumes: DefaultVolumes()}
}

// SetProfile returns false for an unusable bank and keeps the current bank.
func (m *Mixer) SetProfile(p Profile) bool {
	if !p.Valid() {
		return false
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	m.profile = p
	m.phase = 0
	m.rpm = p.IdleRPM
	m.engineLow, m.engineMid, m.engineSmooth = 0, 0, 0
	return true
}

// Update receives the current held-key state, not a key-down event.
func (m *Mixer) Update(s State, hornHeld bool) {
	if math.IsNaN(s.Speed) || math.IsInf(s.Speed, 0) {
		s.Speed = 0
	}
	if math.IsNaN(s.Throttle) || math.IsInf(s.Throttle, 0) {
		s.Throttle = 0
	}
	s.Speed = max(0, s.Speed)
	s.Throttle = min(1, max(0, s.Throttle))
	m.mu.Lock()
	defer m.mu.Unlock()
	m.state = s
	held := hornHeld && s.Active
	if held && !m.hornHeld {
		m.hornPresses++ // Retain separate taps even between audio callbacks.
	}
	m.hornHeld = held
	if !s.Active {
		m.hornPresses, m.hornPulseLeft, m.hornGapLeft = 0, 0, 0
	}
}

func (m *Mixer) sample() float32 {
	if !m.state.Active && m.gain < .00001 {
		m.gain = 0
		m.hornBank = nil
		m.horn = 0
		m.hornGain = 0
		return 0
	}
	targetGain := 0.
	if m.state.Active {
		targetGain = 1
	}
	m.gain += (targetGain - m.gain) * .002 // ~20 ms, avoiding clicks on pause/mute.
	value := 0.
	if !m.state.HornOnly {
		m.load += (m.state.Throttle - m.load) * .0005
		m.speed += (min(m.state.Speed, 120.) - m.speed) * .001
		// Virtual automatic gears provide rising RPM and a drop on each upshift.
		// This affects only sound, including on manual/reverse operation.
		speed := m.speed
		gear := min(5, int(speed/22))
		ratio := (speed - float64(gear)*22) / 22
		targetRPM := m.profile.IdleRPM + (m.profile.MaxRPM-m.profile.IdleRPM)*min(1, .12+ratio*.62+m.load*.26)
		if speed < 1 {
			targetRPM = m.profile.IdleRPM + (m.profile.MaxRPM-m.profile.IdleRPM)*m.load*.4
		}
		m.rpm += (targetRPM - m.rpm) * .0002
		bank := m.profile.EngineLoop
		i := int(m.phase)
		j := (i + 1) % len(bank)
		f := m.phase - float64(i)
		engine := float64(bank[i]) + (float64(bank[j])-float64(bank[i]))*f
		m.phase += m.rpm / m.profile.IdleRPM
		if m.phase >= float64(len(bank)) {
			m.phase = math.Mod(m.phase, float64(len(bank)))
		}
		// Local PRNG: never consumes the traffic simulation's random sequence.
		m.noise ^= m.noise << 13
		m.noise ^= m.noise >> 17
		m.noise ^= m.noise << 5
		noise := float64(int32(m.noise)) / 2147483648.
		// Multi-stage low-pass cascades soften the running bed and strip sharp
		// white-noise transients. The H horn is mixed below and keeps its
		// original attack and timbre.
		m.engineLow += (engine - m.engineLow) * .06
		m.engineMid += (m.engineLow - m.engineMid) * .06
		m.engineSmooth += (m.engineMid - m.engineSmooth) * .06
		m.rolling += (noise - m.rolling) * .022
		m.rollingLow += (m.rolling - m.rollingLow) * .022
		m.rollingMid += (m.rollingLow - m.rollingMid) * .022
		m.rollingSmooth += (m.rollingMid - m.rollingSmooth) * .022
		value = m.engineSmooth*(.18+.13*m.load) + m.rollingSmooth*.055*min(1, speed/70)
	}
	value *= 1.15 * float64(m.volumes.Engine) / 100
	if m.hornGapLeft > 0 {
		m.hornGapLeft--
	} else if len(m.hornBank) == 0 && m.hornPresses > 0 {
		m.hornPresses--
		m.hornBank = m.profile.Horn
		m.horn, m.hornPulseLeft = 0, SampleRate*80/1000
	}
	if len(m.hornBank) > 0 {
		sustain := m.state.Active && (m.hornPulseLeft > 0 || (m.hornHeld && m.hornPresses == 0))
		if m.hornPulseLeft > 0 {
			m.hornPulseLeft--
		}
		if sustain {
			m.hornGain = math.Min(1, m.hornGain+1./(.015*SampleRate))
		} else {
			m.hornGain = math.Max(0, m.hornGain-1./(.04*SampleRate))
		}
		value += float64(m.hornBank[m.horn]) * m.hornGain * float64(m.volumes.Horn) / 100
		m.horn = (m.horn + 1) % len(m.hornBank)
		if m.hornGain == 0 {
			m.hornBank = nil
			m.horn = 0
			m.hornGapLeft = SampleRate * 30 / 1000
		}
	}
	return float32(min(.95, max(-.95, value*m.gain)))
}

func (m *Mixer) Read(p []byte) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	written := 0
	for written < len(p) {
		if m.pendingN == 0 {
			bits := math.Float32bits(m.sample())
			binary.LittleEndian.PutUint32(m.pending[:4], bits)
			binary.LittleEndian.PutUint32(m.pending[4:], bits)
			m.pendingN = 8
		}
		n := copy(p[written:], m.pending[8-m.pendingN:])
		written += n
		m.pendingN -= n
	}
	return written, nil
}

// SetVolumes changes presentation gain without changing vehicle state.
func (m *Mixer) SetVolumes(v Volumes) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.volumes = v.Normalize()
}
