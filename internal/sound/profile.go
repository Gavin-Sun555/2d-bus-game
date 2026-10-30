// Package sound generates vehicle audio independently of the simulation.
package sound

import (
	"math"
	"sync"
)

const SampleRate = 24000

// Profile is an immutable sound bank. EngineLoop is one seamless mono cycle at
// IdleRPM; Horn is a seamless mono loop. Samples are in [-1,1], at SampleRate.
// Future electric engines or recorded vehicle sounds can supply different banks
// without changing driving physics. Do not modify sample slices after registration.
type Profile struct {
	EngineLoop      []float32
	Horn            []float32
	IdleRPM, MaxRPM float64
}

var dieselOnce sync.Once
var diesel Profile

// Diesel is the default bank for every current bus model. These original
// synthesized sounds need no downloaded assets or runtime sound files.
func Diesel() Profile {
	dieselOnce.Do(func() {
		engine := make([]float32, SampleRate/5) // Two revolutions at 600 RPM.
		for i := range engine {
			p := float64(i) / float64(len(engine))
			// Low diesel firing harmonics, without the narrow combustion spikes
			// and high-frequency metallic texture that sounded like a harsh horn.
			// All frequencies fit the loop, avoiding clicks at its boundary.
			cycle := 12 * math.Pi * p
			rumble := .78*math.Sin(cycle) + .22*math.Cos(2*cycle)
			engine[i] = float32(.42 * rumble * (1 + .04*math.Sin(2*math.Pi*p)))
		}
		// Both pitches complete an integer number of periods in half a second,
		// allowing an uninterrupted sustain. The mixer supplies attack/release.
		horn := make([]float32, SampleRate/2)
		for i := range horn {
			t := float64(i) / SampleRate
			// Dual-pitch air horn with brass-like overtones.
			tone := func(hz float64) float64 {
				p := 2 * math.Pi * hz * t
				return math.Sin(p) + .27*math.Sin(2*p) + .13*math.Sin(3*p) + .06*math.Sin(4*p)
			}
			horn[i] = float32(.27 * (tone(370) + tone(466)))
		}
		diesel = Profile{EngineLoop: engine, Horn: horn, IdleRPM: 600, MaxRPM: 2350}
	})
	return diesel
}

// Valid checks that a bank can be played safely at this mixer's sample rate.
func (p Profile) Valid() bool {
	if len(p.EngineLoop) < 2 || len(p.Horn) < 2 || p.IdleRPM <= 0 || p.MaxRPM < p.IdleRPM || math.IsNaN(p.IdleRPM) || math.IsNaN(p.MaxRPM) || math.IsInf(p.IdleRPM, 0) || math.IsInf(p.MaxRPM, 0) {
		return false
	}
	for _, samples := range [][]float32{p.EngineLoop, p.Horn} {
		for _, s := range samples {
			if math.IsNaN(float64(s)) || math.IsInf(float64(s), 0) || s < -1 || s > 1 {
				return false
			}
		}
	}
	return true
}
