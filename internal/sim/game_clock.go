package sim

import (
	"encoding/json"
	"fmt"
	"math"
)

// ClockRate is game seconds per real second, with real time as the zero/default.
type ClockRate int

const (
	ClockRealTime ClockRate = iota
	ClockThirtySeconds
	ClockOneMinute
)

func (r ClockRate) Valid() bool { return r >= ClockRealTime && r <= ClockOneMinute }
func (r ClockRate) Multiplier() float64 {
	switch r {
	case ClockThirtySeconds:
		return 30
	case ClockOneMinute:
		return 60
	default:
		return 1
	}
}

// Zero is a new game at 08:00:00 at real-time speed.
type GameClock struct {
	ElapsedMinutes float64   `json:",omitempty"`
	Rate           ClockRate `json:",omitempty"`
}

// Early saves called this value ElapsedSeconds but stored game minutes.
// Preserve their time of day while defaulting their missing rate to real time.
func (c *GameClock) UnmarshalJSON(data []byte) error {
	var saved struct {
		ElapsedMinutes *float64
		ElapsedSeconds float64
		Rate           ClockRate
	}
	if err := json.Unmarshal(data, &saved); err != nil {
		return err
	}
	c.ElapsedMinutes = saved.ElapsedSeconds
	if saved.ElapsedMinutes != nil {
		c.ElapsedMinutes = *saved.ElapsedMinutes
	}
	c.Rate = saved.Rate
	return nil
}

func (c GameClock) Valid() bool {
	return c.Rate.Valid() && !math.IsNaN(c.ElapsedMinutes) && !math.IsInf(c.ElapsedMinutes, 0) && c.ElapsedMinutes >= 0 && c.ElapsedMinutes <= 1e12
}
func (c *GameClock) Advance(dt float64) {
	if dt > 0 && !math.IsNaN(dt) && !math.IsInf(dt, 0) {
		c.ElapsedMinutes += dt * c.Rate.Multiplier() / 60
	}
}
func (c GameClock) MinuteOfDay() float64 { return math.Mod(8*60+c.ElapsedMinutes, 24*60) }
func (c GameClock) TimeString() string {
	seconds := int(c.MinuteOfDay()*60+1e-7) % (24 * 60 * 60)
	return fmt.Sprintf("%02d:%02d:%02d", seconds/3600, seconds/60%60, seconds%60)
}
func (c GameClock) IsNight() bool      { m := c.MinuteOfDay(); return m < 6*60 || m >= 18*60 }
func smoothDaylight(x float64) float64 { x = Clamp(x, 0, 1); return x * x * (3 - 2*x) }

// A two-hour dawn/dusk transition avoids abruptly repainting the whole city.
func (c GameClock) Darkness() float64 {
	h := c.MinuteOfDay() / 60
	switch {
	case h < 5 || h >= 19:
		return .70
	case h < 7:
		return .70 * (1 - smoothDaylight((h-5)/2))
	case h < 17:
		return 0
	default:
		return .70 * smoothDaylight((h-17)/2)
	}
}
