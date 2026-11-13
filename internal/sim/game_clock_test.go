package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestGameClockRateBoundariesAndDaylight(t *testing.T) {
	c := GameClock{}
	if c.TimeString() != "08:00:00" || c.IsNight() || c.Darkness() != 0 {
		t.Fatal("wrong initial time")
	}
	for i := 0; i < 60; i++ {
		c.Advance(1. / 60)
	}
	if c.TimeString() != "08:00:01" {
		t.Fatal("one real second must advance one game second", c.TimeString())
	}
	for _, tc := range []struct {
		elapsed float64
		time    string
		night   bool
	}{{599, "17:59:00", false}, {600, "18:00:00", true}, {960, "00:00:00", true}, {1319, "05:59:00", true}, {1320, "06:00:00", false}, {1440, "08:00:00", false}} {
		c.ElapsedMinutes = tc.elapsed
		if c.TimeString() != tc.time || c.IsNight() != tc.night {
			t.Fatalf("elapsed %.0f: %s %v", tc.elapsed, c.TimeString(), c.IsNight())
		}
	}
	for minute := 0.; minute < 1440; minute++ {
		c.ElapsedMinutes = math.Mod(minute-480+1440, 1440)
		d := c.Darkness()
		if d < 0 || d > .7 {
			t.Fatal("invalid ambient darkness")
		}
		c.Advance(.01)
		if math.Abs(c.Darkness()-d) > .001 {
			t.Fatal("abrupt dawn or dusk transition")
		}
	}
	c.ElapsedMinutes = math.NaN()
	if c.Valid() {
		t.Fatal("invalid clock accepted")
	}
}

func TestGameClockRatesAndPersistence(t *testing.T) {
	for _, rate := range []ClockRate{ClockRealTime, ClockThirtySeconds, ClockOneMinute} {
		c := GameClock{Rate: rate}
		for i := 0; i < 60; i++ {
			c.Advance(1. / 60)
		}
		if math.Abs(c.ElapsedMinutes-rate.Multiplier()/60) > 1e-9 {
			t.Fatalf("rate %v: %v", rate, c)
		}
		before := c.ElapsedMinutes
		c.Rate = ClockOneMinute
		if c.ElapsedMinutes != before {
			t.Fatal("rate change moved clock")
		}
		data, err := json.Marshal(c)
		if err != nil {
			t.Fatal(err)
		}
		var restored GameClock
		if err = json.Unmarshal(data, &restored); err != nil || restored != c {
			t.Fatalf("roundtrip: %s %v", data, err)
		}
	}
	for _, data := range []string{`{"ElapsedSeconds":780}`, `{"ElapsedMinutes":780}`, `{"ElapsedSeconds":1,"ElapsedMinutes":780}`} {
		var c GameClock
		if err := json.Unmarshal([]byte(data), &c); err != nil || c.TimeString() != "21:00:00" || c.Rate != ClockRealTime {
			t.Fatalf("legacy: %s %+v", data, c)
		}
	}
	c := GameClock{ElapsedMinutes: 960 - 1./60}
	if c.TimeString() != "23:59:59" {
		t.Fatal(c.TimeString())
	}
	c.Advance(1)
	if c.TimeString() != "00:00:00" {
		t.Fatal(c.TimeString())
	}
	if (GameClock{Rate: 3}).Valid() {
		t.Fatal("invalid rate accepted")
	}
}
