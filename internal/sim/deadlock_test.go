package sim

import "testing"

func TestCompleteSignalCycleTimeout(t *testing.T) {
	for _, tc := range []struct {
		name                      string
		major                     bool
		green, yellow, left, want float64
	}{
		{"minor", false, 8, 2.5, 5.5, 42}, {"major", true, 8, 2.5, 5.5, 74}, {"custom", true, 12, 3, 7, 100},
	} {
		t.Run(tc.name, func(t *testing.T) {
			inter := &TrafficIntersection{IsMajor: tc.major, GreenDuration: tc.green, YellowDuration: tc.yellow, LeftGreenDuration: tc.left}
			tls := &TrafficLightSystem{Intersections: []*TrafficIntersection{inter}}
			if got := tls.DeadlockTimeoutAt(Vec2{}); got != tc.want {
				t.Fatalf("timeout=%v want=%v", got, tc.want)
			}
		})
	}
}
func TestStationaryJamMustBeContinuous(t *testing.T) {
	var tracker StationaryTracker
	timer := 0.0
	for i := 0; i < 55; i++ {
		timer = tracker.Update(timer, 1, Vec2{}, 0, true)
	}
	if timer != 55 {
		t.Fatal(timer)
	}
	timer = tracker.Update(timer, 1, Vec2{}, 0, false)
	if timer != 0 {
		t.Fatal("Unblocked time did not reset clock")
	}
	for i := 0; i < 100; i++ {
		timer = tracker.Update(timer, 1, Vec2{X: float64(i)}, 1, true)
		if timer > 3 {
			t.Fatal("Slow movement accumulated stationary time")
		}
	}
}
