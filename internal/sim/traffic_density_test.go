package sim

import "testing"

func TestTrafficDensityCounts(t *testing.T) {
	for level, want := range map[int]int{-1: 288, 0: 288, 1: 144, 2: 288, 3: 576, 4: 864, 5: 288} {
		if got := TrafficCarCount(level); got != want {
			t.Fatalf("level%d count%d want%d", level, got, want)
		}
	}
	if MaxTrafficCars != TrafficCarCount(TrafficDensityLevels) {
		t.Fatal("maximum population differs from congestion setting")
	}
}
