package sim

import "testing"

func TestSignalGeometryCacheTracksRoadEdits(t *testing.T) {
	inter := &TrafficIntersection{}
	for _, lw := range []float64{42, 52, 64, 78} {
		for _, lanes := range []int{2, 4, 6, 8, 2} {
			inter.ApproachHasRoad = [4]bool{true, true, true, true}
			inter.ApproachLanes = [4]int{lanes, lanes, lanes, lanes}
			a, b, c, d := inter.GetApproachDimensions(lw)
			half := lw + 6
			if lanes >= 4 {
				half = lw*float64(lanes)/2 + 12
			}
			want := NewJunctionGeometry(half, half, lw).HalfX + JunctionStopInset
			if a != want || c != want || b != want-25 || d != want-25 {
				t.Fatalf("Stale geometry: width=%v lanes=%v got=%v want=%v", lw, lanes, a, want)
			}
			if again, _, _, _ := inter.GetApproachDimensions(lw); again != a {
				t.Fatal("Cached query changed result")
			}
		}
		inter.ApproachHasRoad = [4]bool{}
		if got, _, _, _ := inter.GetApproachDimensions(lw); got != lw*1.6+18 {
			t.Fatal("Removed road retained cached dimensions")
		}
	}
}
