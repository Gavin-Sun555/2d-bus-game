package sim

import (
	"math"
	"testing"
)

func TestCurrentRoadInfoRetainsRoadAtShoulderAndBay(t *testing.T) {
	for _, lanes := range []int{2, 4, 6} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			u := Vec2{X: math.Cos(h), Y: math.Sin(h)}
			right := Vec2{X: -u.Y, Y: u.X}
			w := &World{Roads: []RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: lanes}}}
			for _, extra := range []float64{0, 20, 38} {
				pos := right.Mul(float64(lanes)*52/2 + extra)
				center, ew, multi, six := CurrentRoadInfo(w, pos, h, 52)
				if math.Abs(center) > 1e-6 || ew != (math.Abs(u.X) > .5) || multi != (lanes >= 4) || six != (lanes == 6) {
					t.Fatalf("lanes%d heading%.2f at%v misidentified: center%.1f EW%v multi%v six%v", lanes, h, pos, center, ew, multi, six)
				}
			}
		}
	}
}
