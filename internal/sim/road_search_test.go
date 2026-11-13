package sim

import "testing"

func TestRoadSearchPrefersNoUTurnWithinOneKilometer(t *testing.T) {
	const width = 6000.
	for _, height := range []float64{3999, 4000, 4001} {
		xs, ys := []float64{-5000, 0, width}, []float64{0, height}
		w := &World{Roads: []RoadSegment{
			{P1: Vec2{X: xs[0]}, P2: Vec2{X: width}},
			{P2: Vec2{Y: height}},
			{P1: Vec2{X: width}, P2: Vec2{X: width, Y: height}},
			{P1: Vec2{Y: height}, P2: Vec2{X: width, Y: height}},
		}}
		path, found := shortestRoadStates(w, xs, ys, roadState{1, 0, 0}, roadState{1, 0, 2})
		if !found {
			t.Fatalf("height %.0f: route not found", height)
		}
		want := 0.
		if height <= 4000 {
			want = 2 * (width + height)
		}
		if got := roadStatesDistance(path, xs, ys); got != want {
			t.Fatalf("height %.0f: route length %.0f, want %.0f", height, got, want)
		}
		if want == 0 && len(path) != 2 {
			t.Fatal("a U-turn must be represented as one reversal at the junction")
		}
	}
}
