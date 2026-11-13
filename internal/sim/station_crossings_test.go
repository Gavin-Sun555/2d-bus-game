package sim

import (
	"math"
	"testing"
)

func TestLargeGridStationCrossingsAreBalancedAndCoverOrdinaryStops(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, theme := range BuiltinMapThemes() {
			for _, brt := range []bool{false, true} {
				w := &World{LargeGrid: true, CenterBRT: brt}
				w.BuildMap(theme, lw)
				var counts [4]int
				for _, crossing := range w.StationCrossings() {
					counts[crossing.Kind]++
					if w.InsideJunction(crossing.Center, lw, 75) || w.IsAtRoundabout(crossing.Center) {
						t.Errorf("%s width %.0f BRT=%t: crossing inside a junction at %v", theme.ShortName(), lw, brt, crossing.Center)
					}
				}
				for _, count := range counts {
					if count == 0 || math.Abs(float64(count-counts[0])) > 1 {
						t.Fatalf("%s: unbalanced crossings %v", theme.ShortName(), counts)
					}
				}
				for _, stop := range w.Stops {
					if stop.CenterBRT {
						continue
					}
					u := UnitFromAngle(stop.Heading)
					r := Vec2{X: -u.Y, Y: u.X}
					found := false
					for _, crossing := range w.StationCrossings() {
						delta := crossing.Center.Sub(stop.RoadCenter)
						if math.Abs(delta.Dot(r)) < 1 && math.Abs(delta.Dot(u)) <= 950 &&
							math.Abs(math.Cos(crossing.Heading-stop.Heading)) > .99 {
							found = true
							break
						}
					}
					if !found {
						t.Errorf("%s BRT=%t: stop %d at %v has no nearby crossing", theme.ShortName(), brt, stop.ID, stop.RoadCenter)
					}
				}
				w.LargeGrid = false
				w.BuildMap(theme, lw)
				if len(w.StationCrossings()) != 0 {
					t.Fatalf("%s: crossings survived a switch to the small grid", theme.ShortName())
				}
			}
		}
	}
}

func TestStationCrossingRedStopsBothDirections(t *testing.T) {
	const lw = 52.
	w := &World{LargeGrid: true}
	w.BuildMap(MapThemeMetropolis, lw)
	for _, crossing := range w.StationCrossings() {
		if crossing.Kind != StationSignalZebra {
			continue
		}
		forward := UnitFromAngle(crossing.Heading)
		right := Vec2{X: -forward.Y, Y: forward.X}
		lights := &TrafficLightSystem{StationCrossingElapsed: 30 + 36 - crossing.phaseOffset}
		for _, dir := range []float64{-1, 1} {
			before := crossing.Center.Sub(forward.Mul(dir * (StationCrossingStopOffset + 9))).Add(right.Mul(dir * lw / 2))
			heading := math.Atan2(dir*forward.Y, dir*forward.X)
			if got := w.StationCrossingSpeedLimit(lights, before, heading, 25, lw, 135); got != 0 {
				t.Errorf("direction %.0f: red signal stop limit is %g", dir, got)
			}
			if got := w.StationCrossingSpeedLimit(lights, crossing.Center, heading, 25, lw, 135); !math.IsInf(got, 1) {
				t.Errorf("direction %.0f: red signal stopped a vehicle that had crossed the line", dir)
			}
		}
		return
	}
	t.Fatal("no signal-controlled crossing")
}
