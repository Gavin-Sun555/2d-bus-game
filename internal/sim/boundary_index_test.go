package sim

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"testing"
)

func checkBoundaryOracle(t *testing.T, w *World, p Vec2, radius, width float64) {
	t.Helper()
	want := w.checkBoundaries(p, radius, width, nil)
	got := w.CheckBoundaries(p, radius, width)
	if got != want {
		t.Fatalf("point%v radius%g width%g: indexed%+v full%+v", p, radius, width, got, want)
	}
}

func TestBoundaryIndexMatchesFullMapQueries(t *testing.T) {
	for _, theme := range []MapTheme{MapThemeMetropolis, MapThemeCoastalBay, MapThemeRiverValley, MapThemeEmeraldLake, MapThemeTwinCities, MapThemeCustom} {
		for _, width := range []float64{20, 42, 52, 78, 100} {
			t.Run(fmt.Sprintf("map%d/width%g", theme, width), func(t *testing.T) {
				w := NewWorld(width, theme)
				w.BeginBoundaryQueries(width)
				defer w.EndBoundaryQueries()
				if w.boundaryCache == nil {
					t.Fatal("missing index")
				}
				rng := rand.New(rand.NewSource(8241024))
				bx, by := w.PerimeterBounds()
				count := 3000
				if testing.Short() {
					count = 200
				}
				for i := 0; i < count; i++ {
					p := Vec2{(rng.Float64()*2 - 1) * (bx + 100), (rng.Float64()*2 - 1) * (by + 100)}
					checkBoundaryOracle(t, w, p, rng.Float64()*110, width)
				}
				// Probe the queue, shelter edges and long-platform exit taper.
				for _, st := range w.Stops {
					for _, along := range []float64{-1200.000001, -1200, -1199.999999, -st.Length / 2, 0, 42, st.Length / 2, st.Length/2 + 110, st.Length/2 + 260} {
						for _, across := range []float64{-float64(st.RoadLanes)/2*width - st.Width*.48 - 15, 24, 36, 48, st.Width/2 + 12} {
							p := st.Pos.Add(Vec2{math.Cos(st.Heading)*along - math.Sin(st.Heading)*across, math.Sin(st.Heading)*along + math.Cos(st.Heading)*across})
							checkBoundaryOracle(t, w, p, .5, width)
						}
					}
				}
			})
		}
	}
}

func TestBoundaryIndexArbitrarySegmentsAndOriginalTies(t *testing.T) {
	rng := rand.New(rand.NewSource(9912))
	w := &World{}
	for i := 0; i < 100; i++ {
		a := Vec2{rng.Float64()*10000 - 5000, rng.Float64()*6000 - 3000}
		b := a.Add(Vec2{rng.Float64()*6000 - 3000, rng.Float64()*3000 - 1500})
		if i%7 == 0 {
			b = a
		}
		w.Roads = append(w.Roads, RoadSegment{P1: a, P2: b, Lanes: 2 + 2*(i%5)})
	}
	// Diagonal, rotated platforms and overlapping shelters.
	for i := 0; i < 20; i++ {
		w.Stops = append(w.Stops, &BusStop{Pos: Vec2{rng.Float64()*10000 - 5000, rng.Float64()*6000 - 3000}, Heading: rng.Float64() * math.Pi * 2, Length: 336, Width: 44, RoadLanes: 8})
	}
	w.BeginBoundaryQueries(52)
	for i := 0; i < 15000; i++ {
		checkBoundaryOracle(t, w, Vec2{rng.Float64()*12000 - 6000, rng.Float64()*8000 - 4000}, float64(i%100), 52)
	}
	w.EndBoundaryQueries()
	// Equal-distance roads must retain the first road's pushback normal.
	w = &World{Roads: []RoadSegment{{P1: Vec2{-2000, -100}, P2: Vec2{2000, -100}, Lanes: 2}, {P1: Vec2{-2000, 100}, P2: Vec2{2000, 100}, Lanes: 2}, {P1: Vec2{-100, -2000}, P2: Vec2{-100, 2000}, Lanes: 2}, {P1: Vec2{100, -2000}, P2: Vec2{100, 2000}, Lanes: 2}}}
	for i := 0; i < 12; i++ {
		w.Roads = append(w.Roads, RoadSegment{P1: Vec2{4000 + float64(i)*200, -2000}, P2: Vec2{4000 + float64(i)*200, 2000}, Lanes: 2})
	}
	w.BeginBoundaryQueries(20)
	defer w.EndBoundaryQueries()
	for _, p := range []Vec2{{0, 0}, {2000, 2000}, {-2000, -2000}, {4000, 0}} {
		checkBoundaryOracle(t, w, p, 1, 20)
	}
}

func TestBoundaryIndexOverlapsPreserveFirstBarrier(t *testing.T) {
	w := &World{}
	for i := 0; i < 20; i++ {
		w.Buildings = append(w.Buildings, Building{Pos: Vec2{float64(i) * 10, 0}, W: 500, H: 1000})
		w.Stops = append(w.Stops, &BusStop{Pos: Vec2{float64(i), 0}, Heading: float64(i) * .15, Width: 44, Length: 336, RoadLanes: 8})
	}
	w.BeginBoundaryQueries(52)
	for _, p := range []Vec2{{80, 0}, {-5, 36}, {250, 300}, {-1200, 24}} {
		checkBoundaryOracle(t, w, p, 16, 52)
	}
	w.EndBoundaryQueries()
	w.Buildings = nil
	w.BeginBoundaryQueries(52)
	defer w.EndBoundaryQueries()
	for x := -1200.; x < 600; x += 7 {
		for _, y := range []float64{24, 36, 48, -100} {
			checkBoundaryOracle(t, w, Vec2{x, y}, .5, 52)
		}
	}
}

func TestBoundaryIndexBatchInvalidationAndSave(t *testing.T) {
	w := NewWorld(52)
	raw, _ := json.Marshal(w)
	w.BeginBoundaryQueries(52)
	old := w.boundaryCache
	w.BeginBoundaryQueries(78)
	checkBoundaryOracle(t, w, w.Stops[0].Pos, 16, 78)
	w.EndBoundaryQueries()
	w.EndBoundaryQueries()
	if w.boundaryDepth != 0 {
		t.Fatal("unbalanced batch")
	}
	encoded, _ := json.Marshal(w)
	if string(raw) != string(encoded) {
		t.Fatal("cache leaked into save")
	}
	w.Stops[0].WaitingPassengers++
	w.BeginBoundaryQueries(52)
	if w.boundaryCache != old {
		t.Fatal("passengers rebuilt geometry")
	}
	w.EndBoundaryQueries()
	mutations := []func(){func() { w.Buildings[0].Pos.X += 200 }, func() { w.Roads[0].P2.X += 100 }, func() { w.Stops[0].Pos.Y += 100 }, func() { w.Stops[0].Heading += .17 }, func() { w.Stops[0].Length += 100 }, func() { w.Stops[0].Width += 7 }, func() { w.Stops[0].RoadLanes += 2 }, func() { w.Stops = append(w.Stops, &BusStop{Pos: Vec2{123, 456}, Length: 336, Width: 44, RoadLanes: 8}) }}
	for _, mutate := range mutations {
		old = w.boundaryCache
		mutate()
		// Standalone queries must ignore the retained old cache.
		checkBoundaryOracle(t, w, w.Stops[0].Pos, 16, 52)
		w.BeginBoundaryQueries(52)
		if w.boundaryCache == old {
			t.Fatal("missed in-place map edit")
		}
		checkBoundaryOracle(t, w, w.Stops[0].Pos, 16, 52)
		w.EndBoundaryQueries()
	}
	w.BeginBoundaryQueries(78)
	checkBoundaryOracle(t, w, w.Stops[0].Pos, 16, 78)
	w.EndBoundaryQueries()
	var loaded World
	encoded, _ = json.Marshal(w)
	if err := json.Unmarshal(encoded, &loaded); err != nil {
		t.Fatal(err)
	}
	if loaded.boundaryCache != nil || loaded.boundaryDepth != 0 {
		t.Fatal("restored transient index")
	}
	loaded.BeginBoundaryQueries(78)
	checkBoundaryOracle(t, &loaded, loaded.Stops[0].Pos, 16, 78)
	loaded.EndBoundaryQueries()
	// Explicit geometry changes during a batch disable stale predictions.
	w.Stops[0].Length = 140
	w.BeginBoundaryQueries(52)
	EnsureLongBusBays(w, &Route{StopIDs: []int{0}})
	if w.boundaryCache != nil {
		t.Fatal("bay extension retained stale index")
	}
	checkBoundaryOracle(t, w, w.Stops[0].Pos, 16, 52)
	w.EndBoundaryQueries()
}
