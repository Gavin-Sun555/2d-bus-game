package sim

import (
	"fmt"
	"math"
	"sort"
)

// DefaultTransitLines contains six independent editable itineraries. Four local
// lines cover the 48 platforms; lines five and six offer overlapping cross-city service.
func DefaultTransitLines(w *World) []*Route {
	if w == nil {
		return nil
	}
	if len(w.Stops) < 48 || w.Theme == MapThemeCustom {
		r := newStandardRoute(w.Theme, len(w.Stops))
		if w.Theme == MapThemeCustom {
			r.StopIDs = customMapStopOrder(w)
		}
		lines := []*Route{r}
		for i := 1; i < 6; i++ {
			q := NewRoute()
			q.StopIDs = append([]int(nil), r.StopIDs...)
			q.Name = fmt.Sprintf("Line %d", i+1)
			lines = append(lines, q)
		}
		return lines
	}
	lines := make([]*Route, 0, 6)
	core := newStandardRoute(w.Theme, 12)
	lines = append(lines, core)
	remaining := make([]int, 0, 36)
	for i := 12; i < 48; i++ {
		remaining = append(remaining, i)
	}
	// Spatial bands avoid arbitrary ID-order zigzags through distant neighborhoods.
	sort.SliceStable(remaining, func(i, j int) bool {
		a, b := w.Stops[remaining[i]].RoadCenter, w.Stops[remaining[j]].RoadCenter
		if a.X != b.X {
			return a.X < b.X
		}
		return a.Y < b.Y
	})
	groups := [][]int{remaining[:12], remaining[12:24], remaining[24:]}
	for _, g := range groups {
		lines = append(lines, localLine(w, g))
	}
	for k := 0; k < 2; k++ {
		ids := []int{}
		for i := k; i < 48; i += 4 {
			ids = append(ids, i)
		}
		lines = append(lines, localLine(w, ids))
	}
	colors := [][3]uint8{{16, 185, 129}, {59, 130, 246}, {245, 158, 11}, {168, 85, 247}, {236, 72, 153}, {6, 182, 212}}
	for i, r := range lines {
		r.Name = fmt.Sprintf("Line %d", i+1)
		c := colors[i]
		r.Color = RGBA(c[0], c[1], c[2], 255)
	}
	return lines
}
func localLine(w *World, ids []int) *Route {
	r := NewRoute()
	left := append([]int(nil), ids...)
	if len(left) == 0 {
		return r
	}
	current := left[0]
	r.StopIDs = append(r.StopIDs, current)
	left = left[1:]
	for len(left) > 0 {
		best, d := 0, math.Inf(1)
		for i, id := range left {
			a, b := w.Stops[current], w.Stops[id]
			distance := a.RoadCenter.Distance(b.RoadCenter)
			if distance < d {
				best, d = i, distance
			}
		}
		current = left[best]
		r.StopIDs = append(r.StopIDs, current)
		left = append(left[:best], left[best+1:]...)
	}
	r.StopIDs = append(r.StopIDs, r.StopIDs[0])
	return r
}
