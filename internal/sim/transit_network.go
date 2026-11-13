package sim

import "fmt"

const MaxBusesPerLine = 12
const MaxRouteUTurns = 2

// RouteUTurnCount includes the return to the first stop. Count each leg's
// larger total across its platform approach and its road-only bypass.
func RouteUTurnCount(w *World, stops []int, lw float64) int {
	if w == nil || len(stops) < 2 {
		return 0
	}
	total := 0
	for i, from := range stops {
		to := stops[(i+1)%len(stops)]
		if from == to || from < 0 || from >= len(w.Stops) || to < 0 || to >= len(w.Stops) {
			continue
		}
		platform, bypass := 0, 0
		computeRoadPathBetweenStops(w, w.Stops[from], w.Stops[to].ForwardDockStop(), lw, &platform)
		computeRoadPathBetweenStops(w, w.Stops[from], w.Stops[to], lw, &bypass)
		total += max(platform, bypass)
	}
	return total
}

// DefaultTransitLines contains six independent editable itineraries. Four local
// lines cover the 48 platforms; lines five and six offer overlapping cross-city service.
func DefaultTransitLines(w *World) []*Route {
	if w == nil {
		return nil
	}
	circuits, builtIn := secondaryTransitCircuits[w.Theme]
	if len(w.Stops) < 48 || !builtIn {
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
	lines := []*Route{newStandardRoute(w.Theme, 12)}
	for _, circuit := range circuits {
		line := NewRoute()
		line.StopIDs = append(line.StopIDs, circuit[:]...)
		line.StopIDs = append(line.StopIDs, circuit[0])
		lines = append(lines, line)
	}
	colors := [][3]uint8{{16, 185, 129}, {59, 130, 246}, {245, 158, 11}, {168, 85, 247}, {236, 72, 153}, {6, 182, 212}}
	for i, r := range lines {
		r.Name = fmt.Sprintf("Line %d", i+1)
		c := colors[i]
		r.Color = RGBA(c[0], c[1], c[2], 255)
	}
	return lines
}

// These station orders account for approach headings on both grid sizes and
// the curb and center BRT variants. Each circuit has at most two U-turns,
// including the closing leg. The first three cover the remaining 36 platforms.
var secondaryTransitCircuits = map[MapTheme][5][12]int{
	MapThemeMetropolis: {
		{13, 30, 39, 33, 34, 43, 17, 37, 21, 42, 12, 19},
		{25, 44, 45, 28, 20, 29, 23, 47, 26, 18, 27, 46},
		{22, 16, 36, 32, 24, 40, 35, 41, 15, 38, 31, 14},
		{0, 4, 32, 16, 36, 40, 24, 12, 28, 20, 44, 8},
		{1, 5, 17, 21, 25, 13, 33, 37, 9, 45, 29, 41},
	},
	MapThemeCoastalBay: {
		{13, 37, 33, 47, 12, 36, 25, 16, 39, 30, 21, 35},
		{26, 45, 32, 22, 44, 31, 17, 23, 29, 18, 46, 34},
		{38, 24, 14, 42, 28, 20, 43, 15, 19, 40, 27, 41},
		{0, 4, 40, 20, 24, 28, 32, 8, 16, 44, 12, 36},
		{1, 9, 13, 37, 33, 25, 17, 21, 45, 29, 41, 5},
	},
	MapThemeRiverValley: {
		{14, 22, 18, 26, 29, 17, 44, 12, 33, 36, 38, 24},
		{21, 46, 35, 41, 30, 42, 31, 20, 40, 45, 34, 47},
		{27, 19, 23, 15, 25, 39, 37, 16, 28, 43, 32, 13},
		{0, 8, 4, 20, 32, 16, 28, 40, 36, 24, 44, 12},
		{1, 41, 9, 5, 17, 45, 29, 33, 21, 25, 37, 13},
	},
	MapThemeTwinCities: {
		{14, 32, 20, 34, 42, 24, 46, 18, 43, 12, 33, 30},
		{16, 22, 28, 36, 29, 40, 39, 41, 23, 17, 35, 26},
		{44, 27, 31, 38, 13, 47, 19, 25, 37, 15, 21, 45},
		{0, 4, 40, 44, 24, 16, 12, 32, 20, 28, 36, 8},
		{1, 33, 9, 29, 21, 45, 37, 13, 41, 17, 25, 5},
	},
}
