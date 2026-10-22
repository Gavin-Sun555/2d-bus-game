package sim

import "math"

// GetStandardRouteForMap returns an all-stop loop starting and ending at stop 0.
// Built-in itineraries follow the road network and each platform's travel direction.
// Custom maps use directed road distances and include every station.
func GetStandardRouteForMap(theme MapTheme, world *World) *Route {
	if world == nil {
		return newStandardRoute(theme, 0)
	}
	route := newStandardRoute(theme, len(world.Stops))
	if theme == MapThemeCustom {
		route.StopIDs = customMapStopOrder(world)
	}
	return route
}

func newStandardRoute(theme MapTheme, numStops int) *Route {
	route := &Route{Name: "Line 1 (城市全站环线)", Color: RGBA(16, 185, 129, 255), cachedDescDirty: true}
	if numStops == 0 {
		return route
	}
	// These twelve-stop circuits are planned together with maps_stops.go. A new
	// map layout falls back to visiting every station rather than silently omitting it.
	if numStops == 12 {
		switch theme {
		case MapThemeMetropolis:
			// Center -> northeast -> southeast -> north railway -> west -> center.
			route.StopIDs = []int{0, 1, 2, 4, 3, 11, 10, 5, 6, 7, 9, 8, 0}
		case MapThemeCoastalBay:
			// Fishing harbor -> north beach -> offshore port -> south cape -> harbor.
			route.StopIDs = []int{0, 9, 8, 2, 3, 5, 6, 11, 4, 1, 7, 10, 0}
		case MapThemeRiverValley:
			// North bank -> south bank -> western bridge -> rail district -> eastern bridge.
			route.StopIDs = []int{0, 8, 9, 7, 5, 2, 3, 4, 11, 10, 6, 1, 0}
		case MapThemeEmeraldLake:
			// Island -> east bank -> north causeway -> south causeway -> west bank.
			route.StopIDs = []int{0, 11, 3, 9, 2, 1, 4, 5, 10, 6, 7, 8, 0}
		case MapThemeTwinCities:
			// West industry -> expressway -> east technology district -> airport -> hub.
			route.StopIDs = []int{0, 9, 2, 3, 4, 7, 11, 6, 5, 10, 8, 1, 0}
		}
	}
	if len(route.StopIDs) == 0 {
		route.StopIDs = make([]int, numStops, numStops+1)
		for i := range route.StopIDs {
			route.StopIDs[i] = i
		}
		if numStops > 1 {
			route.StopIDs = append(route.StopIDs, 0)
		}
	}
	return route
}

// customMapStopOrder uses directed road distances, including platform approach
// headings. Cheapest insertion keeps nearby stops together without moving any
// stations authored in a custom map. It runs only when loading that map.
func customMapStopOrder(world *World) []int {
	n := len(world.Stops)
	if n < 2 {
		if n == 1 {
			return []int{0}
		}
		return nil
	}
	// Infer the current lane width from the platform geometry built by World.
	first := world.Stops[0]
	laneWidth := (first.Pos.Distance(first.RoadCenter) - first.Width*.48) * 2 / float64(first.RoadLanes)
	if laneWidth <= 0 || math.IsNaN(laneWidth) || math.IsInf(laneWidth, 0) {
		laneWidth = 42
	}
	cost := make([][]float64, n)
	for i := range cost {
		cost[i] = make([]float64, n)
		for j := range cost[i] {
			if i == j {
				continue
			}
			path := ComputeRoadPathBetweenStops(world, world.Stops[i], world.Stops[j], laneWidth)
			for k := 1; k < len(path); k++ {
				cost[i][j] += path[k].Distance(path[k-1])
			}
		}
	}
	order := []int{0, 0}
	used := make([]bool, n)
	used[0] = true
	for len(order) <= n {
		bestStop, bestAt := -1, 0
		bestCost := math.Inf(1)
		for stop := 1; stop < n; stop++ {
			if used[stop] {
				continue
			}
			for at := 1; at < len(order); at++ {
				from, to := order[at-1], order[at]
				added := cost[from][stop] + cost[stop][to] - cost[from][to]
				if added < bestCost {
					bestCost, bestStop, bestAt = added, stop, at
				}
			}
		}
		order = append(order, 0)
		copy(order[bestAt+1:], order[bestAt:])
		order[bestAt] = bestStop
		used[bestStop] = true
	}
	return order
}
