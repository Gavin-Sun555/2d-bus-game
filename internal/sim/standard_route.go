package sim

// GetStandardRouteForMap returns the single primary continuous loop bus route for the active city map
func GetStandardRouteForMap(theme MapTheme, world *World) *Route {
	route := &Route{
		Name:            "Line 1 (城市主干环线)",
		Color:           RGBA(16, 185, 129, 255), // Emerald green
		CurrentIndex:    0,
		cachedDescDirty: true,
	}

	numStops := len(world.Stops)
	if numStops == 0 {
		return route
	}

	switch theme {
	case MapThemeMetropolis:
		// Central Terminal (0) -> Civic Center (2) -> Creative District (10) -> City Hospital (8) -> 0
		route.StopIDs = []int{0, 2, 10, 8, 0}
	case MapThemeCoastalBay:
		// Coastal Plaza (0) -> Ocean Pier (1) -> Golden Beach (3) -> Marina (5) -> 0
		route.StopIDs = []int{0, 1, 3, 5, 0}
	case MapThemeRiverValley:
		// Valley Hub (0) -> Riverside Ave (3) -> Suspension Bridge (6) -> High-Tech (9) -> 0
		route.StopIDs = []int{0, 3, 6, 9, 0}
	case MapThemeEmeraldLake:
		// Lake Hub (0) -> East Causeway (9) -> Ancient Town (3) -> West Wetlands (6) -> 0
		route.StopIDs = []int{0, 9, 3, 6, 0}
	case MapThemeTwinCities:
		// Logistics Hub (0) -> Smart City Center (4) -> Interchange East (7) -> West Industrial (2) -> 0
		route.StopIDs = []int{0, 4, 7, 2, 0}
	case MapThemeCustom:
		if numStops >= 5 {
			route.StopIDs = []int{0, 1, 3, 4, 0}
		} else if numStops >= 3 {
			route.StopIDs = []int{0, 1, 2, 0}
		} else if numStops >= 2 {
			route.StopIDs = []int{0, 1, 0}
		} else {
			route.StopIDs = []int{0}
		}
	}
	return route
}
