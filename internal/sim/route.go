package sim

import (
	"image/color"
	"math"
	"sync"
)

type routePathCacheKey struct {
	fromID int
	toID   int
	lwKey  int
}

var (
	routePathCacheMu sync.RWMutex
	routePathCache   = make(map[routePathCacheKey][]Vec2)
)

// ClearRoutePathCache clears all cached Dijkstra road paths between stops
func ClearRoutePathCache() {
	routePathCacheMu.Lock()
	routePathCache = make(map[routePathCacheKey][]Vec2)
	routePathCacheMu.Unlock()
}

// Route manages the planned sequence of bus stops
type Route struct {
	Name         string
	Color        color.RGBA
	StopIDs      []int // Sequence of stop IDs
	CurrentIndex int   // Index within StopIDs that bus is currently navigating towards
	TotalTrips   int   // Completed full loops

	cachedDesc      string
	cachedDescDirty bool
}

func NewRoute() *Route {
	return &Route{
		Name:            "Custom Route",
		Color:           RGBA(14, 165, 233, 255),
		cachedDescDirty: true,
	}
}

// NewDefaultRoute uses the same all-stop itinerary as the Metropolis map.
func NewDefaultRoute() *Route {
	return newStandardRoute(MapThemeMetropolis, 12)
}

func (r *Route) AddStop(stopID int) {
	// Avoid immediate consecutive duplicates
	if len(r.StopIDs) > 0 && r.StopIDs[len(r.StopIDs)-1] == stopID {
		return
	}
	r.StopIDs = append(r.StopIDs, stopID)
	r.cachedDescDirty = true
}

func (r *Route) Clear() {
	r.StopIDs = make([]int, 0)
	r.CurrentIndex = 0
	r.cachedDescDirty = true
}

// RemoveLastStop removes the last station from the route.
// Returns the removed stop ID and true, or -1 and false if route is already empty.
func (r *Route) RemoveLastStop() (int, bool) {
	if len(r.StopIDs) == 0 {
		return -1, false
	}
	lastIdx := len(r.StopIDs) - 1
	removedID := r.StopIDs[lastIdx]
	r.StopIDs = r.StopIDs[:lastIdx]
	if r.CurrentIndex >= len(r.StopIDs) {
		if len(r.StopIDs) > 0 {
			r.CurrentIndex = len(r.StopIDs) - 1
		} else {
			r.CurrentIndex = 0
		}
	}
	r.cachedDescDirty = true
	return removedID, true
}

func (r *Route) GetCurrentTargetStopID() int {
	if len(r.StopIDs) == 0 {
		return -1
	}
	if r.CurrentIndex >= len(r.StopIDs) {
		r.CurrentIndex = 0
	}
	return r.StopIDs[r.CurrentIndex]
}

func (r *Route) AdvanceToNextStop() {
	if len(r.StopIDs) == 0 {
		return
	}
	prevStopID := r.StopIDs[r.CurrentIndex%len(r.StopIDs)]
	r.CurrentIndex++
	if r.CurrentIndex >= len(r.StopIDs) {
		r.CurrentIndex = 0
		r.TotalTrips++
		// Closed loops repeat the terminal as first and last stop ([0, a, b, 0]):
		// having just served the terminal, continue straight on to the next distinct stop.
		if len(r.StopIDs) > 1 && r.StopIDs[0] == prevStopID {
			r.CurrentIndex = 1
		}
	}
	r.cachedDescDirty = true
}

// IsTerminusIndex checks if the given stop sequence index is the terminal of the route.
func (r *Route) IsTerminusIndex(index int) bool {
	n := len(r.StopIDs)
	if n == 0 {
		return false
	}
	return index == n-1
}

// IsTerminus checks if the given stopID is the terminal stop of the route.
func (r *Route) IsTerminus(stopID int) bool {
	n := len(r.StopIDs)
	if n == 0 {
		return false
	}
	return r.StopIDs[n-1] == stopID
}

// ContainsStop returns true if stopID is included in the route's stop sequence.
func (r *Route) ContainsStop(stopID int) bool {
	if r == nil {
		return false
	}
	for _, id := range r.StopIDs {
		if id == stopID {
			return true
		}
	}
	return false
}

// GetUpcomingCandidateStopIDs returns valid destination stop IDs for passengers
// boarding at the specified index within the route's StopIDs sequence.
// Ensures passengers only choose downstream stops on the current run and never
// pick their boarding station or ride a full loop back to the same terminus.
func (r *Route) GetUpcomingCandidateStopIDs(stopIndex int) []int {
	n := len(r.StopIDs)
	if n <= 1 {
		return nil
	}
	if stopIndex < 0 || stopIndex >= n {
		stopIndex = stopIndex % n
		if stopIndex < 0 {
			stopIndex += n
		}
	}
	currentStopID := r.StopIDs[stopIndex]
	isClosedLoop := (n >= 2 && r.StopIDs[0] == r.StopIDs[n-1])

	var candidateSlice []int
	if isClosedLoop {
		if stopIndex == 0 || stopIndex == n-1 {
			// Boarding at terminal of closed loop: upcoming stops on the loop are intermediate stops (indices 1 to n-2).
			// This specifically prevents passengers boarding at the terminal from riding an entire loop back to the terminal.
			if n > 2 {
				candidateSlice = r.StopIDs[1 : n-1]
			} else {
				candidateSlice = []int{r.StopIDs[1]}
			}
		} else {
			// Boarding at intermediate stop: upcoming stops up to the terminus (indices stopIndex+1 to n-1).
			candidateSlice = r.StopIDs[stopIndex+1 : n]
		}
	} else {
		// Open route
		if stopIndex < n-1 {
			candidateSlice = r.StopIDs[stopIndex+1 : n]
		} else {
			candidateSlice = r.StopIDs[0 : n-1]
		}
	}

	// Deduplicate and ensure currentStopID is never included
	seen := make(map[int]bool)
	var result []int
	for _, id := range candidateSlice {
		if id != currentStopID && !seen[id] {
			seen[id] = true
			result = append(result, id)
		}
	}

	// Fallback safety: if result is empty, find any other stop in route that is not currentStopID
	if len(result) == 0 {
		for _, id := range r.StopIDs {
			if id != currentStopID && !seen[id] {
				seen[id] = true
				result = append(result, id)
			}
		}
	}
	return result
}

// GetUpcomingCandidateStopIDsForStop returns candidate destination stop IDs
// for passengers boarding at a specific bus stop ID.
func (r *Route) GetUpcomingCandidateStopIDsForStop(currentStopID int) []int {
	n := len(r.StopIDs)
	if n == 0 {
		return nil
	}
	// Prefer current index if it matches the stop
	if r.CurrentIndex < n && r.StopIDs[r.CurrentIndex] == currentStopID {
		return r.GetUpcomingCandidateStopIDs(r.CurrentIndex)
	}
	// Otherwise look forward from CurrentIndex
	for i := 0; i < n; i++ {
		idx := (r.CurrentIndex + i) % n
		if r.StopIDs[idx] == currentStopID {
			return r.GetUpcomingCandidateStopIDs(idx)
		}
	}
	return r.GetUpcomingCandidateStopIDs(r.CurrentIndex)
}

// GetRoadPathBetweenStops calculates an orthogonal path strictly along the driving lane on the proper side of each road,
// ensuring the line is drawn on the right-hand side of the road rather than on the center divider median.
// Results are cached for instant O(1) lookup during real-time rendering.
func GetRoadPathBetweenStops(s1, s2 *BusStop, laneWidth ...float64) []Vec2 {
	return GetRoadPathBetweenStopsInWorld(ActiveWorldForRoute, s1, s2, laneWidth...)
}

// GetRoadPathBetweenStopsInWorld calculates an orthogonal path between stops within the specified world
func GetRoadPathBetweenStopsInWorld(w *World, s1, s2 *BusStop, laneWidth ...float64) []Vec2 {
	if s1 == nil || s2 == nil {
		return nil
	}
	if s1.ID == s2.ID {
		return []Vec2{s1.Pos, s2.Pos}
	}
	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	key := routePathCacheKey{
		fromID: s1.ID,
		toID:   s2.ID,
		lwKey:  int(math.Round(lw * 10)),
	}

	routePathCacheMu.RLock()
	if cached, ok := routePathCache[key]; ok {
		routePathCacheMu.RUnlock()
		return cached
	}
	routePathCacheMu.RUnlock()

	res := ComputeRoadPathBetweenStops(w, s1, s2, lw)

	routePathCacheMu.Lock()
	routePathCache[key] = res
	routePathCacheMu.Unlock()

	return res
}

// InvalidateDescription marks the cached route label for rebuilding after edits.
func (r *Route) InvalidateDescription() { r.cachedDescDirty = true }
