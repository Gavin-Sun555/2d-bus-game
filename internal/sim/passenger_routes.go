package sim

import "slices"

type passengerRouteKey struct {
	stops        []int
	currentIndex int
	absent       bool
}

type passengerRouteCache struct {
	keys    []passengerRouteKey
	service map[int][]int
}

// Route editors can replace StopIDs in place. Keep a copy, and include the
// current occurrence because a loop can visit the same stop more than once.
func (c *passengerRouteCache) forRoutes(routes []*Route) map[int][]int {
	if c.matches(routes) {
		return c.service
	}
	c.keys = make([]passengerRouteKey, len(routes))
	serving := make(map[int][]*Route)
	for i, route := range routes {
		if route == nil {
			c.keys[i].absent = true
			continue
		}
		c.keys[i] = passengerRouteKey{stops: slices.Clone(route.StopIDs), currentIndex: route.CurrentIndex}
		seen := make(map[int]bool, len(route.StopIDs))
		for _, id := range route.StopIDs {
			if !seen[id] {
				seen[id] = true
				serving[id] = append(serving[id], route)
			}
		}
	}
	c.service = make(map[int][]int, len(serving))
	for id, lines := range serving {
		var ids []int
		members := make(map[int]bool)
		for _, line := range lines {
			ids = appendPassengerDestinations(ids, members, line.GetUpcomingCandidateStopIDsForStop(id), id)
		}
		if len(ids) == 0 {
			for _, line := range lines {
				ids = appendPassengerDestinations(ids, members, line.StopIDs, id)
			}
		}
		c.service[id] = ids
	}
	return c.service
}

// ReconcileWaitingPassengers applies a committed network change. A passenger
// can wait for any visit by any line that can take them to their destination.
func (w *World) ReconcileWaitingPassengers(routes ...*Route) {
	reachable := make(map[int]map[int]bool)
	for _, route := range routes {
		if route == nil {
			continue
		}
		for index, stopID := range route.StopIDs {
			destinations := reachable[stopID]
			if destinations == nil {
				destinations = make(map[int]bool)
				reachable[stopID] = destinations
			}
			for _, dest := range route.GetUpcomingCandidateStopIDs(index) {
				destinations[dest] = true
			}
		}
	}
	for _, stop := range w.Stops {
		destinations, served := reachable[stop.ID]
		if !served {
			stop.clearWaitingQueue()
			continue
		}
		stop.SyncWaitingQueue(nil)
		kept := 0
		for i, dest := range stop.WaitingDestinations {
			if dest == -1 || destinations[dest] {
				stop.WaitingDestinations[kept] = dest
				stop.WaitingSince[kept] = stop.WaitingSince[i]
				kept++
			}
		}
		stop.WaitingDestinations = stop.WaitingDestinations[:kept]
		stop.WaitingSince = stop.WaitingSince[:kept]
		stop.WaitingPassengers = kept
	}
}

func (c *passengerRouteCache) matches(routes []*Route) bool {
	if len(c.keys) != len(routes) {
		return false
	}
	for i, route := range routes {
		key := c.keys[i]
		if route == nil {
			if !key.absent {
				return false
			}
		} else if key.absent || key.currentIndex != route.CurrentIndex || !slices.Equal(key.stops, route.StopIDs) {
			return false
		}
	}
	return true
}

func appendPassengerDestinations(ids []int, members map[int]bool, candidates []int, boardingStop int) []int {
	for _, id := range candidates {
		if id != boardingStop && !members[id] {
			members[id] = true
			ids = append(ids, id)
		}
	}
	return ids
}
