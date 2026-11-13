package traffic

import (
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Sites already account for usable lane length. Favor each general-traffic
// site on a wider road to keep more civilian traffic on the main network.
var trafficRoadWeights = [3]int{1, 3, 4}

func trafficRoadClass(road sim.RoadSegment) int {
	lanes := road.LaneCount()
	switch {
	case lanes >= 8:
		return 2
	case lanes >= 6:
		return 1
	default:
		return 0
	}
}

func preferTrafficRoad(road sim.RoadSegment) bool {
	return sim.RandomIntn(trafficRoadWeights[2]) < trafficRoadWeights[trafficRoadClass(road)]
}

type trafficSpawnPool [3][]trafficSpawnSite

func newTrafficSpawnPool(w *sim.World, laneWidth float64) *trafficSpawnPool {
	pool := new(trafficSpawnPool)
	for _, site := range trafficSpawnSites(w, laneWidth) {
		pool[site.roadClass] = append(pool[site.roadClass], site)
	}
	return pool
}

func (pool *trafficSpawnPool) counts() [3]int {
	return [3]int{len(pool[0]), len(pool[1]), len(pool[2])}
}

// The caller supplies a nonempty pool. Each site occupies its road class's
// weight in the draw.
func drawTrafficSite(counts [3]int) (class, index int) {
	total := 0
	for class, count := range counts {
		total += count * trafficRoadWeights[class]
	}
	draw := sim.RandomIntn(total)
	for class, count := range counts {
		weight := trafficRoadWeights[class]
		if draw < count*weight {
			return class, draw / weight
		}
		draw -= count * weight
	}
	panic("traffic spawn draw out of range")
}

func (pool *trafficSpawnPool) clone() *trafficSpawnPool {
	out := new(trafficSpawnPool)
	for class := range pool {
		out[class] = slices.Clone(pool[class])
	}
	return out
}

// Initial dispatch uses each physical site at most once. The full pool stays
// available for later replacements when traffic has moved away.
func (pool *trafficSpawnPool) take(remaining *[3]int) trafficSpawnSite {
	class, index := drawTrafficSite(*remaining)
	site := pool[class][index]
	remaining[class]--
	last := remaining[class]
	pool[class][index], pool[class][last] = pool[class][last], site
	return site
}
