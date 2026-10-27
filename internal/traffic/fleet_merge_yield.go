package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// Publish the yield as well as braking for it, so the merging controller sees
// the granted slot instead of demanding its normal 70px overtaking rear buffer.
func (abm *AIBusManager) updateMergeYield(bus, player *sim.Bus, laneWidth float64) {
	if bus == nil {
		return
	}
	previous := bus.YieldingToBus
	bus.YieldingToBus = nil
	best := math.Inf(1)
	consider := func(other *sim.Bus) {
		if limit, ok := sim.BusMergeYieldLimit(bus, other, laneWidth, previous == other); ok && limit < best {
			bus.YieldingToBus = other
			best = limit
		}
	}
	consider(player)
	for _, other := range abm.Buses {
		if other != nil {
			consider(other.Bus)
		}
	}
}

// UpdatePlayerMergeYield publishes the same safe rear slot for the player's
// automatic driver that fleet buses publish for one another.
func (abm *AIBusManager) UpdatePlayerMergeYield(player *sim.Bus, laneWidth float64) {
	abm.updateMergeYield(player, nil, laneWidth)
}
