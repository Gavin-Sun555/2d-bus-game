package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) ObserveSignalDemand(tls *sim.TrafficLightSystem, world *sim.World, laneWidth float64, density int) {
	tls.PrepareJunctionIndex(laneWidth)
	ids, nearby := tm.nearbyJunctionIDs[:0], tm.nearbyJunctions[:0]
	for _, car := range tm.GetActiveCars(density) {
		if car.InRoundabout {
			continue
		}
		radius := tls.CarJunctionQueryRadius(laneWidth, car.Speed, car.Length, .2) + math.Abs(car.Speed)*1.2
		ids, nearby = tls.NearbyJunctions(car.Pos, radius, ids, nearby)
		for _, inter := range nearby {
			if turn := car.JunctionTurn; turn != nil && !turn.LaneChange && turn.Center == inter.Center {
				if inter.IncomingSignalApproach(car.Pos, turn.EntryHeading, car.Length, car.Speed, laneWidth) {
					inter.ObserveCarSignalDemand(turn.EntryHeading, sim.NormalizeAngle(turn.ExitHeading-turn.EntryHeading) < -.35)
				}
				continue
			}
			if !inter.IncomingSignalApproach(car.Pos, car.Heading, car.Length, car.Speed, laneWidth) {
				continue
			}
			left, _ := carIntersectionTurns(carFrame{car: car, world: world, laneWidth: laneWidth}, inter)
			inter.ObserveCarSignalDemand(car.Heading, left)
		}
	}
	tm.nearbyJunctionIDs, tm.nearbyJunctions = ids, nearby
}
