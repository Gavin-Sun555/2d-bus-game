package autopilot

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

// Scoped to one ComputeControls call. Fleet updates are sequential: retaining
// occupancy into the next bus/tick would change admission and priority rules.
// The key also rejects a different source pose, stop or obstacle view inside
// the decision. Returned berth slices are read-only with no append capacity.
type stationDecisionCache struct {
	bus                                                        *sim.Bus
	stop                                                       *sim.BusStop
	first                                                      *VehicleObstacle
	obstacleCount                                              int
	pos                                                        sim.Vec2
	heading, tail, length, width                               float64
	model                                                      sim.BusModel
	trailerInitialized                                         bool
	capacityReady, capacity, queueReady, occupied, berthsReady bool
	queue                                                      sim.Vec2
	berths                                                     [2]sim.Vec2
	count                                                      int
}

func (ad *AutopilotDriver) stationQuery(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) *stationDecisionCache {
	if !ad.stationQueriesActive {
		return nil
	}
	var first *VehicleObstacle
	if len(obstacles) > 0 {
		first = &obstacles[0]
	}
	c := &ad.stationQueries
	if c.bus != bus || c.stop != stop || c.first != first || c.obstacleCount != len(obstacles) || c.pos != bus.Pos || c.heading != bus.Heading || c.tail != bus.TrailerHeading || c.length != bus.Length || c.width != bus.Width || c.model != bus.Model || c.trailerInitialized != bus.TrailerInitialized {
		*c = stationDecisionCache{bus: bus, stop: stop, first: first, obstacleCount: len(obstacles), pos: bus.Pos, heading: bus.Heading, tail: bus.TrailerHeading, length: bus.Length, width: bus.Width, model: bus.Model, trailerInitialized: bus.TrailerInitialized}
	}
	return c
}

func (ad *AutopilotDriver) stationBoardingCapacity(bus *sim.Bus, stop *sim.BusStop, obs []VehicleObstacle) bool {
	c := ad.stationQuery(bus, stop, obs)
	if c == nil {
		return stationBoardingCapacity(bus, stop, obs)
	}
	if !c.capacityReady {
		c.capacity = stationBoardingCapacity(bus, stop, obs)
		c.capacityReady = true
	}
	return c.capacity
}

func (ad *AutopilotDriver) stationQueueTarget(bus *sim.Bus, stop *sim.BusStop, obs []VehicleObstacle) (sim.Vec2, bool) {
	c := ad.stationQuery(bus, stop, obs)
	if c == nil {
		return stationQueueTarget(bus, stop, obs)
	}
	if !c.queueReady {
		c.queue, c.occupied = stationQueueTarget(bus, stop, obs)
		c.queueReady = true
	}
	return c.queue, c.occupied
}

func (ad *AutopilotDriver) availableStationBerths(bus *sim.Bus, stop *sim.BusStop, obs []VehicleObstacle) []sim.Vec2 {
	c := ad.stationQuery(bus, stop, obs)
	if c == nil {
		return availableStationBerths(bus, stop, obs)
	}
	if !c.berthsReady {
		if ad.stationBoardingCapacity(bus, stop, obs) {
			var queue sim.Vec2
			var occupied bool
			if stop.HasTwoBerths() {
				queue, occupied = ad.stationQueueTarget(bus, stop, obs)
			}
			c.count = len(availableStationBerthsInto(bus, stop, obs, c.berths[:0], queue, occupied))
		}
		c.berthsReady = true
	}
	if c.count == 0 {
		return nil
	}
	return c.berths[:c.count:c.count]
}
