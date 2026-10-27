package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) StepCar(dt float64, car *sim.TrafficCar, index int, bus *sim.Bus, tls *sim.TrafficLightSystem, activeCount int, laneWidth float64, w *sim.World) {
	// 1. Perception & Sensors
	localizeRoundaboutCar(car)
	if car.JunctionCorridor != nil {
		car.JunctionCorridor.Remaining = 0
	}
	targetSpd := car.TargetSpeed
	car.IsBraking = false

	if car.RoundaboutCooldown > 0 {
		car.RoundaboutCooldown -= dt
	}

	// Fleet poses stay fixed while civilian cars advance sequentially.
	if !tm.busPerceptionActive {
		tm.prepareBusPerception(bus, w)
	}
	busesToCheck := tm.observedBuses

	frame := carFrame{car: car, bus: bus, tls: tls, world: w, index: index, activeCount: activeCount, laneWidth: laneWidth, buses: busesToCheck, parkedBuses: tm.observedParked[:len(busesToCheck)]}
	frame.scratch = tm.scratchForCar(car, index)
	tm.localCarsReady = false
	if tm.spatialActive {
		// Freeze the original broad-phase reach before rules can brake the car.
		// Only collect candidates if a rule actually consumes this observation.
		frame.localCarRadius = localCarQueryRadius(car)
	}
	if tls != nil && !tm.disableJunctionIndex {
		radius := tls.CarJunctionQueryRadius(laneWidth, car.Speed, car.Length, dt)
		// A committed turn keeps owning its original junction through its tail,
		// even for fixtures/custom maps whose signal and world extents differ.
		if turn := car.JunctionTurn; turn != nil && !turn.LaneChange {
			radius = math.Max(radius, car.Pos.Distance(turn.Center)+1)
		}
		tm.nearbyJunctionIDs, tm.nearbyJunctions = tls.NearbyJunctions(car.Pos, radius, tm.nearbyJunctionIDs, tm.nearbyJunctions)
		frame.junctions, frame.indexedJunctions = tm.nearbyJunctions, true
	}
	forcedLeftLimit := tm.prepareForcedLeftApproach(frame)
	if forcedLeftLimit < targetSpd {
		car.IsBraking = true
	}
	targetSpd = math.Min(targetSpd, forcedLeftLimit)
	if math.IsInf(forcedLeftLimit, 1) {
		tm.prepareEightLaneApproach(frame)
		targetSpd = math.Min(targetSpd, tm.prepareCarPassing(frame))
	}
	targetSpd = tm.signalSpeed(frame, targetSpd)
	targetSpd = tm.junctionQueueSpeed(frame, targetSpd)
	targetSpd = tm.busMergeYieldSpeed(frame, targetSpd)
	targetSpd = tm.busFollowingSpeed(frame, targetSpd)
	targetSpd = tm.carFollowingSpeed(frame, targetSpd)
	targetSpd = tm.intersectionYieldSpeed(frame, targetSpd)
	targetSpd = tm.roundaboutSpeed(frame, targetSpd)

	if car.JunctionTurn != nil && !car.JunctionTurn.LaneChange && !car.JunctionTurn.Straight {
		targetSpd = math.Min(targetSpd, 85)
	}

	if car.JunctionTurn != nil && car.JunctionTurn.SpeedLimit > 0 {
		targetSpd = math.Min(targetSpd, car.JunctionTurn.SpeedLimit)
	}

	// 2. Acceleration / Braking Physics
	if car.Speed < targetSpd {
		car.Speed = math.Min(targetSpd, car.Speed+120*dt)
	} else if car.Speed > targetSpd {
		car.Speed = math.Max(targetSpd, car.Speed-280*dt)
	}

	moveCar(car, dt)

	tm.turnCarAtIntersection(frame)

	// 4. Preserve lane tracking until the car leaves the map.
	tm.handleCarNavigation(car, laneWidth, w)
	publishCarJunctionCorridor(frame)
	publishRoundaboutCorridor(car)
}

// carFrame shares the tick's observations across perception rules. Rules run in
// signal, bus, car, intersection and roundabout order and may apply emergency
// speed corrections before motion is integrated.
type carFrame struct {
	scratch            *carJunctionScratch
	junctions          []*sim.TrafficIntersection
	indexedJunctions   bool
	car                *sim.TrafficCar
	bus                *sim.Bus
	tls                *sim.TrafficLightSystem
	world              *sim.World
	buses              []*sim.Bus
	parkedBuses        []bool
	candidates         []*sim.TrafficCar
	localCarRadius     float64
	index, activeCount int
	laneWidth          float64
}

func (f carFrame) nearbyJunctions() []*sim.TrafficIntersection {
	if f.indexedJunctions {
		return f.junctions
	}
	if f.tls == nil {
		return nil
	}
	return f.tls.Intersections
}

// Refresh once per population update, or for each standalone StepCar call.
// No geometry or observation buffer is retained by a live car's turn plan.
func (tm *TrafficManager) prepareBusPerception(bus *sim.Bus, w *sim.World) {
	busesToCheck := tm.busStorage[:0]
	if bus != nil {
		busesToCheck = append(busesToCheck, bus)
	}
	if tm.AIBusMgr != nil {
		for _, ab := range tm.AIBusMgr.Buses {
			if ab != nil && ab.Bus != nil && ab.Bus != bus {
				busesToCheck = append(busesToCheck, ab.Bus)
			}
		}
	}
	clear(tm.busStorage[len(busesToCheck):])
	clear(tm.observedParked[:])
	if w != nil {
		for i, b := range busesToCheck {
			for _, stop := range w.Stops {
				if stop.ContainsBus(b) {
					tm.observedParked[i] = true
					break
				}
			}
		}
	}
	tm.observedBuses = busesToCheck
}
