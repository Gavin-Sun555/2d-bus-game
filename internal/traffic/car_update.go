package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) StepCar(dt float64, car *sim.TrafficCar, index int, bus *sim.Bus, tls *sim.TrafficLightSystem, activeCount int, laneWidth float64, w *sim.World) {
	// 1. Perception & Sensors
	if car.JunctionCorridor != nil {
		car.JunctionCorridor.Remaining = 0
	}
	targetSpd := car.TargetSpeed
	car.IsBraking = false

	if car.RoundaboutCooldown > 0 {
		car.RoundaboutCooldown -= dt
	}

	// The supported fleet has eight buses plus the player; keep the
	// per-car observation list on the stack instead of growing a heap slice.
	var busStorage [9]*sim.Bus
	busesToCheck := busStorage[:0]
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

	frame := carFrame{car: car, bus: bus, tls: tls, world: w, index: index, activeCount: activeCount, laneWidth: laneWidth, buses: busesToCheck}
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
}

// carFrame shares the tick's observations across perception rules. Rules run in
// signal, bus, car, intersection and roundabout order and may apply emergency
// speed corrections before motion is integrated.
type carFrame struct {
	car                *sim.TrafficCar
	bus                *sim.Bus
	tls                *sim.TrafficLightSystem
	world              *sim.World
	buses              []*sim.Bus
	index, activeCount int
	laneWidth          float64
}
