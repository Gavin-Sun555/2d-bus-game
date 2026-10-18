package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) StepCar(dt float64, car *sim.TrafficCar, index int, bus *sim.Bus, tls *sim.TrafficLightSystem, activeCount int, laneWidth float64, w *sim.World) {
	// 1. Perception & Sensors
	targetSpd := car.TargetSpeed
	car.IsBraking = false

	if car.RoundaboutCooldown > 0 {
		car.RoundaboutCooldown -= dt
	}

	var busesToCheck []*sim.Bus
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
	targetSpd = tm.signalSpeed(frame, targetSpd)
	targetSpd = tm.busFollowingSpeed(frame, targetSpd)
	targetSpd = tm.carFollowingSpeed(frame, targetSpd)
	targetSpd = tm.intersectionYieldSpeed(frame, targetSpd)
	targetSpd = tm.roundaboutSpeed(frame, targetSpd)

	// 2. Acceleration / Braking Physics
	if car.Speed < targetSpd {
		car.Speed = math.Min(targetSpd, car.Speed+120*dt)
	} else if car.Speed > targetSpd {
		car.Speed = math.Max(targetSpd, car.Speed-280*dt)
	}

	moveCar(car, dt)

	tm.turnCarAtIntersection(frame)

	// 4. Map Boundary Wrapping & Lane Tracking with customizable lane width
	tm.handleCarNavigation(car, laneWidth, w)
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
