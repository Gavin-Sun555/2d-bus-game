package game

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (g *Game) rollAlightingForNextStop() {
	g.bellToastShown = false
	if g.Bus == nil || g.Route == nil {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}
	targetStopID := g.Route.GetCurrentTargetStopID()
	if targetStopID < 0 || g.Bus.PassengerCount == 0 {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}

	alighting := g.Bus.CountPassengersAlightingAt(targetStopID)
	// If heading to the terminus of the route, all remaining passengers on board will alight
	if g.Route.IsTerminusIndex(g.Route.CurrentIndex) {
		if g.Bus.PassengerCount > alighting {
			alighting = g.Bus.PassengerCount
		}
	}

	g.passengersAlighting = alighting
	g.stopBellRung = (g.passengersAlighting > 0)
}

func (g *Game) updateDriverMode(dt float64) {
	currentLaneW := g.Settings.GetLaneWidth()

	g.updateAutopilotHotkeys()

	var throttle, brake, steer float64
	var handbrake, toggleDoors bool

	if g.Settings.AutopilotEnabled {
		throttle, brake, steer, handbrake, toggleDoors = g.autopilotControls(dt, currentLaneW)
	} else {
		throttle, brake, steer, handbrake, toggleDoors = g.manualControls()
	}

	if g.updateDriverHUD() {
		return
	}

	// Update bus kinematics
	g.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)
	if g.skipCooldown > 0 {
		g.skipCooldown -= dt
	}

	g.applyRoadBoundaries(currentLaneW)

	// Collision check between player bus and AI cars (resolves penetration immediately after bus moved)
	g.checkBusCarCollisions()

	// Camera follows bus
	g.Camera.FollowBus(g.Bus)

	g.updateStopApproach()

	g.updatePassengerExchange(toggleDoors)
}

// IsStopOccupiedByOtherVehicle checks if a bus stop bay is currently occupied by another stopped or docking vehicle
func (g *Game) IsStopOccupiedByOtherVehicle(stop *sim.BusStop) bool {
	if stop == nil {
		return false
	}
	// 1. Check AI route buses
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus != nil {
				// Docked strictly inside yellow bay box
				if stop.ContainsBus(ab.Bus) {
					return true
				}
				// Autopilot driver is in docked state at this stop
				if ab.Driver != nil && ab.Driver.State == autopilot.AutopilotStateDocked && ab.Driver.DockedStopID == stop.ID {
					return true
				}
				// Close to bay center and stopped or crawling, facing the stop direction
				if ab.Bus.Pos.Distance(stop.Pos) < (stop.Length*0.5+25.0) && math.Abs(ab.Bus.Speed) < 8.0 &&
					math.Cos(ab.Bus.Heading-stop.Heading) > 0.4 {
					return true
				}
			}
		}
	}
	// 2. Check civilian traffic cars that might be stalled or parked in the bay
	if g.TrafficMgr != nil {
		for _, c := range g.TrafficMgr.Cars {
			if c.Pos.Distance(stop.Pos) < 55.0 && math.Abs(c.Speed) < 8.0 &&
				math.Cos(c.Heading-stop.Heading) > 0.4 {
				return true
			}
		}
	}
	return false
}
