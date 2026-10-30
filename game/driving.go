package game

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// RollAlightingForNextStop determines if any passengers onboard wish to alight at the next stop and rings the bell.
func (g *Game) RollAlightingForNextStop() {
	g.rollAlightingForNextStop()
}

func (g *Game) rollAlightingForNextStop() {
	g.bellToastShown = false
	if g.Bus == nil || g.Route == nil {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}
	targetStopID := g.Route.GetCurrentTargetStopID()
	staying := max(0, g.Bus.PassengerCount-g.Bus.PassengerService.AlightingTarget)
	if targetStopID < 0 || staying == 0 {
		g.stopBellRung = false
		g.passengersAlighting = 0
		return
	}

	alighting := g.Bus.CountPassengersAlightingAt(targetStopID)
	if g.Bus.PassengerService.Active && g.Bus.PassengerService.StopID == targetStopID {
		alighting = max(0, alighting-g.Bus.PassengerService.AlightingTarget)
	}
	// If heading to the terminus of the route, all remaining passengers on board will alight
	if g.Route.IsTerminusIndex(g.Route.CurrentIndex) {
		if staying > alighting {
			alighting = staying
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
	if g.vehicleAudio != nil {
		g.vehicleAudio.throttle = throttle
	}
	g.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)
	if g.skipCooldown > 0 {
		g.skipCooldown -= dt
	}

	g.applyRoadBoundaries(currentLaneW)

	// Collision check between player bus and AI cars (resolves penetration immediately after bus moved)
	g.checkBusCarCollisions()

	// Camera follows bus
	g.Camera.FollowBus(g.Bus)

	// Departure reminders run first, so an urgent next-stop announcement
	// cannot be overwritten by safety speech later in the same tick.
	g.trackAnnouncementDeparture()
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
