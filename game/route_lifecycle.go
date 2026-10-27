package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// ResetBusesToRouteStart resets both the player's bus and all AI fleet buses
// to the first stop of the currently planned route.
func (g *Game) ResetBusesToRouteStart() { g.resetBusesToRouteStart(true) }

func (g *Game) resetBusesToRouteStart(resetFleet bool) {
	if g.Route == nil || len(g.Route.StopIDs) == 0 || g.World == nil || len(g.World.Stops) == 0 {
		return
	}

	firstStopID := g.Route.StopIDs[0]
	if firstStopID < 0 || firstStopID >= len(g.World.Stops) {
		firstStopID = 0
	}
	firstStop := g.World.Stops[firstStopID]

	// 1. Reset player bus
	if g.Bus != nil {
		g.Bus.Pos = firstStop.FrontBerthPosition()
		g.Bus.Heading = firstStop.Heading
		g.Bus.Speed = 0
		g.Bus.SteeringAngle = 0
		g.Bus.Gear = sim.GearDrive
		g.Bus.TransmissionMode = sim.TransmissionAuto
		g.Bus.IsHandbraking = false
		g.Bus.IsBraking = false
		g.Bus.DoorsOpen = true // Ready at the starting station bay for passenger boarding
		g.Bus.OnShoulder = false
		g.Bus.ClearPassengers()
		g.Bus.ResetArticulation()
	}

	g.boardedThisStop = false
	g.skipCooldown = 0
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.bellToastShown = false
	g.Route.CurrentIndex = 0
	g.Route.InvalidateDescription()

	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
	}
	if !resetFleet {
		g.queuePlayerAtStop(firstStop)
	}

	// 2. Ensure initial stop has passengers for immediate departure
	if firstStop.WaitingPassengers < 4 {
		firstStop.WaitingPassengers = 4 + sim.RandomIntn(4)
	}

	// 3. Reset AI bus fleet queuing behind player bus at first stop

	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.Routes = g.activeLines()
		if resetFleet {
			g.buildLineFleets(true)
		}
	}

	// 4. Follow player bus with camera
	if g.Camera != nil && g.Bus != nil {
		g.Camera.FollowBus(g.Bus)
		g.Camera.Pos = g.Bus.Pos
		g.Camera.TargetPos = g.Bus.Pos
		g.Camera.RestoreDrivingZoom()
		g.Camera.Rotation = g.Camera.TargetRotation
	}

	g.ensureOperations()
	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()
}

// ResetToStartingStation resets the player's bus back to the starting station (起点站) of the active route.
func (g *Game) ResetToStartingStation() {
	if g.Route == nil || len(g.Route.StopIDs) == 0 || g.World == nil || len(g.World.Stops) == 0 {
		return
	}

	g.AutopilotDeadlockTimer = 0
	g.DeadlockTracker = sim.StationaryTracker{}

	startID := g.Route.StopIDs[0]
	if startID < 0 || startID >= len(g.World.Stops) {
		startID = 0
	}
	startStop := g.World.Stops[startID]

	if startStop.WaitingPassengers < 4 {
		startStop.WaitingPassengers = 4 + sim.RandomIntn(4)
	}

	g.Route.CurrentIndex = 0
	g.Route.InvalidateDescription()

	if g.Bus != nil {
		g.Bus.Pos = startStop.FrontBerthPosition()
		g.Bus.Heading = startStop.Heading
		g.Bus.Speed = 0
		g.Bus.SteeringAngle = 0
		g.Bus.Gear = sim.GearDrive
		g.Bus.TransmissionMode = sim.TransmissionAuto
		g.Bus.IsHandbraking = false
		g.Bus.IsBraking = false
		g.Bus.DoorsOpen = false
		g.Bus.ClearPassengers()
		g.Bus.ResetArticulation()
	}

	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
		g.AutopilotDriver.State = autopilot.AutopilotStateDocked
		g.AutopilotDriver.DockedStopID = startStop.ID
		g.AutopilotDriver.DwellTimer = 4.0
		g.AutopilotDriver.DwellFinished = false
	}

	g.boardedThisStop = true
	g.skipCooldown = 0
	g.stopBellRung = false
	g.passengersAlighting = 0
	g.bellToastShown = false
	g.queuePlayerAtStop(startStop)

	if g.Camera != nil && g.Bus != nil {
		g.Camera.Pos = g.Bus.Pos
		g.Camera.TargetPos = g.Bus.Pos
	}

	g.HUD.AddToast(locale.T("【⚠️ 道路严重拥堵超时】AI公交车已重置回起点站，重新投入运营", "【⚠️ Severe Gridlock Timeout】AI Bus reset to starting station to resume route"), true)
}

// SyncStopsWithRoute ensures that ONLY bus stops included in the planned route
// have waiting passengers and can generate passengers.
// Any stops not on the planned route are cleared (waiting = 0, timer = 0).
func (g *Game) SyncStopsWithRoute() {
	if g.World == nil || g.Route == nil {
		return
	}
	for _, s := range g.World.Stops {
		served := false
		for _, r := range g.activeLines() {
			if r.ContainsStop(s.ID) {
				served = true
				break
			}
		}
		if !served {
			s.WaitingPassengers = 0
			s.SpawnTimer = 0
		} else if s.WaitingPassengers == 0 {
			// Ensure route stops have passengers ready for service
			s.WaitingPassengers = 4 + sim.RandomIntn(4)
		}
	}
}

// ReplanRoute updates the active route to the given stop sequence and resets
// the player bus and AI fleet to the first station.
func (g *Game) ReplanRoute(stopIDs []int) {
	if len(stopIDs) == 0 {
		return
	}
	g.Route.Clear()
	for _, id := range stopIDs {
		g.Route.AddStop(id)
	}
	g.ResetBusesToRouteStart()
}
