package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (g *Game) activeDockedStop() (activeStop, misalignedStop *sim.BusStop) {
	isPlayerBerserk := g.Settings.AutopilotBerserk || (g.AutopilotDriver != nil && g.AutopilotDriver.BerserkMode)
	currentLaneW := g.Settings.GetLaneWidth()
	for _, s := range g.World.Stops {
		isBerserkOuterLane := isPlayerBerserk && s.ContainsBusInOuterLane(g.Bus, currentLaneW)
		isAutopilotDockedAtThisStop := g.Settings.AutopilotEnabled && g.AutopilotDriver != nil &&
			g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == s.ID &&
			(s.ContainsBus(g.Bus) || isBerserkOuterLane || s.Pos.Distance(g.Bus.Pos) < s.Length/2+65.0)
		if s.ContainsBus(g.Bus) || isBerserkOuterLane || isAutopilotDockedAtThisStop {
			isOuterDock := isBerserkOuterLane || (isAutopilotDockedAtThisStop && !s.ContainsBus(g.Bus))
			if !isOuterDock && (!g.Bus.PassengerService.Active || g.Bus.PassengerService.StopID != s.ID) {
				others := 0
				for _, b := range g.dockedBusesAtStop(s) {
					if b != g.Bus {
						others++
					}
				}
				if others >= 2 {
					continue
				}
			}
			activeStop = s
			break
		} else if s.Pos.Distance(g.Bus.Pos) < s.Length/2+65 || s.RoadCenter.Distance(g.Bus.Pos) < 80 {
			misalignedStop = s
		}
	}
	return activeStop, misalignedStop
}

func (g *Game) dockedBusesAtStop(stop *sim.BusStop) []*sim.Bus {
	if stop == nil {
		return nil
	}
	var buses []*sim.Bus
	if g.Bus != nil && g.Bus.DoorsOpen && math.Abs(g.Bus.Speed) <= 3 && !g.Bus.TerminalReplacementHold {
		buses = append(buses, g.Bus)
	}
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus == nil || !ab.Bus.DoorsOpen || math.Abs(ab.Bus.Speed) > 3 || ab.Retiring {
				continue
			}
			if ab.Bus.PendingModel != nil && ab.Route != nil && ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) {
				continue
			}
			atStop := (ab.Bus.PassengerService.Active && ab.Bus.PassengerService.StopID == stop.ID) ||
				(ab.Driver != nil && ab.Driver.State == autopilot.AutopilotStateDocked &&
					(ab.Driver.DockedStopID == stop.ID || (ab.Route != nil && ab.Route.GetCurrentTargetStopID() == stop.ID))) ||
				stop.ContainsBus(ab.Bus)
			if atStop {
				buses = append(buses, ab.Bus)
			}
		}
	}
	return buses
}

func (g *Game) updatePassengerExchange(toggleDoors bool) {
	g.syncPassengerRates()
	g.ensureOperations()
	activeStop, misalignedStop := g.activeDockedStop()
	isPlayerBerserk := g.Settings.AutopilotBerserk || (g.AutopilotDriver != nil && g.AutopilotDriver.BerserkMode)

	// Feedback if driver tries to open doors near a stop from wrong side or reverse orientation
	if toggleDoors && activeStop == nil && misalignedStop != nil {
		angleDiff := math.Abs(sim.NormalizeAngle(g.Bus.Heading - misalignedStop.Heading))
		if angleDiff >= math.Pi/4 {
			g.HUD.AddWarningToast(fmt.Sprintf("【靠站方向错误】[%s] 车门背离站台！公交车只能靠马路右侧顺行停靠", misalignedStop.LocalizedName()))
		} else {
			g.HUD.AddWarningToast(fmt.Sprintf("【未准确进站】[%s] 请驶入马路右侧黄色停靠港湾方可上下客", misalignedStop.LocalizedName()))
		}
	}

	// Boarding logic when parked at active stop
	if activeStop != nil {
		g.lastSkippedStopID = -1
		if !g.Bus.DoorsOpen {
			// Doors closed while parked: summarize station exchange and reset flag
			g.finishStationServiceSummary()
			g.boardedThisStop = false
			g.Bus.PassengerService = sim.PassengerService{}
		} else {
			isTerminus := g.Route.IsTerminusIndex(g.Route.CurrentIndex) && activeStop.ID == g.Route.GetCurrentTargetStopID()
			// Wait for a full stop before the terminal vehicle handover.
			if g.Bus.PendingModel != nil && !g.boardedThisStop && isTerminus && math.Abs(g.Bus.Speed) > 3 {
				return
			}
			candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
			if len(candidates) == 0 && len(g.Route.StopIDs) > 0 {
				for _, id := range g.Route.StopIDs {
					if id != activeStop.ID {
						candidates = append(candidates, id)
					}
				}
			}

			if !g.boardedThisStop {
				g.boardedThisStop = true
				g.announceArrival(activeStop)

				offCount := g.Bus.CountPassengersAlightingAt(activeStop.ID)
				if isTerminus {
					offCount = len(g.Bus.Passengers)
				}
				g.Bus.BeginStationService(activeStop.ID, g.Route.CurrentIndex, offCount, isTerminus, candidates, g.Settings.ModelPassengerRates(g.Bus.Model))

				if g.Bus.PendingModel != nil && isTerminus {
					g.Bus.TerminalReplacementHold = true
					if offCount == 0 {
						site := g.vehicleHandoverSite()
						site.Stop = activeStop
						if !g.Bus.ReplaceModelAtTerminus(g.Route, activeStop.ID, site) {
							if g.Bus.PendingModel != nil && *g.Bus.PendingModel == sim.BusArticulated && site.OtherPlatformBus() {
								g.Bus.TerminalReplacementHold = false
							}
							g.boardedThisStop = false
							return
						}
						g.Bus.SetStationServiceRates(g.Settings.ModelPassengerRates(g.Bus.Model))
						g.HUD.AddToast(locale.Tf("终点站换车完成：%s · %d 人", "Terminal replacement: %s · %d passengers", locale.T(g.Bus.Model.Name(), g.Bus.Model.EnglishName()), g.Bus.Capacity), true)
					}
				}

				g.stopBellRung = false
				g.passengersAlighting = 0

				sim.RebalanceStationBoarding(activeStop, g.dockedBusesAtStop(activeStop), true)

				toastMsg := locale.Tf("[%s] 下客 %d 人！",
					"[%s] %d alighted!",
					activeStop.CleanName(), offCount)
				if isPlayerBerserk && !activeStop.ContainsBus(g.Bus) {
					toCenter := g.Bus.Pos.Sub(activeStop.RoadCenter)
					sinH := math.Sin(activeStop.Heading)
					cosH := math.Cos(activeStop.Heading)
					sideDist := toCenter.X*(-sinH) + toCenter.Y*cosH
					if activeStop.RoadLanes <= 2 && sideDist < -5.0 {
						toastMsg = locale.Tf("【⚡ 狂暴对向借道靠站】[%s] 下客 %d 人！",
							"【⚡ Berserk Opposing-Lane Dock】[%s] %d alighted!",
							activeStop.CleanName(), offCount)
					} else {
						toastMsg = locale.Tf("【⚡ 狂暴外道靠站】[%s] 下客 %d 人！",
							"【⚡ Berserk Outer-Lane Dock】[%s] %d alighted!",
							activeStop.CleanName(), offCount)
					}
				}
				g.HUD.AddToast(toastMsg, offCount > 0)

				// Advance to next stop if this was the planned target stop
				if activeStop.ID == g.Route.GetCurrentTargetStopID() {
					g.Route.AdvanceToNextStop()
				}
				g.rollAlightingForNextStop()
			}

			alightedNow, boardedNow, fareNow := g.Bus.StepStationService(0, activeStop, g.Route, g.Operations)
			g.Cash += fareNow

			if g.Bus.PendingModel != nil && g.Bus.PassengerService.IsTerminus && g.Bus.PassengerService.AlightingTarget == 0 && len(g.Bus.Passengers) == 0 {
				g.Bus.TerminalReplacementHold = true
				site := g.vehicleHandoverSite()
				site.Stop = activeStop
				if g.Bus.ReplaceModelAtTerminus(g.Route, activeStop.ID, site) {
					g.Bus.SetStationServiceRates(g.Settings.ModelPassengerRates(g.Bus.Model))
					g.HUD.AddToast(locale.Tf("终点站换车完成：%s · %d 人", "Terminal replacement: %s · %d passengers", locale.T(g.Bus.Model.Name(), g.Bus.Model.EnglishName()), g.Bus.Capacity), true)
					sim.RebalanceStationBoarding(activeStop, g.dockedBusesAtStop(activeStop), true)
				} else if g.Bus.PendingModel != nil {
					if *g.Bus.PendingModel == sim.BusArticulated && site.OtherPlatformBus() {
						g.Bus.TerminalReplacementHold = false
					} else if g.Bus.PassengerServiceRemaining() <= 0 {
						g.Bus.TerminalReplacementHold = false
					}
				}
			}

			dockedBuses := g.dockedBusesAtStop(activeStop)
			assignedBefore := 0
			for _, b := range dockedBuses {
				assignedBefore += b.PassengerService.BoardingTarget
			}
			prevTarget := g.Bus.PassengerService.BoardingTarget
			sim.RebalanceStationBoarding(activeStop, dockedBuses, false)
			if g.Bus.PassengerService.BoardingTarget > prevTarget {
				newArrivals := max(g.Bus.PassengerService.BoardingTarget-prevTarget, activeStop.WaitingPassengers-assignedBefore)
				toastMsg := locale.Tf("[%s] 站台新增 %d 名乘客！",
					"[%s] %d additional passengers at platform!",
					activeStop.CleanName(), newArrivals)
				g.HUD.AddToast(toastMsg, true)
			}

			if alightedNow > 0 || boardedNow > 0 {
				g.rollAlightingForNextStop()
			}
		}
	} else {
		// Left the stop bay, reset boarding flag
		g.finishStationServiceSummary()
		g.boardedThisStop = false
		g.Bus.PassengerService = sim.PassengerService{}
	}
}

func (g *Game) finishStationServiceSummary() {
	svc := g.Bus.PassengerService
	if !g.boardedThisStop || !svc.Active || g.World == nil || svc.StopID < 0 || svc.StopID >= len(g.World.Stops) {
		return
	}
	stop := g.World.Stops[svc.StopID]
	toastMsg := locale.Tf("[%s] 本站共下客 %d 人，上客 %d 人，营收 +$%d！",
		"[%s] Total: %d alighted, %d boarded, revenue +$%d!",
		stop.CleanName(), svc.AlightedSoFar, svc.BoardedSoFar, svc.FareSoFar)
	g.HUD.AddToast(toastMsg, svc.BoardedSoFar > 0 || svc.FareSoFar > 0)
}

func (g *Game) vehicleHandoverSite() sim.VehicleHandoverSite {
	site := sim.VehicleHandoverSite{World: g.World, LaneWidth: g.Settings.GetLaneWidth()}
	for _, car := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
		site.Obstacles = append(site.Obstacles, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: car.Length/2 + 2, HalfWid: car.Width/2 + 2})
	}
	for _, b := range g.AIBusMgr.Buses {
		for _, part := range b.Bus.BodyParts() {
			if part.HalfLen > 0 {
				part.HalfLen += 2
				part.HalfWid += 2
				site.Obstacles = append(site.Obstacles, part)
			}
		}
	}
	return site
}

func (g *Game) syncPassengerRates() {
	if g.AIBusMgr == nil {
		return
	}
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		g.AIBusMgr.PassengerRates[model] = g.Settings.ModelPassengerRates(model)
	}
}
