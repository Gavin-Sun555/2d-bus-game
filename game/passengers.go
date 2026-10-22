package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (g *Game) updatePassengerExchange(toggleDoors bool) {
	g.ensureOperations()
	// Check if parked at any bus stop
	var activeStop *sim.BusStop
	var misalignedStop *sim.BusStop
	isPlayerBerserk := g.Settings.AutopilotBerserk || (g.AutopilotDriver != nil && g.AutopilotDriver.BerserkMode)
	currentLaneW := g.Settings.GetLaneWidth()
	for _, s := range g.World.Stops {
		isBerserkOuterLane := isPlayerBerserk && s.ContainsBusInOuterLane(g.Bus, currentLaneW)
		isAutopilotDockedAtThisStop := g.Settings.AutopilotEnabled && g.AutopilotDriver != nil &&
			g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == s.ID &&
			(s.ContainsBus(g.Bus) || isBerserkOuterLane || s.Pos.Distance(g.Bus.Pos) < s.Length/2+65.0)
		if s.ContainsBus(g.Bus) || isBerserkOuterLane || isAutopilotDockedAtThisStop {
			activeStop = s
			break
		} else if s.Pos.Distance(g.Bus.Pos) < s.Length/2+65 || s.RoadCenter.Distance(g.Bus.Pos) < 80 {
			misalignedStop = s
		}
	}

	// Feedback if driver tries to open doors near a stop from wrong side or reverse orientation
	if toggleDoors && activeStop == nil && misalignedStop != nil {
		angleDiff := math.Abs(sim.NormalizeAngle(g.Bus.Heading - misalignedStop.Heading))
		if angleDiff >= math.Pi/4 {
			g.HUD.AddWarningToast(fmt.Sprintf("【靠站方向错误】[%s] 车门背离站台！公交车只能靠马路右侧顺行停靠", misalignedStop.Name))
		} else {
			g.HUD.AddWarningToast(fmt.Sprintf("【未准确进站】[%s] 请驶入马路右侧黄色停靠港湾方可上下客", misalignedStop.Name))
		}
	}

	// Boarding logic when parked at active stop
	if activeStop != nil {
		g.lastSkippedStopID = -1
		if !g.Bus.DoorsOpen {
			// Doors closed while parked: reset flag so opening doors at this same stop allows boarding again
			g.boardedThisStop = false
		} else {
			// Doors are open
			boardSpace := g.Bus.Capacity - g.Bus.PassengerCount

			if !g.boardedThisStop {
				g.boardedThisStop = true

				// Passengers getting off:
				// Remove passengers whose target destination is this stop
				offCount := g.Bus.RemovePassengersAlightingAt(activeStop.ID)

				// If this active stop is the terminus of the route, all remaining passengers must alight
				if g.Route.IsTerminusIndex(g.Route.CurrentIndex) && activeStop.ID == g.Route.GetCurrentTargetStopID() {
					offCount += g.Bus.ClearPassengers()
				}
				g.Operations.Alight(0, activeStop.ID, offCount)
				g.stopBellRung = false
				g.passengersAlighting = 0

				// Passengers getting on
				boardCount := activeStop.WaitingPassengers
				if boardCount > boardSpace {
					boardCount = boardSpace
				}

				if boardCount > 0 {
					candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
					if len(candidates) == 0 && len(g.Route.StopIDs) > 0 {
						for _, id := range g.Route.StopIDs {
							if id != activeStop.ID {
								candidates = append(candidates, id)
							}
						}
					}
					actualBoarded, fareEarned := g.Operations.Board(0, g.Bus, g.Route, activeStop.ID, boardCount, candidates)
					activeStop.WaitingPassengers -= actualBoarded
					g.Cash += fareEarned
					toastMsg := locale.Tf("[%s] 下客 %d 人，上客 %d 人，营收 +$%d！",
						"[%s] %d alighted, %d boarded, revenue +$%d!",
						activeStop.CleanName(), offCount, actualBoarded, fareEarned)
					if isPlayerBerserk && !activeStop.ContainsBus(g.Bus) {
						toCenter := g.Bus.Pos.Sub(activeStop.RoadCenter)
						sinH := math.Sin(activeStop.Heading)
						cosH := math.Cos(activeStop.Heading)
						sideDist := toCenter.X*(-sinH) + toCenter.Y*cosH
						if activeStop.RoadLanes <= 2 && sideDist < -5.0 {
							toastMsg = locale.Tf("【⚡ 狂暴对向借道靠站】[%s] 下客 %d 人，上客 %d 人，营收 +$%d！",
								"【⚡ Berserk Opposing-Lane Dock】[%s] %d alighted, %d boarded, revenue +$%d!",
								activeStop.CleanName(), offCount, actualBoarded, fareEarned)
						} else {
							toastMsg = locale.Tf("【⚡ 狂暴外道靠站】[%s] 下客 %d 人，上客 %d 人，营收 +$%d！",
								"【⚡ Berserk Outer-Lane Dock】[%s] %d alighted, %d boarded, revenue +$%d!",
								activeStop.CleanName(), offCount, actualBoarded, fareEarned)
						}
					}
					g.HUD.AddToast(toastMsg, true)
				} else if boardSpace <= 0 && activeStop.WaitingPassengers > 0 {
					var toastMsg string
					if offCount > 0 {
						toastMsg = locale.Tf("[%s] 下客 %d 人，当前车厢已满载，无法上客！",
							"[%s] %d alighted, bus is at full capacity!",
							activeStop.CleanName(), offCount)
					} else {
						toastMsg = locale.Tf("[%s] 当前车厢已满载，无法上客！",
							"[%s] Bus is at full capacity!",
							activeStop.CleanName())
					}
					g.HUD.AddToast(toastMsg, false)
				} else if offCount > 0 {
					toastMsg := locale.Tf("[%s] 下客 %d 人，本站无乘客上车！",
						"[%s] %d alighted, no passengers waiting!",
						activeStop.CleanName(), offCount)
					g.HUD.AddToast(toastMsg, false)
				} else {
					toastMsg := locale.Tf("[%s] 本站无乘客上下车！",
						"[%s] No passengers boarding or alighting!",
						activeStop.CleanName())
					g.HUD.AddToast(toastMsg, false)
				}

				// Advance to next stop if this was the planned target stop
				if activeStop.ID == g.Route.GetCurrentTargetStopID() {
					g.Route.AdvanceToNextStop()
				}
				g.rollAlightingForNextStop()
			} else {
				// Doors remain open: if new passengers arrived while parked, board them as long as there is space!
				if activeStop.WaitingPassengers > 0 && boardSpace > 0 {
					boardCount := activeStop.WaitingPassengers
					if boardCount > boardSpace {
						boardCount = boardSpace
					}
					candidates := g.Route.GetUpcomingCandidateStopIDsForStop(activeStop.ID)
					if len(candidates) == 0 && len(g.Route.StopIDs) > 0 {
						for _, id := range g.Route.StopIDs {
							if id != activeStop.ID {
								candidates = append(candidates, id)
							}
						}
					}
					actualBoarded, fareEarned := g.Operations.Board(0, g.Bus, g.Route, activeStop.ID, boardCount, candidates)
					activeStop.WaitingPassengers -= actualBoarded
					g.Cash += fareEarned
					toastMsg := locale.Tf("[%s] 站台新增乘客，上客 %d 人，营收 +$%d！",
						"[%s] Additional passengers: %d boarded, revenue +$%d!",
						activeStop.CleanName(), actualBoarded, fareEarned)
					g.HUD.AddToast(toastMsg, true)
				}
			}
		}
	} else {
		// Left the stop bay, reset boarding flag
		g.boardedThisStop = false
	}
}
