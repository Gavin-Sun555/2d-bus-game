package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) updateStopApproach() {
	// Target stop navigation & Stop Bell / Skip Stop (飞站) processing
	targetStopID := g.Route.GetCurrentTargetStopID()
	var targetStop *sim.BusStop
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		targetStop = g.World.Stops[targetStopID]
	}

	if targetStop != nil {
		// Boarding and route endpoints use the forward berth marker. Measuring
		// passage from the platform center fines a long-platform arrival before
		// it reaches that marker, including while returning from a passing lane.
		servicePosition := targetStop.FrontBerthPosition()
		distToTarget := g.Bus.Pos.Distance(servicePosition)

		headingVec := sim.UnitFromAngle(targetStop.Heading)
		perpVec := sim.Vec2{X: -headingVec.Y, Y: headingVec.X}
		dVec := g.Bus.Pos.Sub(servicePosition)
		forwardProj := dVec.X*headingVec.X + dVec.Y*headingVec.Y

		// 1. Heading alignment: Bus must be traveling in the same general direction as the stop
		// (Prevents oncoming vehicles driving past on the opposite side of the road from triggering)
		cosHeading := math.Cos(g.Bus.Heading - targetStop.Heading)
		isAlignedHeading := cosHeading > 0.55

		// 2. Road side: Bus must be on the stop's side of the road (or borrowing oncoming lane on 2-lane road while overtaking)
		// Oncoming lanes of 4/6-lane roads, opposite carriageways, or parallel roads must NOT match
		toCenter := g.Bus.Pos.Sub(targetStop.RoadCenter)
		sideDist := toCenter.X*perpVec.X + toCenter.Y*perpVec.Y
		curbDist := targetStop.Pos.Distance(targetStop.RoadCenter)
		minSideDist := -25.0
		if targetStop.RoadLanes <= 2 {
			// On 2-lane roads, bus may borrow opposing lane during overtake
			minSideDist = -curbDist - 20.0
		}
		maxSideDist := curbDist + targetStop.Width*0.5 + 35.0
		isOnStopRoad := sideDist >= minSideDist && sideDist <= maxSideDist

		// 3. Autopilot route progress:
		// If Autopilot is driving, verify it is on the route approach to targetStop,
		// not on an earlier leg or opposite road that happens to pass near targetStop.
		isArrivingAtStop := true
		if g.Settings.AutopilotEnabled && g.AutopilotDriver != nil && g.AutopilotDriver.PathTargetStopID() == targetStop.ID {
			if forwardProj > 55.0 {
				// For overshooting past the stop: must have reached the end of the leg
				if g.AutopilotDriver.LastRemaining > 200.0 {
					isArrivingAtStop = false
				}
			} else {
				// For approaching/skipping the stop: must be on the approach to the stop (not 450+ px away on earlier leg)
				if g.AutopilotDriver.LastRemaining > 450.0 && g.AutopilotDriver.LastRemaining > distToTarget+100.0 {
					isArrivingAtStop = false
				}
			}
		}

		isValidStopApproach := isAlignedHeading && isOnStopRoad && isArrivingAtStop

		currentLaneW := g.Settings.GetLaneWidth()
		isPlayerBerserk := g.Settings.AutopilotBerserk || (g.AutopilotDriver != nil && g.AutopilotDriver.BerserkMode)
		isDocked := targetStop.ContainsBus(g.Bus) ||
			(g.AutopilotDriver != nil && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == targetStop.ID) ||
			(g.Bus.DoorsOpen && isPlayerBerserk && targetStop.ContainsBusInOuterLane(g.Bus, currentLaneW))

		if distToTarget > 450.0 && g.lastSkippedStopID == targetStop.ID {
			g.lastSkippedStopID = -1
		}

		isCirclingNearStop := g.Settings.AutopilotEnabled && !isDocked && distToTarget < 350.0 &&
			isOnStopRoad && isArrivingAtStop &&
			(g.AutopilotDriver != nil && g.AutopilotDriver.PathTargetStopID() == targetStop.ID && g.AutopilotDriver.LastRemaining < 120.0) &&
			forwardProj > 15.0 && cosHeading < 0.55

		if !isDocked && g.shouldPreannounceStop(targetStop, distToTarget, isValidStopApproach && forwardProj < 55) {
			g.announceNextStop(targetStop)
		}

		// 1. Chime bell notification when approaching within 550px
		if distToTarget < 550 && g.stopBellRung && !g.bellToastShown && isAlignedHeading && isOnStopRoad && isArrivingAtStop {
			g.bellToastShown = true
			g.HUD.AddToast(locale.T("[🔔 下车铃响] 叮咚~ 有乘客请求在下一站下车！", "[🔔 Stop requested] A passenger has requested the next stop!"), false)
		}

		// 2. Skip Stop (飞站 / 漏站) check when approaching or past within 380px
		if distToTarget <= 380.0 && (isValidStopApproach || isCirclingNearStop) {
			isOccupied := g.IsStopOccupiedByOtherVehicle(targetStop)
			canSkip := !g.stopBellRung && (targetStop.WaitingPassengers == 0 || isOccupied)

			// Proactive AI Autopilot Skip when station can be skipped and no bell rung (both empty and occupied station)
			if g.Settings.AutopilotEnabled && canSkip && forwardProj < 55.0 && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 {
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.LocalizedName()
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					g.AutopilotDriver.ContinuePastSkippedStop(g.Bus, targetStop, currentLaneW)
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if isOccupied {
					g.HUD.AddToast(fmt.Sprintf(locale.T("【🤖 AI自主飞站】[%s] 检测到前方已有车靠站且无下车乘客，自动飞站提速！下一站: [%s]",
						"[🤖 AI Skip] [%s] Station occupied & no alighting pax, skipping stop! Next: [%s]"), oldName, newTarget.LocalizedName()), true)
				} else {
					g.HUD.AddToast(fmt.Sprintf(locale.T("【🤖 AI自主飞站】[%s] 站台无人且车内无下车乘客，自动飞站提速！下一站: [%s]",
						"[🤖 AI Skip] [%s] Station empty & no alighting pax, skipping stop! Next: [%s]"), oldName, newTarget.LocalizedName()), true)
				}
			}

			// Manual skip with F key
			if inpututil.IsKeyJustPressed(ebiten.KeyF) {
				if canSkip {
					g.lastSkippedStopID = targetStop.ID
					g.skipCooldown = 0.5
					oldName := targetStop.LocalizedName()
					g.Route.AdvanceToNextStop()
					newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
					g.rollAlightingForNextStop()
					if g.AutopilotDriver != nil {
						g.AutopilotDriver.ContinuePastSkippedStop(g.Bus, targetStop, currentLaneW)
					}
					g.Bus.IsBraking = false
					g.Bus.IsHandbraking = false
					if isOccupied {
						g.HUD.AddToast(fmt.Sprintf("【⚡ 飞站成功】[%s] 前方站台已有车靠站且本车无人下车，已飞站！下一站: [%s]", oldName, newTarget.LocalizedName()), true)
					} else {
						g.HUD.AddToast(fmt.Sprintf("【⚡ 飞站成功】[%s] 站内无人且无人按铃，已飞站！下一站: [%s]", oldName, newTarget.LocalizedName()), true)
					}
				} else {
					reasons := ""
					if g.stopBellRung {
						reasons += locale.T("下车铃已响 ", "Stop requested ")
					}
					if targetStop.WaitingPassengers > 0 && !isOccupied {
						reasons += fmt.Sprintf("站台有 %d 人候车且前方无其他车辆靠站 ", targetStop.WaitingPassengers)
					}
					g.HUD.AddWarningToast(fmt.Sprintf("【不可飞站】[%s] %s，必须进站停靠！", targetStop.LocalizedName(), reasons))
				}
			}

			// Drive-by auto skip / AI missed stop check when driving past the stop
			latDist := math.Abs(-dVec.X*headingVec.Y + dVec.Y*headingVec.X)

			// Point 2: Missed stop penalty only applies when bus has physically missed or overshot the bay:
			// A. Front-section center has passed the forward berth by >55px.
			isCenterOvershot := forwardProj > 55.0 && distToTarget < 350.0

			// B. The front-section reference has passed the forward berth while
			// in an outer/passing lane. A protruding nose alone is not a miss:
			// all models must still be allowed to reach the boarding marker.
			// 1. Bus is stalled/crawling past the berth (Speed <8, latDist >65).
			//    — this immediately resolves the exact deadlock where front has passed and bus stops at 0 km/h!
			// 2. Bus is actively cruising/passing through in outer lane without docking (TargetLaneOffset in outer lane, forwardProj > 20.0)
			isStalledInOuterLane := !isOccupied && latDist > 65.0 && forwardProj > 20.0 && distToTarget < 350.0 && math.Abs(g.Bus.Speed) < 8.0
			isPassingInOuterLane := distToTarget < 350.0 && forwardProj > 20.0 &&
				(g.AutopilotDriver != nil && (g.AutopilotDriver.TargetLaneOffset < -currentLaneW*0.25 || g.AutopilotDriver.LaneOffset < -currentLaneW*0.25))

			isAIMissed := !canSkip && g.Settings.AutopilotEnabled && !isDocked &&
				(isCenterOvershot || isStalledInOuterLane || isPassingInOuterLane || isCirclingNearStop)

			if (forwardProj > 35.0 || (latDist > 65.0 && forwardProj > 15.0) || isCirclingNearStop) && distToTarget < 350.0 && !isDocked && canSkip && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 {
				// Lawful skip when station is empty or occupied with no bell rung (Point 4: pass straight through in outer or through lane)
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.LocalizedName()
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					g.AutopilotDriver.ContinuePastSkippedStop(g.Bus, targetStop, currentLaneW)
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if isOccupied {
					g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ 快速飞站】顺利掠过 [%s]（站台已有车靠站）！已自动飞站，下一站: [%s]", "【⚡ Fast Skip】Glided past [%s] (occupied)! Next stop: [%s]"), oldName, newTarget.LocalizedName()), true)
				} else {
					g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ 快速飞站】顺利掠过 [%s]！已自动飞站，下一站: [%s]", "【⚡ Fast Skip】Glided past [%s]! Next stop: [%s]"), oldName, newTarget.LocalizedName()), true)
				}
			} else if isAIMissed && g.lastSkippedStopID != targetStop.ID && g.skipCooldown <= 0 {
				// AI Autopilot genuinely missed the station after overshooting:
				// Directly skip this station and deduct $50 fine (only in AI mode), and immediately continue driving!
				g.skipCooldown = 0.5
				g.lastSkippedStopID = targetStop.ID
				oldName := targetStop.LocalizedName()
				if g.Settings.TrafficFinesEnabled {
					g.Cash -= 50
				}
				g.Route.AdvanceToNextStop()
				newTarget := g.World.Stops[g.Route.GetCurrentTargetStopID()]
				// Reassign destination for passengers on board destined for the missed stop to next stop
				for i := range g.Bus.Passengers {
					if g.Bus.Passengers[i].TargetStopID == targetStop.ID {
						g.Bus.Passengers[i].TargetStopID = newTarget.ID
					}
				}
				g.rollAlightingForNextStop()
				if g.AutopilotDriver != nil {
					// A missed stop is also a through passage. Re-anchor the
					// next leg to the actual lane instead of applying an old
					// bay-relative offset to its new departure taper.
					g.AutopilotDriver.ContinuePastSkippedStop(g.Bus, targetStop, currentLaneW)
					g.AutopilotDriver.State = autopilot.AutopilotStateCruising
				}
				g.Bus.IsBraking = false
				g.Bus.IsHandbraking = false
				if g.Settings.TrafficFinesEnabled {
					g.HUD.AddWarningToast(fmt.Sprintf(locale.T("【⚠️ 站台过头·漏站罚款 -$50】AI未能及时靠站，已越过 [%s] 站！扣除罚款 -$50，下一站: [%s]",
						"[⚠️ Missed Stop Fine -$50] AI failed to dock in time and overshot [%s]! Fined -$50, next stop: [%s]"), oldName, newTarget.LocalizedName()))
				} else {
					g.HUD.AddWarningToast(fmt.Sprintf(locale.T("【⚠️ 站台过头·漏站免罚】AI未能及时靠站，已越过 [%s] 站 (已豁免罚款)！下一站: [%s]",
						"[⚠️ Missed Stop Waived] AI failed to dock in time and overshot [%s] (fines waived)! Next stop: [%s]"), oldName, newTarget.LocalizedName()))
				}
			}
		}
	}

}
