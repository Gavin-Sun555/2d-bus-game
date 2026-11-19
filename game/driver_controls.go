package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) syncPlayerSpeedLimit() {
	g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
	g.Bus.ApplyCatalogLimits()
}

// autopilotControls updates controller configuration before producing this tick's inputs.
func (g *Game) autopilotControls(dt, currentLaneW float64) (throttle, brake, steer float64, handbrake, toggleDoors bool) {
	if g.AIBusMgr != nil {
		g.AIBusMgr.UpdatePlayerMergeYield(g.Bus, currentLaneW)
	}
	// Collect dynamic obstacles (civilian traffic cars + route AI buses)
	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	obs := g.obstacles[:0]
	if g.AIBusMgr != nil && len(activeCars) > 0 {
		sight := 0.
		if g.Settings.AutopilotBerserk {
			_, _, multi, six := sim.CurrentRoadInfo(g.World, g.Bus.Pos, g.Bus.Heading, currentLaneW)
			if !multi && !six {
				// Include the same runway the two-lane passing decision uses.
				sight = autopilot.OncomingPassSightDistance(math.Abs(g.Bus.Speed), g.Settings.GetBerserkCruiseSpeed(), sim.SpeedKPH(sim.TrafficMaxSpeedKPH)) + currentLaneW
			}
		}
		obs = g.AIBusMgr.GetLocalObstacles(g.Bus.Pos, g.Bus.Speed, obs, sight)
	} else {
		for _, c := range activeCars {
			obs = append(obs, autopilot.VehicleObstacle{
				Car:              c,
				Pos:              c.Pos,
				YieldingTo:       c.YieldingToBus,
				JunctionTurn:     c.JunctionTurn,
				JunctionCorridor: c.JunctionCorridor,
				Heading:          c.Heading,
				Speed:            c.Speed,
				Length:           c.Length,
				Width:            c.Width,
			})
		}
		if g.AIBusMgr != nil {
			for _, ab := range g.AIBusMgr.Buses {
				obs = append(obs, autopilot.ObserveBus(ab.Bus))
			}
		}
	}

	g.obstacles = obs
	g.syncPlayerSpeedLimit()
	g.configurePlayerAutopilot()
	ad := g.AutopilotDriver
	targetStopID := g.Route.GetCurrentTargetStopID()
	var currentTargetStop *sim.BusStop
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		currentTargetStop = g.World.Stops[targetStopID]
	}
	if g.Settings.AutopilotBerserk {
		g.autopilotSkip.Reset()
		ad.CanSkipTargetStop = currentTargetStop != nil &&
			g.canAutopilotSkipStop(currentTargetStop, g.IsStopOccupiedByOtherVehicle(currentTargetStop))
	} else {
		ad.CanSkipTargetStop = currentTargetStop != nil && !g.stopBellRung &&
			g.autopilotSkip.Update(g.World, g.Bus, g.Route, ad, false)
	}

	var docked bool
	throttle, brake, steer, handbrake, toggleDoors, docked = ad.ComputeControls(
		g.Bus, g.Route, g.World, g.TrafficLights, obs, currentLaneW, dt,
	)
	// Standard autopilot shares the fleet's last-resort stall recoveries.
	if !g.Settings.AutopilotBerserk {
		held := throttle == 0 && brake > 0 && !docked
		if !ad.CanSkipTargetStop {
			if stop := traffic.RecoverPassedStation(g.World, g.Bus, g.Route, ad, false, &g.autopilotLateArrival, currentLaneW, dt, held); stop != nil {
				g.finishAutopilotStationPass(stop, currentLaneW, true)
			}
		}
		if ad.RecoverStrandedStationDeparture(g.Bus, g.World, g.TrafficLights, obs, currentLaneW, dt, held) ||
			ad.RecoverStrandedMedianApproach(g.Bus, g.World, obs, currentLaneW, dt, g.TrafficLights) {
			g.AutopilotDeadlockTimer = 0
			g.DeadlockTracker = sim.StationaryTracker{}
			return 0, 1, 0, false, false
		}
	}

	g.updateAutopilotDeadlock(dt, currentLaneW, obs, activeCars)

	return
}

// configurePlayerAutopilot gives standard autopilot exactly the controller
// setup of a route fleet bus running at the player's top speed (see
// traffic.AIBus.assignLineSpeed and the fleet update loop). Berserk keeps its
// uncapped straightaway runs and oncoming-lane overtaking.
func (g *Game) configurePlayerAutopilot() {
	ad := g.AutopilotDriver
	ad.BerserkMode = true
	if g.Settings.AutopilotBerserk {
		ad.BerserkMaxSpeed = g.Settings.GetBerserkCruiseSpeed()
		ad.CruiseSpeedLimit = 0
		ad.DisallowOncomingOvertake = false
		ad.FleetBus = false
		return
	}
	// A fleet bus overtakes only when it runs above its line's slowest speed.
	r := sim.SpeedRange{MinKPH: g.Settings.AIBusMinSpeed, MaxKPH: g.Settings.AIBusMaxSpeed}.Normalize(50, sim.TrafficMaxSpeedKPH)
	ad.BerserkMode = g.Settings.BerserkSpeedKPH() > r.MinKPH
	ad.BerserkMaxSpeed = g.Bus.MaxSpeedFwd
	ad.CruiseSpeedLimit = g.Bus.MaxSpeedFwd
	ad.DisallowOncomingOvertake = true
	ad.FleetBus = true
}

// standardAutopilot reports whether the fleet controller drives the bus.
func (g *Game) standardAutopilot() bool {
	return g.Settings.AutopilotEnabled && !g.Settings.AutopilotBerserk && g.AutopilotDriver != nil
}

// advanceStandardAutopilotPastStop is the fleet's post-move step: a bus
// skipping its stop moves on to the next one only at the end of the road
// approach, never while still short of the platform.
func (g *Game) advanceStandardAutopilotPastStop(dt, laneWidth float64) {
	if !g.standardAutopilot() {
		return
	}
	if stop := traffic.AdvancePastEmptyStation(g.World, g.Bus, g.Route, g.AutopilotDriver, laneWidth, dt); stop != nil {
		g.finishAutopilotStationPass(stop, laneWidth, false)
	}
}

// finishAutopilotStationPass mirrors AIBus.continuePastStation and keeps the
// player's next-stop bell and HUD in step. missed charges the usual missed
// stop fine.
func (g *Game) finishAutopilotStationPass(stop *sim.BusStop, laneWidth float64, missed bool) {
	ad := g.AutopilotDriver
	ad.ContinuePastSkippedStop(g.Bus, stop, laneWidth)
	ad.CanSkipTargetStop = false
	g.autopilotSkip.Reset()
	g.autopilotLateArrival = 0
	g.Bus.StationMergeRequest = nil
	g.Bus.LaneMergeRequest = nil
	g.lastSkippedStopID = stop.ID
	g.rollAlightingForNextStop()
	next := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	if !missed {
		g.HUD.AddToast(fmt.Sprintf(locale.T("【🤖 AI自主飞站】[%s] 站台无人且车内无下车乘客，自动飞站！下一站: [%s]",
			"[🤖 AI Skip] [%s] No one to board or alight, skipping stop! Next: [%s]"), stop.LocalizedName(), next.LocalizedName()), true)
		return
	}
	if g.Settings.TrafficFinesEnabled {
		g.Cash -= 50
		g.HUD.AddWarningToast(fmt.Sprintf(locale.T("【⚠️ 站台过头·漏站罚款 -$50】AI未能及时靠站，已越过 [%s] 站！扣除罚款 -$50，下一站: [%s]",
			"[⚠️ Missed Stop Fine -$50] AI failed to dock in time and overshot [%s]! Fined -$50, next stop: [%s]"), stop.LocalizedName(), next.LocalizedName()))
	} else {
		g.HUD.AddWarningToast(fmt.Sprintf(locale.T("【⚠️ 站台过头·漏站免罚】AI未能及时靠站，已越过 [%s] 站 (已豁免罚款)！下一站: [%s]",
			"[⚠️ Missed Stop Waived] AI failed to dock in time and overshot [%s] (fines waived)! Next stop: [%s]"), stop.LocalizedName(), next.LocalizedName()))
	}
}

// canAutopilotSkipStop is the skip rule for the current tick. Standard
// autopilot reports the fleet decision already latched for this tick.
func (g *Game) canAutopilotSkipStop(stop *sim.BusStop, occupied bool) bool {
	if !g.Settings.AutopilotBerserk {
		return g.AutopilotDriver != nil && g.AutopilotDriver.CanSkipTargetStop &&
			stop.ID == g.Route.GetCurrentTargetStopID()
	}
	if g.stopBellRung || g.Bus.DoorsOpen || g.Bus.DoorProgress > .05 || g.Bus.TerminalReplacementHold ||
		(g.Bus.PendingModel != nil && g.Route.IsTerminusIndex(g.Route.CurrentIndex)) {
		return false
	}
	return stop.WaitingPassengers == 0 || !stop.HasMatchingWaiting(g.Route.GetUpcomingCandidateStopIDsForStop(stop.ID))
}

func (g *Game) manualControls() (throttle, brake, steer float64, handbrake, toggleDoors bool) {
	// Manual inputs cannot guarantee the automatic rear-slot braking policy.
	g.Bus.YieldingToBus = nil
	g.autopilotSkip.Reset()
	g.autopilotLateArrival = 0
	g.AutopilotDeadlockTimer = 0
	g.DeadlockTracker = sim.StationaryTracker{}
	g.syncPlayerSpeedLimit()
	// Driver controls: Separated accelerator throttle and service foot brake
	if ebiten.IsKeyPressed(ebiten.KeyW) || ebiten.IsKeyPressed(ebiten.KeyUp) {
		throttle = 1.0
	}

	if ebiten.IsKeyPressed(ebiten.KeyS) || ebiten.IsKeyPressed(ebiten.KeyDown) {
		brake = 1.0
	}

	if ebiten.IsKeyPressed(ebiten.KeyA) || ebiten.IsKeyPressed(ebiten.KeyLeft) {
		steer -= 1.0
	}
	if ebiten.IsKeyPressed(ebiten.KeyD) || ebiten.IsKeyPressed(ebiten.KeyRight) {
		steer += 1.0
	}

	handbrake = ebiten.IsKeyPressed(ebiten.KeySpace)
	toggleDoors = inpututil.IsKeyJustPressed(ebiten.KeyE)

	// Gear Shifting inputs:
	// 1. Shift Up (升档): Shift key
	if inpututil.IsKeyJustPressed(ebiten.KeyShift) || inpututil.IsKeyJustPressed(ebiten.KeyShiftLeft) || inpututil.IsKeyJustPressed(ebiten.KeyShiftRight) {
		if g.Bus.ShiftUp() {
			g.HUD.AddToast("已升档: "+g.Bus.Gear.Name(), false)
		}
	}
	// 2. Shift Down (降档): Control key or Q key
	if inpututil.IsKeyJustPressed(ebiten.KeyControl) || inpututil.IsKeyJustPressed(ebiten.KeyControlLeft) || inpututil.IsKeyJustPressed(ebiten.KeyControlRight) || inpututil.IsKeyJustPressed(ebiten.KeyQ) {
		if g.Bus.ShiftDown() {
			g.HUD.AddToast("已降档: "+g.Bus.Gear.Name(), false)
		}
	}
	// 3. Direct gear hotkeys:
	// 'R': Directly shift to Reverse (倒挡) / Toggle between D and R
	if inpututil.IsKeyJustPressed(ebiten.KeyR) {
		if g.Bus.Gear == sim.GearReverse {
			if g.Bus.ShiftTo(sim.GearDrive) {
				g.HUD.AddToast("已切回前进挡: "+g.Bus.Gear.Name(), false)
			}
		} else {
			if g.Bus.ShiftTo(sim.GearReverse) {
				g.HUD.AddToast("已挂入倒挡: "+g.Bus.Gear.Name()+" (按 W/↑ 倒车，S/↓ 刹车)", false)
			} else {
				g.HUD.AddWarningToast("车速过快，变速箱保护锁止！请先踩刹车减速")
			}
		}
	}
	// 'N': Directly shift to Neutral (空挡)
	if inpututil.IsKeyJustPressed(ebiten.KeyN) {
		if g.Bus.ShiftTo(sim.GearNeutral) {
			g.HUD.AddToast("已挂入空挡: "+g.Bus.Gear.Name(), false)
		}
	}
	// 'G': Directly shift to Drive (前进挡)
	if inpututil.IsKeyJustPressed(ebiten.KeyG) {
		if g.Bus.ShiftTo(sim.GearDrive) {
			g.HUD.AddToast("已挂入前进挡: "+g.Bus.Gear.Name(), false)
		} else {
			g.HUD.AddWarningToast("倒车速度过快，无法直接切入前进挡！请先踩刹车减速")
		}
	}
	return
}

func (g *Game) updateAutopilotHotkeys() {
	// 0. AI Autopilot Toggle (P key) & Berserk Mode Toggle (B key or Shift+P)
	if inpututil.IsKeyJustPressed(ebiten.KeyP) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) {
			// Shift+P: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
			} else {
				g.HUD.AddToast(locale.T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		} else {
			// Normal P key: Toggle Autopilot ON/OFF
			g.Settings.AutopilotEnabled = !g.Settings.AutopilotEnabled
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
				if g.Settings.AutopilotBerserk {
					g.HUD.AddToast(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
				} else {
					g.HUD.AddToast(locale.T("【🤖 AI自动驾驶已开启】AI已接管车辆，遵循交规循迹靠站 (按 B 开启狂暴，按 P 接管)", "【🤖 AI Autopilot ON】AI controls vehicle, serving stops (Press B for Berserk, P to exit)"), true)
				}
			} else {
				g.HUD.AddToast(locale.T("【👤 已恢复手动驾驶】您已接管公交车控制权 (按 P 键可重新开启 AI)", "【👤 Manual Mode Restored】You have taken control (Press P to enable AI)"), false)
			}
		}
	} else if inpututil.IsKeyJustPressed(ebiten.KeyB) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) || ebiten.IsKeyPressed(ebiten.KeyShiftLeft) || ebiten.IsKeyPressed(ebiten.KeyShiftRight) {
			// Shift+B: Cycle Max Speed Limit (80 -> 120 -> 160 -> 180 -> 80)
			newSpd := g.Settings.CycleBerserkMaxSpeed()
			g.syncPlayerSpeedLimit()
			g.configurePlayerAutopilot()
			g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ 玩家公交车最高限速已切换】%d km/h", "【⚡ Player Bus Top Speed】%d km/h"), newSpd), true)
		} else {
			// Dedicated B key: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行最高时速 %d km/h (Shift+B可切速)，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed %d km/h (Shift+B to cycle speed), multi-lane overtaking enabled!"), g.Settings.AutopilotBerserkMaxSpeed), true)
			} else {
				g.HUD.AddToast(locale.T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		}
	}

}

func (g *Game) updateDriverHUD() bool {
	g.updateDrivingViewHotkey()
	// 4. Perspective toggle hotkey (V key)
	if inpututil.IsKeyJustPressed(ebiten.KeyV) {
		newMode := g.Camera.ToggleMode()
		g.Settings.CameraViewMode = newMode
		switch newMode {
		case sim.CameraModeHeadingUp:
			g.HUD.AddToast("已切换视角: 【车头朝上 (跟随视角)】 - 车头始终朝上，视野随转向旋转", true)
		case sim.CameraModeSouthUp:
			g.HUD.AddToast("已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上", true)
		default:
			g.HUD.AddToast("已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上", true)
		}
	}

	// 5. Mouse click on HUD buttons (Mode badge, Map theme badge, Perspective badge, Gear selectors & Autopilot)
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mouseScreen := sim.Vec2{X: float64(mx), Y: float64(my)}
		if g.HUD.HandleModeBtnClick(mouseScreen) {
			g.TogglePlanMode()
			return true
		} else if toggled := g.HUD.HandleMenuBtnClick(mouseScreen); toggled {
			g.State = StateMainMenu
			return true
		} else if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, true)
		} else if g.HUD.SettingsBtnBounds.Contains(mouseScreen) {
			g.Settings.IsOpen = true
			return true
		} else if shifted, notice := g.HUD.HandleGearClick(mouseScreen, g.Bus); shifted {
			g.HUD.AddToast(notice, false)
		} else if toggled, notice := g.HUD.HandleAutopilotClick(mouseScreen, g.Settings); toggled {
			g.syncPlayerSpeedLimit()
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
			}
			g.HUD.AddToast(notice, g.Settings.AutopilotEnabled)
		} else if notice != "" {
			g.HUD.AddWarningToast(notice)
		}
	}

	return false
}

// outerLaneDockAllowed reports whether the player bus may serve a stop from
// the lane beside the bay: always in Berserk, and in standard autopilot only
// when the driver has fallen back to it as a route bus would.
func (g *Game) outerLaneDockAllowed() bool {
	return g.Settings.AutopilotBerserk ||
		(g.Settings.AutopilotEnabled && g.AutopilotDriver.OuterLaneDockAllowed())
}
