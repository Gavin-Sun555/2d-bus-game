package game

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

// autopilotControls updates controller configuration before producing this tick's inputs.
func (g *Game) autopilotControls(dt, currentLaneW float64) (throttle, brake, steer float64, handbrake, toggleDoors bool) {
	if g.AIBusMgr != nil {
		g.AIBusMgr.UpdatePlayerMergeYield(g.Bus, currentLaneW)
	}
	// Collect dynamic obstacles (civilian traffic cars + route AI buses)
	activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	obs := g.obstacles[:0]
	for _, c := range activeCars {
		obs = append(obs, autopilot.VehicleObstacle{
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

	g.obstacles = obs
	g.AutopilotDriver.BerserkMode = g.Settings.AutopilotBerserk
	g.AutopilotDriver.BerserkMaxSpeed = g.Settings.GetBerserkCruiseSpeed()
	if g.Settings.AutopilotBerserk {
		g.Bus.MaxSpeedFwd = math.Max(g.Settings.GetBusMaxSpeed(), g.AutopilotDriver.GetBerserkCruiseSpeed()+20.0)
	} else {
		g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
	}

	targetStopID := g.Route.GetCurrentTargetStopID()
	var currentTargetStop *sim.BusStop
	if targetStopID >= 0 && targetStopID < len(g.World.Stops) {
		currentTargetStop = g.World.Stops[targetStopID]
	}
	canSkipCurrent := false
	if currentTargetStop != nil {
		isOccupied := g.IsStopOccupiedByOtherVehicle(currentTargetStop)
		canSkipCurrent = !g.stopBellRung && (currentTargetStop.WaitingPassengers == 0 || isOccupied)
	}
	g.AutopilotDriver.CanSkipTargetStop = canSkipCurrent

	throttle, brake, steer, handbrake, toggleDoors, _ = g.AutopilotDriver.ComputeControls(
		g.Bus, g.Route, g.World, g.TrafficLights, obs, currentLaneW, dt,
	)

	g.updateAutopilotDeadlock(dt, currentLaneW, obs, activeCars)

	return
}

func (g *Game) manualControls() (throttle, brake, steer float64, handbrake, toggleDoors bool) {
	// Manual inputs cannot guarantee the automatic rear-slot braking policy.
	g.Bus.YieldingToBus = nil
	g.AutopilotDeadlockTimer = 0
	g.DeadlockTracker = sim.StationaryTracker{}
	g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
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
