package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (g *Game) applyRoadBoundaries(currentLaneW float64) {
	// Road Boundaries Check: Soft Shoulder (slowdown) & Hard Barriers (collision, damage & fine)
	bRes := g.Bus.CheckBoundaries(g.World, currentLaneW)
	switch bRes.Zone {
	case sim.ZoneRoad:
		g.Bus.OnShoulder = false
	case sim.ZoneShoulder:
		g.Bus.OnShoulder = !g.Bus.IsOnStationAccessShoulder(g.World, currentLaneW)
	case sim.ZoneHardBarrier:
		g.Bus.OnShoulder = false
		// Repulsion pushback from hard obstacle
		g.Bus.Pos = g.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 5.0))
		impactSpeed := math.Abs(g.Bus.Speed)
		g.Bus.Speed = -g.Bus.Speed * 0.35 // bounce recoil

		// Damage & Fine event
		if g.Bus.CrashCooldown <= 0 {
			g.Bus.CrashCooldown = 1.4
			g.Camera.AddShake(10.0) // visceral camera impact shake

			damageAmount := 15.0
			if impactSpeed > 120 {
				damageAmount = 25.0
			}
			g.Bus.Durability = math.Max(0, g.Bus.Durability-damageAmount)

			fine := 100
			g.Cash -= fine

			msg := fmt.Sprintf("[事故碰撞] 撞击%s！车况 -%.0f%%，赔偿罚金 -$%d",
				bRes.ObstacleName, damageAmount, fine)
			g.HUD.AddWarningToast(msg)

			if g.Bus.Durability <= 0 {
				g.Bus.Durability = 50.0
				g.Cash -= 200
				g.HUD.AddWarningToast("[严重损毁] 车况归零！已紧急呼叫拖车抢修至 50% (维修费 -$200)")
			}
		}
	}

}

func (g *Game) updateAutopilotDeadlock(dt, currentLaneW float64, obs []autopilot.VehicleObstacle, activeCars []*sim.TrafficCar) {
	jammed := false
	if !g.Bus.DoorsOpen && math.Abs(g.Bus.Speed) < 3 {
		for _, ob := range obs {
			if math.Abs(ob.Speed) >= 3 {
				continue
			}
			delta := ob.Pos.Sub(g.Bus.Pos)
			fwd := delta.X*math.Cos(g.Bus.Heading) + delta.Y*math.Sin(g.Bus.Heading)
			lat := math.Abs(-delta.X*math.Sin(g.Bus.Heading) + delta.Y*math.Cos(g.Bus.Heading))
			if math.Cos(ob.Heading-g.Bus.Heading) > 0.96 && lat < currentLaneW*0.45 && fwd > 0 {
				continue
			}
			dist := delta.Length()
			if dist < (g.Bus.Length+ob.Length)*0.5+40 && (fwd > -15 || dist < (g.Bus.Length+ob.Length)*0.5+15) {
				jammed = true
				break
			}
		}
	}
	g.AutopilotDeadlockTimer = g.DeadlockTracker.Update(g.AutopilotDeadlockTimer, dt, g.Bus.Pos, g.Bus.Speed, jammed)
	// Diagnostic only: a stationary player bus is never teleported or reset.
}
