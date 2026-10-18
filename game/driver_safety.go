package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) applyRoadBoundaries(currentLaneW float64) {
	// Road Boundaries Check: Soft Shoulder (slowdown) & Hard Barriers (collision, damage & fine)
	bRes := g.World.CheckBoundaries(g.Bus.Pos, g.Bus.Width/2, currentLaneW)
	switch bRes.Zone {
	case sim.ZoneRoad:
		g.Bus.OnShoulder = false
	case sim.ZoneShoulder:
		// Official bus bays are designated parking areas off the main roadway, not a shoulder violation
		inBusBay := false
		for _, s := range g.World.Stops {
			if s.ContainsBus(g.Bus) || s.Pos.Distance(g.Bus.Pos) < s.Length/2 {
				inBusBay = true
				break
			}
		}
		if inBusBay {
			g.Bus.OnShoulder = false
		} else {
			g.Bus.OnShoulder = true
		}
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
	// Deadlock detection for AI Autopilot: if motionless and blocked by obstacle for > 1 traffic light cycle,
	// reset the bus back to the route's starting station (起点站)
	if math.Abs(g.Bus.Speed) < 8.0 {
		isJammed := false
		jammedCarIdx := -1
		var jammedAIBus *traffic.AIBus
		isNearInter := false
		if g.World != nil {
			for _, pt := range g.World.Intersections {
				if g.Bus.Pos.Distance(pt) < 130.0 {
					isNearInter = true
					break
				}
			}
		}
		for _, ob := range obs {
			if math.Abs(ob.Speed) >= 8.0 {
				continue
			}
			dist := g.Bus.Pos.Distance(ob.Pos)
			if dist < (g.Bus.Length+ob.Length)*0.5+40.0 {
				toOb := ob.Pos.Sub(g.Bus.Pos)
				cosH := math.Cos(g.Bus.Heading)
				sinH := math.Sin(g.Bus.Heading)
				fwd := toOb.X*cosH + toOb.Y*sinH
				lat := math.Abs(-toOb.X*sinH + toOb.Y*cosH)
				headDiff := math.Cos(ob.Heading - g.Bus.Heading)
				if headDiff > 0.96 && lat < currentLaneW*0.45 && fwd > 0 {
					continue // Normal queueing behind vehicle in front
				}
				if fwd > -15.0 || dist < (g.Bus.Length+ob.Length)*0.5+15.0 {
					isJammed = true
					if g.TrafficMgr != nil {
						for cIdx, c := range activeCars {
							if c != nil && c.Pos.Distance(ob.Pos) < 2.0 {
								jammedCarIdx = cIdx
								break
							}
						}
					}
					if g.AIBusMgr != nil {
						for _, ab := range g.AIBusMgr.Buses {
							if ab != nil && ab.Bus != nil && ab.Bus.Pos.Distance(ob.Pos) < 5.0 {
								jammedAIBus = ab
								break
							}
						}
					}
					break
				}
			}
		}
		if isJammed {
			g.AutopilotDeadlockTimer += dt
			// Point 1: Fast deadlock resolution (4.0s) applies ONLY at intersections.
			// Stations and open roads use the full traffic light cycle (38.0s) before resetting.
			if isNearInter && g.AutopilotDeadlockTimer >= traffic.IntersectionDeadlockThreshold {
				if jammedCarIdx >= 0 && g.TrafficMgr != nil {
					g.TrafficMgr.DespawnAndResetCar(jammedCarIdx, g.World, currentLaneW, g.Bus)
					g.AutopilotDeadlockTimer = 0
				} else if jammedAIBus != nil {
					jammedAIBus.ResetToStartingStation(g.World)
					g.AutopilotDeadlockTimer = 0
				}
			} else if g.AutopilotDeadlockTimer >= traffic.DeadlockDurationThreshold {
				if jammedCarIdx >= 0 && g.TrafficMgr != nil {
					g.TrafficMgr.DespawnAndResetCar(jammedCarIdx, g.World, currentLaneW, g.Bus)
					g.AutopilotDeadlockTimer = 0
				} else if jammedAIBus != nil {
					jammedAIBus.ResetToStartingStation(g.World)
					g.AutopilotDeadlockTimer = 0
				} else {
					g.AutopilotDeadlockTimer = 0
					oldBusPos := g.Bus.Pos
					g.ResetToStartingStation()
					// Also reset the jammed AI car(s) so both vehicles are cleared
					if g.TrafficMgr != nil {
						activeCars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
						for cIdx, c := range activeCars {
							if c == nil {
								continue
							}
							dist := oldBusPos.Distance(c.Pos)
							if dist < (g.Bus.Length+c.Length)*0.5+40.0 {
								g.TrafficMgr.DespawnAndResetCar(cIdx, g.World, currentLaneW, g.Bus)
								break
							}
						}
					}
				}
			}
		} else {
			if g.AutopilotDeadlockTimer > 0 {
				g.AutopilotDeadlockTimer = math.Max(0, g.AutopilotDeadlockTimer-dt*2.0)
			}
		}
	} else {
		g.AutopilotDeadlockTimer = 0
	}
}
