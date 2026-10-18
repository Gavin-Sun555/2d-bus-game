package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (g *Game) checkBusCarCollisions() {
	var activeCars []*sim.TrafficCar
	if g.TrafficMgr != nil {
		activeCars = g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	}
	if len(activeCars) == 0 && (g.AIBusMgr == nil || len(g.AIBusMgr.Buses) == 0) {
		return
	}

	busOBB := sim.OBB{
		Center:  g.Bus.Pos,
		HalfLen: g.Bus.Length / 2, // 48.0
		HalfWid: g.Bus.Width / 2,  // 18.0
		Heading: g.Bus.Heading,
	}

	for _, car := range activeCars {
		carOBB := sim.OBB{
			Center:  car.Pos,
			HalfLen: car.Length / 2, // 23.0 ~ 25.0
			HalfWid: car.Width / 2,  // 11.0
			Heading: car.Heading,
		}

		collides, normal, depth := sim.CheckOBBCollision(busOBB, carOBB)
		if collides && depth > 0 {
			// Firm separation buffer to ensure zero visual overlap
			sepDepth := depth + 2.0

			// normal points from Car to Bus
			toBus := g.Bus.Pos.Sub(car.Pos)
			carForward := sim.Vec2{X: math.Cos(car.Heading), Y: math.Sin(car.Heading)}
			busForward := sim.Vec2{X: math.Cos(g.Bus.Heading), Y: math.Sin(g.Bus.Heading)}

			isCarFacingBus := carForward.Dot(toBus) > 0
			isBusFacingCar := busForward.Dot(car.Pos.Sub(g.Bus.Pos)) > 0

			isBusStopped := math.Abs(g.Bus.Speed) < 8.0
			isWrongWay := sim.IsBusFacingWrongWay(g.Bus)

			// 1. Positional Separation:
			// Normal points from Car to Bus.
			if isBusStopped {
				// Heavy stopped bus remains firmly planted; car absorbs almost all separation pushback
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.1))
				car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.9))
			} else if car.Speed < 5.0 {
				// Stationary car: bus absorbs pushback so it cannot penetrate
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth))
			} else {
				// Both moving: bus takes 70%, car takes 30%
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.7))
				car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.3))
			}

			// Update busOBB center for subsequent car checks
			busOBB.Center = g.Bus.Pos

			// 2. Velocity & Momentum Resolution:
			// Relative velocity along collision normal
			busVel := sim.Vec2{X: math.Cos(g.Bus.Heading) * g.Bus.Speed, Y: math.Sin(g.Bus.Heading) * g.Bus.Speed}
			carVel := sim.Vec2{X: math.Cos(car.Heading) * car.Speed, Y: math.Sin(car.Heading) * car.Speed}
			relVelAlongNormal := busVel.Sub(carVel).Dot(normal)

			// Normal points from Car to Bus. When moving into each other, relVelAlongNormal < 0
			if relVelAlongNormal < 0 {
				impactSpeed := math.Abs(relVelAlongNormal)

				if isCarFacingBus && (!isBusFacingCar || car.Speed > math.Abs(g.Bus.Speed)) {
					// Car impacted the bus (e.g. car rear-ended or broadsided the bus):
					// Car MUST immediately brake and stop, not accelerate forward!
					car.Speed = 0
					car.IsBraking = true
				} else if isBusFacingCar && g.Bus.Speed > 5.0 {
					// Bus actively rammed the car from behind: push car ahead
					car.Speed = math.Max(car.Speed+20.0, math.Abs(g.Bus.Speed)*0.8+15.0)
					car.IsBraking = false
				} else {
					// Side glance / mutual contact: kill car's excess forward velocity
					car.Speed = math.Min(car.Speed, 10.0)
					car.IsBraking = true
				}

				busSpeedBeforeImpact := g.Bus.Speed

				// 3. Accident Responsibility & Fines Determination:
				// Bus is at fault ONLY IF:
				//  a) Bus is driving wrong-way (逆行) into traffic, OR
				//  b) Player is manually driving and actively rear-ends / reverses into another vehicle.
				// In Autopilot mode, the player is not driving manually; autonomous cruise is never player fault.
				// In cross-traffic/roundabouts, vehicles failing to yield or broadsiding the bus are at fault.
				isBusInRoundabout := g.World != nil && g.World.IsAtRoundabout(g.Bus.Pos)
				toCar := car.Pos.Sub(g.Bus.Pos)
				distToCar := toCar.Length()
				busDotCar := 0.0
				if distToCar > 0.001 {
					busDotCar = busForward.Dot(toCar.Mul(1.0 / distToCar))
				}
				headAngleDiff := math.Abs(sim.NormalizeAngle(g.Bus.Heading - car.Heading))

				isPlayerAtFault := false
				if !g.Settings.AutopilotEnabled {
					if isWrongWay {
						isPlayerAtFault = true
					} else if !isBusStopped {
						// In-lane rear-end collision: bus directly rams the rear of a vehicle heading same direction
						isRearEndRam := busDotCar > 0.70 && busSpeedBeforeImpact > 18.0 && headAngleDiff < 0.85
						isReverseRam := busDotCar < -0.70 && busSpeedBeforeImpact < -12.0
						if isRearEndRam || isReverseRam {
							isPlayerAtFault = true
						}
					}
				}

				// Clamp and stop the bus from ramming through the car
				if isBusFacingCar && g.Bus.Speed > 0 {
					if impactSpeed > 45.0 {
						// High-speed collision: bounce recoil
						g.Bus.Speed = -g.Bus.Speed * 0.25
					} else {
						// Low-speed contact: clamp bus speed to 0
						g.Bus.Speed = 0
					}
				}

				if g.Bus.CrashCooldown <= 0 {
					g.Bus.CrashCooldown = 2.0
					g.Camera.AddShake(math.Min(12.0, 4.0+impactSpeed*0.15))

					if isPlayerAtFault {
						// Player fault: normal damage and fine
						damage := 8.0
						if impactSpeed > 80.0 {
							damage = 18.0
						}
						g.Bus.Durability = math.Max(0, g.Bus.Durability-damage)

						if g.Settings.TrafficFinesEnabled {
							fine := 50
							g.Cash -= fine
							if isWrongWay {
								g.HUD.AddWarningToast(fmt.Sprintf("[严重违章] 逆行引发交通事故！车况 -%.0f%%，罚款 -$%d", damage, fine))
							} else {
								g.HUD.AddWarningToast(fmt.Sprintf("[事故责任] 主动撞击社会车辆！车况 -%.0f%%，赔偿 -$%d", damage, fine))
							}
						} else {
							g.HUD.AddWarningToast(fmt.Sprintf("[事故碰撞] 撞击社会车辆！车况 -%.0f%% (已豁免罚金)", damage))
						}
					} else {
						// Victim (car struck stopped, roundabout right-of-way, or AI autopilot driving): ZERO FINE!
						damage := 3.0
						if impactSpeed > 80.0 {
							damage = 6.0
						}
						g.Bus.Durability = math.Max(0, g.Bus.Durability-damage)
						if g.Settings.AutopilotEnabled {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通碰撞] AI巡航已紧急制动避险！车况 -%.0f%% (免除事故赔偿)", damage))
						} else if isBusInRoundabout {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通事故] 环岛内社会车辆未按规定让行！车况 -%.0f%% (对方全责，免予罚款)", damage))
						} else {
							g.HUD.AddWarningToast(fmt.Sprintf("[交通事故] 被社会车辆碰撞！车况 -%.0f%% (对方全责，免予罚款)", damage))
						}
					}

					if g.Bus.Durability <= 0 {
						g.Bus.Durability = 50.0
						g.Cash -= 200
						g.HUD.AddWarningToast("[严重损毁] 车况归零！已紧急呼叫拖车抢修至 50% (维修费 -$200)")
					}
				}
			} else {
				// Static pushing / rubbing
				if isCarFacingBus {
					car.Speed = 0
					car.IsBraking = true
				}
				if isBusFacingCar && g.Bus.Speed > car.Speed {
					g.Bus.Speed = math.Max(0, car.Speed*0.5)
				}
			}
		}
	}

	// Collision check between player bus and route AI buses
	if g.AIBusMgr != nil {
		for _, ai := range g.AIBusMgr.Buses {
			if ai.Bus == nil {
				continue
			}
			aiOBB := sim.OBB{
				Center:  ai.Bus.Pos,
				HalfLen: ai.Bus.Length / 2,
				HalfWid: ai.Bus.Width / 2,
				Heading: ai.Bus.Heading,
			}
			collides, normal, depth := sim.CheckOBBCollision(busOBB, aiOBB)
			if collides && depth > 0 {
				sepDepth := depth + 2.0
				g.Bus.Pos = g.Bus.Pos.Add(normal.Mul(sepDepth * 0.5))
				ai.Bus.Pos = ai.Bus.Pos.Sub(normal.Mul(sepDepth * 0.5))
				busOBB.Center = g.Bus.Pos

				g.Bus.Speed = -g.Bus.Speed * 0.25
				ai.Bus.Speed = -ai.Bus.Speed * 0.25
			}
		}

		// Collision separation between AI buses and civilian traffic cars
		for _, ai := range g.AIBusMgr.Buses {
			if ai.Bus == nil {
				continue
			}
			for _, car := range activeCars {
				sim.ResolveBusCarSeparation(ai.Bus, car)
			}
		}

		// Collision separation between AI buses themselves
		for i := 0; i < len(g.AIBusMgr.Buses); i++ {
			for j := i + 1; j < len(g.AIBusMgr.Buses); j++ {
				sim.ResolveBusBusSeparation(g.AIBusMgr.Buses[i].Bus, g.AIBusMgr.Buses[j].Bus)
			}
		}
	}

	// Collision separation between civilian traffic cars themselves
	for i := 0; i < len(activeCars); i++ {
		for j := i + 1; j < len(activeCars); j++ {
			sim.ResolveCarCarSeparation(activeCars[i], activeCars[j])
		}
	}
}
