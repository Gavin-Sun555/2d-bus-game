package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// DeadlockDurationThreshold defines the duration in seconds of deadlock (exceeding one full 37s traffic light cycle)
// before an unresolvable jammed non-player car disappears on open roads.
const DeadlockDurationThreshold = 38.0

// IntersectionDeadlockThreshold defines the fast-timeout threshold (4.0s) for clearing civilian traffic jams
// inside or near intersections, or when wedged against a turning bus, so turning buses never get stuck.
const IntersectionDeadlockThreshold = 4.0

// DespawnCar removes a car from active traffic
func (tm *TrafficManager) DespawnCar(index int) {
	if tm == nil || index < 0 || index >= len(tm.Cars) {
		return
	}
	tm.Cars = append(tm.Cars[:index], tm.Cars[index+1:]...)
}

// DespawnAndResetCar resets a deadlocked car by repositioning it to a clear section of road,
// or despawns it if no world or valid position is available.
func (tm *TrafficManager) DespawnAndResetCar(index int, w *sim.World, laneWidth float64, bus *sim.Bus) {
	if tm == nil || index < 0 || index >= len(tm.Cars) {
		return
	}
	car := tm.Cars[index]
	if w == nil || len(w.Roads) == 0 {
		tm.DespawnCar(index)
		return
	}

	// Try to find a clear road position away from intersections, stops, roundabouts, bus, and other cars
	resetSuccess := false
	for attempt := 0; attempt < 25; attempt++ {
		rIdx := sim.RandomIntn(len(w.Roads))
		r := w.Roads[rIdx]
		dir := r.P2.Sub(r.P1)
		lenR := dir.Length()
		if lenR < 350 {
			continue
		}
		unitDir := dir.Mul(1.0 / lenR)
		isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		var roadCoord float64
		if isEW {
			roadCoord = r.P1.Y
		} else {
			roadCoord = r.P1.X
		}

		numLanesPerDir := 1
		if r.Lanes >= 6 {
			numLanesPerDir = 3
		} else if r.Lanes >= 4 {
			numLanesPerDir = 2
		}

		lane := sim.RandomIntn(numLanesPerDir)
		offset := (float64(lane) + 0.5) * laneWidth
		dist := 260.0 + sim.RandomFloat64()*(lenR-520.0)
		pt := r.P1.Add(unitDir.Mul(dist))

		var testPos sim.Vec2
		var heading float64
		if sim.RandomFloat64() < 0.5 {
			if isEW {
				testPos = sim.Vec2{X: pt.X, Y: roadCoord + offset}
				heading = 0
			} else {
				testPos = sim.Vec2{X: roadCoord - offset, Y: pt.Y}
				heading = math.Pi / 2
			}
		} else {
			if isEW {
				testPos = sim.Vec2{X: pt.X, Y: roadCoord - offset}
				heading = math.Pi
			} else {
				testPos = sim.Vec2{X: roadCoord + offset, Y: pt.Y}
				heading = -math.Pi / 2
			}
		}

		// Ensure clear of intersections
		nearInter := false
		for _, ipt := range w.Intersections {
			if testPos.Distance(ipt) < 220 {
				nearInter = true
				break
			}
		}
		if nearInter {
			continue
		}

		// Ensure clear of stops
		nearStop := false
		for _, st := range w.Stops {
			if testPos.Distance(st.Pos) < 160 {
				nearStop = true
				break
			}
		}
		if nearStop {
			continue
		}

		// Ensure clear of bus
		if bus != nil && testPos.Distance(bus.Pos) < 200 {
			continue
		}

		// Ensure clear of other cars
		nearOtherCar := false
		for _, oc := range tm.Cars {
			if oc != car && testPos.Distance(oc.Pos) < 120 {
				nearOtherCar = true
				break
			}
		}
		if nearOtherCar {
			continue
		}

		// Found valid clear position! Reset car here.
		car.Pos = testPos
		car.Heading = heading
		car.RoadCenter = roadCoord
		car.IsEW = isEW
		car.LaneIndex = lane
		car.Speed = car.TargetSpeed
		car.DeadlockTimer = 0
		car.IsBraking = false
		car.InRoundabout = false
		car.RoundaboutCooldown = 0
		resetSuccess = true
		break
	}

	if !resetSuccess {
		tm.DespawnCar(index)
	}
}

func (tm *TrafficManager) ResolveDeadlocks(dt float64, bus *sim.Bus, activeCount int, laneWidth float64, w *sim.World) {
	if tm == nil || len(tm.Cars) == 0 {
		return
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]

		// 1. Moving vehicles are never deadlocked
		if math.Abs(car.Speed) >= 3.0 {
			car.DeadlockTimer = 0
			continue
		}

		// Detect if car is near an intersection (within 130px of intersection center)
		isNearIntersection := false
		if w != nil {
			for _, inter := range w.Intersections {
				if car.Pos.Distance(inter) < 130.0 {
					isNearIntersection = true
					break
				}
			}
		}

		isJammed := false
		jammedWithPlayerBus := false

		// 2. Check relation to player bus
		if bus != nil && math.Abs(bus.Speed) < 8.0 {
			distToBus := car.Pos.Distance(bus.Pos)
			if distToBus < 180.0 {
				toBus := bus.Pos.Sub(car.Pos)
				cosH := math.Cos(car.Heading)
				sinH := math.Sin(car.Heading)
				fwdBus := toBus.X*cosH + toBus.Y*sinH
				latBus := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
				headDiff := math.Cos(car.Heading - bus.Heading)
				// If car is queued behind player bus heading in the same direction on open road:
				if !isNearIntersection && headDiff > 0.5 && fwdBus > 10.0 && latBus < laneWidth*1.5 {
					car.DeadlockTimer = 0
					continue
				}
				// If car is jammed against player bus (crosswise, oncoming, turning, or trapped against front bumper):
				maxJamDist := (car.Length+bus.Length)*0.5 + 40.0
				if distToBus < maxJamDist && (fwdBus > -15.0 || distToBus < (car.Length+bus.Length)*0.5+15.0) {
					isJammed = true
					jammedWithPlayerBus = true
				}
			}
		}

		// 3. Check against other civilian AI cars
		if !isJammed {
			for j := 0; j < activeCount; j++ {
				if j == i {
					continue
				}
				other := tm.Cars[j]
				if math.Abs(other.Speed) >= 3.0 {
					continue
				}
				dist := car.Pos.Distance(other.Pos)
				maxJamDist := (car.Length+other.Length)*0.5 + 35.0
				if dist < maxJamDist {
					toOther := other.Pos.Sub(car.Pos)
					cosH := math.Cos(car.Heading)
					sinH := math.Sin(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					if fwd > -15.0 || dist < (car.Length+other.Length)*0.5+10.0 {
						isJammed = true
						break
					}
				}
			}
		}

		var jammedAIBus *AIBus

		// 4. Check against AI route buses
		if !isJammed && tm.AIBusMgr != nil {
			for _, ab := range tm.AIBusMgr.Buses {
				if ab.Bus == nil || math.Abs(ab.Bus.Speed) >= 3.0 {
					continue
				}
				dist := car.Pos.Distance(ab.Bus.Pos)
				maxJamDist := (car.Length+ab.Bus.Length)*0.5 + 40.0
				if dist < maxJamDist {
					toOther := ab.Bus.Pos.Sub(car.Pos)
					cosH := math.Cos(car.Heading)
					sinH := math.Sin(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					lat := math.Abs(-toOther.X*sinH + toOther.Y*cosH)
					headDiff := math.Cos(car.Heading - ab.Bus.Heading)
					if !isNearIntersection && headDiff > 0.5 && fwd > 10.0 && lat < laneWidth*1.5 {
						continue
					}
					if fwd > -15.0 || dist < (car.Length+ab.Bus.Length)*0.5+15.0 {
						isJammed = true
						jammedAIBus = ab
						break
					}
				}
			}
		}

		if isJammed {
			car.DeadlockTimer += dt
		} else {
			if car.DeadlockTimer > 0 {
				car.DeadlockTimer = math.Max(0, car.DeadlockTimer-dt*2.0)
			}
		}

		// 5. Threshold determination:
		// Normal open-road deadlock threshold is DeadlockDurationThreshold (38.0s)
		// Near intersection or jammed with turning/stopping bus is IntersectionDeadlockThreshold (4.0s)
		threshold := DeadlockDurationThreshold
		if isNearIntersection {
			threshold = IntersectionDeadlockThreshold
		}

		if car.DeadlockTimer >= threshold {
			if jammedAIBus != nil {
				jammedAIBus.ResetToStartingStation(w)
			}
			if jammedWithPlayerBus && !isNearIntersection && tm.OnPlayerBusDeadlockReset != nil {
				tm.OnPlayerBusDeadlockReset()
			}
			tm.DespawnAndResetCar(i, w, laneWidth, bus)
			break // Resetting breaks the deadlock for others in the jam
		}
	}
}
