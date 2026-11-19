package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Legacy exported names share the full default timeout; intersections have no shortcut.
const DeadlockDurationThreshold = 2 * sim.DefaultSignalCycle
const IntersectionDeadlockThreshold = DeadlockDurationThreshold

// DespawnCar removes a car from active traffic
func (tm *TrafficManager) DespawnCar(index int) {
	if tm == nil || index < 0 || index >= len(tm.Cars) {
		return
	}
	tm.Cars = append(tm.Cars[:index], tm.Cars[index+1:]...)
	// Keep immutable geometry scratch with the same vehicle after slot shifts.
	// The owner check remains the fallback for untracked population edits.
	if index < len(tm.junctionScratch) {
		copy(tm.junctionScratch[index:], tm.junctionScratch[index+1:])
		tm.junctionScratch[len(tm.junctionScratch)-1] = carJunctionScratch{}
	}
}

// DespawnAndResetCar resets a deadlocked car by repositioning it to a clear section of road,
// or despawns it if no world or valid position is available.
func (tm *TrafficManager) DespawnAndResetCar(index int, w *sim.World, laneWidth float64, bus *sim.Bus) {
	if tm == nil || index < 0 || index >= len(tm.Cars) {
		return
	}
	car := tm.Cars[index]
	if tm.BeforeRecovery != nil {
		tm.BeforeRecovery(sim.TrafficRecoveryEvent{Kind: "car", ID: car.ID, Pos: car.Pos, Reason: "deadlock-reset"})
	}
	if w == nil || len(w.Roads) == 0 {
		tm.DespawnCar(index)
		return
	}

	// Try to find a clear road position away from intersections, stops, roundabouts, bus, and other cars
	resetSuccess := false
	for attempt := 0; attempt < 25; attempt++ {
		rIdx := sim.RandomIntn(len(w.Roads))
		r := w.Roads[rIdx]
		if attempt < 16 && !preferTrafficRoad(r) {
			continue
		}
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

		numLanesPerDir := r.GeneralTrafficLanes()

		lane := r.FirstGeneralTrafficLane() + sim.RandomIntn(numLanesPerDir)
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

		if tooCloseToForcedLeftBend(w, testPos, heading, roadCoord, isEW, lane-r.FirstGeneralTrafficLane(), laneWidth) {
			continue
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

		for _, rb := range w.Roundabouts {
			if testPos.Distance(rb.Center) < rb.YieldLineDistance()+60 {
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
		car.DeadlockTracker = sim.StationaryTracker{}
		car.IsBraking = false
		car.PassRetreatActive = false
		car.JunctionReverseActive = false
		car.JunctionReverseYieldBus = false
		car.JunctionReverseYieldID = 0
		car.JunctionTurn = nil
		car.Passing = false
		car.PassingLeadBus = nil
		car.YieldingToBus = nil
		car.RoundaboutCurve = nil
		car.InRoundabout = false
		car.RoundaboutCooldown = 0
		resetSuccess = true
		break
	}

	if !resetSuccess {
		tm.DespawnCar(index)
	}
}

func (tm *TrafficManager) ResolveDeadlocks(dt float64, bus *sim.Bus, activeCount int, laneWidth float64, w *sim.World, signals ...*sim.TrafficLightSystem) {
	var tls *sim.TrafficLightSystem
	if len(signals) > 0 {
		tls = signals[0]
	}
	tm.resolveDeadlocks(dt, bus, activeCount, laneWidth, w, tls, false)
}

func (tm *TrafficManager) resolveDeadlocks(dt float64, bus *sim.Bus, activeCount int, laneWidth float64, w *sim.World, tls *sim.TrafficLightSystem, indexReady bool) {
	if tm == nil || len(tm.Cars) == 0 {
		return
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}

	indexed := tm.prepareRecoveryQueries(activeCount, w, tls, indexReady)
	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]

		// 0. Ring motion re-derives its radius from the actual position after
		// collision separation. A car pushed well off the carriageway by an
		// approach queue then "circulates" through that queue on a large
		// circle, never reaching its exit, while still counting as inside
		// the roundabout. That state has no valid road position: replace it.
		if roundaboutCarDisplaced(car) {
			if tm.BeforeRecovery != nil {
				tm.BeforeRecovery(sim.TrafficRecoveryEvent{Kind: "car", ID: car.ID, Pos: car.Pos, Reason: "roundabout-displaced"})
			}
			tm.queueCarReplacement(i)
			break
		}

		// 1. Moving vehicles and cars whose controls just released the brake
		// are not deadlocked. Clear a saved timeout on the first acceleration tick.
		if car.ReverseRemaining > 0 || car.ReverseHold > 0 {
			car.DeadlockTimer = 0
			continue
		}
		if math.Abs(car.Speed) >= 3.0 || (!car.IsBraking && car.Speed > .5) {
			car.DeadlockTimer = 0
			continue
		}
		if yielding := car.YieldingToBus; yielding != nil && ((yielding.StationMergeRequest != nil && yielding.StationMergeRequest.Remaining > 0) || (yielding.LaneMergeRequest != nil && yielding.LaneMergeRequest.Remaining > 0)) {
			car.DeadlockTimer = 0
			continue
		}

		// Detect if car is near an intersection (within 130px of intersection center)
		isNearIntersection := false
		if indexed {
			isNearIntersection = tm.recoveryQueries.nearIntersection(car.Pos)
		} else if w != nil {
			for _, inter := range w.Intersections {
				if car.Pos.Distance(inter) < 130.0 {
					isNearIntersection = true
					break
				}
			}
		}

		isJammed := false

		// 2. Check relation to player bus
		if bus != nil && math.Abs(bus.Speed) < 8.0 {
			// The player has the same articulated geometry as a fleet bus.
			// A car beside its trailer can be far from the powered section;
			// measuring only the cab prevents its checked reverse from starting.
			section := bus.Pos
			distToBus := car.Pos.Distance(section)
			for _, part := range bus.BodyParts() {
				if part.HalfLen > 0 {
					if distance := car.Pos.Distance(part.Center); distance < distToBus {
						section, distToBus = part.Center, distance
					}
				}
			}
			if distToBus < 180.0 {
				toBus := section.Sub(car.Pos)
				sinH, cosH := math.Sincos(car.Heading)
				fwdBus := toBus.X*cosH + toBus.Y*sinH
				latBus := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
				headDiff := math.Cos(car.Heading - bus.Heading)
				// Protect aligned queues on open roads. A diagonal merging bus can
				// be mutually blocked with this car; exempting a 60-degree cone
				// resets its timer forever (also after loading a saved jam).
				queueCos := 0.992
				if bus.DoorsOpen || bus.PassengerServiceRemaining() > 0 {
					queueCos = 0.98
				}
				if !isNearIntersection && headDiff > queueCos && fwdBus > 10.0 && latBus < laneWidth*1.5 &&
					(!bus.IsArticulated() || (math.Abs(sim.NormalizeAngle(bus.Heading-bus.TailHeading())) <= 0.12 && math.Cos(car.Heading-bus.TailHeading()) > queueCos)) {
					car.DeadlockTimer = 0
					continue
				}
				// If car is jammed against player bus (crosswise, oncoming, turning, or trapped against front bumper):
				maxJamDist := (car.Length+bus.Length)*0.5 + 40.0
				if distToBus < maxJamDist && (fwdBus > -15.0 || distToBus < (car.Length+bus.Length)*0.5+15.0) && carBusDeadlockCorridor(car, bus) {
					isJammed = true
				}
			}
		}

		// 3. Check against other civilian AI cars
		if !isJammed {
			ids, local := tm.recoveryCandidates(car, indexed)
			count := activeCount
			if local {
				count = len(ids)
			}
			for candidate := 0; candidate < count; candidate++ {
				j := candidate
				if local {
					j = ids[candidate]
				}
				if j == i {
					continue
				}
				other := tm.Cars[j]
				if math.Abs(other.Speed) >= 3.0 {
					continue
				}
				dist := car.Pos.Distance(other.Pos)
				maxJamDist := (car.Length+other.Length)*0.5 + 35.0
				if dist < maxJamDist && carDeadlockCorridor(car, other) {
					toOther := other.Pos.Sub(car.Pos)
					sinH, cosH := math.Sincos(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					if fwd > -15.0 || dist < (car.Length+other.Length)*0.5+10.0 {
						isJammed = true
						break
					}
				}
			}
		}

		// 4. Check against AI route buses
		if !isJammed && tm.AIBusMgr != nil {
			for _, ab := range tm.AIBusMgr.Buses {
				if ab.Bus == nil || math.Abs(ab.Bus.Speed) >= 3.0 {
					continue
				}
				maxJamDist := (car.Length+ab.Bus.Length)*0.5 + 40.0
				broadLimit := maxJamDist + ab.Bus.OverallLength()
				d0 := ab.Bus.Pos.Sub(car.Pos)
				if d0.Dot(d0) >= broadLimit*broadLimit {
					continue
				}
				// A trailer blocks a lane as solidly as the powered section:
				// measure from the nearest body section.
				section := ab.Bus.Pos
				bestDistSq := d0.Dot(d0)
				for _, part := range ab.Bus.BodyParts() {
					if part.HalfLen > 0 {
						dp := part.Center.Sub(car.Pos)
						if dSq := dp.Dot(dp); dSq < bestDistSq {
							section = part.Center
							bestDistSq = dSq
						}
					}
				}
				if bestDistSq < maxJamDist*maxJamDist {
					dist := math.Sqrt(bestDistSq)
					toOther := section.Sub(car.Pos)
					sinH, cosH := math.Sincos(car.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					lat := math.Abs(-toOther.X*sinH + toOther.Y*cosH)
					headDiff := math.Cos(car.Heading - ab.Bus.Heading)
					// Match the player-bus queue rule: crossing a lane is not
					// an aligned queue behind a stopped bus.
					queueCos := 0.992
					if ab.Bus.DoorsOpen || ab.Bus.PassengerServiceRemaining() > 0 {
						queueCos = 0.98
					}
					if !isNearIntersection && headDiff > queueCos && fwd > 10.0 && lat < laneWidth*1.5 &&
						(!ab.Bus.IsArticulated() || (math.Abs(sim.NormalizeAngle(ab.Bus.Heading-ab.Bus.TailHeading())) <= 0.12 && math.Cos(car.Heading-ab.Bus.TailHeading()) > queueCos)) {
						continue
					}
					if (fwd > -15.0 || dist < (car.Length+ab.Bus.Length)*0.5+15.0) && carBusDeadlockCorridor(car, ab.Bus) {
						isJammed = true
						break
					}
				}
			}
		}

		car.DeadlockTimer = car.DeadlockTracker.Update(car.DeadlockTimer, dt, car.Pos, car.Speed, isJammed)
		timeout := 0.0
		if indexed {
			timeout = tm.recoveryQueries.timeoutAt(i, car.Pos)
		} else {
			timeout = tls.DeadlockTimeoutAt(car.Pos)
		}
		if car.DeadlockTimer >= timeout {
			if tm.BeforeRecovery != nil {
				tm.BeforeRecovery(sim.TrafficRecoveryEvent{Kind: "car", ID: car.ID, Pos: car.Pos, Reason: "deadlock-reset"})
			}
			tm.queueCarReplacement(i)
			break
		}
	}
}

// Adjacent queues can be close in Euclidean distance without blocking each
// other. Project the other body's width onto this car's lateral axis so that
// parallel lanes stay distinct while crossing and oncoming wedges still count.
func carDeadlockCorridor(car, other *sim.TrafficCar) bool {
	u := sim.UnitFromAngle(car.Heading)
	d := other.Pos.Sub(car.Pos)
	yaw := other.Heading - car.Heading
	// A clean nose-to-tail queue inherits the leader's wait. Removing its
	// followers cannot resolve a blocked junction and needlessly churns cars.
	// Apply this symmetrically: a close but non-overlapping rear follower must
	// not make the leading car look wedged either.
	otherHalfLength := other.Length/2*math.Abs(math.Cos(yaw)) + other.Width/2*math.Abs(math.Sin(yaw))
	queueCos := .88
	if car.JunctionTurn != nil {
		queueCos = .60
	}
	if math.Cos(yaw) > queueCos && math.Abs(d.Dot(u)) > car.Length/2+otherHalfLength+.5 {
		return false
	}
	return obbInCarDeadlockCorridor(car, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2})
}

// carBusDeadlockCorridor prevents a stopped car from counting an opposing or
// laterally separated bus across another lane—or a same-direction bus queued
// cleanly ahead of or behind the car—as deadlocking it.
func carBusDeadlockCorridor(car *sim.TrafficCar, bus *sim.Bus) bool {
	u := sim.UnitFromAngle(car.Heading)
	allAhead := true
	anyAligned := false
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		d := part.Center.Sub(car.Pos)
		yaw := part.Heading - car.Heading
		partHalfLength := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
		if math.Cos(yaw) > .85 && d.Dot(u) < -(car.Length/2+partHalfLength+.5) {
			allAhead = false
			continue
		}
		sinY, cosY := math.Sincos(yaw)
		partHalfWidth := part.HalfLen*math.Abs(sinY) + part.HalfWid*math.Abs(cosY)
		if cosY > .95 && math.Abs(d.X*u.Y-d.Y*u.X) > car.Width/2+partHalfWidth+0.5 {
			allAhead = false
			continue
		}
		if cosY > .95 && d.Dot(u) > car.Length/2+partHalfLength+.5 {
			anyAligned = true
			continue
		}
		allAhead = false
		if math.Abs(d.Dot(u)) <= car.Length/2+partHalfLength+40 && obbInCarDeadlockCorridor(car, part) {
			return true
		}
	}
	if allAhead && anyAligned {
		return false
	}
	return false
}

func obbInCarDeadlockCorridor(car *sim.TrafficCar, part sim.OBB) bool {
	u := sim.UnitFromAngle(car.Heading)
	d := part.Center.Sub(car.Pos)
	yaw := part.Heading - car.Heading
	sinY, cosY := math.Sincos(yaw)
	partHalfWidth := part.HalfLen*math.Abs(sinY) + part.HalfWid*math.Abs(cosY)
	return math.Abs(d.X*u.Y-d.Y*u.X) <= car.Width/2+partHalfWidth+8
}

func roundaboutCarDisplaced(car *sim.TrafficCar) bool {
	return car.InRoundabout && car.RoundaboutCurve == nil && car.RoundaboutOuterRadius > 0 &&
		car.Pos.Distance(car.RoundaboutCenter) > car.RoundaboutOuterRadius+car.Length+24
}
