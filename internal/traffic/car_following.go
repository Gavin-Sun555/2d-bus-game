package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (tm *TrafficManager) busFollowingSpeed(f carFrame, targetSpd float64) float64 {
	// B. Check Buses ahead (Player Bus & AI Route Buses)
	if f.car.InRoundabout {
		return targetSpd
	}
	sinH, cosH := math.Sincos(f.car.Heading)
	carHalfLen := f.car.Length * 0.5 // ~24.0
	carHalfWid := f.car.Width * 0.5  // ~11.0

	lookahead := math.Max(260, f.car.Speed*f.car.Speed/(2*220)+100)
	ownCorridorActive := f.car.JunctionCorridor != nil && f.car.JunctionCorridor.Remaining > 0 && len(f.car.JunctionCorridor.Bodies) >= 2
	for busIndex, b := range f.buses {
		if !ownCorridorActive || b.JunctionCorridor == nil || b.JunctionCorridor.Remaining <= 0 {
			d := f.car.Pos.Sub(b.Pos)
			reach := lookahead + b.OverallLength()
			if d.Dot(d) >= reach*reach {
				continue
			}
		}
		body := b.BodyEnvelope()
		// A checked, physically clear exit takes precedence over a loose
		// corridor leader projection. Otherwise that earlier zero survives
		// even after the actual-body following rule grants the car its exit.
		delta := f.car.Pos.Sub(body.Center)
		if delta.Dot(delta) < lookahead*lookahead {
			if carClearsStoppedBusAhead(f.car, b) || recoveryLaneChangeClearsStoppedBus(f.car, b) || committedCarClearsStoppedTurn(f.car, b) {
				continue
			}
			if carClearsParallelStoppedBus(f.car, b) || carClearsStoppedBusAcrossDivider(f, b) {
				targetSpd = math.Min(targetSpd, 45)
				continue
			}
		}
		if ownCorridorActive {
			if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, b.JunctionCorridor, body, f.car.Length, f.car.Width); ok {
				targetSpd = math.Min(targetSpd, sim.JunctionFollowingSpeed(gap, b.Speed, f.car.Speed, 220))
			}
		}
		deltaToBus := f.car.Pos.Sub(body.Center)
		if deltaToBus.Dot(deltaToBus) >= lookahead*lookahead {
			continue
		}

		if canPassWaitingDepartureBus(f, b) {
			continue
		}

		// If bus is safely parked inside an off-road bus stop bay, traffic vehicles in travel lanes drive past normally
		if f.world != nil {
			isParkedInBay := false
			if busIndex < len(f.parkedBuses) {
				isParkedInBay = f.parkedBuses[busIndex]
			} else {
				for _, s := range f.world.Stops {
					if s.ContainsBus(b) {
						isParkedInBay = true
						break
					}
				}
			}
			if isParkedInBay && busClearsCarLane(f.car, b) {
				continue
			}
		}

		if carClearsStoppedBusAhead(f.car, b) || recoveryLaneChangeClearsStoppedBus(f.car, b) || committedCarClearsStoppedTurn(f.car, b) {
			continue
		}

		isBusStopped := math.Abs(b.Speed) < 8.0
		isWrongWay := sim.IsBusFacingWrongWay(b)

		// Driving corridor half-width for the car
		corridorW := carHalfWid + 5.0
		if isBusStopped {
			corridorW = carHalfWid + 7.0
		}

		// Clip each physical section against the car travel corridor.
		// An articulated envelope can occupy empty space between the sections.
		for _, section := range b.BodyParts() {
			if section.HalfLen <= 0 {
				continue
			}
			sinB, cosB := math.Sincos(section.Heading)
			hLenB := section.HalfLen
			hWidB := section.HalfWid
			fwdB := sim.Vec2{X: cosB, Y: sinB}
			latB := sim.Vec2{X: -sinB, Y: cosB}
			corners := [4]sim.Vec2{
				section.Center.Add(fwdB.Mul(hLenB)).Add(latB.Mul(hWidB)),
				section.Center.Add(fwdB.Mul(hLenB)).Sub(latB.Mul(hWidB)),
				section.Center.Sub(fwdB.Mul(hLenB)).Sub(latB.Mul(hWidB)),
				section.Center.Sub(fwdB.Mul(hLenB)).Add(latB.Mul(hWidB)),
			}

			// Transform corners into car's local coordinate frame:
			// fwd = distance along car's heading from car.Pos
			// lat = signed lateral offset perpendicular to car's heading
			type localPoint struct {
				fwd float64
				lat float64
			}
			localCorners := [4]localPoint{}
			for k := 0; k < 4; k++ {
				dx := corners[k].X - f.car.Pos.X
				dy := corners[k].Y - f.car.Pos.Y
				localCorners[k] = localPoint{
					fwd: dx*cosH + dy*sinH,
					lat: -dx*sinH + dy*cosH,
				}
			}

			// Find minimum forward distance of any part of the bus inside the car's forward travel corridor:
			// By convex geometry, the minimum fwd within |lat| <= corridorW must occur at a corner inside the corridor,
			// or at an intersection of a bus edge with the corridor boundaries lat = +corridorW or lat = -corridorW.
			minFwdAhead, maxFwdAhead := math.Inf(1), math.Inf(-1)
			checkPoint := func(fwd, lat float64) {
				if math.Abs(lat) <= corridorW+1e-8 {
					minFwdAhead = math.Min(minFwdAhead, fwd)
					maxFwdAhead = math.Max(maxFwdAhead, fwd)
				}
			}

			// 1. Check all 4 bus corners
			for k := 0; k < 4; k++ {
				checkPoint(localCorners[k].fwd, localCorners[k].lat)
			}

			// 2. Check all 4 bus edges intersecting the corridor boundaries
			for k := 0; k < 4; k++ {
				pA := localCorners[k]
				pB := localCorners[(k+1)%4]
				dLat := pB.lat - pA.lat
				if math.Abs(dLat) > 1e-4 {
					t1 := (corridorW - pA.lat) / dLat
					if t1 > 0 && t1 < 1 {
						fwd1 := pA.fwd + t1*(pB.fwd-pA.fwd)
						checkPoint(fwd1, corridorW)
					}
					t2 := (-corridorW - pA.lat) / dLat
					if t2 > 0 && t2 < 1 {
						fwd2 := pA.fwd + t2*(pB.fwd-pA.fwd)
						checkPoint(fwd2, -corridorW)
					}
				}
			}

			if maxFwdAhead > -carHalfLen && minFwdAhead < lookahead {
				bumperDist := minFwdAhead - carHalfLen

				if isBusStopped && !isWrongWay {
					// Lawfully stopped bus directly ahead in our travel corridor
					if f.world != nil && math.Cos(b.Heading-f.car.Heading) > .8 {
						bumper := f.car.Pos.Add(sim.Vec2{X: cosH * carHalfLen, Y: sinH * carHalfLen})
						if railDist, clamped := f.world.ClampStopBeforeRailCrossing(bumper, f.car.Heading, bumperDist-11, f.car.Length, 14); clamped {
							bumperDist = railDist + 11
						}
					}
					if bumperDist <= 28.0 {
						targetSpd = 0
						f.car.IsBraking = true
						if bumperDist <= 12.0 && f.car.Speed > 0 {
							f.car.Speed = 0
						}
					} else {
						f.car.IsBraking = true
						decelSpd := math.Min(math.Max(0, (bumperDist-25)*1.8), math.Sqrt(2*220*math.Max(0, bumperDist-25)))
						targetSpd = math.Min(targetSpd, decelSpd)
					}
				} else if !isWrongWay {
					// Moving bus in legal flow directly ahead: adaptive cruise following
					if bumperDist <= 20.0 {
						targetSpd = 0
						f.car.IsBraking = true
						if bumperDist <= 8.0 && f.car.Speed > 0 {
							f.car.Speed = 0
						}
					} else {
						f.car.IsBraking = true
						lead := math.Max(0, b.Speed*math.Cos(b.Heading-f.car.Heading))
						desiredGap := 20.
						if b.IsArticulated() {
							// A constant 20px moving gap leaves no reaction room
							// behind the long rear section when its leader brakes.
							desiredGap = 30 + .35*math.Max(0, f.car.Speed)
						}
						decelSpd := math.Min(math.Max(0, lead+(bumperDist-desiredGap)*.8), math.Sqrt(lead*lead+2*220*math.Max(0, bumperDist-20)))
						targetSpd = math.Min(targetSpd, decelSpd)
					}
				} else {
					// Bus is driving wrong-way (逆行) into oncoming traffic:
					if bumperDist <= 15.0 {
						targetSpd = 0
						f.car.IsBraking = true
					} else if bumperDist < 90.0 {
						targetSpd = math.Min(targetSpd, math.Max(0, (bumperDist-15.0)*1.5))
						f.car.IsBraking = true
					}
				}
			}
		}
	}

	return targetSpd
}

func carClearsStoppedBusAcrossDivider(f carFrame, bus *sim.Bus) bool {
	car := f.car
	if f.world == nil || car.JunctionTurn != nil || car.YieldingToBus == bus || math.Abs(bus.Speed) > .1 ||
		math.Cos(bus.Heading-car.Heading) < .9 || bus.DoorsOpen || bus.LaneMergeRequest != nil {
		return false
	}
	forward := sim.UnitFromAngle(car.Heading)
	if math.Max(math.Abs(forward.X), math.Abs(forward.Y)) < .999 {
		return false
	}
	delta := car.Pos.Sub(bus.Pos)
	if along := delta.Dot(forward); along < 0 || along >= car.Length/2 {
		return false
	}
	if _, ok := f.world.RoadAt(car.RoadCenter, car.IsEW, car.Pos); !ok || f.world.InsideJunction(car.Pos, f.laneWidth, 0) {
		return false
	}
	origin := sim.Vec2{X: car.RoadCenter}
	if car.IsEW {
		origin = sim.Vec2{Y: car.RoadCenter}
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	carSide, busSide := car.Pos.Sub(origin).Dot(right), bus.Pos.Sub(origin).Dot(right)
	// The car already overlaps the stopped bus longitudinally but has a
	// clear lane of travel. Let it clear the gap needed for the bus to return.
	return carSide >= car.Width/2 && busSide < 0 && busSide > -bus.Width/2 && carPathClearsBus(car, bus, .25)
}

func (tm *TrafficManager) carFollowingSpeed(f carFrame, targetSpd float64) float64 {
	if f.car.InRoundabout {
		return targetSpd
	}
	u := sim.UnitFromAngle(f.car.Heading)
	hasCorridor := f.car.JunctionCorridor != nil && f.car.JunctionCorridor.Remaining > 0 && len(f.car.JunctionCorridor.Bodies) >= 2
	for _, other := range tm.followingCars(f) {
		if other == f.car || (other.InRoundabout && (other.RoundaboutCurve == nil || other.RoundaboutCurve.Exiting)) {
			continue
		}
		if hasCorridor && other.JunctionCorridor != nil && other.JunctionCorridor.Remaining > 0 && len(other.JunctionCorridor.Bodies) >= 2 {
			if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, other.JunctionCorridor, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}, f.car.Length, f.car.Width); ok {
				targetSpd = math.Min(targetSpd, sim.JunctionFollowingSpeed(gap, other.Speed, f.car.Speed, 220))
				continue
			}
		}
		delta := other.Pos.Sub(f.car.Pos)
		forward := delta.Dot(u)
		if forward <= f.car.Length/2 {
			continue
		}
		lateral := math.Abs(-delta.X*u.Y + delta.Y*u.X)
		if lateral > f.car.Width*0.5+(other.Length+other.Width)*0.5+5 {
			continue
		}
		var extent, alongExtent, cosYaw float64
		if other.Heading == f.car.Heading {
			extent = other.Width * .5
			alongExtent = other.Length * .5
			cosYaw = 1
		} else {
			yaw := other.Heading - f.car.Heading
			sinY, cosY := math.Sincos(yaw)
			absSinY, absCosY := math.Abs(sinY), math.Abs(cosY)
			extent = other.Length/2*absSinY + other.Width/2*absCosY
			alongExtent = other.Length/2*absCosY + other.Width/2*absSinY
			cosYaw = cosY
		}
		if lateral > f.car.Width/2+extent+5 {
			continue
		}
		gap := forward - f.car.Length/2 - alongExtent
		lead := math.Max(0, other.Speed*cosYaw)
		if lead < 15 && f.world != nil && cosYaw > .8 {
			bumper := f.car.Pos.Add(u.Mul(f.car.Length / 2))
			if railDist, clamped := f.world.ClampStopBeforeRailCrossing(bumper, f.car.Heading, gap-4, f.car.Length, 14); clamped {
				gap = railDist + 4
				lead = 0
			}
		}
		closing := math.Max(0, f.car.Speed-lead)
		if gap > math.Max(90, closing*closing/(2*220)+closing*.5+30) {
			continue
		}
		// The tangent ray can point into a stopped source-lane queue even
		// though the committed turn or lane-change curve bends away from it.
		if math.Abs(other.Speed) < 1 && committedCarPathClear(f.car, other) {
			if f.car.JunctionTurn.LaneChange {
				targetSpd = math.Min(targetSpd, carRecoveryLaneChangeSpeed)
			}
			continue
		}
		allowed := math.Min(math.Max(0, lead+.9*(gap-18)), math.Sqrt(lead*lead+2*220*math.Max(0, gap-18)))
		if gap < 12 {
			allowed = 0
			if gap < 5 {
				f.car.Speed = 0
			}
		}
		if allowed < targetSpd {
			f.car.IsBraking = true
			targetSpd = allowed
		}
	}
	return targetSpd
}

// A committed curve can disprove the straight following ray.
// Moving crossing traffic still uses the conservative following/admission rules.
func committedCarPathClear(car, other *sim.TrafficCar) bool {
	turn := car.JunctionTurn
	if turn == nil {
		return false
	}
	pad := 0.0
	if turn.LaneChange {
		pad = 1
	} else if turn.Straight {
		if math.Abs(math.Cos(other.Heading-car.Heading)) >= .35 {
			return false
		}
		pad = 1.5
	}
	obstacle := sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}
	horizon := math.Max(120, car.Speed*car.Speed/(2*220)+math.Abs(car.Speed)*.3+50)
	for distance := 0.; distance <= horizon; distance += 6 {
		pos, heading := turn.Predict(distance)
		body := sim.OBB{Center: pos, Heading: heading, HalfLen: car.Length/2 + pad, HalfWid: car.Width/2 + pad}
		if hit, _, _ := sim.CheckOBBCollision(body, obstacle); hit {
			return false
		}
	}
	return true
}

func busStoppedWithoutMerge(car *sim.TrafficCar, bus *sim.Bus) bool {
	stationArrival := bus.StationMergeRequest != nil && bus.StationMergeRequest.Remaining > 0
	laneMerge := bus.LaneMergeRequest != nil && !stationArrival
	if math.Abs(bus.Speed) > .1 || car.YieldingToBus == bus || (laneMerge && (car.JunctionTurn == nil || car.JunctionTurn.LaneChange || !car.JunctionTurn.Straight)) || car.InRoundabout {
		return false
	}
	return true
}

func carPathClearsBus(car *sim.TrafficCar, bus *sim.Bus, pad float64) bool {
	u := sim.UnitFromAngle(car.Heading)
	for distance := 0.; distance <= 120; distance += 2 {
		pos, heading := car.Pos.Add(u.Mul(distance)), car.Heading
		if car.JunctionTurn != nil {
			pos, heading = car.JunctionTurn.Predict(distance)
		}
		body := sim.OBB{Center: pos, Heading: heading, HalfLen: car.Length/2 + pad, HalfWid: car.Width/2 + pad}
		if hit, _, _ := bus.CollidesBody(body); hit {
			return false
		}
	}
	return true
}

// A car whose nose already leads a stopped, almost parallel bus can clear a
// closed merge gap. Prove clearance along its actual committed path, including
// both bus sections, rather than treating a side safety margin as a leader.
func carClearsStoppedBusAhead(car *sim.TrafficCar, bus *sim.Bus) bool {
	// A station arrival also publishes a lane request, but only traffic behind
	// the bus yields to it. A car already leading the bus must be free to leave,
	// otherwise the bus waits for that car while the car waits for the bus.
	if !busStoppedWithoutMerge(car, bus) || math.Cos(bus.Heading-car.Heading) < .75 {
		return false
	}
	u := sim.UnitFromAngle(car.Heading)
	if car.Pos.Sub(bus.Pos).Dot(u) < car.Length/2 {
		return false
	}
	return carPathClearsBus(car, bus, 1)
}

// carClearsParallelStoppedBus allows a car already alongside a stopped,
// same-direction bus's front section to creep past when its unpadded forward
// path is physically clear and not converging into the bus.
func carClearsParallelStoppedBus(car *sim.TrafficCar, bus *sim.Bus) bool {
	turn := car.JunctionTurn
	straightExit := turn != nil && !turn.LaneChange && turn.Segment == len(turn.Points)-2 && len(turn.Points) >= 2 &&
		math.Cos(car.Heading-turn.ExitHeading) > .999 && turn.Points[len(turn.Points)-1].Sub(turn.Points[len(turn.Points)-2]).Normalize().Dot(sim.UnitFromAngle(car.Heading)) > .999
	if (turn != nil && !turn.Straight && !straightExit) ||
		math.Abs(bus.Speed) > .1 || car.YieldingToBus == bus || car.InRoundabout {
		return false
	}
	u := sim.UnitFromAngle(car.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	rel := car.Pos.Sub(bus.Pos)
	if station := bus.StationMergeRequest; station != nil && station.Remaining > 0 {
		// An alongside car may clear a stationary station maneuver whose
		// requested lane is on the opposite side. Keep the arrival exception
		// ahead of the cab. For departure, the merge must still be waiting.
		lane := bus.LaneMergeRequest
		if lane == nil || lane.Remaining <= 0 ||
			rel.Dot(r)*lane.Target.Sub(bus.Pos).Dot(r) >= 0 ||
			math.Cos(bus.Heading-car.Heading) < .95 {
			return false
		}
		if (station.Departing && lane.Joined) || (!station.Departing && rel.Dot(u) <= 0) {
			return false
		}
	}
	if math.Cos(bus.Heading-car.Heading) < .95 {
		lane := bus.LaneMergeRequest
		// A stopped bus correcting into the turn lane can be too angled
		// for the parallel check. The car already ahead in the other lane
		// can still clear if the actual bus body is outside its path.
		if turn == nil && lane != nil && lane.LeftTurn && lane.Remaining > 0 && !lane.Joined &&
			math.Cos(lane.Heading-car.Heading) > .98 && math.Cos(bus.Heading-car.Heading) > .9 &&
			rel.Dot(u) >= car.Length/2 && rel.Dot(r)*lane.Target.Sub(bus.Pos).Dot(r) < 0 &&
			carPathClearsBus(car, bus, .5) {
			return true
		}
		corridor := bus.JunctionCorridor
		return turn != nil && !turn.LaneChange && corridor != nil && corridor.Remaining > 0 && corridor.Center.Distance(turn.Center) < 1 &&
			car.Pos.Distance(bus.Pos) < bus.OverallLength()/2+car.Length+30 && carPathClearsBus(car, bus, .5)
	}
	if rel.Dot(u) <= -bus.Length/2 {
		// The car may already be alongside a straight trailer. If the
		// bus is waiting for that lane, let the car leave along its actual
		// clear path instead of holding it against the cab's rear margin.
		lane := bus.LaneMergeRequest
		if turn != nil || !bus.IsArticulated() || lane == nil || lane.Remaining <= 0 || lane.Joined ||
			math.Cos(bus.Heading-car.Heading) < .999 || math.Cos(bus.TailHeading()-car.Heading) < .999 ||
			car.Pos.Sub(bus.TrailerBody().Center).Dot(u) <= -bus.Length/2 {
			return false
		}
	}
	lat := rel.Dot(r)
	busFwd := sim.UnitFromAngle(bus.Heading)
	converging := (lat > 0 && busFwd.Dot(r) > .01) || (lat < 0 && busFwd.Dot(r) < -.01)
	if converging {
		// A diagonal but stopped bus may be waiting for this leading car.
		// Its heading alone must not hold a car whose entire forward path
		// clears both sections. Keep followers yielding; the caller caps this
		// physically separated movement at a crawl, as for a parallel bus.
		return (rel.Dot(u) > 0 && carPathClearsBus(car, bus, 0)) || (rel.Dot(u) > -car.Length/4 && carPathClearsBus(car, bus, 2))
	}
	return carPathClearsBus(car, bus, 0)
}

func recoveryLaneChangeClearsStoppedBus(car *sim.TrafficCar, bus *sim.Bus) bool {
	turn := car.JunctionTurn
	if turn == nil || !turn.LaneChange || turn.SpeedLimit != carRecoveryLaneChangeSpeed || math.Abs(bus.Speed) >= 1 {
		return false
	}
	for distance := 0.; distance <= 120; distance += 3 {
		p, h := turn.Predict(distance)
		if hit, _, _ := bus.CollidesBody(sim.OBB{Center: p, Heading: h, HalfLen: car.Length / 2, HalfWid: car.Width / 2}); hit {
			return false
		}
	}
	return true
}

// A committed car can clear a stopped oncoming turn along its selected path.
// Check the whole curve rather than extending the current tangent into a
// fictitious straight following corridor. Physical sections retain their margin.
func committedCarClearsStoppedTurn(car *sim.TrafficCar, bus *sim.Bus) bool {
	turn := car.JunctionTurn
	corridor := bus.JunctionCorridor
	if turn == nil || turn.LaneChange || car.InRoundabout || math.Abs(bus.Speed) > .1 || corridor == nil || corridor.Remaining <= 0 || corridor.Center.Distance(turn.Center) > 1 || (turn.Straight && math.Cos(bus.Heading-car.Heading) > -.9) {
		return false
	}
	forward := sim.UnitFromAngle(car.Heading)
	horizon := bus.Pos.Sub(car.Pos).Dot(forward) + bus.OverallLength() + car.Length + 20
	if horizon <= 0 || horizon > 400 {
		return false
	}
	for distance := 0.; distance <= horizon; distance += 2 {
		pos, heading := turn.Predict(distance)
		body := sim.OBB{Center: pos, Heading: heading, HalfLen: car.Length/2 + .5, HalfWid: car.Width/2 + .5}
		for _, part := range bus.BodyParts() {
			if part.HalfLen > 0 {
				if hit, _, _ := sim.CheckOBBCollision(part, body); hit {
					return false
				}
			}
		}
	}
	return true
}
