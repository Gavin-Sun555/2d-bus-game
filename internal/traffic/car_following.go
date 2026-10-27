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
	for busIndex, b := range f.buses {
		body := b.BodyEnvelope()
		if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, b.JunctionCorridor, body, f.car.Length, f.car.Width); ok {
			targetSpd = math.Min(targetSpd, sim.JunctionFollowingSpeed(gap, b.Speed, f.car.Speed, 220))
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

		if carClearsStoppedBusAhead(f.car, b) || committedThroughCarClearsStoppedTurn(f.car, b) {
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

func (tm *TrafficManager) carFollowingSpeed(f carFrame, targetSpd float64) float64 {
	if f.car.InRoundabout {
		return targetSpd
	}
	u := sim.UnitFromAngle(f.car.Heading)
	for _, other := range tm.followingCars(f) {
		if other == f.car || other.InRoundabout {
			continue
		}
		if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, other.JunctionCorridor, sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}, f.car.Length, f.car.Width); ok {
			targetSpd = math.Min(targetSpd, sim.JunctionFollowingSpeed(gap, other.Speed, f.car.Speed, 220))
			continue
		}
		delta := other.Pos.Sub(f.car.Pos)
		forward := delta.Dot(u)
		if forward <= f.car.Length/2 {
			continue
		}
		yaw := other.Heading - f.car.Heading
		extent := other.Length/2*math.Abs(math.Sin(yaw)) + other.Width/2*math.Abs(math.Cos(yaw))
		lateral := math.Abs(-delta.X*u.Y + delta.Y*u.X)
		if lateral > f.car.Width/2+extent+5 {
			continue
		}
		alongExtent := other.Length/2*math.Abs(math.Cos(yaw)) + other.Width/2*math.Abs(math.Sin(yaw))
		gap := forward - f.car.Length/2 - alongExtent
		lead := math.Max(0, other.Speed*math.Cos(yaw))
		closing := math.Max(0, f.car.Speed-lead)
		if gap > math.Max(90, closing*closing/(2*220)+closing*.5+30) {
			continue
		}
		// While turning, the tangent ray can point into a stopped adjacent
		// through queue even though the committed curve bends away from it.
		if math.Abs(other.Speed) < 1 && committedCarPathClear(f.car, other) {
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

// Only a committed junction path can disprove the straight following ray.
// Moving crossing traffic still uses the conservative following/admission rules.
func committedCarPathClear(car, other *sim.TrafficCar) bool {
	turn := car.JunctionTurn
	if turn == nil || turn.LaneChange || turn.Straight {
		return false
	}
	obstacle := sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}
	horizon := math.Max(120, car.Speed*car.Speed/(2*220)+math.Abs(car.Speed)*.3+50)
	for distance := 0.; distance <= horizon; distance += 6 {
		pos, heading := turn.Predict(distance)
		body := sim.OBB{Center: pos, Heading: heading, HalfLen: car.Length / 2, HalfWid: car.Width / 2}
		if hit, _, _ := sim.CheckOBBCollision(body, obstacle); hit {
			return false
		}
	}
	return true
}

// A car whose nose already leads a stopped, almost parallel bus can clear a
// closed merge gap. Prove clearance along its actual committed path, including
// both bus sections, rather than treating a side safety margin as a leader.
func carClearsStoppedBusAhead(car *sim.TrafficCar, bus *sim.Bus) bool {
	if math.Abs(bus.Speed) > .1 || bus.StationMergeRequest != nil || (bus.LaneMergeRequest != nil && (car.JunctionTurn == nil || car.JunctionTurn.LaneChange || !car.JunctionTurn.Straight)) || car.InRoundabout || math.Cos(bus.Heading-car.Heading) < .95 {
		return false
	}
	u := sim.UnitFromAngle(car.Heading)
	if car.Pos.Sub(bus.Pos).Dot(u) < car.Length/2 {
		return false
	}
	for distance := 0.; distance <= 120; distance += 2 {
		pos, heading := car.Pos.Add(u.Mul(distance)), car.Heading
		if car.JunctionTurn != nil {
			pos, heading = car.JunctionTurn.Predict(distance)
		}
		body := sim.OBB{Center: pos, Heading: heading, HalfLen: car.Length/2 + 2, HalfWid: car.Width/2 + 2}
		if hit, _, _ := sim.CheckBusAtBody(bus, bus.BodyParts()[0], body); hit {
			return false
		}
	}
	return true
}

// A committed through car clears a stopped oncoming turn before that turn
// resumes. A generous straight following margin must not freeze both vehicles
// when their actual bodies are separated throughout the car's crossing.
func committedThroughCarClearsStoppedTurn(car *sim.TrafficCar, bus *sim.Bus) bool {
	turn := car.JunctionTurn
	corridor := bus.JunctionCorridor
	if turn == nil || turn.LaneChange || !turn.Straight || car.InRoundabout || math.Abs(bus.Speed) > .1 || corridor == nil || corridor.Remaining <= 0 || corridor.Center.Distance(turn.Center) > 1 || math.Cos(bus.Heading-car.Heading) > -.9 {
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
