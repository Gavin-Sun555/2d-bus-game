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
	cosH := math.Cos(f.car.Heading)
	sinH := math.Sin(f.car.Heading)
	carHalfLen := f.car.Length * 0.5 // ~24.0
	carHalfWid := f.car.Width * 0.5  // ~11.0

	lookahead := math.Max(260, f.car.Speed*f.car.Speed/(2*220)+100)
	for _, b := range f.buses {
		if gap, ok := sim.JunctionLeaderGap(f.car.JunctionCorridor, b.JunctionCorridor, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}, f.car.Length, f.car.Width); ok {
			targetSpd = math.Min(targetSpd, sim.JunctionFollowingSpeed(gap, b.Speed, f.car.Speed, 220))
			continue
		}
		distToBus := f.car.Pos.Distance(b.Pos)
		if distToBus >= lookahead {
			continue
		}

		if canPassWaitingDepartureBus(f, b) {
			continue
		}

		// If bus is safely parked inside an off-road bus stop bay, traffic vehicles in travel lanes drive past normally
		if f.world != nil {
			isParkedInBay := false
			for _, s := range f.world.Stops {
				if s.ContainsBus(b) {
					isParkedInBay = true
					break
				}
			}
			if isParkedInBay && busClearsCarLane(f.car, b) {
				continue
			}
		}

		isBusStopped := math.Abs(b.Speed) < 8.0
		isWrongWay := sim.IsBusFacingWrongWay(b)

		// Driving corridor half-width for the car
		corridorW := carHalfWid + 5.0
		if isBusStopped {
			corridorW = carHalfWid + 7.0
		}

		// 4 corners of the bus in world coordinates
		cosB := math.Cos(b.Heading)
		sinB := math.Sin(b.Heading)
		hLenB := b.Length * 0.5 // 48.0
		hWidB := b.Width * 0.5  // 18.0
		fwdB := sim.Vec2{X: cosB, Y: sinB}
		latB := sim.Vec2{X: -sinB, Y: cosB}
		corners := [4]sim.Vec2{
			b.Pos.Add(fwdB.Mul(hLenB)).Add(latB.Mul(hWidB)),
			b.Pos.Add(fwdB.Mul(hLenB)).Sub(latB.Mul(hWidB)),
			b.Pos.Sub(fwdB.Mul(hLenB)).Sub(latB.Mul(hWidB)),
			b.Pos.Sub(fwdB.Mul(hLenB)).Add(latB.Mul(hWidB)),
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
		minFwdAhead := 9999.0
		hasObstacleAhead := false

		checkPoint := func(fwd, lat float64) {
			if math.Abs(lat) <= corridorW {
				// Must be strictly ahead of the car's front bumper!
				// Vehicles alongside or behind the car must never cause it to brake.
				if fwd > carHalfLen && fwd < lookahead {
					if fwd < minFwdAhead {
						minFwdAhead = fwd
						hasObstacleAhead = true
					}
				}
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

		if hasObstacleAhead {
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
					decelSpd := math.Min(math.Max(0, lead+(bumperDist-20)*.8), math.Sqrt(lead*lead+2*220*math.Max(0, bumperDist-20)))
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

	return targetSpd
}

func (tm *TrafficManager) carFollowingSpeed(f carFrame, targetSpd float64) float64 {
	if f.car.InRoundabout {
		return targetSpd
	}
	u := sim.Vec2{X: math.Cos(f.car.Heading), Y: math.Sin(f.car.Heading)}
	for j, other := range tm.Cars {
		if j == f.index || j >= f.activeCount || other.InRoundabout {
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
