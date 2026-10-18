package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func hasRoadInDirection(w *sim.World, center sim.Vec2, heading float64) bool {
	if w == nil {
		return true
	}
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	isEW := math.Abs(cosH) > math.Abs(sinH)

	for _, r := range w.Roads {
		roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		if roadIsEW != isEW {
			continue
		}
		if isEW {
			if math.Abs(r.P1.Y-center.Y) > 25.0 {
				continue
			}
			minX := math.Min(r.P1.X, r.P2.X)
			maxX := math.Max(r.P1.X, r.P2.X)
			if cosH > 0 { // East
				if maxX >= center.X+40.0 && minX <= center.X+15.0 {
					return true
				}
			} else { // West
				if minX <= center.X-40.0 && maxX >= center.X-15.0 {
					return true
				}
			}
		} else {
			if math.Abs(r.P1.X-center.X) > 25.0 {
				continue
			}
			minY := math.Min(r.P1.Y, r.P2.Y)
			maxY := math.Max(r.P1.Y, r.P2.Y)
			if sinH > 0 { // South
				if maxY >= center.Y+40.0 && minY <= center.Y+15.0 {
					return true
				}
			} else { // North
				if minY <= center.Y-40.0 && maxY >= center.Y-15.0 {
					return true
				}
			}
		}
	}
	return false
}

func (tm *TrafficManager) handleCarNavigation(car *sim.TrafficCar, laneWidth float64, w *sim.World) {
	if car.InRoundabout {
		return // Do not apply straight lane Lerp while circulating in roundabout!
	}

	minX, maxX := -2600.0, 2600.0
	minY, maxY := -1200.0, 1200.0
	if w != nil && len(w.Roads) > 0 {
		minX, maxX = 1e9, -1e9
		minY, maxY = 1e9, -1e9
		for _, r := range w.Roads {
			if r.P1.X < minX {
				minX = r.P1.X
			}
			if r.P2.X < minX {
				minX = r.P2.X
			}
			if r.P1.X > maxX {
				maxX = r.P1.X
			}
			if r.P2.X > maxX {
				maxX = r.P2.X
			}
			if r.P1.Y < minY {
				minY = r.P1.Y
			}
			if r.P2.Y < minY {
				minY = r.P2.Y
			}
			if r.P1.Y > maxY {
				maxY = r.P1.Y
			}
			if r.P2.Y > maxY {
				maxY = r.P2.Y
			}
		}
	}

	// Boundary loop wrapping strictly along road limits
	if car.IsEW {
		if car.Pos.X > maxX+30.0 {
			car.Pos.X = minX - 10.0
		} else if car.Pos.X < minX-30.0 {
			car.Pos.X = maxX + 10.0
		}
	} else {
		if car.Pos.Y > maxY+30.0 {
			car.Pos.Y = minY - 10.0
		} else if car.Pos.Y < minY-30.0 {
			car.Pos.Y = maxY + 10.0
		}
	}

	lanes := 2
	if w != nil {
		lanes = w.GetRoadLanes(car.RoadCenter, car.IsEW, car.Pos)
	} else {
		if car.IsEW {
			if car.RoadCenter == -1200 {
				lanes = 6
			} else if car.RoadCenter == 0 || car.RoadCenter == 1200 {
				lanes = 4
			}
		} else {
			if car.RoadCenter == 0 {
				lanes = 4
			}
		}
	}

	laneOffset := laneWidth * 0.5 // Standard 2-lane road: center of travel lane
	if lanes >= 6 {
		if car.LaneIndex == 0 {
			laneOffset = laneWidth * 0.5 // Inner lane (dedicated left turn)
		} else if car.LaneIndex == 1 {
			laneOffset = laneWidth * 1.5 // Middle lane (straight through)
		} else {
			laneOffset = laneWidth * 2.5 // Outer lane (straight through / slow lane)
		}
	} else if lanes >= 4 {
		if car.LaneIndex == 0 {
			laneOffset = laneWidth * 0.5 // Inner lane (dedicated left turn)
		} else {
			laneOffset = laneWidth * 1.5 // Outer lane (straight through)
		}
	} else {
		car.LaneIndex = 0 // 2-lane road has single lane in each direction
	}

	if car.IsEW {
		targetY := car.RoadCenter
		if math.Cos(car.Heading) > 0 {
			targetY += laneOffset // Eastbound (+Y)
		} else {
			targetY -= laneOffset // Westbound (-Y)
		}
		car.Pos.Y = sim.Lerp(car.Pos.Y, targetY, 0.08)
	} else {
		targetX := car.RoadCenter
		if math.Sin(car.Heading) > 0 {
			targetX -= laneOffset // Southbound (-X)
		} else {
			targetX += laneOffset // Northbound (+X)
		}
		car.Pos.X = sim.Lerp(car.Pos.X, targetX, 0.08)
	}
}
