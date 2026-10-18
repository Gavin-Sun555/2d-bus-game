package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) obstacleSpeed(f autopilotFrame, targetSpd float64) float64 {
	targetSpd = math.Min(targetSpd, ad.innerEntryObstacleSpeed(f))
	// E. Forward Vehicle Obstacle Avoidance
	cosH := math.Cos(f.bus.Heading)
	sinH := math.Sin(f.bus.Heading)
	fwdLimit := 140.0
	if ad.BerserkMode {
		fwdLimit = math.Max(140.0, f.speedAbs*f.speedAbs/(2*130.0)+80.0)
	}
	for _, obs := range f.obstacles {
		delta := obs.Pos.Sub(f.bus.Pos)
		fwdDist := delta.X*cosH + delta.Y*sinH
		latDist := math.Abs(-delta.X*sinH + delta.Y*cosH)

		if ad.clearsThroughQueueDuringMerge(f, obs) || ad.isSafelyWaitingOutsideRoundabout(f, obs) || roundaboutLanesSeparated(f, obs, cosH, sinH, latDist) {
			continue
		}

		// For oncoming vehicles (facing opposite direction), ignore if safely separated:
		cosHeadDiff := math.Cos(obs.Heading - f.bus.Heading)
		if cosHeadDiff < -0.2 {
			// 1. Safely separated laterally in local vehicle frame (standard straight-road case)
			if latDist > (f.bus.Width/2 + obs.Width/2 + 2.0) {
				continue
			}
			// 2. Safely separated by road centerline / double yellow line on multi-lane roads
			if f.world != nil {
				separated := false
				for _, r := range f.world.Roads {
					roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
					c := r.P1.Y
					if !roadIsEW {
						c = r.P1.X
					}
					is6 := (r.Lanes >= 6 || r.Type == sim.RoadExpressway6Lane || f.world.IsRoad6Lane(c, roadIsEW, f.bus.Pos))
					isMulti := (is6 || r.Lanes >= 4 || r.Type == sim.RoadAvenue4Lane || f.world.IsRoad4Lane(c, roadIsEW, f.bus.Pos))
					if !isMulti {
						continue
					}
					rWidth := r.GetWidth(f.laneWidth)
					if roadIsEW {
						minX := math.Min(r.P1.X, r.P2.X) - 30.0
						maxX := math.Max(r.P1.X, r.P2.X) + 30.0
						if f.bus.Pos.X >= minX && f.bus.Pos.X <= maxX && math.Abs(f.bus.Pos.Y-c) <= rWidth*0.5+15.0 &&
							obs.Pos.X >= minX && obs.Pos.X <= maxX && math.Abs(obs.Pos.Y-c) <= rWidth*0.5+15.0 {
							dBus := f.bus.Pos.Y - c
							dObs := obs.Pos.Y - c
							if (dBus <= 2.0 && dObs >= -2.0) || (dBus >= -2.0 && dObs <= 2.0) {
								if math.Abs(f.bus.Pos.Y-obs.Pos.Y) >= (f.bus.Width/2 + obs.Width/2) {
									separated = true
									break
								}
							}
						}
					} else {
						minY := math.Min(r.P1.Y, r.P2.Y) - 30.0
						maxY := math.Max(r.P1.Y, r.P2.Y) + 30.0
						if f.bus.Pos.Y >= minY && f.bus.Pos.Y <= maxY && math.Abs(f.bus.Pos.X-c) <= rWidth*0.5+15.0 &&
							obs.Pos.Y >= minY && obs.Pos.Y <= maxY && math.Abs(obs.Pos.X-c) <= rWidth*0.5+15.0 {
							dBus := f.bus.Pos.X - c
							dObs := obs.Pos.X - c
							if (dBus <= 2.0 && dObs >= -2.0) || (dBus >= -2.0 && dObs <= 2.0) {
								if math.Abs(f.bus.Pos.X-obs.Pos.X) >= (f.bus.Width/2 + obs.Width/2) {
									separated = true
									break
								}
							}
						}
					}
				}
				if separated {
					continue
				}
			}
			// 3. Safely outside our tracked drive path corridor (e.g. turning across intersection while oncoming car waits in its lane)
			if len(ad.drivePath) >= 2 {
				dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
				if dPath > (f.bus.Width/2 + obs.Width/2 + 6.0) {
					continue
				}
			}
		}

		roadLatDist := latDist
		if f.world != nil {
			_, isEW, isMulti, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
			if isMulti || is6 {
				if isEW {
					roadLatDist = math.Abs(f.bus.Pos.Y - obs.Pos.Y)
				} else {
					roadLatDist = math.Abs(f.bus.Pos.X - obs.Pos.X)
				}
			} else {
				cosH := math.Abs(math.Cos(f.bus.Heading))
				sinH := math.Abs(math.Sin(f.bus.Heading))
				if cosH >= sinH {
					roadLatDist = math.Abs(f.bus.Pos.Y - obs.Pos.Y)
				} else {
					roadLatDist = math.Abs(f.bus.Pos.X - obs.Pos.X)
				}
			}
		}

		// Immediate physical proximity / collision containment:
		// If obstacle directly overlaps with or is right against the bus front envelope:
		directDist := obs.Pos.Distance(f.bus.Pos)
		collisionDist := f.bus.Length/2 + obs.Length/2 + 8.0
		if directDist < collisionDist && fwdDist > -(f.bus.Length/2) {
			isLaneSeparated := (math.Abs(ad.LaneOffset) > 8.0 || ad.OvertakeState != OvertakeNone) &&
				(latDist >= (f.bus.Width/2+obs.Width/2-2.0) || roadLatDist >= (f.bus.Width/2+obs.Width/2+2.0))
			if !isLaneSeparated && latDist < (f.bus.Width/2+obs.Width/2+8.0) {
				targetSpd = 0.0
				continue
			}
		}

		if fwdDist > 2.0 && fwdDist < fwdLimit && (latDist < (f.bus.Width/2+obs.Width/2+18.0) || roadLatDist < (f.bus.Width/2+obs.Width/2+18.0)) {
			// On multi-lane roads, detect if obstacle is in an adjacent lane corridor
			isAdjacentLane := false
			if f.world != nil {
				_, _, isMulti, is6 := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
				if (isMulti || is6 || f.nearRoundabout) && (roadLatDist >= (f.bus.Width/2+obs.Width/2+2.0) || latDist >= (f.bus.Width/2+obs.Width/2+2.0)) {
					isAdjacentLane = true
				}
			}

			// For vehicles where we have a valid tracked path: verify obstacle is within our path corridor
			if len(ad.drivePath) >= 2 {
				obsPos := ad.basePathPosition(obs.Pos, f.world, f.bus.Heading, f.laneWidth)
				dPath := minDistanceToPath(obsPos, ad.drivePath, ad.WaypointIdx)
				if isAdjacentLane || dPath > (f.bus.Width/2+obs.Width/2+10.0) {
					// Obstacle is in adjacent lane corridor.
					minClearLat := f.bus.Width/2 + obs.Width/2 + 6.0
					actualLat := roadLatDist
					if actualLat < minClearLat && fwdDist > 0 {
						isPassingCorridor := math.Abs(ad.LaneOffset) > 8.0 || ad.OvertakeState == OvertakePassing || ad.OvertakeState == OvertakeChangingLane
						roadLatSep := actualLat >= (f.bus.Width/2 + obs.Width/2 - 2.0)
						if isPassingCorridor && roadLatSep {
							// Vehicle is in an adjacent lane. The bus is driving forward along its own lane corridor.
							// Never brake to 0 km/h and freeze next to a stopped vehicle; maintain forward driving speed!
							minPassingSpd := 35.0
							if ad.BerserkMode {
								minPassingSpd = math.Max(minPassingSpd, ad.GetBerserkCruiseSpeed()*0.6)
							}
							safeDist := f.bus.Length/2 + obs.Length/2 + 25.0
							if fwdDist >= safeDist {
								decelRate := 160.0
								decelSpd := obs.Speed + math.Sqrt(2*decelRate*math.Max(0, fwdDist-safeDist))
								targetSpd = math.Min(targetSpd, math.Max(minPassingSpd, decelSpd))
							} else {
								targetSpd = math.Min(targetSpd, minPassingSpd)
							}
						} else {
							safeDist := f.bus.Length/2 + obs.Length/2 + 25.0
							if fwdDist < safeDist {
								targetSpd = math.Min(targetSpd, math.Max(0.0, obs.Speed-5.0))
							} else {
								decelRate := 160.0
								decelSpd := obs.Speed + math.Sqrt(2*decelRate*math.Max(0, fwdDist-safeDist))
								targetSpd = math.Min(targetSpd, decelSpd)
							}
						}
					}
					continue
				}
			}

			safeStopDist := f.bus.Length/2 + obs.Length/2 + 30.0
			if ad.BerserkMode && ad.OvertakeState == OvertakeChangingLane {
				safeStopDist = f.bus.Length/2 + obs.Length/2 + 25.0
			}
			if fwdDist < safeStopDist {
				targetSpd = 0.0
			} else {
				leadSpd := math.Max(0.0, obs.Speed)
				decelRate := 135.0
				if ad.BerserkMode {
					decelRate = 180.0
				}
				decelSpd := leadSpd + math.Sqrt(2*decelRate*math.Max(0, fwdDist-safeStopDist))
				// If closing in at close quarters while still overlapping laterally:
				// Clamp closing rate so bus cannot close faster than available buffer distance
				gap := fwdDist - safeStopDist
				if gap < 50.0 {
					maxClosing := math.Max(0.0, gap*0.75)
					decelSpd = math.Min(decelSpd, leadSpd+maxClosing)
				}
				targetSpd = math.Min(targetSpd, decelSpd)
			}
		}
	}

	return targetSpd
}

// isSafelyWaitingOutsideRoundabout excludes stopped vehicles outside the curb
// only when they also clear the tracked path or the bus's physical envelope.
func (ad *AutopilotDriver) isSafelyWaitingOutsideRoundabout(f autopilotFrame, obs VehicleObstacle) bool {
	if f.world == nil || !(obs.Speed < 5.0) {
		return false
	}
	for _, rb := range f.world.Roundabouts {
		if !(f.bus.Pos.Distance(rb.Center) < rb.Radius+10.0) || !(obs.Pos.Distance(rb.Center) > rb.Radius+15.0) {
			continue
		}
		if len(ad.drivePath) >= 2 {
			if minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx) > f.bus.Width/2+obs.Width/2+6.0 {
				return true
			}
			continue
		}
		if obs.Pos.Distance(f.bus.Pos) > f.bus.Length/2+obs.Length/2+25.0 {
			return true
		}
	}
	return false
}

func roundaboutLanesSeparated(f autopilotFrame, obs VehicleObstacle, cosH, sinH, latDist float64) bool {
	if !f.nearRoundabout || f.world == nil {
		return false
	}
	for _, rb := range f.world.Roundabouts {
		dBus, dObs := f.bus.Pos.Distance(rb.Center), obs.Pos.Distance(rb.Center)
		if !(dBus <= rb.Radius+100.0 && dObs <= rb.Radius+100.0) {
			continue
		}
		entryLatDist := math.Abs(f.bus.Pos.X - obs.Pos.X)
		if math.Abs(cosH) >= math.Abs(sinH) {
			entryLatDist = math.Abs(f.bus.Pos.Y - obs.Pos.Y)
		}
		if entryLatDist >= f.bus.Width/2+obs.Width/2+2.0 {
			return true
		}
		if math.Abs(dBus-dObs) >= f.laneWidth*0.75 && latDist >= f.bus.Width/2+obs.Width/2-4.0 {
			return true
		}
	}
	return false
}
