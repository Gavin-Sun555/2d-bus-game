package sim

import "math"

const StandardStopLength = 336.0
const StationBerthGap = 24.0
const StationFrontMargin = 24.0

// All vehicles align their front section's nose with the same forward marker.
func (s *BusStop) FrontBerthPosition() Vec2 {
	offset := math.Max(0, s.Length/2-StationFrontMargin-48)
	return s.Pos.Add(UnitFromAngle(s.Heading).Mul(offset))
}
func (s *BusStop) RearBerthPosition() Vec2 {
	return s.FrontBerthPosition().Sub(UnitFromAngle(s.Heading).Mul(96 + StationBerthGap))
}
func (s *BusStop) HasTwoBerths() bool { return s.Length >= StandardStopLength }

// A body overlaps the boarding strip, as distinct from the adjacent road lane.
func (s *BusStop) OccupiesBerth(body OBB) bool {
	u := UnitFromAngle(s.Heading)
	r := Vec2{X: -u.Y, Y: u.X}
	d := body.Center.Sub(s.Pos)
	yaw := body.Heading - s.Heading
	along := math.Abs(math.Cos(yaw))*body.HalfLen + math.Abs(math.Sin(yaw))*body.HalfWid
	across := math.Abs(math.Sin(yaw))*body.HalfLen + math.Abs(math.Cos(yaw))*body.HalfWid
	return math.Abs(d.Dot(u)) < s.Length/2+along && math.Abs(d.Dot(r)) < s.Width/2+across-2 && math.Cos(yaw) > .8
}

// Path-only copy; physical station geometry stays centered on the platform.
func (s *BusStop) ForwardDockStop() *BusStop {
	copy := *s
	delta := s.FrontBerthPosition().Sub(s.Pos)
	copy.Pos = copy.Pos.Add(delta)
	copy.RoadCenter = copy.RoadCenter.Add(delta)
	copy.ShelterPos = copy.ShelterPos.Add(delta)
	return &copy
}
func StandardizeStopBays(w *World, laneWidths ...float64) {
	if w == nil {
		return
	}
	lw := 52.
	if len(laneWidths) > 0 {
		lw = laneWidths[0]
	}
	for _, s := range w.Stops {
		if s == nil {
			continue
		}
		s.Length = StandardStopLength
		if !w.InsideJunction(s.Pos, lw, s.Length/2+64) {
			continue
		}
		origin := s.RoadCenter
		u := UnitFromAngle(s.Heading)
		found := false
		for offset := 16.; offset <= 640 && !found; offset += 16 {
			for _, sign := range []float64{-1, 1} {
				s.RoadCenter = origin.Add(u.Mul(sign * offset))
				s.UpdateGeometry(lw)
				onRoad := false
				for _, road := range w.Roads {
					direction := road.P2.Sub(road.P1).Normalize()
					distance, _ := DistToSegment(s.RoadCenter, road.P1, road.P2)
					if math.Abs(direction.Dot(u)) > .99 && distance < 1 {
						onRoad = true
						break
					}
				}
				if !onRoad {
					continue
				}
				if !w.InsideJunction(s.Pos, lw, s.Length/2+64) && w.CheckBoundaries(s.Pos, 18, lw).Zone != ZoneHardBarrier {
					found = true
					break
				}
			}
		}
		if !found {
			s.RoadCenter = origin
			s.UpdateGeometry(lw)
		}
	}
}
