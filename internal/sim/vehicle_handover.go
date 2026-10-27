package sim

type VehicleHandoverSite struct {
	Stop      *BusStop
	World     *World
	LaneWidth float64
	Obstacles []OBB
}

func placeReplacement(fresh *Bus, site VehicleHandoverSite) bool {
	if fresh.IsArticulated() && site.OtherPlatformBus() {
		return false
	}
	if fresh.IsArticulated() && site.Stop != nil {
		fresh.Pos = site.Stop.FrontBerthPosition()
	}
	origin := fresh.Pos
	u := UnitFromAngle(fresh.Heading)
	maxOffset := 0.
	if fresh.IsArticulated() && site.Stop == nil {
		maxOffset = fresh.Length
	}
	for offset := 0.; offset <= maxOffset; offset += 4 {
		fresh.Pos = origin.Add(u.Mul(offset))
		fresh.ResetArticulation()
		clear := true
		for _, part := range fresh.BodyParts() {
			if part.HalfLen == 0 {
				continue
			}
			for _, ob := range site.Obstacles {
				if hit, _, _ := CheckOBBCollision(part, ob); hit {
					clear = false
					break
				}
			}
			if site.World != nil {
				for _, p := range BodyCorners(part) {
					if site.World.CheckBoundaries(p, 1, site.LaneWidth).Zone == ZoneHardBarrier {
						clear = false
						break
					}
				}
			}
		}
		if clear {
			return true
		}
	}
	return false
}

// Two parked short buses must finish independently before a long replacement
// can reserve their entire platform.
func (site VehicleHandoverSite) OtherPlatformBus() bool {
	if site.Stop == nil || !site.Stop.HasTwoBerths() {
		return false
	}
	for _, body := range site.Obstacles {
		if body.HalfLen >= 32 && site.Stop.OccupiesBerth(body) {
			return true
		}
	}
	return false
}
