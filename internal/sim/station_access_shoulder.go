package sim

// A short apron before/after the platform allows normal entry and departure
// speed. This changes shoulder drag only, never the physical paved boundary.
const StationShoulderApron = 160.0

func (w *World) stationAccessShoulderPoint(pos Vec2) bool {
	for _, st := range w.Stops {
		if st == nil {
			continue
		}
		u := UnitFromAngle(st.Heading)
		d := pos.Sub(st.Pos)
		along, across := d.Dot(u), d.Dot(Vec2{X: -u.Y, Y: u.X})
		taper := 110.
		if st.Length > 200 {
			taper = 260
		}
		if along >= -st.Length/2-StationShoulderApron && along <= st.Length/2+taper+StationShoulderApron &&
			across >= -st.Width*.48-2 && across <= st.Width/2+ShoulderWidth {
			return true
		}
	}
	return false
}

// Match the probes used by CheckBoundaries. Every point actually on a shoulder
// must belong to a station apron; a trailer reaching ordinary shoulder retains
// its drag. Road points outside the apron do not disqualify a partial departure.
func (b *Bus) IsOnStationAccessShoulder(w *World, lw float64) bool {
	if w == nil {
		return false
	}
	shoulder := false
	check := func(pos Vec2, radius float64) bool {
		result := w.CheckBoundaries(pos, radius, lw)
		if result.Zone == ZoneHardBarrier {
			return false
		}
		if result.Zone == ZoneShoulder {
			shoulder = true
			return w.stationAccessShoulderPoint(pos)
		}
		return true
	}
	if !check(b.Pos, b.Width/2) {
		return false
	}
	if b.IsArticulated() {
		for _, part := range b.BodyParts() {
			for _, corner := range BodyCorners(part) {
				if !check(corner, .5) {
					return false
				}
			}
		}
	}
	return shoulder
}
