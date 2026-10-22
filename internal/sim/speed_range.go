package sim

// SpeedRange describes spawn-time speed limits, in discrete 10 km/h steps.
type SpeedRange struct{ MinKPH, MaxKPH int }

func (r SpeedRange) Normalize(defaultMax, ceiling int) SpeedRange {
	if r.MinKPH == 0 && r.MaxKPH == 0 {
		return SpeedRange{30, defaultMax}
	}
	r.MinKPH = max(30, min(ceiling, r.MinKPH/10*10))
	r.MaxKPH = max(r.MinKPH, min(ceiling, r.MaxKPH/10*10))
	return r
}
func (r SpeedRange) Sample() int { return r.MinKPH + 10*RandomIntn((r.MaxKPH-r.MinKPH)/10+1) }
func SpeedKPH(kph int) float64   { return float64(kph) / .18 }
