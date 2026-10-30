package sim

import "slices"

type LineOperations struct {
	Name                                                     string
	Vehicles, CompletedLaps, Legs                            int
	TotalKM, AverageSpacingKM, CompletedKM, CompletedSeconds float64
}

func (l LineOperations) AverageKPH() (float64, bool) {
	if l.CompletedLaps == 0 || l.CompletedSeconds <= 0 {
		return 0, false
	}
	return l.CompletedKM / l.CompletedSeconds * 3600, true
}

// AI routes are independent cursors over the same itinerary. Compare both
// service name and stop sequence instead of route pointers or vehicle IDs.
func (o *TransitOperations) LineSummary(route *Route) LineOperations {
	var l LineOperations
	if o == nil || route == nil {
		return l
	}
	l.Name = route.Name
	d := o.DistancesFor(route)
	l.TotalKM = d.TotalKM
	for _, leg := range d.Legs {
		if leg.From != leg.To && leg.KM > 0 {
			l.Legs++
			l.AverageSpacingKM += leg.KM
		}
	}
	if l.Legs > 0 {
		l.AverageSpacingKM /= float64(l.Legs)
	}
	for _, v := range o.Vehicles {
		if v.Route == nil || v.LineName != route.Name || !slices.Equal(v.Route.StopIDs, route.StopIDs) {
			continue
		}
		l.Vehicles++
		l.CompletedLaps += v.CompletedLaps
		l.CompletedKM += v.CompletedKM
		l.CompletedSeconds += v.CompletedSeconds
	}
	return l
}
