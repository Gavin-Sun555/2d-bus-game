package sim

import "slices"

type LineOperations struct {
	Name                                                     string
	Vehicles, CompletedLaps, Legs                            int
	TotalKM, AverageSpacingKM, CompletedKM, CompletedSeconds float64
}

// ResetLine discards one service's statistics before its vehicles are dispatched
// on a new itinerary. The player record is shared with the currently driven line.
func (o *TransitOperations) ResetLine(route *Route, player bool) {
	for id, v := range o.Vehicles {
		if (id == 0 && player) || (id != 0 && v.LineName == route.Name) {
			delete(o.routeDistances, v.Route)
			delete(o.Vehicles, id)
		}
	}
	for stopID, activity := range o.LineStations[route.Name] {
		if activity == nil {
			continue
		}
		o.Revenue = max(0, o.Revenue-activity.Revenue)
		if total := o.Stations[stopID]; total != nil {
			total.Boarded = max(0, total.Boarded-activity.Boarded)
			total.Alighted = max(0, total.Alighted-activity.Alighted)
			total.Revenue = max(0, total.Revenue-activity.Revenue)
		}
	}
	delete(o.LineStations, route.Name)
	delete(o.routeDistances, route)
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
		if !v.ServesLine(route) || v.ServiceEnded {
			continue
		}
		l.Vehicles++
		l.CompletedLaps += v.CompletedLaps
		l.CompletedKM += v.CompletedKM
		l.CompletedSeconds += v.CompletedSeconds
	}
	return l
}

// ServesLine matches a vehicle's itinerary to the selected service.
func (v *VehicleOperations) ServesLine(route *Route) bool {
	return v != nil && v.Route != nil && route != nil && v.LineName == route.Name && slices.Equal(v.Route.StopIDs, route.StopIDs)
}
