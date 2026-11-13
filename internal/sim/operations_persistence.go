package sim

// OperationsSnapshot includes the unfinished lap so a saved journey remains one lap.
type OperationsSnapshot struct {
	Revenue      int
	Stations     map[int]*StationActivity
	Vehicles     map[int]VehicleSnapshot
	LineStations map[string]map[int]*StationActivity `json:",omitempty"`
}
type VehicleSnapshot struct {
	Public              VehicleOperations
	LastPos             Vec2
	LastTrips           int
	LoopKM, LoopSeconds float64
	ValidLoop           bool
	Tracking            bool
}

func (o *TransitOperations) Snapshot() OperationsSnapshot {
	s := OperationsSnapshot{Revenue: o.Revenue, Stations: o.Stations, LineStations: o.LineStations, Vehicles: make(map[int]VehicleSnapshot)}
	for id, v := range o.Vehicles {
		s.Vehicles[id] = VehicleSnapshot{*v, v.lastPos, v.lastTrips, v.loopKM, v.loopSeconds, v.validLoop, v.bus != nil}
	}
	return s
}
func RestoreOperations(s OperationsSnapshot, w *World, r *Route, lw float64, p FarePolicy, buses map[int]*Bus, routes map[int]*Route) *TransitOperations {
	o := NewTransitOperations(w, r, lw, p)
	o.Revenue = s.Revenue
	o.LineStations = s.LineStations
	if s.Stations != nil {
		o.Stations = s.Stations
	}
	for id, saved := range s.Vehicles {
		v := saved.Public
		if id > 0 && buses[id] == nil {
			v.ServiceEnded = true
		}
		if saved.Tracking && !v.ServiceEnded {
			v.bus = buses[id]
		}
		if active := routes[id]; active != nil {
			v.Route = active
		}
		if v.Stations == nil {
			v.Stations = make(map[int]*StationActivity)
		}
		v.lastPos = saved.LastPos
		v.lastTrips = saved.LastTrips
		v.loopKM = saved.LoopKM
		v.loopSeconds = saved.LoopSeconds
		v.validLoop = saved.ValidLoop
		o.Vehicles[id] = &v
		// Legacy saves had vehicle totals only. Use each saved assignment to
		// recover the historical boarding and alighting counts where possible.
		if s.LineStations == nil && v.LineName != "" {
			for stop, activity := range v.Stations {
				if activity != nil {
					line := o.lineStationActivity(v.LineName, stop)
					line.Boarded += activity.Boarded
					line.Alighted += activity.Alighted
					line.Revenue += activity.Revenue
				}
			}
		}
	}
	return o
}
