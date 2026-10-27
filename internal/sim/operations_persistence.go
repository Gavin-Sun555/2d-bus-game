package sim

// OperationsSnapshot includes the unfinished lap so a saved journey remains one lap.
type OperationsSnapshot struct {
	Revenue  int
	Stations map[int]*StationActivity
	Vehicles map[int]VehicleSnapshot
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
	s := OperationsSnapshot{Revenue: o.Revenue, Stations: o.Stations, Vehicles: make(map[int]VehicleSnapshot)}
	for id, v := range o.Vehicles {
		s.Vehicles[id] = VehicleSnapshot{*v, v.lastPos, v.lastTrips, v.loopKM, v.loopSeconds, v.validLoop, v.bus != nil}
	}
	return s
}
func RestoreOperations(s OperationsSnapshot, w *World, r *Route, lw float64, p FarePolicy, buses map[int]*Bus, routes map[int]*Route) *TransitOperations {
	o := NewTransitOperations(w, r, lw, p)
	o.Revenue = s.Revenue
	if s.Stations != nil {
		o.Stations = s.Stations
	}
	for id, saved := range s.Vehicles {
		v := saved.Public
		if saved.Tracking {
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
	}
	return o
}
