package sim

import "math"

type StationActivity struct{ Boarded, Alighted, Revenue int }
type VehicleOperations struct {
	ID                                         int
	Revenue, Boarded, Alighted, CompletedLaps  int
	KM, Seconds, CompletedKM, CompletedSeconds float64
	Stations                                   map[int]*StationActivity
	bus                                        *Bus
	lastPos                                    Vec2
	lastTrips                                  int
	loopKM, loopSeconds                        float64
	validLoop                                  bool
}

func (v *VehicleOperations) AverageKPH() (float64, bool) {
	if v.CompletedLaps == 0 || v.CompletedSeconds <= 0 {
		return 0, false
	}
	return v.CompletedKM / v.CompletedSeconds * 3600, true
}

type TransitOperations struct {
	Policy    FarePolicy
	Distances RouteDistances
	Vehicles  map[int]*VehicleOperations // 0 = player; positive IDs = fleet buses
	Stations  map[int]*StationActivity
	Revenue   int
}

func NewTransitOperations(w *World, r *Route, lw float64, p FarePolicy) *TransitOperations {
	return &TransitOperations{Policy: p.Normalize(), Distances: BuildRouteDistances(w, r, lw), Vehicles: make(map[int]*VehicleOperations), Stations: make(map[int]*StationActivity)}
}
func (o *TransitOperations) Vehicle(id int) *VehicleOperations {
	v := o.Vehicles[id]
	if v == nil {
		v = &VehicleOperations{ID: id, Stations: make(map[int]*StationActivity)}
		o.Vehicles[id] = v
	}
	return v
}
func stationActivity(m map[int]*StationActivity, id int) *StationActivity {
	s := m[id]
	if s == nil {
		s = &StationActivity{}
		m[id] = s
	}
	return s
}
func (o *TransitOperations) Board(id int, b *Bus, r *Route, stop, count int, candidates []int) (int, int) {
	before := len(b.Passengers)
	boarded := b.BoardPassengers(count, candidates)
	policy := DefaultFarePolicy()
	if o != nil {
		policy = o.Policy
	}
	revenue := 0
	for i := before; i < len(b.Passengers); i++ {
		stops, km := 0, 0.
		if o != nil && r != nil {
			stops, km = o.Distances.Journey(stop, b.Passengers[i].TargetStopID, r.CurrentIndex)
		}
		fare := policy.Quote(stops, km)
		b.Passengers[i].PaidFare = fare
		revenue += fare
	}
	if o != nil && boarded > 0 {
		v := o.Vehicle(id)
		v.Boarded += boarded
		v.Revenue += revenue
		o.Revenue += revenue
		for _, s := range []*StationActivity{stationActivity(o.Stations, stop), stationActivity(v.Stations, stop)} {
			s.Boarded += boarded
			s.Revenue += revenue
		}
	}
	return boarded, revenue
}
func (o *TransitOperations) Alight(id, stop, count int) {
	if o == nil || count <= 0 {
		return
	}
	v := o.Vehicle(id)
	v.Alighted += count
	stationActivity(o.Stations, stop).Alighted += count
	stationActivity(v.Stations, stop).Alighted += count
}

// Call only during simulation ticks. Paused settings/planning time is excluded;
// traffic lights, jams and dwell count. Speed integration excludes teleport and
// collision corrections from the distance, and invalidates interrupted laps.
func (o *TransitOperations) Track(id int, b *Bus, r *Route, dt float64) {
	if o == nil || b == nil || r == nil || dt < 0 {
		return
	}
	v := o.Vehicle(id)
	if v.bus != b {
		v.bus = b
		v.lastPos = b.Pos
		v.lastTrips = r.TotalTrips
		v.validLoop = r.CurrentIndex == 0 && len(r.StopIDs) > 1 && r.StopIDs[0] == r.StopIDs[len(r.StopIDs)-1]
		v.loopKM = 0
		v.loopSeconds = 0
	}
	if dt == 0 {
		return
	}
	step := math.Abs(b.Speed) * dt * MetersPerPixel / 1000
	v.KM += step
	v.Seconds += dt
	v.loopKM += step
	v.loopSeconds += dt
	if b.Pos.Distance(v.lastPos) > math.Max(60, math.Abs(b.Speed)*dt+10) || r.TotalTrips < v.lastTrips {
		v.validLoop = false
	}
	if r.TotalTrips > v.lastTrips {
		if v.validLoop && r.TotalTrips == v.lastTrips+1 && v.loopKM >= o.Distances.TotalKM*.5 && o.Distances.TotalKM > 0 {
			v.CompletedLaps++
			v.CompletedKM += v.loopKM
			v.CompletedSeconds += v.loopSeconds
		}
		v.loopKM = 0
		v.loopSeconds = 0
		v.validLoop = true
	}
	v.lastPos = b.Pos
	v.lastTrips = r.TotalTrips
}
