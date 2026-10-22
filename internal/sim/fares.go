package sim

import "math"

// Keep distance and speed consistent: 1 px/s is 0.18 km/h in the dashboard.
const MetersPerPixel = 0.05

type FareMode int

const (
	FareFixed FareMode = iota
	FareByStops
	FareByDistance
)

type FarePolicy struct {
	Mode                                     FareMode
	Fixed, Base, IncludedStops, StopsPerYuan int
	IncludedKM, PerKM, Cap                   int
}

func DefaultFarePolicy() FarePolicy {
	return FarePolicy{Fixed: 3, Base: 2, IncludedStops: 3, StopsPerYuan: 3, IncludedKM: 1, PerKM: 1, Cap: 10}
}

func (p FarePolicy) Normalize() FarePolicy {
	if p.Mode < FareFixed || p.Mode > FareByDistance {
		p.Mode = FareFixed
	}
	p.Fixed = max(0, min(10, p.Fixed))
	p.Base = max(0, min(8, p.Base))
	p.IncludedStops = max(0, min(99, p.IncludedStops))
	p.StopsPerYuan = max(1, min(99, p.StopsPerYuan))
	p.IncludedKM = max(0, min(100, p.IncludedKM))
	p.PerKM = max(0, min(10, p.PerKM))
	p.Cap = max(p.Base+1, min(10, p.Cap))
	return p
}

func (p FarePolicy) Quote(stops int, km float64) int {
	p = p.Normalize()
	switch p.Mode {
	case FareByStops:
		extra := max(0, stops-p.IncludedStops)
		return min(p.Cap, p.Base+(extra+p.StopsPerYuan-1)/p.StopsPerYuan)
	case FareByDistance:
		if math.IsNaN(km) || math.IsInf(km, 0) {
			km = 0
		}
		return min(p.Cap, int(math.Round(float64(p.Base)+math.Max(0, km-float64(p.IncludedKM))*float64(p.PerKM))))
	default:
		return p.Fixed
	}
}

type RouteLegDistance struct {
	From, To int
	KM       float64
}
type RouteDistances struct {
	StopIDs []int
	Legs    []RouteLegDistance
	TotalKM float64
}

func BuildRouteDistances(w *World, route *Route, lw float64) RouteDistances {
	var d RouteDistances
	if w == nil || route == nil {
		return d
	}
	d.StopIDs = append([]int(nil), route.StopIDs...)
	if len(d.StopIDs) > 1 && d.StopIDs[0] == d.StopIDs[len(d.StopIDs)-1] {
		d.StopIDs = d.StopIDs[:len(d.StopIDs)-1]
	}
	if len(d.StopIDs) < 2 {
		return d
	}
	for i, from := range d.StopIDs {
		to := d.StopIDs[(i+1)%len(d.StopIDs)]
		leg := RouteLegDistance{From: from, To: to}
		if from >= 0 && from < len(w.Stops) && to >= 0 && to < len(w.Stops) {
			points := ComputeRoadPathBetweenStops(w, w.Stops[from], w.Stops[to], lw)
			for j := 1; j < len(points); j++ {
				leg.KM += points[j].Distance(points[j-1]) * MetersPerPixel / 1000
			}
		}
		d.Legs = append(d.Legs, leg)
		d.TotalKM += leg.KM
	}
	return d
}

// Use the boarding occurrence nearest the route cursor, so repeated station
// IDs and the duplicated terminal do not accidentally bill a whole extra lap.
func (d RouteDistances) Journey(from, to, cursor int) (int, float64) {
	n := len(d.StopIDs)
	if n < 2 || len(d.Legs) != n || from == to {
		return 0, 0
	}
	cursor = (cursor%n + n) % n
	start := -1
	for _, i := range []int{cursor % n, (cursor + n - 1) % n} {
		if i >= 0 && d.StopIDs[i] == from {
			start = i
			break
		}
	}
	if start < 0 {
		for i, id := range d.StopIDs {
			if id == from {
				start = i
				break
			}
		}
	}
	if start < 0 {
		return 0, 0
	}
	km := 0.
	for step := 1; step <= n; step++ {
		km += d.Legs[(start+step-1)%n].KM
		if d.StopIDs[(start+step)%n] == to {
			return step, km
		}
	}
	return 0, 0
}
