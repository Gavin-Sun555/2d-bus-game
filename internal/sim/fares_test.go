package sim

import (
	"math"
	"testing"
)

func TestFarePoliciesAndStrictCap(t *testing.T) {
	p := DefaultFarePolicy()
	for price := 0; price <= 10; price++ {
		p.Fixed = price
		if p.Quote(20, 100) != price {
			t.Fatal("flat fare changed by journey")
		}
	}
	p = FarePolicy{Mode: FareByStops, Base: 2, IncludedStops: 3, StopsPerYuan: 2, Cap: 7}
	for _, c := range []struct{ stops, want int }{{0, 2}, {3, 2}, {4, 3}, {5, 3}, {6, 4}, {100, 7}} {
		if got := p.Quote(c.stops, 0); got != c.want {
			t.Fatalf("%d stops: %d want %d", c.stops, got, c.want)
		}
	}
	p = FarePolicy{Mode: FareByDistance, Base: 2, IncludedKM: 1, PerKM: 2, Cap: 7}
	for _, c := range []struct {
		km   float64
		want int
	}{{0, 2}, {1, 2}, {1.24, 2}, {1.25, 3}, {2.25, 5}, {100, 7}} {
		if got := p.Quote(0, c.km); got != c.want {
			t.Fatalf("%.2f km: %d want%d", c.km, got, c.want)
		}
	}
	for base := -2; base <= 12; base++ {
		for cap := -2; cap <= 12; cap++ {
			q := FarePolicy{Base: base, Cap: cap}.Normalize()
			if q.Base < 0 || q.Base > 8 || q.Cap <= q.Base || q.Cap > 10 {
				t.Fatalf("invalid normalized fare: %+v", q)
			}
		}
	}
}

func TestFareJourneyUsesRoadsAndRouteOccurrence(t *testing.T) {
	d := RouteDistances{StopIDs: []int{0, 1, 0, 2}, Legs: []RouteLegDistance{{KM: 1}, {KM: 2}, {KM: 3}, {KM: 4}}}
	if n, km := d.Journey(0, 2, 2); n != 1 || km != 3 {
		t.Fatal("used first occurrence instead of current boarding position")
	}
	if n, km := d.Journey(2, 1, 0); n != 2 || km != 5 {
		t.Fatal("wraparound journey wrong")
	}
	w := NewWorld(52, MapThemeMetropolis)
	r := GetStandardRouteForMap(w.Theme, w)
	distances := BuildRouteDistances(w, r, 52)
	if len(distances.Legs) != len(r.StopIDs)-1 || distances.TotalKM <= 0 {
		t.Fatal("closed route distance invalid")
	}
	for _, leg := range distances.Legs {
		direct := w.Stops[leg.From].Pos.Distance(w.Stops[leg.To].Pos) * MetersPerPixel / 1000
		if leg.KM+1e-6 < direct {
			t.Fatal("road distance shorter than direct line")
		}
	}
	if math.Abs(SpeedKPH(60)*MetersPerPixel/1000*3600-60) > .001 {
		t.Fatal("distance and dashboard speed units disagree")
	}
}
