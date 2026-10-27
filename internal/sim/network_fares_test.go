package sim

import "testing"

func TestOtherLineFareUsesItsOwnItinerary(t *testing.T) {
	w := NewWorld(52, MapThemeMetropolis)
	lines := DefaultTransitLines(w)
	r := lines[1]
	from, to := r.StopIDs[0], r.StopIDs[3]
	for _, mode := range []FareMode{FareByStops, FareByDistance} {
		policy := FarePolicy{Mode: mode, Base: 0, IncludedStops: 0, StopsPerYuan: 1, IncludedKM: 0, PerKM: 10, Cap: 10}
		o := NewTransitOperations(w, lines[0], 52, policy)
		bus := NewBus(w.Stops[from].Pos.X, w.Stops[from].Pos.Y, w.Stops[from].Heading)
		count, fare := o.Board(1, bus, r, from, 1, []int{to})
		stops, km := BuildRouteDistances(w, r, 52).Journey(from, to, 0)
		if count != 1 || fare != policy.Quote(stops, km) || fare == 0 {
			t.Fatalf("mode%d: fare %d, expected %d for %d stops / %.3f km", mode, fare, policy.Quote(stops, km), stops, km)
		}
	}
}
