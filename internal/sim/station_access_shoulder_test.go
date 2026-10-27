package sim

import "testing"

func TestStationApronBoundsAndArticulatedProbes(t *testing.T) {
	const lw = 52.
	st := NewBusStop(0, "stop", Vec2{}, 0, 6, RGBA(0, 0, 0, 255), lw)
	w := &World{Roads: []RoadSegment{{P1: Vec2{-3000, 0}, P2: Vec2{3000, 0}, Lanes: 6}}, Stops: []*BusStop{st}}
	for _, tc := range []struct {
		x      float64
		inside bool
	}{{-st.Length/2 - StationShoulderApron, true}, {-st.Length/2 - StationShoulderApron - .01, false}, {st.Length/2 + 260 + StationShoulderApron, true}, {st.Length/2 + 260 + StationShoulderApron + .01, false}} {
		if got := w.stationAccessShoulderPoint(Vec2{tc.x, 3*lw + 15}); got != tc.inside {
			t.Fatalf("apron boundary x%.2f: %v want%v", tc.x, got, tc.inside)
		}
	}
	b := NewBus(650, 3*lw-25, 0)
	b.SetModel(BusArticulated)
	b.TrailerHeading = -.3
	b.TrailerInitialized = true
	if zone := b.CheckBoundaries(w, lw).Zone; zone != ZoneShoulder {
		t.Fatalf("tail shoulder fixture zone%v", zone)
	}
	if !b.IsOnStationAccessShoulder(w, lw) {
		t.Fatal("road-going front disqualified tail still inside station apron")
	}
	b.Pos.X += 100
	if b.IsOnStationAccessShoulder(w, lw) {
		t.Fatal("tail on ordinary shoulder outside apron escaped slowdown")
	}
	b.Pos = st.ShelterPos
	if b.CheckBoundaries(w, lw).Zone != ZoneHardBarrier || b.IsOnStationAccessShoulder(w, lw) {
		t.Fatal("shelter stopped being a hard barrier")
	}
	b = NewBus(500, 3*lw+15, 0)
	w.Buildings = []Building{{Pos: b.Pos, W: 40, H: 40}}
	if b.CheckBoundaries(w, lw).Zone != ZoneHardBarrier || b.IsOnStationAccessShoulder(w, lw) {
		t.Fatal("apron waived a building collision")
	}
}
