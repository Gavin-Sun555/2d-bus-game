package sim

import (
	"math"
	"testing"
)

func TestBRTAvailabilityMatchesActualMapRoads(t *testing.T) {
	for _, theme := range BuiltinMapThemes() {
		w := &World{CenterBRT: true}
		w.BuildMap(theme, 52)
		active := ActiveWorldForRoute
		if got := BuiltinMapSupportsCenterBRT(theme); got != w.HasBRTEligibleRoads() {
			t.Fatal("menu eligibility differs from actual roads", theme, got)
		}
		if ActiveWorldForRoute != active {
			t.Fatal("menu eligibility replaced active navigation world")
		}
	}
}

func TestCenterBRTCustomMapEligibilityAndGeometry(t *testing.T) {
	for _, busLane := range []bool{false, true} {
		data := GetDefaultCustomMapData()
		data.Roads[0].Lanes = 8
		data.Roads[0].BusLane = busLane
		if CustomMapSupportsCenterBRT(data) != busLane {
			t.Fatal("custom map eligibility ignored bus_lane")
		}
		w := &World{CenterBRT: true, CustomMap: data}
		w.BuildMap(MapThemeCustom, 52)
		if w.CenterBRT != busLane {
			t.Fatal("custom map BRT variant not applied")
		}
		roads, stops := 0, 0
		for _, r := range w.Roads {
			if r.HasMedianBusLane() {
				roads++
			}
		}
		for _, s := range w.Stops {
			if s.CenterBRT {
				stops++
				if s.Pos.Distance(s.RoadCenter) != 26 {
					t.Fatal("custom stop outside median bus lane", s.ID)
				}
			}
		}
		if busLane && (roads == 0 || stops == 0) {
			t.Fatalf("custom BRT roads/stops missing: %d/%d", roads, stops)
		}
		if !busLane && (roads != 0 || stops != 0) {
			t.Fatal("ordinary eight-lane road converted to BRT")
		}
	}
}

func TestBuiltinBRTOverpassCoversOuterLanes(t *testing.T) {
	for _, large := range []bool{false, true} {
		w := &World{CenterBRT: true, LargeGrid: large}
		w.BuildMap(MapThemeTwinCities, 42)
		for _, lw := range []float64{42, 52, 78} {
			w.UpdateLaneWidth(lw)
			checked := 0
			for _, br := range w.Bridges {
				if !br.IsOverpass {
					continue
				}
				right := UnitFromAngle(br.Heading + math.Pi/2)
				for _, side := range []float64{-1, 1} {
					if !w.IsOverpassAt(br.Pos.Add(right.Mul(side * lw * 3.5))) {
						t.Fatalf("outer BRT road lane outside overpass: large=%v width=%.0f", large, lw)
					}
				}
				checked++
			}
			if checked == 0 {
				t.Fatal("missing overpass fixture")
			}
		}
	}
}
