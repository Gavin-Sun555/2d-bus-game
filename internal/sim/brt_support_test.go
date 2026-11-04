package sim

import "testing"

func TestBRTAvailabilityMatchesActualMapRoads(t *testing.T) {
	for _, theme := range BuiltinMapThemes() {
		w := NewWorld(52, theme)
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
