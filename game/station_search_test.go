package game

import (
	"slices"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestStationSearchFindsBilingualNamesAndAliases(t *testing.T) {
	stops := []*sim.BusStop{
		{ID: 0, Name: "市民广场·北行 (Civic Plaza Northbound)", CustomNameZH: "人民广场", CustomNameEN: "People's Square"},
		{ID: 1, Name: "市民广场·南行 (Civic Plaza Southbound)"},
		{ID: 2, Name: "中央枢纽 (Central Terminal)"},
	}
	search := &stationSearch{}
	for _, tc := range []struct {
		query string
		want  []*sim.BusStop
	}{
		{"市民广场", stops[:2]},
		{"  PLAZA   southbound ", stops[1:2]},
		{"人民", stops[:1]},
		{"people's square", stops[:1]},
		{"missing", nil},
		{" ", nil},
	} {
		search.fields[0] = tc.query
		search.refresh(stops)
		if !slices.Equal(search.results, tc.want) {
			t.Errorf("%q matched %v, want %v", tc.query, search.results, tc.want)
		}
	}
}

func TestLocatingStationKeepsPlannedRoute(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.TogglePlanMode()
	route := g.planningRoute()
	before := slices.Clone(route.StopIDs)
	g.openStationSearch()
	stop := g.World.Stops[5]
	g.focusSearchedStation(stop)
	if g.stationSearch != nil || !g.IsPlanMode || g.Camera.Pos != stop.ShelterPos || g.Camera.TargetPos != stop.ShelterPos || g.Camera.Zoom < .42 {
		t.Fatal("did not close search and focus the selected station in planning mode")
	}
	if g.planModeRouteModified || !slices.Equal(route.StopIDs, before) {
		t.Fatal("locating a station changed the planned route")
	}
}
