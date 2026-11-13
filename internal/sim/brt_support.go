package sim

import (
	"math"
	"strings"
)

func (w *World) HasBRTEligibleRoads() bool {
	if w != nil {
		for _, r := range w.Roads {
			if r.HasBusLane() && r.LaneCount() == 8 {
				return true
			}
		}
	}
	return false
}
func CustomMapSupportsCenterBRT(data *CustomMapData) bool {
	if data == nil {
		return false
	}
	for _, r := range data.Roads {
		if r.BusLane && r.Lanes == 8 {
			return true
		}
	}
	return false
}

// Each selectable built-in city has a BRT road variant. Availability describes
// that variant, rather than requiring bus lanes in the ordinary road layout.
func BuiltinMapSupportsCenterBRT(theme MapTheme) bool {
	switch theme {
	case MapThemeMetropolis, MapThemeCoastalBay, MapThemeRiverValley, MapThemeTwinCities:
		return true
	default:
		return false
	}
}

// The other cities retain their original streets with BRT disabled. In the BRT
// variant their expressway corridors become eight-lane bus boulevards; River
// Valley also connects its north/south riverfront avenues to those corridors.
// Run before scaling, stop matching and intersection generation.
func (w *World) prepareBuiltinBRTRoads() {
	if !w.CenterBRT || !BuiltinMapSupportsCenterBRT(w.Theme) || w.Theme == MapThemeMetropolis {
		return
	}
	for i := range w.Roads {
		r := &w.Roads[i]
		riverfront := w.Theme == MapThemeRiverValley && r.P1.Y == r.P2.Y && r.LaneCount() == 4
		if r.LaneCount() >= 6 || riverfront {
			r.Lanes = 8
			r.Type = RoadBusBoulevard8Lane
			r.Name = strings.ReplaceAll(r.Name, "六车道", "八车道公交")
		}
	}
}

// Keep the physical bridge footprint wide enough for the widened BRT roads,
// including when the player changes lane width after entering the city.
func (w *World) fitBuiltinBRTBridges(lw float64) {
	if !w.CenterBRT || !BuiltinMapSupportsCenterBRT(w.Theme) || w.Theme == MapThemeMetropolis {
		return
	}
	for i := range w.Bridges {
		br := &w.Bridges[i]
		forward := UnitFromAngle(br.Heading)
		for _, road := range w.Roads {
			if !road.HasMedianBusLane() {
				continue
			}
			dir := road.P2.Sub(road.P1).Normalize()
			diff := br.Pos.Sub(road.P1)
			along := diff.Dot(dir)
			if math.Abs(dir.Dot(forward)) > .95 && along >= 0 && along <= road.P2.Distance(road.P1) && math.Abs(diff.X*dir.Y-diff.Y*dir.X) < 20 {
				br.Width = math.Max(br.Width, road.GetWidth(lw)+12)
			}
		}
	}
}
