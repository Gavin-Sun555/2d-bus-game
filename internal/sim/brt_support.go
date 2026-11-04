package sim

import "sync"

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

var builtinBRTSupport sync.Map

// Inspect the generated road configuration once. Do not call BuildMap here:
// menu previews must not replace the active world or clear navigation caches.
func BuiltinMapSupportsCenterBRT(theme MapTheme) bool {
	if supported, ok := builtinBRTSupport.Load(theme); ok {
		return supported.(bool)
	}
	w := &World{Theme: theme}
	switch theme {
	case MapThemeMetropolis:
		w.buildMap1Metropolis(52)
	case MapThemeCoastalBay:
		w.buildMap2CoastalBay(52)
	case MapThemeRiverValley:
		w.buildMap3RiverValley(52)
	case MapThemeEmeraldLake:
		w.buildMap4EmeraldLake(52)
	case MapThemeTwinCities:
		w.buildMap5TwinCities(52)
	default:
		return false
	}
	if theme != MapThemeEmeraldLake {
		w.expandCity(52)
	}
	supported := w.HasBRTEligibleRoads()
	builtinBRTSupport.Store(theme, supported)
	return supported
}
