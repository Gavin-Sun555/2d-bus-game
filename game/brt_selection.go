package game

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

func (g *Game) centerBRTAvailable(theme sim.MapTheme) bool {
	if theme != sim.MapThemeCustom {
		return sim.BuiltinMapSupportsCenterBRT(theme)
	}
	if g.World.CustomMap != nil {
		return sim.CustomMapSupportsCenterBRT(g.World.CustomMap)
	}
	if !g.menuCustomMapChecked {
		g.menuCustomMapChecked = true
		data, err := sim.LoadCustomMapFromFile(sim.DefaultCustomMapFileName)
		if err != nil {
			data = sim.GetDefaultCustomMapData()
		}
		g.menuCustomMapPreview = data
	}
	return sim.CustomMapSupportsCenterBRT(g.menuCustomMapPreview)
}
