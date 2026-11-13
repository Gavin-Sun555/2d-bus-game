package view

import (
	"image/color"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type scenePalette struct {
	ground, grain, paving, asphalt, curb              color.RGBA
	water, shallows, foam, foliage, leafLight, accent color.RGBA
}

func sceneColors(theme sim.MapTheme) scenePalette {
	switch theme {
	case sim.MapThemeCoastalBay:
		return scenePalette{
			ground:    rgb(58, 106, 76),
			grain:     rgb(72, 120, 88),
			paving:    rgb(196, 188, 168),
			asphalt:   rgb(38, 48, 56),
			curb:      rgb(228, 222, 206),
			water:     rgb(20, 105, 145),
			shallows:  rgb(52, 172, 186),
			foam:      rgb(215, 245, 238),
			foliage:   rgb(26, 108, 72),
			leafLight: rgb(88, 174, 118),
			accent:    rgb(242, 148, 65),
		}
	case sim.MapThemeRiverValley:
		return scenePalette{
			ground:    rgb(52, 94, 64),
			grain:     rgb(66, 108, 76),
			paving:    rgb(136, 144, 142),
			asphalt:   rgb(36, 46, 52),
			curb:      rgb(178, 186, 184),
			water:     rgb(18, 96, 108),
			shallows:  rgb(54, 142, 138),
			foam:      rgb(172, 224, 212),
			foliage:   rgb(28, 82, 54),
			leafLight: rgb(78, 142, 88),
			accent:    rgb(218, 142, 72),
		}
	case sim.MapThemeEmeraldLake:
		return scenePalette{
			ground:    rgb(139, 162, 138),
			grain:     rgb(159, 178, 148),
			paving:    rgb(201, 206, 181),
			asphalt:   rgb(52, 65, 62),
			curb:      rgb(222, 221, 194),
			water:     rgb(40, 109, 99),
			shallows:  rgb(79, 145, 122),
			foam:      rgb(177, 213, 176),
			foliage:   rgb(45, 109, 70),
			leafLight: rgb(125, 169, 95),
			accent:    rgb(168, 77, 55),
		}
	case sim.MapThemeTwinCities:
		return scenePalette{
			ground:    rgb(48, 88, 70),
			grain:     rgb(62, 102, 82),
			paving:    rgb(142, 156, 168),
			asphalt:   rgb(32, 42, 52),
			curb:      rgb(182, 196, 208),
			water:     rgb(24, 98, 128),
			shallows:  rgb(58, 152, 172),
			foam:      rgb(176, 228, 238),
			foliage:   rgb(32, 96, 72),
			leafLight: rgb(92, 166, 126),
			accent:    rgb(56, 189, 248),
		}
	default:
		return scenePalette{
			ground:    rgb(56, 94, 68),
			grain:     rgb(68, 108, 78),
			paving:    rgb(118, 128, 136),
			asphalt:   rgb(32, 43, 55),
			curb:      rgb(162, 174, 182),
			water:     rgb(28, 92, 116),
			shallows:  rgb(48, 126, 144),
			foam:      rgb(148, 198, 208),
			foliage:   rgb(34, 86, 58),
			leafLight: rgb(94, 152, 98),
			accent:    rgb(84, 186, 204),
		}
	}
}

func rgb(r, g, b uint8) color.RGBA { return color.RGBA{R: r, G: g, B: b, A: 255} }
func tint(a, b color.RGBA, t float64) color.RGBA {
	return rgb(uint8(float64(a.R)*(1-t)+float64(b.R)*t), uint8(float64(a.G)*(1-t)+float64(b.G)*t), uint8(float64(a.B)*(1-t)+float64(b.B)*t))
}
