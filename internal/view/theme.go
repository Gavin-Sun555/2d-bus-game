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
		return scenePalette{rgb(174, 160, 119), rgb(197, 181, 137), rgb(209, 199, 166), rgb(47, 60, 67), rgb(239, 223, 181), rgb(23, 100, 133), rgb(67, 168, 175), rgb(205, 237, 218), rgb(37, 114, 82), rgb(106, 167, 100), rgb(231, 158, 91)}
	case sim.MapThemeRiverValley:
		return scenePalette{rgb(73, 91, 65), rgb(92, 108, 74), rgb(145, 141, 116), rgb(44, 54, 52), rgb(189, 185, 156), rgb(35, 102, 106), rgb(73, 135, 123), rgb(156, 192, 170), rgb(30, 76, 56), rgb(82, 126, 75), rgb(191, 135, 91)}
	case sim.MapThemeEmeraldLake:
		return scenePalette{rgb(139, 162, 138), rgb(159, 178, 148), rgb(201, 206, 181), rgb(52, 65, 62), rgb(222, 221, 194), rgb(40, 109, 99), rgb(79, 145, 122), rgb(177, 213, 176), rgb(45, 109, 70), rgb(125, 169, 95), rgb(168, 77, 55)}
	case sim.MapThemeTwinCities:
		return scenePalette{rgb(35, 47, 62), rgb(43, 61, 76), rgb(91, 110, 126), rgb(33, 43, 56), rgb(139, 167, 182), rgb(32, 104, 123), rgb(55, 139, 149), rgb(145, 204, 210), rgb(43, 111, 93), rgb(94, 170, 132), rgb(116, 192, 213)}
	default:
		return scenePalette{rgb(24, 35, 47), rgb(34, 49, 62), rgb(75, 91, 109), rgb(30, 41, 55), rgb(139, 161, 178), rgb(26, 85, 109), rgb(43, 117, 135), rgb(140, 190, 199), rgb(40, 91, 71), rgb(91, 141, 99), rgb(81, 189, 206)}
	}
}

func rgb(r, g, b uint8) color.RGBA { return color.RGBA{R: r, G: g, B: b, A: 255} }
func tint(a, b color.RGBA, t float64) color.RGBA {
	return rgb(uint8(float64(a.R)*(1-t)+float64(b.R)*t), uint8(float64(a.G)*(1-t)+float64(b.G)*t), uint8(float64(a.B)*(1-t)+float64(b.B)*t))
}
