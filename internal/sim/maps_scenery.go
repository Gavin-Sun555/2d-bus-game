package sim

import (
	"image/color"
)

func (w *World) buildMetropolisBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		{cx: 650, cy: 300, w: 340, h: 160, col: RGBA(15, 23, 42, 255), roof: RGBA(30, 41, 59, 255)},
		{cx: -650, cy: 300, w: 320, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1720, cy: -810, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: 650, cy: -820, w: 360, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(236, 72, 153, 255)},
		{cx: -650, cy: -820, w: 360, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -1950, cy: -820, w: 360, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: -1950, cy: 850, w: 380, h: 160, col: RGBA(39, 39, 42, 255), roof: RGBA(63, 63, 70, 255)},
		{cx: -1950, cy: 300, w: 340, h: 150, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: -650, cy: 850, w: 340, h: 160, col: RGBA(248, 250, 252, 255), roof: RGBA(239, 68, 68, 255)},
		{cx: 1950, cy: 850, w: 380, h: 160, col: RGBA(24, 24, 27, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1950, cy: 300, w: 360, h: 150, col: RGBA(15, 23, 42, 255), roof: RGBA(217, 70, 239, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildMetropolisGreenery() {
	treeColor := RGBA(22, 101, 52, 255)
	for x := -1150.0; x <= -150.0; x += 100.0 {
		for y := 720.0; y <= 1080.0; y += 90.0 {
			w.Trees = append(w.Trees, Tree{Pos: Vec2{X: x, Y: y}, Radius: 16, Color: treeColor})
		}
	}
	for _, x := range []float64{-1625, -975, 325, 975, 1625} {
		for _, y := range []float64{-1050, -450, 450, 1050} {
			w.Trees = append(w.Trees,
				Tree{Pos: Vec2{X: x - 120, Y: y}, Radius: 14, Color: treeColor},
				Tree{Pos: Vec2{X: x + 120, Y: y}, Radius: 14, Color: treeColor},
			)
		}
	}
}

func (w *World) buildCoastalBayBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// Mainland Coast Blocks (Centered in 600px tall blocks)
		{cx: -1625, cy: -900, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: -900, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: -325, cy: -900, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(100, 116, 139, 255)},
		{cx: -1625, cy: -300, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(217, 119, 6, 255)},
		{cx: -975, cy: -300, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(6, 182, 212, 255)},
		{cx: -325, cy: -300, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: -1625, cy: 300, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: 300, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(6, 182, 212, 255)},
		{cx: -325, cy: 300, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -1625, cy: 900, w: 260, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: 900, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(16, 185, 129, 255)},
		// Island Port Buildings
		{cx: 2200, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: 2200, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(245, 158, 11, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildCoastalBayGreenery() {
	palmColor := RGBA(16, 185, 129, 255)
	for _, x := range []float64{-1625, -975, -325} {
		for _, y := range []float64{-900, -300, 300, 900} {
			w.Trees = append(w.Trees,
				Tree{Pos: Vec2{X: x - 180, Y: y}, Radius: 15, Color: palmColor},
				Tree{Pos: Vec2{X: x + 180, Y: y}, Radius: 15, Color: palmColor},
			)
		}
	}
}

func (w *World) buildRiverValleyBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// North Bank Foothill Town (Centered in 1300px x 600px blocks at Y = -900)
		{cx: -1950, cy: -950, w: 260, h: 120, col: RGBA(241, 245, 249, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: -650, cy: -850, w: 240, h: 120, col: RGBA(248, 250, 252, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: 650, cy: -900, w: 240, h: 120, col: RGBA(241, 245, 249, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1950, cy: -900, w: 260, h: 120, col: RGBA(248, 250, 252, 255), roof: RGBA(71, 85, 105, 255)},
		// South Bank Railway City (Centered in 1300px x 600px blocks at Y = 900)
		{cx: -1950, cy: 900, w: 260, h: 120, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -650, cy: 900, w: 240, h: 120, col: RGBA(15, 23, 42, 255), roof: RGBA(30, 41, 59, 255)},
		{cx: 650, cy: 900, w: 240, h: 120, col: RGBA(30, 41, 59, 255), roof: RGBA(168, 85, 247, 255)},
		{cx: 1950, cy: 900, w: 260, h: 120, col: RGBA(15, 23, 42, 255), roof: RGBA(234, 179, 8, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildRiverValleyGreenery() {
	pineColor := RGBA(5, 150, 105, 255)
	for _, x := range []float64{-1950, -650, 650, 1950} {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: x - 180, Y: -900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x + 180, Y: -900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x - 180, Y: 900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x + 180, Y: 900}, Radius: 16, Color: pineColor},
		)
	}
}

func (w *World) buildEmeraldLakeBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// Outer Shore Courtyards
		{cx: -1950, cy: -900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: 1950, cy: -900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: -1950, cy: 900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: 1950, cy: 900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		// Lake Island Pavilions on Shore
		{cx: -320, cy: -180, w: 140, h: 80, col: RGBA(254, 242, 242, 255), roof: RGBA(180, 83, 9, 255)},
		{cx: 320, cy: 180, w: 140, h: 80, col: RGBA(254, 242, 242, 255), roof: RGBA(180, 83, 9, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildEmeraldLakeGreenery() {
	willowColor := RGBA(34, 197, 94, 255)
	for _, x := range []float64{-320, 320} {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: x, Y: -180}, Radius: 14, Color: willowColor},
			Tree{Pos: Vec2{X: x, Y: 180}, Radius: 14, Color: willowColor},
		)
	}
}

func (w *World) buildTwinCitiesBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// West District: Logistics Hangars & Factories
		{cx: -1625, cy: -300, w: 260, h: 140, col: RGBA(51, 65, 85, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: -2275, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -1625, cy: 300, w: 260, h: 140, col: RGBA(51, 65, 85, 255), roof: RGBA(100, 116, 139, 255)},
		{cx: -2275, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(245, 158, 11, 255)},
		// East District: Software Towers & Campuses
		{cx: 1625, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(139, 92, 246, 255)},
		{cx: 2275, cy: -300, w: 260, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: 1625, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: 2275, cy: 300, w: 260, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(236, 72, 153, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildTwinCitiesGreenery() {
	parkColor := RGBA(16, 185, 129, 255)
	for y := -900.0; y <= 900.0; y += 180.0 {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: -250, Y: y}, Radius: 15, Color: parkColor},
			Tree{Pos: Vec2{X: 250, Y: y}, Radius: 15, Color: parkColor},
		)
	}
}
