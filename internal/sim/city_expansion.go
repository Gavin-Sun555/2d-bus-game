package sim

import (
	"fmt"
	"math"
	"sort"
)

// Stable theme IDs preserve authored custom maps; the retired lake is no longer selectable.
func BuiltinMapThemes() []MapTheme {
	return []MapTheme{MapThemeMetropolis, MapThemeCoastalBay, MapThemeRiverValley, MapThemeTwinCities}
}
func SelectableMapThemes() []MapTheme { return append(BuiltinMapThemes(), MapThemeCustom) }

func (w *World) PerimeterBounds() (float64, float64) {
	scale := w.GridScale()
	x, y := 3200.0*scale, 1480.0*scale
	for _, r := range w.Roads {
		for _, p := range []Vec2{r.P1, r.P2} {
			x = math.Max(x, math.Abs(p.X)+600*scale)
			y = math.Max(y, math.Abs(p.Y)+280*scale)
		}
	}
	return x, y
}

// Extend the city footprint, not the blocks: new streets retain the original
// 600-unit north/south and 650/1300-unit east/west spacing before MapGridScale.
func (w *World) expandCity(lw float64) {
	for i := range w.Roads {
		r := &w.Roads[i]
		if r.P1.Y == r.P2.Y {
			if r.P1.X == -2600 {
				r.P1.X = -5200
			}
			if r.P2.X == 2600 {
				r.P2.X = 5200
			}
		} else if !(w.Theme == MapThemeTwinCities && math.Abs(r.P1.X) < 1300) {
			if r.P1.Y == -1200 {
				r.P1.Y = -2400
			}
			if r.P2.Y == 1200 {
				r.P2.Y = 2400
			}
		}
		if w.Theme == MapThemeMetropolis {
			if r.Type == RoadBusBoulevard8Lane || r.Lanes == 2 {
				r.Lanes = 8
				r.Type = RoadBusBoulevard8Lane
			}
		}
	}
	add := func(a, b Vec2, lanes int, typ RoadType, name string) {
		w.Roads = append(w.Roads, RoadSegment{P1: a, P2: b, Lanes: lanes, Type: typ, HasLines: true, Name: name})
	}
	for _, y := range []float64{-2400, -1800, 1800, 2400} {
		lanes, typ := 4, RoadAvenue4Lane
		if w.Theme == MapThemeMetropolis {
			lanes, typ = 8, RoadBusBoulevard8Lane
			if math.Abs(y) == 2400 {
				lanes, typ = 6, RoadExpressway6Lane
			}
		}
		if w.Theme == MapThemeCoastalBay {
			add(Vec2{-5200, y}, Vec2{0, y}, lanes, typ, "海岸新区大道")
			add(Vec2{1800, y}, Vec2{5200, y}, 4, RoadAvenue4Lane, "深水港新区大道")
		} else if w.Theme == MapThemeTwinCities {
			add(Vec2{-5200, y}, Vec2{-1300, y}, lanes, typ, "临空新区大道")
			add(Vec2{1300, y}, Vec2{5200, y}, lanes, typ, "科创新城大道")
		} else {
			add(Vec2{-5200, y}, Vec2{5200, y}, lanes, typ, "城市拓展大道")
		}
	}
	xs := []float64{-5200, -3900, 3900, 5200}
	if w.Theme == MapThemeTwinCities {
		xs = []float64{-5200, -4550, -3900, -3250, 3250, 3900, 4550, 5200}
	}
	if w.Theme == MapThemeCoastalBay {
		xs = []float64{-5200, -4550, -3900, -3250, 3400, 4200, 5200}
	}
	for _, x := range xs {
		lanes, typ := 4, RoadAvenue4Lane
		if w.Theme == MapThemeMetropolis {
			lanes, typ = 8, RoadBusBoulevard8Lane
		}
		if w.Theme == MapThemeRiverValley {
			add(Vec2{x, -2400}, Vec2{x, -600}, lanes, typ, "江北新区路")
			add(Vec2{x, 600}, Vec2{x, 2400}, lanes, typ, "江南新区路")
		} else {
			add(Vec2{x, -2400}, Vec2{x, 2400}, lanes, typ, "新区纵向大道")
		}
	}
	// Complete the offshore grid, without building streets across the open bay.
	if w.Theme == MapThemeCoastalBay {
		for _, x := range []float64{1800, 2600} {
			lanes, typ, south := 4, RoadAvenue4Lane, 600.0
			if x == 1800 {
				lanes, typ, south = 2, RoadStreet2Lane, 1200
			}
			add(Vec2{x, -2400}, Vec2{x, -600}, lanes, typ, "港北联络路")
			add(Vec2{x, south}, Vec2{x, 2400}, lanes, typ, "港南联络路")
		}
	}
	for i := range w.Waters {
		water := &w.Waters[i]
		if w.Theme == MapThemeCoastalBay {
			water.Center = Vec2{900, 0}
			water.Width = 1800
			water.Height = 5600
		} else if w.Theme == MapThemeTwinCities {
			water.Height = 5400
		} else {
			water.Width = 12000
		}
	}
	w.RebuildIntersections()
	// Candidate platforms are centered between actual junctions; never place a bay
	// inside a junction, on a bridge, or in a lane-change approach taper.
	type candidate struct {
		p     Vec2
		h     float64
		lanes int
	}
	candidates := []candidate{}
	for _, r := range w.Roads {
		ew := r.P1.Y == r.P2.Y
		cuts := []float64{}
		for _, p := range []Vec2{r.P1, r.P2} {
			if ew {
				cuts = append(cuts, p.X)
			} else {
				cuts = append(cuts, p.Y)
			}
		}
		for _, p := range w.Intersections {
			if ew && p.Y == r.P1.Y && p.X >= r.P1.X && p.X <= r.P2.X {
				cuts = append(cuts, p.X)
			}
			if !ew && p.X == r.P1.X && p.Y >= r.P1.Y && p.Y <= r.P2.Y {
				cuts = append(cuts, p.Y)
			}
		}
		sort.Float64s(cuts)
		for j := 1; j < len(cuts); j++ {
			if cuts[j]-cuts[j-1] < 550 {
				continue
			}
			mid := (cuts[j] + cuts[j-1]) / 2
			p := Vec2{r.P1.X, mid}
			h := math.Pi / 2
			if ew {
				p = Vec2{mid, r.P1.Y}
				h = 0
			}
			if math.Abs(p.X) <= 2600 && math.Abs(p.Y) <= 1200 {
				continue
			}
			candidates = append(candidates, candidate{p, h, r.Lanes})
		}
	}
	// Farthest-first distributes all 36 added stations across the new districts.
	for len(w.Stops) < 48 && len(candidates) > 0 {
		best, bestD := 0, -1.0
		for i, c := range candidates {
			d := math.Inf(1)
			for _, st := range w.Stops {
				d = math.Min(d, c.p.Distance(st.RoadCenter))
			}
			if d > bestD {
				best, bestD = i, d
			}
		}
		c := candidates[best]
		candidates = append(candidates[:best], candidates[best+1:]...)
		if bestD < 200 {
			continue
		}
		id := len(w.Stops)
		district := "新城"
		en := "New District"
		switch w.Theme {
		case MapThemeMetropolis:
			district = "都会新区"
			en = "CBD District"
		case MapThemeCoastalBay:
			district = "海岸新区"
			en = "Coastal District"
			if c.p.X > 0 {
				district = "港口新区"
				en = "Port District"
			}
		case MapThemeRiverValley:
			district = "江北新区"
			en = "North Bank"
			if c.p.Y > 0 {
				district = "江南新区"
				en = "South Bank"
			}
		case MapThemeTwinCities:
			district = "临空新区"
			en = "Airport District"
			if c.p.X > 0 {
				district = "科创新城"
				en = "Tech District"
			}
		}
		w.Stops = append(w.Stops, NewBusStop(id, fmt.Sprintf("%s%d站 (%s %d)", district, id-11, en, id-11), c.p, c.h, c.lanes, RGBA(14, 165, 233, 255), lw, 4))
	}
}
