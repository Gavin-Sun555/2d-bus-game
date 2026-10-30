package sim

import (
	"fmt"
	"math"
	"sort"
)

type BlockStopPair struct {
	Center     Vec2
	Heading    float64
	Lanes      int
	Start, End float64
}

// EligibleBlockStops lists street blocks with a junction at both ends. Bridges
// and water crossings are excluded rather than creating unusable platforms.
func (w *World) EligibleBlockStops(lw float64) []BlockStopPair {
	blocks := []BlockStopPair{}
	seen := map[[3]int]bool{}
	for _, r := range w.Roads {
		ew := math.Abs(r.P2.Y-r.P1.Y) < 1
		lo, hi, coord := r.P1.Y, r.P2.Y, r.P1.X
		if ew {
			lo, hi, coord = r.P1.X, r.P2.X, r.P1.Y
		}
		if lo > hi {
			lo, hi = hi, lo
		}
		cuts := []float64{}
		for _, j := range w.Intersections {
			along, cross := j.Y, j.X
			if ew {
				along, cross = j.X, j.Y
			}
			if math.Abs(cross-coord) < 1 && along >= lo && along <= hi {
				cuts = append(cuts, along)
			}
		}
		sort.Float64s(cuts)
		for i := 1; i < len(cuts); i++ {
			a, b := cuts[i-1], cuts[i]
			if b-a < 550*w.GridScale() {
				continue
			}
			center := Vec2{X: coord, Y: (a + b) / 2}
			h := math.Pi / 2
			if ew {
				center = Vec2{X: (a + b) / 2, Y: coord}
				h = 0
			}
			key := [3]int{int(center.X), int(center.Y), int(h * 100)}
			if seen[key] {
				continue
			}
			seen[key] = true
			water := false
			for _, v := range w.Waters {
				if math.Abs(center.X-v.Center.X) < v.Width/2-r.GetWidth(lw) && math.Abs(center.Y-v.Center.Y) < v.Height/2-r.GetWidth(lw) {
					water = true
					break
				}
			}
			if water {
				continue
			}
			bridge := false
			for _, v := range w.Bridges {
				if math.Abs(math.Sin(v.Heading-h)) > .1 {
					continue
				}
				along, cross := v.Pos.Y, v.Pos.X
				if ew {
					along, cross = v.Pos.X, v.Pos.Y
				}
				if math.Abs(cross-coord) < 1 && along+v.Length/2 > a && along-v.Length/2 < b {
					bridge = true
					break
				}
			}
			if bridge {
				continue
			}
			blocks = append(blocks, BlockStopPair{Center: center, Heading: h, Lanes: r.LaneCount(), Start: a, End: b})
		}
	}
	return blocks
}
func (w *World) addBlockStops(lw float64) {
	for _, block := range w.EligibleBlockStops(lw) {
		for side := 0; side < 2; side++ {
			h := NormalizeAngle(block.Heading + float64(side)*math.Pi)
			ew := math.Abs(math.Cos(h)) > .5
			exists := false
			for _, s := range w.Stops {
				along, cross := s.RoadCenter.Y, s.RoadCenter.X
				coord := block.Center.X
				if ew {
					along, cross = s.RoadCenter.X, s.RoadCenter.Y
					coord = block.Center.Y
				}
				if math.Abs(cross-coord) < 1 && along > block.Start && along < block.End && math.Cos(s.Heading-h) > .99 {
					exists = true
					break
				}
			}
			if exists {
				continue
			}
			id := len(w.Stops)
			direction, en := "南行", "Southbound"
			if ew {
				direction, en = "东行", "Eastbound"
			}
			if side == 1 {
				direction, en = "北行", "Northbound"
				if ew {
					direction, en = "西行", "Westbound"
				}
			}
			st := NewBusStop(id, fmt.Sprintf("街区%d·%s站 (Block %d %s)", id+1, direction, id+1, en), block.Center, h, block.Lanes, RGBA(56, 189, 248, 255), lw, 0)
			if w.InsideJunction(st.Pos, lw, 70+160) {
				continue
			}
			collision := false
			// Both the bay and the boarding shelter must be clear of authored buildings.
			for _, building := range w.Buildings {
				for _, p := range []Vec2{st.Pos, st.ShelterPos} {
					if math.Abs(p.X-building.Pos.X) < building.W/2+70 && math.Abs(p.Y-building.Pos.Y) < building.H/2+st.Width/2 {
						collision = true
						break
					}
				}
				if collision {
					break
				}
			}
			if !collision {
				w.Stops = append(w.Stops, st)
			}
		}
	}
}

// Screen-space picking remains usable when a dense city is fully zoomed out.
func (w *World) StopAtScreen(cam *Camera, p Vec2, radius float64) int {
	id := -1
	nearest := radius
	for _, s := range w.Stops {
		d := cam.WorldToScreen(s.Pos).Distance(p)
		if d < nearest {
			id = s.ID
			nearest = d
		}
	}
	return id
}
