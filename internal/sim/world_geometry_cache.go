package sim

import (
	"math"
	"slices"
)

type junctionCell struct{ x, y int32 }

func junctionCellAt(p Vec2) junctionCell {
	return junctionCell{int32(math.Floor(p.X / 1024)), int32(math.Floor(p.Y / 1024))}
}

type cachedWorldJunction struct {
	ew, ns     float64
	geometry   JunctionGeometry
	roundabout bool
}
type junctionBounds struct {
	center       Vec2
	halfX, halfY float64
}

type worldJunctionTableEntry struct {
	valid  bool
	center Vec2
	value  cachedWorldJunction
}

type worldGeometryCache struct {
	roads              []RoadSegment
	roundabouts        []Roundabout
	centers            []Vec2
	width              float64
	revision           uint64
	entries            map[Vec2]cachedWorldJunction
	table              []worldJunctionTableEntry
	tableMask          uint64
	tableOverflow      bool
	cells              map[junctionCell][]junctionBounds
	cellGrid           [][]junctionBounds
	gridOrigin         junctionCell
	gridW, gridH       int32
	maxHalfX, maxHalfY float64
}

func worldJunctionSlot(p Vec2) uint64 {
	x := math.Float64bits(p.X + 0)
	y := math.Float64bits(p.Y + 0)
	x ^= x >> 32
	y ^= y >> 32
	h := x*0x9e3779b97f4a7c15 ^ y*0xbf58476d1ce4e5b9
	return h ^ (h >> 30)
}

func (c *worldGeometryCache) lookup(p Vec2) (cachedWorldJunction, bool) {
	if len(c.table) == 0 {
		return cachedWorldJunction{}, false
	}
	slot := worldJunctionSlot(p) & c.tableMask
	e := &c.table[slot]
	if !e.valid {
		return cachedWorldJunction{}, false
	}
	if e.center == p {
		return e.value, true
	}
	for probe := uint64(1); probe < 8; probe++ {
		e = &c.table[(slot+probe)&c.tableMask]
		if !e.valid {
			return cachedWorldJunction{}, false
		}
		if e.center == p {
			return e.value, true
		}
	}
	if !c.tableOverflow {
		return cachedWorldJunction{}, false
	}
	v, ok := c.entries[p]
	return v, ok
}

// PrepareJunctionGeometry validates static inputs once per simulation update.
// Public map slices can be edited in place, so pointer/length checks alone are
// insufficient. Queries within the update then share immutable geometry.
func (w *World) PrepareJunctionGeometry(width float64) {
	if w == nil {
		return
	}
	old := w.geometryCache
	if old != nil && old.width == width && slices.Equal(old.roads, w.Roads) && slices.Equal(old.roundabouts, w.Roundabouts) && slices.Equal(old.centers, w.Intersections) {
		return
	}
	w.geometryRevision++
	revision := w.geometryRevision
	tableSize := 256
	for tableSize < len(w.Intersections)*4 {
		tableSize <<= 1
	}
	c := &worldGeometryCache{
		roads:       slices.Clone(w.Roads),
		roundabouts: slices.Clone(w.Roundabouts),
		centers:     slices.Clone(w.Intersections),
		width:       width,
		revision:    revision,
		entries:     make(map[Vec2]cachedWorldJunction, len(w.Intersections)),
		table:       make([]worldJunctionTableEntry, tableSize),
		tableMask:   uint64(tableSize - 1),
		cells:       make(map[junctionCell][]junctionBounds),
	}
	for _, p := range w.Intersections {
		ew, ns := w.uncachedIntersectionRoadDimensions(p, width)
		g := NewJunctionGeometry(ew, ns, width)
		rb := w.uncachedIsAtRoundabout(p)
		val := cachedWorldJunction{ew, ns, g, rb}
		c.entries[p] = val
		slot := worldJunctionSlot(p) & c.tableMask
		stored := false
		for probe := uint64(0); probe < 8; probe++ {
			idx := (slot + probe) & c.tableMask
			if !c.table[idx].valid || c.table[idx].center == p {
				c.table[idx] = worldJunctionTableEntry{valid: true, center: p, value: val}
				stored = true
				break
			}
		}
		if !stored {
			c.tableOverflow = true
		}
		if !rb {
			cell := junctionCellAt(p)
			c.cells[cell] = append(c.cells[cell], junctionBounds{p, g.HalfX, g.HalfY})
		}
		c.maxHalfX = math.Max(c.maxHalfX, g.HalfX)
		c.maxHalfY = math.Max(c.maxHalfY, g.HalfY)
	}
	if len(c.cells) > 0 {
		first := true
		var minX, minY, maxX, maxY int32
		for cell := range c.cells {
			if first {
				minX, maxX, minY, maxY = cell.x, cell.x, cell.y, cell.y
				first = false
			} else {
				if cell.x < minX {
					minX = cell.x
				}
				if cell.x > maxX {
					maxX = cell.x
				}
				if cell.y < minY {
					minY = cell.y
				}
				if cell.y > maxY {
					maxY = cell.y
				}
			}
		}
		gw := int64(maxX) - int64(minX) + 1
		gh := int64(maxY) - int64(minY) + 1
		if gw > 0 && gh > 0 && gw*gh <= 16384 {
			c.gridOrigin = junctionCell{minX, minY}
			c.gridW, c.gridH = int32(gw), int32(gh)
			c.cellGrid = make([][]junctionBounds, gw*gh)
			for cell, list := range c.cells {
				c.cellGrid[int32(cell.x-minX)*c.gridH+int32(cell.y-minY)] = list
			}
		}
	}
	w.geometryCache = c
}

// GeometryRevision identifies the validated topology used by route caches.
func (w *World) GeometryRevision() uint64 {
	if w == nil || w.geometryCache == nil {
		return 0
	}
	return w.geometryCache.revision
}

func (c *worldGeometryCache) inside(pos Vec2, margin float64) bool {
	lo := junctionCellAt(pos.Sub(Vec2{X: c.maxHalfX + math.Max(0, margin), Y: c.maxHalfY + math.Max(0, margin)}))
	hi := junctionCellAt(pos.Add(Vec2{X: c.maxHalfX + math.Max(0, margin), Y: c.maxHalfY + math.Max(0, margin)}))
	for x := lo.x; x <= hi.x; x++ {
		for y := lo.y; y <= hi.y; y++ {
			var list []junctionBounds
			if len(c.cellGrid) > 0 {
				gx, gy := x-c.gridOrigin.x, y-c.gridOrigin.y
				if gx < 0 || gx >= c.gridW || gy < 0 || gy >= c.gridH {
					continue
				}
				list = c.cellGrid[gx*c.gridH+gy]
			} else {
				list = c.cells[junctionCell{x, y}]
			}
			for _, j := range list {
				if math.Abs(pos.X-j.center.X) <= j.halfX+margin && math.Abs(pos.Y-j.center.Y) <= j.halfY+margin {
					return true
				}
			}
		}
	}
	return false
}
