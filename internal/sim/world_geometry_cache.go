package sim

import (
	"math"
	"slices"
)

type junctionCell struct{ x, y int }

func junctionCellAt(p Vec2) junctionCell {
	return junctionCell{int(math.Floor(p.X / 1024)), int(math.Floor(p.Y / 1024))}
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

type worldGeometryCache struct {
	roads              []RoadSegment
	roundabouts        []Roundabout
	centers            []Vec2
	width              float64
	revision           uint64
	entries            map[Vec2]cachedWorldJunction
	cells              map[junctionCell][]junctionBounds
	maxHalfX, maxHalfY float64
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
	c := &worldGeometryCache{roads: slices.Clone(w.Roads), roundabouts: slices.Clone(w.Roundabouts), centers: slices.Clone(w.Intersections), width: width, revision: revision, entries: make(map[Vec2]cachedWorldJunction, len(w.Intersections)), cells: make(map[junctionCell][]junctionBounds)}
	for _, p := range w.Intersections {
		ew, ns := w.uncachedIntersectionRoadDimensions(p, width)
		g := NewJunctionGeometry(ew, ns, width)
		rb := w.uncachedIsAtRoundabout(p)
		c.entries[p] = cachedWorldJunction{ew, ns, g, rb}
		if !rb {
			cell := junctionCellAt(p)
			c.cells[cell] = append(c.cells[cell], junctionBounds{p, g.HalfX, g.HalfY})
		}
		c.maxHalfX = math.Max(c.maxHalfX, g.HalfX)
		c.maxHalfY = math.Max(c.maxHalfY, g.HalfY)
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
			for _, j := range c.cells[junctionCell{x, y}] {
				if math.Abs(pos.X-j.center.X) <= j.halfX+margin && math.Abs(pos.Y-j.center.Y) <= j.halfY+margin {
					return true
				}
			}
		}
	}
	return false
}
