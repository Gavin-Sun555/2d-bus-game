package sim

import (
	"math"
	"slices"
)

// A static broad phase. Queries retain original object order; road nearest
// queries retain the original tie breaker and evaluate the original distance.
type boundaryBox struct{ min, max Vec2 }
type boundaryNode struct {
	box                     boundaryBox
	left, right, first, end int
	maxHalf                 float64
}
type boundaryTree struct {
	nodes []boundaryNode
	ids   []int
}
type boundaryStop struct {
	pos                    Vec2
	heading, length, width float64
	lanes                  int
	cos, sin               float64
}
type boundaryGeometryCache struct {
	width                            float64
	roads                            []RoadSegment
	buildings                        []Building
	stops                            []boundaryStop
	buildingTree, stopTree, roadTree boundaryTree
	roadHalves                       []float64
	boundX, boundY                   float64
	roadMin, roadMax                 [2]float64
}

// BeginBoundaryQueries validates public, editable map inputs once per batch.
// Pair with EndBoundaryQueries. Outside a batch CheckBoundaries always reads
// live geometry, so standalone calls after direct edits cannot use stale data.
// Nested controllers share the enclosing batch; map geometry is immutable in
// a batch (passenger counts may change). Different widths fall back to scans.
func (w *World) BeginBoundaryQueries(width float64) {
	if w.boundaryDepth == 0 && !w.boundaryCache.matches(w, width) {
		w.boundaryCache = buildBoundaryGeometry(w, width)
	}
	w.boundaryDepth++
}
func (w *World) EndBoundaryQueries() { w.boundaryDepth-- }

func boundaryStopKey(s *BusStop) boundaryStop {
	return boundaryStop{pos: s.Pos, heading: s.Heading, length: s.Length, width: s.Width, lanes: s.RoadLanes}
}
func (c *boundaryGeometryCache) matches(w *World, width float64) bool {
	if c == nil || c.width != width || !slices.Equal(c.roads, w.Roads) || !slices.Equal(c.buildings, w.Buildings) || len(c.stops) != len(w.Stops) {
		return false
	}
	for i, s := range w.Stops {
		if s == nil {
			return false
		}
		key := boundaryStopKey(s)
		old := c.stops[i]
		old.cos, old.sin = 0, 0
		if old != key {
			return false
		}
	}
	return true
}

func buildBoundaryGeometry(w *World, width float64) *boundaryGeometryCache {
	if !finiteBoundary(width) || width <= 0 {
		return nil
	}
	c := &boundaryGeometryCache{width: width, roads: slices.Clone(w.Roads), buildings: slices.Clone(w.Buildings), roadMin: [2]float64{math.Inf(1), math.Inf(1)}, roadMax: [2]float64{math.Inf(-1), math.Inf(-1)}}
	buildings := make([]boundaryBox, len(w.Buildings))
	for i, b := range w.Buildings {
		if !finiteBoundary(b.Pos.X, b.Pos.Y, b.W, b.H) || b.W < 0 || b.H < 0 {
			return nil
		}
		buildings[i] = boundaryBox{b.Pos.Sub(Vec2{b.W / 2, b.H / 2}), b.Pos.Add(Vec2{b.W / 2, b.H / 2})}.padded()
	}
	stops := make([]boundaryBox, len(w.Stops))
	for _, s := range w.Stops {
		if s == nil || !finiteBoundary(s.Pos.X, s.Pos.Y, s.Heading, s.Length, s.Width) || s.Length < 0 || s.Width < 0 || s.RoadLanes < 0 {
			return nil
		}
		k := boundaryStopKey(s)
		k.cos, k.sin = math.Cos(k.heading), math.Sin(k.heading)
		taper := 110.0
		if s.Length > 200 {
			taper = 260
		}
		// Include the full approach queue, exit taper AND shelter.
		loLong, hiLong := -1200.0, math.Max(42, s.Length/2+taper)
		loLat := math.Min(24, -float64(s.RoadLanes)/2*width-s.Width*.48-15)
		hiLat := math.Max(48, s.Width*.5+12)
		box := boundaryBox{Vec2{math.Inf(1), math.Inf(1)}, Vec2{math.Inf(-1), math.Inf(-1)}}
		for _, longitudinal := range []float64{loLong, hiLong} {
			for _, lateral := range []float64{loLat, hiLat} {
				p := s.Pos.Add(Vec2{k.cos*longitudinal - k.sin*lateral, k.sin*longitudinal + k.cos*lateral})
				box = box.include(boundaryBox{p, p})
			}
		}
		stops[len(c.stops)] = box.padded()
		c.stops = append(c.stops, k)
	}
	roads := make([]boundaryBox, len(w.Roads))
	for _, r := range w.Roads {
		if !finiteBoundary(r.P1.X, r.P1.Y, r.P2.X, r.P2.Y, r.GetWidth(width)) || r.GetWidth(width) < 0 {
			return nil
		}
		for axis := 0; axis < 2; axis++ {
			a, b := r.P1.X, r.P2.X
			if axis == 1 {
				a, b = r.P1.Y, r.P2.Y
			}
			c.roadMin[axis] = math.Min(c.roadMin[axis], math.Min(a, b))
			c.roadMax[axis] = math.Max(c.roadMax[axis], math.Max(a, b))
		}
		box := boundaryBox{r.P1, r.P1}.include(boundaryBox{r.P2, r.P2})
		roads[len(c.roadHalves)] = box.padded()
		c.roadHalves = append(c.roadHalves, r.GetWidth(width)/2)
	}
	c.buildingTree.build(buildings, nil)
	c.stopTree.build(stops, nil)
	c.roadTree.build(roads, c.roadHalves)
	c.boundX, c.boundY = w.PerimeterBounds()
	return c
}
func finiteBoundary(values ...float64) bool {
	for _, v := range values {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}
func (b boundaryBox) padded() boundaryBox {
	pad := 1e-8 * (1 + math.Max(math.Max(math.Abs(b.min.X), math.Abs(b.max.X)), math.Max(math.Abs(b.min.Y), math.Abs(b.max.Y))))
	b.min = b.min.Sub(Vec2{pad, pad})
	b.max = b.max.Add(Vec2{pad, pad})
	return b
}
func (b boundaryBox) include(a boundaryBox) boundaryBox {
	return boundaryBox{Vec2{math.Min(b.min.X, a.min.X), math.Min(b.min.Y, a.min.Y)}, Vec2{math.Max(b.max.X, a.max.X), math.Max(b.max.Y, a.max.Y)}}
}
func (b boundaryBox) distanceSquared(p Vec2) float64 {
	x := math.Max(0, math.Max(b.min.X-p.X, p.X-b.max.X))
	y := math.Max(0, math.Max(b.min.Y-p.Y, p.Y-b.max.Y))
	return x*x + y*y
}
func (t *boundaryTree) build(boxes []boundaryBox, halves []float64) {
	for i := range boxes {
		t.ids = append(t.ids, i)
	}
	if len(boxes) > 0 {
		t.buildNode(boxes, halves, 0, len(boxes))
	}
}
func (t *boundaryTree) buildNode(boxes []boundaryBox, halves []float64, first, end int) int {
	box := boxes[t.ids[first]]
	half := 0.0
	for _, id := range t.ids[first:end] {
		box = box.include(boxes[id])
		if halves != nil {
			half = math.Max(half, halves[id])
		}
	}
	n := len(t.nodes)
	t.nodes = append(t.nodes, boundaryNode{box: box, first: first, end: end, maxHalf: half, left: -1, right: -1})
	if end-first > 4 {
		xAxis := box.max.X-box.min.X >= box.max.Y-box.min.Y
		slices.SortFunc(t.ids[first:end], func(a, b int) int {
			ac, bc := boxes[a].min.Y+boxes[a].max.Y, boxes[b].min.Y+boxes[b].max.Y
			if xAxis {
				ac, bc = boxes[a].min.X+boxes[a].max.X, boxes[b].min.X+boxes[b].max.X
			}
			if ac < bc {
				return -1
			}
			if ac > bc {
				return 1
			}
			return a - b
		})
		mid := (first + end) / 2
		left := t.buildNode(boxes, halves, first, mid)
		right := t.buildNode(boxes, halves, mid, end)
		t.nodes[n].left, t.nodes[n].right = left, right
	}
	return n
}
func (t *boundaryTree) candidates(p Vec2, radius float64, dst []int) []int {
	if len(t.nodes) > 0 {
		dst = t.query(0, p, math.Max(0, radius), dst)
	}
	slices.Sort(dst)
	return dst
}
func (t *boundaryTree) query(index int, p Vec2, radius float64, dst []int) []int {
	n := &t.nodes[index]
	if p.X+radius < n.box.min.X || p.X-radius > n.box.max.X || p.Y+radius < n.box.min.Y || p.Y-radius > n.box.max.Y {
		return dst
	}
	if n.left < 0 {
		return append(dst, t.ids[n.first:n.end]...)
	}
	dst = t.query(n.left, p, radius, dst)
	return t.query(n.right, p, radius, dst)
}

func (c *boundaryGeometryCache) nearestRoad(index int, p Vec2, best *float64, bestID *int, push *Vec2) {
	n := &c.roadTree.nodes[index]
	// Distance to a segment is at least distance to its AABB. Subtract
	// the widest road in this subtree; pad outward to retain float ties.
	limit := *best + n.maxHalf
	if limit < 0 {
		return
	}
	limit += 1e-7 * (1 + math.Abs(limit))
	if n.box.distanceSquared(p) > limit*limit {
		return
	}
	if n.left < 0 {
		for _, id := range c.roadTree.ids[n.first:n.end] {
			r := c.roads[id]
			d, normal := DistToSegment(p, r.P1, r.P2)
			d -= c.roadHalves[id]
			if d < *best || (d == *best && id < *bestID) {
				*best, *bestID, *push = d, id, normal
			}
		}
		return
	}
	first, second := n.left, n.right
	if c.roadTree.nodes[second].box.distanceSquared(p) < c.roadTree.nodes[first].box.distanceSquared(p) {
		first, second = second, first
	}
	c.nearestRoad(first, p, best, bestID, push)
	c.nearestRoad(second, p, best, bestID, push)
}
