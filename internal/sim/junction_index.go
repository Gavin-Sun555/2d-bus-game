package sim

import (
	"math"
	"slices"
)

type indexedJunctionKey struct {
	inter      *TrafficIntersection
	center     Vec2
	dimensions approachDimensionsKey
}
type junctionIndex struct {
	keys             []indexedJunctionKey
	cells            map[junctionCell][]int
	maxStop, maxHalf float64
}

// PrepareJunctionIndex refreshes topology only; phases remain live pointers.
// Keeping original slice indices preserves first-match signal/priority rules.
func (tls *TrafficLightSystem) PrepareJunctionIndex(lw float64) {
	same := tls.junctionIndex != nil && len(tls.junctionIndex.keys) == len(tls.Intersections)
	if same {
		for i, it := range tls.Intersections {
			if tls.junctionIndex.keys[i] != (indexedJunctionKey{it, it.Center, approachDimensionsKey{lw, it.ApproachLanes, it.ApproachHasRoad}}) {
				same = false
				break
			}
		}
	}
	if same {
		return
	}
	c := &junctionIndex{cells: make(map[junctionCell][]int), keys: make([]indexedJunctionKey, len(tls.Intersections))}
	for i, it := range tls.Intersections {
		c.keys[i] = indexedJunctionKey{it, it.Center, approachDimensionsKey{lw, it.ApproachLanes, it.ApproachHasRoad}}
		k := junctionCellAt(it.Center)
		c.cells[k] = append(c.cells[k], i)
		ew, _, ns, _ := it.GetApproachDimensions(lw)
		c.maxStop = math.Max(c.maxStop, math.Max(ew, ns))
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			c.maxHalf = math.Max(c.maxHalf, it.ApproachRoadHalfWidth(heading, lw))
		}
	}
	tls.junctionIndex = c
}

// NearbyJunctions is a broad phase only: callers still apply all original rules.
func (tls *TrafficLightSystem) NearbyJunctions(pos Vec2, radius float64, ids []int, dst []*TrafficIntersection) ([]int, []*TrafficIntersection) {
	c := tls.junctionIndex
	if c == nil {
		return ids[:0], append(dst[:0], tls.Intersections...)
	}
	ids = ids[:0]
	dst = dst[:0]
	lo, hi := junctionCellAt(pos.Sub(Vec2{X: radius, Y: radius})), junctionCellAt(pos.Add(Vec2{X: radius, Y: radius}))
	for x := lo.x; x <= hi.x; x++ {
		for y := lo.y; y <= hi.y; y++ {
			ids = append(ids, c.cells[junctionCell{x, y}]...)
		}
	}
	slices.Sort(ids)
	for _, i := range ids {
		dst = append(dst, tls.Intersections[i])
	}
	return ids, dst
}

func (tls *TrafficLightSystem) CarJunctionQueryRadius(lw, speed, length, dt float64) float64 {
	if tls.junctionIndex == nil {
		return math.Inf(1)
	}
	look := math.Max(180, math.Max(lw*1.5+85, speed*speed/(2*220)+60))
	// Cover both road axes, vehicle motion and the post-move corridor publication.
	return tls.junctionIndex.maxStop + math.Max(tls.junctionIndex.maxHalf, 4.8*lw) + look + 2*length + (math.Abs(speed)+120*math.Abs(dt))*math.Abs(dt) + 1
}
