package view

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type scenicCell struct{ x, y int }

func scenicCellAt(p sim.Vec2) scenicCell {
	const cellSize = 768
	return scenicCell{int(math.Floor(p.X / cellSize)), int(math.Floor(p.Y / cellSize))}
}

type scenicBlocker struct {
	center       sim.Vec2
	halfX, halfY float64
}

// Placement uses the same clearance rectangles for static features and
// previously accepted lots, indexed by the cells they intersect.
type scenicPlacement map[scenicCell][]scenicBlocker

func (s scenicPlacement) add(center sim.Vec2, halfX, halfY float64) {
	extent := sim.Vec2{X: halfX, Y: halfY}
	lo, hi := scenicCellAt(center.Sub(extent)), scenicCellAt(center.Add(extent))
	blocker := scenicBlocker{center, halfX, halfY}
	for y := lo.y; y <= hi.y; y++ {
		for x := lo.x; x <= hi.x; x++ {
			cell := scenicCell{x, y}
			s[cell] = append(s[cell], blocker)
		}
	}
}

func newScenicPlacement(w *sim.World, laneWidth float64) scenicPlacement {
	s := make(scenicPlacement)
	for _, center := range w.Intersections {
		if !w.IsAtRoundabout(center) {
			box := w.JunctionAt(center, laneWidth)
			s.add(center, box.HalfX+45, box.HalfY+45)
		}
	}
	for _, road := range w.Roads {
		padding := road.GetWidth(laneWidth)/2 + sim.ShoulderWidth + 8
		delta := road.P2.Sub(road.P1)
		s.add(road.P1.Add(road.P2).Mul(.5), math.Abs(delta.X)/2+padding, math.Abs(delta.Y)/2+padding)
	}
	for _, stop := range w.Stops {
		along, depth := stop.Length/2+25, stop.Width/2+18
		if math.Abs(math.Sin(stop.Heading)) > .5 {
			along, depth = depth, along
		}
		s.add(stop.Pos, along, depth)
		s.add(stop.ShelterPos, 34, 34)
	}
	for _, b := range w.Buildings {
		s.add(b.Pos, b.W/2+16, b.H/2+16)
	}
	for _, rail := range w.Railways() {
		access := rail.StationCenter.Sub(rail.StationApproach)
		s.add(rail.StationApproach.Add(access.Mul(.5)), 56, math.Abs(access.Y)/2)
	}
	for _, crossing := range w.StationCrossings() {
		if crossing.Kind != sim.StationFootbridge && crossing.Kind != sim.StationUnderpass {
			continue
		}
		forward := sim.UnitFromAngle(crossing.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		landing := float64(crossing.Lanes)*laneWidth/2 + sim.ShoulderWidth + 11
		for _, side := range [...]float64{-1, 1} {
			s.add(crossing.Center.Add(right.Mul(side*landing)), 28, 28)
		}
	}
	return s
}

func (s scenicPlacement) clear(w *sim.World, b sim.Building, waterfront bool) bool {
	extent := sim.Vec2{X: b.W / 2, Y: b.H / 2}
	lo, hi := scenicCellAt(b.Pos.Sub(extent)), scenicCellAt(b.Pos.Add(extent))
	for y := lo.y; y <= hi.y; y++ {
		for x := lo.x; x <= hi.x; x++ {
			for _, other := range s[scenicCell{x, y}] {
				if math.Abs(b.Pos.X-other.center.X) < extent.X+other.halfX &&
					math.Abs(b.Pos.Y-other.center.Y) < extent.Y+other.halfY {
					return false
				}
			}
		}
	}
	if !waterfront {
		for _, water := range w.Waters {
			if math.Abs(b.Pos.X-water.Center.X) < (b.W+water.Width)/2+20 &&
				math.Abs(b.Pos.Y-water.Center.Y) < (b.H+water.Height)/2+20 {
				return false
			}
		}
	}
	for _, river := range w.Rivers {
		if math.Abs(b.Pos.Y-river.Y) < (b.H+river.Height)/2+30 {
			return false
		}
	}
	for _, rb := range w.Roundabouts {
		if b.Pos.Distance(rb.Center) < rb.Radius+math.Hypot(b.W, b.H)/2+30 {
			return false
		}
	}
	return true
}
