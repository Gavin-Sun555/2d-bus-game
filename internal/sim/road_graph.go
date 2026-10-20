package sim

import (
	"math"
	"sort"
)

type gridStopInfo struct {
	entryI, entryJ int
	exitI, exitJ   int
	dir            int // 0: East, 1: South, 2: West, 3: North
}

func getGridCoordinates(w *World) ([]float64, []float64) {
	if w == nil || len(w.Roads) == 0 {
		return []float64{-2600 * MapGridScale, -1300 * MapGridScale, 0, 1300 * MapGridScale, 2600 * MapGridScale}, []float64{-1200 * MapGridScale, -600 * MapGridScale, 0, 600 * MapGridScale, 1200 * MapGridScale}
	}
	xVals := make([]float64, 0, 16)
	yVals := make([]float64, 0, 16)

	addX := func(x float64) {
		for _, existing := range xVals {
			if math.Abs(existing-x) < 15.0 {
				return
			}
		}
		xVals = append(xVals, x)
	}
	addY := func(y float64) {
		for _, existing := range yVals {
			if math.Abs(existing-y) < 15.0 {
				return
			}
		}
		yVals = append(yVals, y)
	}

	for _, r := range w.Roads {
		addX(r.P1.X)
		addX(r.P2.X)
		addY(r.P1.Y)
		addY(r.P2.Y)
	}
	for _, inter := range w.Intersections {
		addX(inter.X)
		addY(inter.Y)
	}

	sort.Float64s(xVals)
	sort.Float64s(yVals)

	if len(xVals) < 2 {
		xVals = []float64{-2600 * MapGridScale, -1300 * MapGridScale, 0, 1300 * MapGridScale, 2600 * MapGridScale}
	}
	if len(yVals) < 2 {
		yVals = []float64{-1200 * MapGridScale, -600 * MapGridScale, 0, 600 * MapGridScale, 1200 * MapGridScale}
	}

	return xVals, yVals
}

func isRoadConnecting(p1, p2 Vec2, w *World) bool {
	if w == nil || len(w.Roads) == 0 {
		return true
	}
	const tol = 20.0
	isEW := math.Abs(p1.Y-p2.Y) < tol
	isNS := math.Abs(p1.X-p2.X) < tol
	if isEW {
		minX := math.Min(p1.X, p2.X)
		maxX := math.Max(p1.X, p2.X)
		if maxX-minX < 1.0 {
			return true
		}
		y := (p1.Y + p2.Y) * 0.5
		for _, r := range w.Roads {
			if math.Abs(r.P1.Y-r.P2.Y) < tol && math.Abs(r.P1.Y-y) < tol {
				rMinX := math.Min(r.P1.X, r.P2.X)
				rMaxX := math.Max(r.P1.X, r.P2.X)
				if minX >= rMinX-tol && maxX <= rMaxX+tol {
					return true
				}
			}
		}
		return false
	}
	if isNS {
		minY := math.Min(p1.Y, p2.Y)
		maxY := math.Max(p1.Y, p2.Y)
		if maxY-minY < 1.0 {
			return true
		}
		x := (p1.X + p2.X) * 0.5
		for _, r := range w.Roads {
			if math.Abs(r.P1.X-r.P2.X) < tol && math.Abs(r.P1.X-x) < tol {
				rMinY := math.Min(r.P1.Y, r.P2.Y)
				rMaxY := math.Max(r.P1.Y, r.P2.Y)
				if minY >= rMinY-tol && maxY <= rMaxY+tol {
					return true
				}
			}
		}
		return false
	}
	return false
}

func getStopGridInfo(s *BusStop, xs, ys []float64) gridStopInfo {
	isEW := math.Abs(math.Cos(s.Heading)) > math.Abs(math.Sin(s.Heading))
	if isEW {
		j := 0
		minDist := math.MaxFloat64
		for idx, yk := range ys {
			d := math.Abs(s.RoadCenter.Y - yk)
			if d < minDist {
				minDist = d
				j = idx
			}
		}
		i := 0
		found := false
		for idx := 0; idx < len(xs)-1; idx++ {
			if s.RoadCenter.X >= xs[idx]-20 && s.RoadCenter.X <= xs[idx+1]+20 {
				i = idx
				found = true
				break
			}
		}
		if !found {
			minD := math.MaxFloat64
			for idx := 0; idx < len(xs)-1; idx++ {
				midX := (xs[idx] + xs[idx+1]) * 0.5
				d := math.Abs(s.RoadCenter.X - midX)
				if d < minD {
					minD = d
					i = idx
				}
			}
		}
		if math.Cos(s.Heading) > 0 {
			// Eastbound (Dir 0): moves from xs[i] to xs[i+1]
			return gridStopInfo{entryI: i, entryJ: j, exitI: i + 1, exitJ: j, dir: 0}
		} else {
			// Westbound (Dir 2): moves from xs[i+1] to xs[i]
			return gridStopInfo{entryI: i + 1, entryJ: j, exitI: i, exitJ: j, dir: 2}
		}
	} else {
		i := 0
		minDist := math.MaxFloat64
		for idx, xk := range xs {
			d := math.Abs(s.RoadCenter.X - xk)
			if d < minDist {
				minDist = d
				i = idx
			}
		}
		j := 0
		found := false
		for idx := 0; idx < len(ys)-1; idx++ {
			if s.RoadCenter.Y >= ys[idx]-20 && s.RoadCenter.Y <= ys[idx+1]+20 {
				j = idx
				found = true
				break
			}
		}
		if !found {
			minD := math.MaxFloat64
			for idx := 0; idx < len(ys)-1; idx++ {
				midY := (ys[idx] + ys[idx+1]) * 0.5
				d := math.Abs(s.RoadCenter.Y - midY)
				if d < minD {
					minD = d
					j = idx
				}
			}
		}
		if math.Sin(s.Heading) > 0 {
			// Southbound (Dir 1): moves from ys[j] to ys[j+1]
			return gridStopInfo{entryI: i, entryJ: j, exitI: i, exitJ: j + 1, dir: 1}
		} else {
			// Northbound (Dir 3): moves from ys[j+1] to ys[j]
			return gridStopInfo{entryI: i, entryJ: j + 1, exitI: i, exitJ: j, dir: 3}
		}
	}
}

func RoadLaneOffset(roadCoord float64, isEW bool, lw float64, w *World, posOptional ...Vec2) float64 {
	lanes := 2
	if w != nil {
		lanes = w.GetRoadLanes(roadCoord, isEW, posOptional...)
	} else {
		if isEW {
			if math.Abs(roadCoord-(-1200*MapGridScale)) < 20 {
				lanes = 6
			} else if math.Abs(roadCoord) < 20 || math.Abs(roadCoord-1200*MapGridScale) < 20 {
				lanes = 4
			}
		} else {
			if math.Abs(roadCoord) < 20 {
				lanes = 4
			}
		}
	}

	if lanes == 8 && w != nil {
		pos := Vec2{X: roadCoord, Y: roadCoord}
		if len(posOptional) > 0 {
			pos = posOptional[0]
		} else {
			for _, road := range w.Roads {
				if (math.Abs(road.P1.Y-road.P2.Y) < 1) == isEW {
					coord := road.P1.Y
					if !isEW {
						coord = road.P1.X
					}
					if math.Abs(coord-roadCoord) < 25 {
						pos = road.P1.Add(road.P2).Mul(.5)
						break
					}
				}
			}
		}
		if road, ok := w.RoadAt(roadCoord, isEW, pos); ok && road.HasBusLane() {
			return lw * 3.5
		}
	}
	if lanes >= 6 {
		return lw * 2.5
	}
	if lanes >= 4 {
		return lw * 1.5
	}
	return lw * 0.5
}

func calcSegOffset(roadCoord float64, isEW bool, dir int, lw float64, w *World, posOptional ...Vec2) float64 {
	d := RoadLaneOffset(roadCoord, isEW, lw, w, posOptional...)
	if isEW {
		if dir == 0 { // East: driving lane is on south side (+Y)
			return d
		}
		// West: driving lane is on north side (-Y)
		return -d
	}
	if dir == 1 { // South: driving lane is on west side (-X)
		return -d
	}
	// North: driving lane is on east side (+X)
	return d
}

func getDirUnit(dir int) Vec2 {
	switch dir {
	case 0:
		return Vec2{X: 1, Y: 0}
	case 1:
		return Vec2{X: 0, Y: 1}
	case 2:
		return Vec2{X: -1, Y: 0}
	case 3:
		return Vec2{X: 0, Y: -1}
	default:
		return Vec2{}
	}
}

func getDirPerp(dir int) Vec2 {
	u := getDirUnit(dir)
	return Vec2{X: -u.Y, Y: u.X}
}

func isRoad4LaneAt(w *World, coord float64, isEW bool, pos Vec2) bool {
	if w != nil {
		return w.IsRoad4Lane(coord, isEW, pos)
	}
	if isEW {
		return math.Abs(coord-(-1200*MapGridScale)) < 20 || math.Abs(coord) < 20 || math.Abs(coord-1200*MapGridScale) < 20
	}
	return math.Abs(coord) < 20 || math.Abs(coord-(-2600*MapGridScale)) < 20 || math.Abs(coord-2600*MapGridScale) < 20
}

type roadSegmentLine struct {
	isEW      bool
	roadCoord float64
	dir       int
	offset    float64
}

func getLanePt(i, j, dir int, xs, ys []float64, lw float64, w *World) Vec2 {
	if dir == 0 || dir == 2 { // East or West
		yRoad := ys[j]
		var probePos Vec2
		if dir == 0 {
			nextX := xs[i] + 50.0
			if i+1 < len(xs) {
				nextX = (xs[i] + xs[i+1]) * 0.5
			}
			probePos = Vec2{X: nextX, Y: yRoad}
		} else {
			prevX := xs[i] - 50.0
			if i > 0 {
				prevX = (xs[i] + xs[i-1]) * 0.5
			}
			probePos = Vec2{X: prevX, Y: yRoad}
		}
		off := calcSegOffset(yRoad, true, dir, lw, w, probePos)
		return Vec2{X: xs[i], Y: yRoad + off}
	}
	// South or North
	xRoad := xs[i]
	var probePos Vec2
	if dir == 1 {
		nextY := ys[j] + 50.0
		if j+1 < len(ys) {
			nextY = (ys[j] + ys[j+1]) * 0.5
		}
		probePos = Vec2{X: xRoad, Y: nextY}
	} else {
		prevY := ys[j] - 50.0
		if j > 0 {
			prevY = (ys[j] + ys[j-1]) * 0.5
		}
		probePos = Vec2{X: xRoad, Y: prevY}
	}
	off := calcSegOffset(xRoad, false, dir, lw, w, probePos)
	return Vec2{X: xRoad + off, Y: ys[j]}
}
