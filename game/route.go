package game

import (
	"fmt"
	"image/color"
	"math"
	"sort"
	"sync"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type routePathCacheKey struct {
	fromID int
	toID   int
	lwKey  int
}

var (
	routePathCacheMu sync.RWMutex
	routePathCache   = make(map[routePathCacheKey][]Vec2)
)

// ClearRoutePathCache clears all cached Dijkstra road paths between stops
func ClearRoutePathCache() {
	routePathCacheMu.Lock()
	routePathCache = make(map[routePathCacheKey][]Vec2)
	routePathCacheMu.Unlock()
}

// Route manages the planned sequence of bus stops
type Route struct {
	Name         string
	Color        color.RGBA
	StopIDs      []int // Sequence of stop IDs
	CurrentIndex int   // Index within StopIDs that bus is currently navigating towards
	TotalTrips   int   // Completed full loops

	cachedDesc      string
	cachedDescDirty bool
}

func NewRoute() *Route {
	return &Route{
		Name:            "Custom Route",
		Color:           RGBA(14, 165, 233, 255),
		cachedDescDirty: true,
	}
}

func NewDefaultRoute() *Route {
	// Default starting route visiting major stops in a continuous right-hand city loop:
	// Stop 0 (Central Terminal, Eastbound) -> Stop 2 (Civic Center, Southbound) ->
	// Stop 10 (Creative District, Westbound) -> Stop 8 (City Hospital, Northbound) -> Stop 0
	return &Route{
		Name:            "Line 1 (城市核心环线)",
		Color:           RGBA(14, 165, 233, 255), // Sky blue
		StopIDs:         []int{0, 2, 10, 8, 0},
		CurrentIndex:    0,
		TotalTrips:      0,
		cachedDescDirty: true,
	}
}

func (r *Route) AddStop(stopID int) {
	// Avoid immediate consecutive duplicates
	if len(r.StopIDs) > 0 && r.StopIDs[len(r.StopIDs)-1] == stopID {
		return
	}
	r.StopIDs = append(r.StopIDs, stopID)
	r.cachedDescDirty = true
}

func (r *Route) Clear() {
	r.StopIDs = make([]int, 0)
	r.CurrentIndex = 0
	r.cachedDescDirty = true
}

func (r *Route) GetCurrentTargetStopID() int {
	if len(r.StopIDs) == 0 {
		return -1
	}
	if r.CurrentIndex >= len(r.StopIDs) {
		r.CurrentIndex = 0
	}
	return r.StopIDs[r.CurrentIndex]
}

func (r *Route) AdvanceToNextStop() {
	if len(r.StopIDs) == 0 {
		return
	}
	prevStopID := r.StopIDs[r.CurrentIndex%len(r.StopIDs)]
	r.CurrentIndex++
	if r.CurrentIndex >= len(r.StopIDs) {
		r.CurrentIndex = 0
		r.TotalTrips++
		// Closed loops repeat the terminal as first and last stop ([0, a, b, 0]):
		// having just served the terminal, continue straight on to the next distinct stop.
		if len(r.StopIDs) > 1 && r.StopIDs[0] == prevStopID {
			r.CurrentIndex = 1
		}
	}
	r.cachedDescDirty = true
}

type gridStopInfo struct {
	entryI, entryJ int
	exitI, exitJ   int
	dir            int // 0: East, 1: South, 2: West, 3: North
}

func getGridCoordinates(w *World) ([]float64, []float64) {
	if w == nil || len(w.Roads) == 0 {
		return []float64{-2600, -1300, 0, 1300, 2600}, []float64{-1200, -600, 0, 600, 1200}
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
		xVals = []float64{-2600, -1300, 0, 1300, 2600}
	}
	if len(yVals) < 2 {
		yVals = []float64{-1200, -600, 0, 600, 1200}
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

func getRoadLaneOffset(roadCoord float64, isEW bool, lw float64, w *World) float64 {
	is4Lane := false
	if w != nil {
		for _, r := range w.Roads {
			if isEW && math.Abs(r.P1.Y-r.P2.Y) < 15.0 && math.Abs(r.P1.Y-roadCoord) < 15.0 {
				if r.Lanes >= 4 {
					is4Lane = true
					break
				}
			} else if !isEW && math.Abs(r.P1.X-r.P2.X) < 15.0 && math.Abs(r.P1.X-roadCoord) < 15.0 {
				if r.Lanes >= 4 {
					is4Lane = true
					break
				}
			}
		}
	} else {
		if isEW {
			if math.Abs(roadCoord-(-1200)) < 20 || math.Abs(roadCoord) < 20 || math.Abs(roadCoord-1200) < 20 {
				is4Lane = true
			}
		} else {
			if math.Abs(roadCoord) < 20 {
				is4Lane = true
			}
		}
	}

	if is4Lane {
		return lw * 1.5
	}
	return lw * 0.5
}

func calcSegOffset(roadCoord float64, isEW bool, dir int, lw float64, w *World) float64 {
	d := getRoadLaneOffset(roadCoord, isEW, lw, w)
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
		return math.Abs(coord-(-1200)) < 20 || math.Abs(coord) < 20 || math.Abs(coord-1200) < 20
	}
	return math.Abs(coord) < 20 || math.Abs(coord-(-2600)) < 20 || math.Abs(coord-2600) < 20
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
		off := calcSegOffset(yRoad, true, dir, lw, w)
		return Vec2{X: xs[i], Y: yRoad + off}
	}
	// South or North
	xRoad := xs[i]
	off := calcSegOffset(xRoad, false, dir, lw, w)
	return Vec2{X: xRoad + off, Y: ys[j]}
}

// GetRoadPathBetweenStops calculates an orthogonal path strictly along the driving lane on the proper side of each road,
// ensuring the line is drawn on the right-hand side of the road rather than on the center divider median.
// Results are cached for instant O(1) lookup during real-time rendering.
func GetRoadPathBetweenStops(s1, s2 *BusStop, laneWidth ...float64) []Vec2 {
	return GetRoadPathBetweenStopsInWorld(ActiveWorldForRoute, s1, s2, laneWidth...)
}

// GetRoadPathBetweenStopsInWorld calculates an orthogonal path between stops within the specified world
func GetRoadPathBetweenStopsInWorld(w *World, s1, s2 *BusStop, laneWidth ...float64) []Vec2 {
	if s1 == nil || s2 == nil {
		return nil
	}
	if s1.ID == s2.ID {
		return []Vec2{s1.Pos, s2.Pos}
	}
	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	key := routePathCacheKey{
		fromID: s1.ID,
		toID:   s2.ID,
		lwKey:  int(math.Round(lw * 10)),
	}

	routePathCacheMu.RLock()
	if cached, ok := routePathCache[key]; ok {
		routePathCacheMu.RUnlock()
		return cached
	}
	routePathCacheMu.RUnlock()

	res := computeRoadPathBetweenStops(w, s1, s2, lw)

	routePathCacheMu.Lock()
	routePathCache[key] = res
	routePathCacheMu.Unlock()

	return res
}

func computeRoadPathBetweenStops(w *World, s1, s2 *BusStop, lw float64) []Vec2 {
	if s1 == nil || s2 == nil {
		return nil
	}
	if s1.ID == s2.ID {
		return []Vec2{s1.Pos}
	}

	xs, ys := getGridCoordinates(w)
	numX := len(xs)
	numY := len(ys)

	info1 := getStopGridInfo(s1, xs, ys)
	info2 := getStopGridInfo(s2, xs, ys)

	isS1EW := math.Abs(math.Cos(s1.Heading)) > math.Abs(math.Sin(s1.Heading))
	s1Coord := s1.RoadCenter.Y
	if !isS1EW {
		s1Coord = s1.RoadCenter.X
	}
	s1Off := calcSegOffset(s1Coord, isS1EW, info1.dir, lw, w)

	isS2EW := math.Abs(math.Cos(s2.Heading)) > math.Abs(math.Sin(s2.Heading))
	s2Coord := s2.RoadCenter.Y
	if !isS2EW {
		s2Coord = s2.RoadCenter.X
	}
	s2Off := calcSegOffset(s2Coord, isS2EW, info2.dir, lw, w)

	// Check if s1 and s2 are on the exact same road segment in the exact same direction
	if info1.entryI == info2.entryI && info1.entryJ == info2.entryJ && info1.dir == info2.dir {
		cosH := math.Cos(s1.Heading)
		sinH := math.Sin(s1.Heading)
		isAhead := false
		if math.Abs(cosH) > 0.5 {
			isAhead = (s2.RoadCenter.X-s1.RoadCenter.X)*cosH > 0
		} else {
			isAhead = (s2.RoadCenter.Y-s1.RoadCenter.Y)*sinH > 0
		}
		if isAhead {
			var lanePt1, lanePt2 Vec2
			if isS1EW {
				lanePt1 = Vec2{X: s1.Pos.X, Y: s1Coord + s1Off}
				lanePt2 = Vec2{X: s2.Pos.X, Y: s1Coord + s1Off}
			} else {
				lanePt1 = Vec2{X: s1Coord + s1Off, Y: s1.Pos.Y}
				lanePt2 = Vec2{X: s1Coord + s1Off, Y: s2.Pos.Y}
			}
			raw := []Vec2{s1.Pos, lanePt1, lanePt2, s2.Pos}
			filtered := make([]Vec2, 0, len(raw))
			for _, pt := range raw {
				if len(filtered) == 0 || filtered[len(filtered)-1].Distance(pt) > 5 {
					filtered = append(filtered, pt)
				}
			}
			return insertRoundaboutArcs(filtered, w)
		}
	}

	startI, startJ, startDir := info1.exitI, info1.exitJ, info1.dir
	targetI, targetJ, targetDir := info2.entryI, info2.entryJ, info2.dir

	if startI >= numX {
		startI = numX - 1
	}
	if startJ >= numY {
		startJ = numY - 1
	}
	if targetI >= numX {
		targetI = numX - 1
	}
	if targetJ >= numY {
		targetJ = numY - 1
	}

	dx := [4]int{1, 0, -1, 0}
	dy := [4]int{0, 1, 0, -1}

	type state struct {
		i, j, dir int
	}

	dist := make([][][4]float64, numX)
	visited := make([][][4]bool, numX)
	prev := make([][][4]state, numX)
	hasPrev := make([][][4]bool, numX)

	for i := 0; i < numX; i++ {
		dist[i] = make([][4]float64, numY)
		visited[i] = make([][4]bool, numY)
		prev[i] = make([][4]state, numY)
		hasPrev[i] = make([][4]bool, numY)
		for j := 0; j < numY; j++ {
			for d := 0; d < 4; d++ {
				dist[i][j][d] = math.MaxFloat64
			}
		}
	}

	dist[startI][startJ][startDir] = 0

	for {
		// Find unvisited state with minimum distance
		minD := math.MaxFloat64
		var curr state
		found := false
		for i := 0; i < numX; i++ {
			for j := 0; j < numY; j++ {
				for d := 0; d < 4; d++ {
					if !visited[i][j][d] && dist[i][j][d] < minD {
						minD = dist[i][j][d]
						curr = state{i: i, j: j, dir: d}
						found = true
					}
				}
			}
		}
		if !found || minD == math.MaxFloat64 {
			break
		}

		visited[curr.i][curr.j][curr.dir] = true

		if curr.i == targetI && curr.j == targetJ && curr.dir == targetDir {
			break // Reached destination in proper arrival heading
		}

		// Action 1: Move forward to adjacent intersection along current direction
		ni := curr.i + dx[curr.dir]
		nj := curr.j + dy[curr.dir]
		if ni >= 0 && ni < numX && nj >= 0 && nj < numY {
			p1 := Vec2{X: xs[curr.i], Y: ys[curr.j]}
			p2 := Vec2{X: xs[ni], Y: ys[nj]}
			if isRoadConnecting(p1, p2, w) {
				moveCost := math.Abs(xs[ni]-xs[curr.i]) + math.Abs(ys[nj]-ys[curr.j])
				if dist[curr.i][curr.j][curr.dir]+moveCost < dist[ni][nj][curr.dir] {
					dist[ni][nj][curr.dir] = dist[curr.i][curr.j][curr.dir] + moveCost
					prev[ni][nj][curr.dir] = curr
					hasPrev[ni][nj][curr.dir] = true
				}
			}
		}

		// Action 2: Turn at current intersection to a different direction
		turnDirs := []int{(curr.dir + 1) % 4, (curr.dir + 3) % 4, (curr.dir + 2) % 4}
		turnCosts := []float64{30.0, 60.0, 5000.0} // Right turn (30), Left turn (60), U-turn (5000)
		for tIdx, nd := range turnDirs {
			nni := curr.i + dx[nd]
			nnj := curr.j + dy[nd]
			roadAvail := false
			if nni >= 0 && nni < numX && nnj >= 0 && nnj < numY {
				if isRoadConnecting(Vec2{X: xs[curr.i], Y: ys[curr.j]}, Vec2{X: xs[nni], Y: ys[nnj]}, w) {
					roadAvail = true
				}
			}
			if roadAvail {
				tc := turnCosts[tIdx]
				if dist[curr.i][curr.j][curr.dir]+tc < dist[curr.i][curr.j][nd] {
					dist[curr.i][curr.j][nd] = dist[curr.i][curr.j][curr.dir] + tc
					prev[curr.i][curr.j][nd] = curr
					hasPrev[curr.i][curr.j][nd] = true
				}
			}
		}
	}

	// If target was not reached, fallback to straight line
	if !visited[targetI][targetJ][targetDir] && !(startI == targetI && startJ == targetJ && startDir == targetDir) {
		return []Vec2{s1.Pos, s2.Pos}
	}

	// Reconstruct path of states from target back to start
	curr := state{i: targetI, j: targetJ, dir: targetDir}
	revStates := []state{curr}
	for {
		if curr.i == startI && curr.j == startJ && curr.dir == startDir {
			break
		}
		if !hasPrev[curr.i][curr.j][curr.dir] {
			break
		}
		curr = prev[curr.i][curr.j][curr.dir]
		revStates = append(revStates, curr)
	}

	states := make([]state, len(revStates))
	for idx := range revStates {
		states[idx] = revStates[len(revStates)-1-idx]
	}

	// Assemble orthogonal driving-lane waypoints
	pts := make([]Vec2, 0, len(states)*3+6)
	pts = append(pts, s1.Pos)

	// 1. Project s1.Pos onto s1 driving lane
	if isS1EW {
		pts = append(pts, Vec2{X: s1.Pos.X, Y: s1Coord + s1Off})
	} else {
		pts = append(pts, Vec2{X: s1Coord + s1Off, Y: s1.Pos.Y})
	}

	// 2. Waypoints for intersection traversals and corners
	for k := 0; k < len(states); k++ {
		st := states[k]
		interPt := Vec2{X: xs[st.i], Y: ys[st.j]}
		var nearRb *Roundabout
		for idx := range w.Roundabouts {
			if interPt.Distance(w.Roundabouts[idx].Center) < 40.0 {
				nearRb = &w.Roundabouts[idx]
				break
			}
		}

		lastK := k
		for lastK+1 < len(states) && states[lastK+1].i == st.i && states[lastK+1].j == st.j {
			lastK++
		}
		isTurn := (lastK > k && states[lastK].dir != st.dir)

		if isTurn {
			prevDir := st.dir
			currDir := states[lastK].dir

			if nearRb != nil {
				circR := getRoundaboutOuterLaneRadius(*nearRb)
				var pEnter, pExit Vec2
				yOffPrev := calcSegOffset(ys[st.j], true, prevDir, lw, w)
				xOffPrev := calcSegOffset(xs[st.i], false, prevDir, lw, w)
				yOffCurr := calcSegOffset(ys[st.j], true, currDir, lw, w)
				xOffCurr := calcSegOffset(xs[st.i], false, currDir, lw, w)

				switch prevDir {
				case 0: // Eastbound
					pEnter = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOffPrev}
				case 2: // Westbound
					pEnter = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOffPrev}
				case 1: // Southbound
					pEnter = Vec2{X: nearRb.Center.X + xOffPrev, Y: nearRb.Center.Y - circR}
				case 3: // Northbound
					pEnter = Vec2{X: nearRb.Center.X + xOffPrev, Y: nearRb.Center.Y + circR}
				}

				switch currDir {
				case 0: // Eastbound
					pExit = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOffCurr}
				case 2: // Westbound
					pExit = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOffCurr}
				case 1: // Southbound
					pExit = Vec2{X: nearRb.Center.X + xOffCurr, Y: nearRb.Center.Y + circR}
				case 3: // Northbound
					pExit = Vec2{X: nearRb.Center.X + xOffCurr, Y: nearRb.Center.Y - circR}
				}
				pts = append(pts, pEnter, pExit)
			} else {
				turn := (currDir - prevDir + 4) % 4
				uPrev := getDirUnit(prevDir)
				rPrev := getDirPerp(prevDir)
				uCurr := getDirUnit(currDir)
				rCurr := getDirPerp(currDir)

				isPrevEW := (prevDir == 0 || prevDir == 2)
				prevCoord := ys[st.j]
				if !isPrevEW {
					prevCoord = xs[st.i]
				}
				isIncoming4Lane := isRoad4LaneAt(w, prevCoord, isPrevEW, interPt)

				isCurrEW := (currDir == 0 || currDir == 2)
				currCoord := ys[st.j]
				if !isCurrEW {
					currCoord = xs[st.i]
				}
				isOutgoing4Lane := isRoad4LaneAt(w, currCoord, isCurrEW, interPt)

				segLen := 600.0
				if len(pts) > 0 {
					segLen = interPt.Distance(pts[len(pts)-1])
				}

				if turn == 1 {
					// 90-degree right turn from outer lane into outer lane
					var corner Vec2
					if isPrevEW {
						yOff := calcSegOffset(ys[st.j], true, prevDir, lw, w)
						xOff := calcSegOffset(xs[st.i], false, currDir, lw, w)
						corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
					} else {
						xOff := calcSegOffset(xs[st.i], false, prevDir, lw, w)
						yOff := calcSegOffset(ys[st.j], true, currDir, lw, w)
						corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
					}
					pts = append(pts, corner)
				} else if turn == 3 || turn == 2 {
					// Left turn (turn == 3) or U-turn (turn == 2)
					dMerge := math.Min(180.0, segLen*0.55)
					if dMerge < lw*2.0 {
						dMerge = lw * 2.0
					}

					if isIncoming4Lane {
						// 4-lane avenue: merge orthogonally from outer travel lane (1.5*lw) into dedicated left-turn lane (0.5*lw)
						pMergeStart := interPt.Sub(uPrev.Mul(dMerge)).Add(rPrev.Mul(lw * 1.5))
						pMergeEnd := interPt.Sub(uPrev.Mul(dMerge)).Add(rPrev.Mul(lw * 0.5))
						pts = append(pts, pMergeStart, pMergeEnd)
					}

					if turn == 3 {
						// Left turn: corner apex at inner lane junction
						pCornerInner := interPt.Add(rPrev.Mul(lw * 0.5)).Add(rCurr.Mul(lw * 0.5))
						pts = append(pts, pCornerInner)

						if isOutgoing4Lane {
							availDist := 400.0
							if k+2 < len(states) {
								nextPt := Vec2{X: xs[states[k+2].i], Y: ys[states[k+2].j]}
								dNext := nextPt.Sub(interPt).Dot(uCurr)
								if dNext > 50.0 {
									availDist = dNext
								}
							} else {
								dS2 := s2.Pos.Sub(interPt).Dot(uCurr)
								if dS2 > 50.0 {
									availDist = dS2
								}
							}
							dMergeOut := math.Min(180.0, availDist*0.45)
							if dMergeOut > lw*2.0 {
								pTargetInner := interPt.Add(uCurr.Mul(dMergeOut)).Add(rCurr.Mul(lw * 0.5))
								pTargetOuter := interPt.Add(uCurr.Mul(dMergeOut)).Add(rCurr.Mul(lw * 1.5))
								pts = append(pts, pTargetInner, pTargetOuter)
							}
						}
					} else {
						// U-turn (turn == 2): orthogonal loop across center divider from inner lane
						outOff := lw * 0.5
						if isOutgoing4Lane {
							outOff = lw * 1.5
						}
						pA := interPt.Add(rPrev.Mul(lw * 0.5))
						pB := interPt.Add(rCurr.Mul(outOff))
						pts = append(pts, pA, pB)
					}
				}
			}
			// Skip all intermediate states at this intersection
			k = lastK
		} else if k > 0 || nearRb != nil {
			if nearRb != nil {
				circR := getRoundaboutOuterLaneRadius(*nearRb)
				dir := st.dir
				yOff := calcSegOffset(ys[st.j], true, dir, lw, w)
				xOff := calcSegOffset(xs[st.i], false, dir, lw, w)
				var pEnter, pExit Vec2
				switch dir {
				case 0:
					pEnter = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOff}
					pExit = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOff}
				case 2:
					pEnter = Vec2{X: nearRb.Center.X + circR, Y: nearRb.Center.Y + yOff}
					pExit = Vec2{X: nearRb.Center.X - circR, Y: nearRb.Center.Y + yOff}
				case 1:
					pEnter = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y - circR}
					pExit = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y + circR}
				case 3:
					pEnter = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y + circR}
					pExit = Vec2{X: nearRb.Center.X + xOff, Y: nearRb.Center.Y - circR}
				}
				pts = append(pts, pEnter, pExit)
			} else {
				// Straight traversal across intersection
				pt := getLanePt(st.i, st.j, st.dir, xs, ys, lw, w)
				pts = append(pts, pt)
			}
		}
	}

	// 3. Project s2.Pos onto s2 driving lane
	if isS2EW {
		pts = append(pts, Vec2{X: s2.Pos.X, Y: s2Coord + s2Off})
	} else {
		pts = append(pts, Vec2{X: s2Coord + s2Off, Y: s2.Pos.Y})
	}

	// 4. Target stop bay
	pts = append(pts, s2.Pos)

	// Filter out duplicate or extremely close points
	filtered := make([]Vec2, 0, len(pts))
	for _, pt := range pts {
		if len(filtered) == 0 || filtered[len(filtered)-1].Distance(pt) > 5 {
			filtered = append(filtered, pt)
		}
	}

	// Collinear simplification along grid axes
	if len(filtered) >= 3 {
		simplified := make([]Vec2, 0, len(filtered))
		simplified = append(simplified, filtered[0])
		for i := 1; i < len(filtered)-1; i++ {
			pPrev := simplified[len(simplified)-1]
			pCurr := filtered[i]
			pNext := filtered[i+1]
			if math.Abs(pPrev.Y-pCurr.Y) < 1.0 && math.Abs(pCurr.Y-pNext.Y) < 1.0 {
				continue
			}
			if math.Abs(pPrev.X-pCurr.X) < 1.0 && math.Abs(pCurr.X-pNext.X) < 1.0 {
				continue
			}
			simplified = append(simplified, pCurr)
		}
		simplified = append(simplified, filtered[len(filtered)-1])
		filtered = simplified
	}

	return insertRoundaboutArcs(filtered, w)
}

// getRoundaboutOuterLaneRadius returns the center radius of the outer travel lane of a roundabout
func getRoundaboutOuterLaneRadius(rb Roundabout) float64 {
	rApron := rb.InnerRadius + 16.0
	rOuter := rb.Radius
	rMid := (rApron + rOuter) * 0.5
	return (rMid + rOuter) * 0.5
}

// insertRoundaboutArcs curves route line segments that would otherwise cut across a roundabout
// smoothly along the circular carriageway in a counter-clockwise arc (following right-hand traffic rules).
func insertRoundaboutArcs(pts []Vec2, w *World) []Vec2 {
	if w == nil || len(w.Roundabouts) == 0 || len(pts) < 2 {
		return pts
	}

	res := make([]Vec2, 0, len(pts)+16)
	res = append(res, pts[0])

	for k := 0; k < len(pts)-1; k++ {
		p1 := pts[k]
		p2 := pts[k+1]

		segHandled := false
		for _, rb := range w.Roundabouts {
			circR := getRoundaboutOuterLaneRadius(rb)
			v := p2.Sub(p1)
			lenV2 := v.X*v.X + v.Y*v.Y
			if lenV2 < 1e-4 {
				continue
			}

			// Distance from roundabout center to segment
			tClosest := ((rb.Center.X-p1.X)*v.X + (rb.Center.Y-p1.Y)*v.Y) / lenV2
			if tClosest < 0 {
				tClosest = 0
			} else if tClosest > 1 {
				tClosest = 1
			}
			closestPt := p1.Add(v.Mul(tClosest))
			distClosest := closestPt.Distance(rb.Center)

			// If the straight segment passes through the roundabout inner core
			if distClosest < circR-5.0 {
				f := p1.Sub(rb.Center)
				a := lenV2
				b := 2.0 * (f.X*v.X + f.Y*v.Y)
				c := (f.X*f.X + f.Y*f.Y) - circR*circR
				disc := b*b - 4.0*a*c

				var enterPt, exitPt Vec2
				if disc >= 0 {
					sqrtDisc := math.Sqrt(disc)
					t1 := (-b - sqrtDisc) / (2.0 * a)
					t2 := (-b + sqrtDisc) / (2.0 * a)

					if t1 >= -0.15 && t1 <= 1.15 {
						enterPt = p1.Add(v.Mul(math.Max(0, math.Min(1, t1))))
					} else {
						enterPt = p1
					}
					if t2 >= -0.15 && t2 <= 1.15 {
						exitPt = p1.Add(v.Mul(math.Max(0, math.Min(1, t2))))
					} else {
						exitPt = p2
					}
				} else {
					enterPt = p1
					exitPt = p2
				}

				enterAngle := math.Atan2(enterPt.Y-rb.Center.Y, enterPt.X-rb.Center.X)
				exitAngle := math.Atan2(exitPt.Y-rb.Center.Y, exitPt.X-rb.Center.X)

				// Counter-clockwise rotary circulation: angle decreases from enterAngle to exitAngle
				dTheta := enterAngle - exitAngle
				for dTheta < 0 {
					dTheta += 2 * math.Pi
				}
				for dTheta >= 2*math.Pi {
					dTheta -= 2 * math.Pi
				}
				if dTheta < 0.1 {
					dTheta = 2 * math.Pi
				}

				// Generate smooth arc waypoints along circR
				numSteps := int(math.Ceil(dTheta / (math.Pi / 24.0)))
				if numSteps < 12 {
					numSteps = 12
				}
				if res[len(res)-1].Distance(enterPt) > 4.0 {
					res = append(res, enterPt)
				}
				for step := 1; step <= numSteps; step++ {
					ang := enterAngle - dTheta*float64(step)/float64(numSteps)
					arcPt := rb.Center.Add(Vec2{
						X: math.Cos(ang) * circR,
						Y: math.Sin(ang) * circR,
					})
					if res[len(res)-1].Distance(arcPt) > 4.0 {
						res = append(res, arcPt)
					}
				}

				segHandled = true
				break
			}
		}

		if !segHandled {
			if res[len(res)-1].Distance(p2) > 4.0 {
				res = append(res, p2)
			}
		} else {
			inInnerIsland := false
			for _, rb := range w.Roundabouts {
				if p2.Distance(rb.Center) < rb.InnerRadius-2.0 {
					inInnerIsland = true
					break
				}
			}
			if !inInnerIsland && res[len(res)-1].Distance(p2) > 4.0 {
				res = append(res, p2)
			}
		}
	}

	// Filter out any stray waypoint that falls inside any roundabout inner island
	clean := make([]Vec2, 0, len(res))
	for _, pt := range res {
		inIsland := false
		for _, rb := range w.Roundabouts {
			if pt.Distance(rb.Center) < rb.InnerRadius-2.0 {
				inIsland = true
				break
			}
		}
		if !inIsland {
			if len(clean) == 0 || clean[len(clean)-1].Distance(pt) > 3.0 {
				clean = append(clean, pt)
			}
		}
	}

	return clean
}

// DrawRouteLine draws connecting route lines along the driving lanes on the side of the road.
// In Driver Mode (!isPlanMode), it strictly draws ONLY the route to the next target stop, eliminating road confusion.
// In Planning Mode (isPlanMode), it draws the full planned transit line with the upcoming segment highlighted.
func (r *Route) DrawRouteLine(screen *ebiten.Image, cam *Camera, world *World, isPlanMode bool, laneWidth ...float64) {
	if len(r.StopIDs) < 1 {
		return
	}

	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	if !isPlanMode {
		// DRIVER MODE: Only draw the suggested route leading to the next stop!
		targetStopID := r.GetCurrentTargetStopID()
		if targetStopID < 0 || targetStopID >= len(world.Stops) {
			return
		}

		prevIdx := (r.CurrentIndex - 1 + len(r.StopIDs)) % len(r.StopIDs)
		prevStopID := r.StopIDs[prevIdx]

		s1 := world.Stops[prevStopID]
		s2 := world.Stops[targetStopID]

		var path []Vec2
		if s1.ID != s2.ID {
			path = GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		} else if len(r.StopIDs) > 1 {
			path = []Vec2{s1.Pos, s2.Pos}
		}

		if len(path) < 2 {
			return
		}

		lineColor := RGBA(250, 204, 21, 240) // Glowing Gold for active navigation line
		drawSegmentPath(screen, cam, path, lineColor)
		return
	}

	// PLANNING MODE: Draw all planned segments so player can review full line
	for i := 0; i < len(r.StopIDs); i++ {
		currID := r.StopIDs[i]
		nextID := r.StopIDs[(i+1)%len(r.StopIDs)]

		s1 := world.Stops[currID]
		s2 := world.Stops[nextID]
		if s1.ID == s2.ID {
			continue
		}

		path := GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		lineColor := RGBA(r.Color.R, r.Color.G, r.Color.B, 130)
		if i == (r.CurrentIndex-1+len(r.StopIDs))%len(r.StopIDs) || (r.CurrentIndex == 0 && i == len(r.StopIDs)-1) {
			lineColor = RGBA(250, 204, 21, 240) // Highlight upcoming segment in gold
		}
		drawSegmentPath(screen, cam, path, lineColor)
	}
}

func drawSegmentPath(screen *ebiten.Image, cam *Camera, path []Vec2, lineColor color.RGBA) {
	strokeWidth := float32(math.Max(2.5, 5.0*cam.Zoom))
	for sIdx := 0; sIdx < len(path)-1; sIdx++ {
		w1 := path[sIdx]
		w2 := path[sIdx+1]

		p1 := cam.WorldToScreen(w1)
		p2 := cam.WorldToScreen(w2)

		// Route line along the road
		vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y),
			strokeWidth, lineColor, true)

		// Draw directional arrows along segments that are sufficiently long on screen
		segLen := w1.Distance(w2)
		if segLen*cam.Zoom > 22 {
			mid := p1.Add(p2).Mul(0.5)
			dir := p2.Sub(p1).Normalize()
			perp := Vec2{X: -dir.Y, Y: dir.X}
			arrLen := math.Max(8.0, 12*cam.Zoom)
			arrWid := math.Max(4.5, 7*cam.Zoom)
			arr1 := mid.Sub(dir.Mul(arrLen)).Add(perp.Mul(arrWid))
			arr2 := mid.Sub(dir.Mul(arrLen)).Sub(perp.Mul(arrWid))
			arrLw := float32(math.Max(1.8, 3.0*cam.Zoom))
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr1.X), float32(arr1.Y),
				arrLw, lineColor, true)
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr2.X), float32(arr2.Y),
				arrLw, lineColor, true)
		}
	}
}

// DrawNavArrow draws a compass/directional navigation arrow around the bus towards the target stop along roads
func (r *Route) DrawNavArrow(screen *ebiten.Image, cam *Camera, bus *Bus, targetStop *BusStop, worldOptional ...*World) {
	if targetStop == nil {
		return
	}

	dist := bus.Pos.Distance(targetStop.Pos)
	if dist < 65 {
		return // Already arrived at stop
	}

	var w *World = ActiveWorldForRoute
	if len(worldOptional) > 0 && worldOptional[0] != nil {
		w = worldOptional[0]
	}

	targetPoint := targetStop.Pos

	if w != nil && len(r.StopIDs) > 0 {
		prevIdx := (r.CurrentIndex - 1 + len(r.StopIDs)) % len(r.StopIDs)
		prevStopID := r.StopIDs[prevIdx]
		if prevStopID >= 0 && prevStopID < len(w.Stops) {
			prevStop := w.Stops[prevStopID]
			path := GetRoadPathBetweenStopsInWorld(w, prevStop, targetStop)
			if len(path) >= 2 {
				closestIdx := 0
				minDist := math.MaxFloat64
				for idx, pt := range path {
					d := bus.Pos.Distance(pt)
					if d < minDist {
						minDist = d
						closestIdx = idx
					}
				}
				// Look ahead along path to find the next waypoint ahead of the bus
				for idx := closestIdx; idx < len(path); idx++ {
					if bus.Pos.Distance(path[idx]) > 70 {
						targetPoint = path[idx]
						break
					}
				}
			}
		}
	}

	// Direction towards target point
	dir := targetPoint.Sub(bus.Pos).Normalize()
	angleToStop := math.Atan2(dir.Y, dir.X)
	screenAngle := angleToStop - cam.Rotation

	// Draw floating navigation arrow around bus
	busScreen := cam.WorldToScreen(bus.Pos)
	navRadius := 68.0 * cam.Zoom
	arrowCenter := busScreen.Add(Vec2{
		X: math.Cos(screenAngle) * navRadius,
		Y: math.Sin(screenAngle) * navRadius,
	})

	cosA := math.Cos(screenAngle)
	sinA := math.Sin(screenAngle)
	perpX := -sinA
	perpY := cosA

	// Triangle pointer
	tip := arrowCenter.Add(Vec2{X: cosA * 15, Y: sinA * 15})
	base1 := arrowCenter.Add(Vec2{X: -cosA*8 + perpX*8, Y: -sinA*8 + perpY*8})
	base2 := arrowCenter.Add(Vec2{X: -cosA*8 - perpX*8, Y: -sinA*8 - perpY*8})

	vector.DrawFilledCircle(screen, float32(arrowCenter.X), float32(arrowCenter.Y), float32(16*cam.Zoom),
		RGBA(15, 23, 42, 190), true)
	vector.StrokeCircle(screen, float32(arrowCenter.X), float32(arrowCenter.Y), float32(16*cam.Zoom),
		float32(1.5), RGBA(250, 204, 21, 255), true)

	// Draw pointer
	vector.StrokeLine(screen, float32(tip.X), float32(tip.Y), float32(base1.X), float32(base1.Y), 2.5, RGBA(250, 204, 21, 255), true)
	vector.StrokeLine(screen, float32(tip.X), float32(tip.Y), float32(base2.X), float32(base2.Y), 2.5, RGBA(250, 204, 21, 255), true)
	vector.StrokeLine(screen, float32(base1.X), float32(base1.Y), float32(base2.X), float32(base2.Y), 2.5, RGBA(250, 204, 21, 255), true)
}

func (r *Route) GetRouteDescription(world *World) string {
	if !r.cachedDescDirty && r.cachedDesc != "" {
		return r.cachedDesc
	}
	if len(r.StopIDs) == 0 {
		r.cachedDesc = "未规划站点 (点击站点图标添加)"
		r.cachedDescDirty = false
		return r.cachedDesc
	}
	desc := ""
	for i, id := range r.StopIDs {
		prefix := " -> "
		if i == 0 {
			prefix = ""
		}
		if i == r.CurrentIndex {
			desc += fmt.Sprintf("%s[%s ★]", prefix, world.Stops[id].Name)
		} else {
			desc += fmt.Sprintf("%s%s", prefix, world.Stops[id].Name)
		}
	}
	r.cachedDesc = desc
	r.cachedDescDirty = false
	return desc
}
