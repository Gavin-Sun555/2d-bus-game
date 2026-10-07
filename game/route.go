package game

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Route manages the planned sequence of bus stops
type Route struct {
	Name         string
	Color        color.RGBA
	StopIDs      []int // Sequence of stop IDs
	CurrentIndex int   // Index within StopIDs that bus is currently navigating towards
	TotalTrips   int   // Completed full loops
}

func NewDefaultRoute() *Route {
	// Default starting route visiting major stops in a continuous right-hand city loop:
	// Stop 0 (Central Terminal, Eastbound) -> Stop 2 (Civic Center, Southbound) ->
	// Stop 10 (Creative District, Westbound) -> Stop 8 (City Hospital, Northbound) -> Stop 0
	return &Route{
		Name:         "Line 1 (城市核心环线)",
		Color:        RGBA(14, 165, 233, 255), // Sky blue
		StopIDs:      []int{0, 2, 10, 8, 0},
		CurrentIndex: 0,
		TotalTrips:   0,
	}
}

func (r *Route) AddStop(stopID int) {
	// Avoid immediate consecutive duplicates
	if len(r.StopIDs) > 0 && r.StopIDs[len(r.StopIDs)-1] == stopID {
		return
	}
	r.StopIDs = append(r.StopIDs, stopID)
}

func (r *Route) Clear() {
	r.StopIDs = make([]int, 0)
	r.CurrentIndex = 0
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
	r.CurrentIndex++
	if r.CurrentIndex >= len(r.StopIDs) {
		r.CurrentIndex = 0
		r.TotalTrips++
	}
}

type gridStopInfo struct {
	entryI, entryJ int
	exitI, exitJ   int
	dir            int // 0: East, 1: South, 2: West, 3: North
}

func getStopGridInfo(s *BusStop) gridStopInfo {
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}

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
		for idx := 0; idx < len(xs)-1; idx++ {
			if s.RoadCenter.X >= xs[idx]-20 && s.RoadCenter.X <= xs[idx+1]+20 {
				i = idx
				break
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
		for idx := 0; idx < len(ys)-1; idx++ {
			if s.RoadCenter.Y >= ys[idx]-20 && s.RoadCenter.Y <= ys[idx+1]+20 {
				j = idx
				break
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

func getRoadLaneOffset(roadCoord float64, isEW bool, lw float64) float64 {
	is4Lane := false
	if isEW {
		if math.Abs(roadCoord-(-1200)) < 20 || math.Abs(roadCoord) < 20 || math.Abs(roadCoord-1200) < 20 {
			is4Lane = true
		}
	} else {
		if math.Abs(roadCoord) < 20 {
			is4Lane = true
		}
	}

	if is4Lane {
		return 9.0 + lw*0.9
	}
	return 4.0 + lw*0.5
}

func calcSegOffset(roadCoord float64, isEW bool, dir int, lw float64) float64 {
	d := getRoadLaneOffset(roadCoord, isEW, lw)
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

type roadSegmentLine struct {
	isEW      bool
	roadCoord float64
	dir       int
	offset    float64
}

func getLanePt(i, j, dir int, xs, ys []float64, lw float64) Vec2 {
	if dir == 0 || dir == 2 { // East or West
		yRoad := ys[j]
		off := calcSegOffset(yRoad, true, dir, lw)
		return Vec2{X: xs[i], Y: yRoad + off}
	}
	// South or North
	xRoad := xs[i]
	off := calcSegOffset(xRoad, false, dir, lw)
	return Vec2{X: xRoad + off, Y: ys[j]}
}

// GetRoadPathBetweenStops calculates an orthogonal path strictly along the driving lane on the proper side of each road,
// ensuring the line is drawn on the right-hand side of the road rather than on the center divider median.
func GetRoadPathBetweenStops(s1, s2 *BusStop, laneWidth ...float64) []Vec2 {
	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}

	info1 := getStopGridInfo(s1)
	info2 := getStopGridInfo(s2)

	isS1EW := math.Abs(math.Cos(s1.Heading)) > math.Abs(math.Sin(s1.Heading))
	s1Coord := s1.RoadCenter.Y
	if !isS1EW {
		s1Coord = s1.RoadCenter.X
	}
	s1Off := calcSegOffset(s1Coord, isS1EW, info1.dir, lw)

	isS2EW := math.Abs(math.Cos(s2.Heading)) > math.Abs(math.Sin(s2.Heading))
	s2Coord := s2.RoadCenter.Y
	if !isS2EW {
		s2Coord = s2.RoadCenter.X
	}
	s2Off := calcSegOffset(s2Coord, isS2EW, info2.dir, lw)

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
			return []Vec2{s1.Pos, lanePt1, lanePt2, s2.Pos}
		}
	}

	startI, startJ, startDir := info1.exitI, info1.exitJ, info1.dir
	targetI, targetJ, targetDir := info2.entryI, info2.entryJ, info2.dir

	dx := [4]int{1, 0, -1, 0}
	dy := [4]int{0, 1, 0, -1}

	type state struct {
		i, j, dir int
	}

	var dist [5][5][4]float64
	var visited [5][5][4]bool
	var prev [5][5][4]state
	var hasPrev [5][5][4]bool

	for i := 0; i < 5; i++ {
		for j := 0; j < 5; j++ {
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
		for i := 0; i < 5; i++ {
			for j := 0; j < 5; j++ {
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
		if ni >= 0 && ni < 5 && nj >= 0 && nj < 5 {
			moveCost := math.Abs(xs[ni]-xs[curr.i]) + math.Abs(ys[nj]-ys[curr.j])
			if dist[curr.i][curr.j][curr.dir]+moveCost < dist[ni][nj][curr.dir] {
				dist[ni][nj][curr.dir] = dist[curr.i][curr.j][curr.dir] + moveCost
				prev[ni][nj][curr.dir] = curr
				hasPrev[ni][nj][curr.dir] = true
			}
		}

		// Action 2: Turn at current intersection to a different direction
		turnDirs := []int{(curr.dir + 1) % 4, (curr.dir + 3) % 4, (curr.dir + 2) % 4}
		turnCosts := []float64{30.0, 60.0, 5000.0} // Right turn (30), Left turn (60), U-turn (5000)
		for tIdx, nd := range turnDirs {
			tc := turnCosts[tIdx]
			if dist[curr.i][curr.j][curr.dir]+tc < dist[curr.i][curr.j][nd] {
				dist[curr.i][curr.j][nd] = dist[curr.i][curr.j][curr.dir] + tc
				prev[curr.i][curr.j][nd] = curr
				hasPrev[curr.i][curr.j][nd] = true
			}
		}
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
		if k > 0 && states[k-1].i == st.i && states[k-1].j == st.j {
			// Turn at intersection (st.i, st.j)
			prevDir := states[k-1].dir
			currDir := st.dir
			if (prevDir % 2) != (currDir % 2) {
				// 90-degree corner
				var corner Vec2
				if prevDir == 0 || prevDir == 2 { // EW to NS
					yOff := calcSegOffset(ys[st.j], true, prevDir, lw)
					xOff := calcSegOffset(xs[st.i], false, currDir, lw)
					corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
				} else { // NS to EW
					xOff := calcSegOffset(xs[st.i], false, prevDir, lw)
					yOff := calcSegOffset(ys[st.j], true, currDir, lw)
					corner = Vec2{X: xs[st.i] + xOff, Y: ys[st.j] + yOff}
				}
				pts = append(pts, corner)
			} else if prevDir != currDir {
				// 180-degree U-turn across intersection
				pA := getLanePt(st.i, st.j, prevDir, xs, ys, lw)
				pB := getLanePt(st.i, st.j, currDir, xs, ys, lw)
				pts = append(pts, pA, pB)
			}
		} else if k > 0 {
			// Moved along a road segment to intersection (st.i, st.j)
			pt := getLanePt(st.i, st.j, states[k-1].dir, xs, ys, lw)
			pts = append(pts, pt)
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

	return filtered
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
			path = GetRoadPathBetweenStops(s1, s2, lw)
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

		path := GetRoadPathBetweenStops(s1, s2, lw)
		lineColor := RGBA(r.Color.R, r.Color.G, r.Color.B, 130)
		if i == (r.CurrentIndex-1+len(r.StopIDs))%len(r.StopIDs) || (r.CurrentIndex == 0 && i == len(r.StopIDs)-1) {
			lineColor = RGBA(250, 204, 21, 240) // Highlight upcoming segment in gold
		}
		drawSegmentPath(screen, cam, path, lineColor)
	}
}

func drawSegmentPath(screen *ebiten.Image, cam *Camera, path []Vec2, lineColor color.RGBA) {
	for sIdx := 0; sIdx < len(path)-1; sIdx++ {
		w1 := path[sIdx]
		w2 := path[sIdx+1]

		p1 := cam.WorldToScreen(w1)
		p2 := cam.WorldToScreen(w2)

		// Route line along the road
		vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y),
			float32(5.0*cam.Zoom), lineColor, true)

		// Draw directional arrows along segments that are sufficiently long
		segLen := w1.Distance(w2)
		if segLen > 60 {
			mid := p1.Add(p2).Mul(0.5)
			dir := p2.Sub(p1).Normalize()
			perp := Vec2{X: -dir.Y, Y: dir.X}
			arr1 := mid.Sub(dir.Mul(12 * cam.Zoom)).Add(perp.Mul(7 * cam.Zoom))
			arr2 := mid.Sub(dir.Mul(12 * cam.Zoom)).Sub(perp.Mul(7 * cam.Zoom))
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr1.X), float32(arr1.Y),
				float32(3.0*cam.Zoom), lineColor, true)
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr2.X), float32(arr2.Y),
				float32(3.0*cam.Zoom), lineColor, true)
		}
	}
}

// DrawNavArrow draws a compass/directional navigation arrow around the bus towards the target stop along roads
func (r *Route) DrawNavArrow(screen *ebiten.Image, cam *Camera, bus *Bus, targetStop *BusStop) {
	if targetStop == nil {
		return
	}

	dist := bus.Pos.Distance(targetStop.Pos)
	if dist < 65 {
		return // Already arrived at stop
	}

	// Calculate target navigation point along road grid
	targetPoint := targetStop.Pos

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
	if len(r.StopIDs) == 0 {
		return "未规划站点 (点击站点图标添加)"
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
	return desc
}
