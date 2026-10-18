package sim

import "math"

type roadState struct{ i, j, dir int }

// shortestRoadStates performs the heading-aware graph search. Its deterministic
// scan order resolves equal-cost routes consistently across cache rebuilds.
func shortestRoadStates(w *World, xs, ys []float64, start, target roadState) ([]roadState, bool) {
	numX, numY := len(xs), len(ys)
	dx := [4]int{1, 0, -1, 0}
	dy := [4]int{0, 1, 0, -1}

	dist := make([][][4]float64, numX)
	visited := make([][][4]bool, numX)
	prev := make([][][4]roadState, numX)
	hasPrev := make([][][4]bool, numX)

	for i := 0; i < numX; i++ {
		dist[i] = make([][4]float64, numY)
		visited[i] = make([][4]bool, numY)
		prev[i] = make([][4]roadState, numY)
		hasPrev[i] = make([][4]bool, numY)
		for j := 0; j < numY; j++ {
			for d := 0; d < 4; d++ {
				dist[i][j][d] = math.MaxFloat64
			}
		}
	}

	dist[start.i][start.j][start.dir] = 0

	for {
		// Find unvisited roadState with minimum distance
		minD := math.MaxFloat64
		var curr roadState
		found := false
		for i := 0; i < numX; i++ {
			for j := 0; j < numY; j++ {
				for d := 0; d < 4; d++ {
					if !visited[i][j][d] && dist[i][j][d] < minD {
						minD = dist[i][j][d]
						curr = roadState{i: i, j: j, dir: d}
						found = true
					}
				}
			}
		}
		if !found || minD == math.MaxFloat64 {
			break
		}

		visited[curr.i][curr.j][curr.dir] = true

		if curr.i == target.i && curr.j == target.j && curr.dir == target.dir {
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
	if !visited[target.i][target.j][target.dir] && !(start.i == target.i && start.j == target.j && start.dir == target.dir) {
		return nil, false
	}

	// Reconstruct path of states from target back to start
	curr := roadState{i: target.i, j: target.j, dir: target.dir}
	revStates := []roadState{curr}
	for {
		if curr.i == start.i && curr.j == start.j && curr.dir == start.dir {
			break
		}
		if !hasPrev[curr.i][curr.j][curr.dir] {
			break
		}
		curr = prev[curr.i][curr.j][curr.dir]
		revStates = append(revStates, curr)
	}

	states := make([]roadState, len(revStates))
	for idx := range revStates {
		states[idx] = revStates[len(revStates)-1-idx]
	}

	return states, true
}
