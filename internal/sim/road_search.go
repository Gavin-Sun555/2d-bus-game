package sim

import "math"

type roadState struct{ i, j, dir int }

func shortestRoadStates(w *World, xs, ys []float64, start, target roadState) ([]roadState, bool) {
	states, found := searchRoadStates(w, xs, ys, start, target, true)
	for i := 1; i < len(states); i++ {
		if (states[i].dir-states[i-1].dir+4)%4 == 2 {
			withoutTurn, ok := searchRoadStates(w, xs, ys, start, target, false)
			if ok && roadStatesDistance(withoutTurn, xs, ys) <= roadStatesDistance(states, xs, ys)+1000/MetersPerPixel {
				return withoutTurn, true
			}
			break
		}
	}
	return states, found
}

func roadStatesDistance(states []roadState, xs, ys []float64) float64 {
	distance := 0.
	for i := 1; i < len(states); i++ {
		a, b := states[i-1], states[i]
		distance += math.Abs(xs[b.i]-xs[a.i]) + math.Abs(ys[b.j]-ys[a.j])
	}
	return distance
}

// Each edge combines the turn with travel to the next intersection. An arrival
// may turn onto its final road, but consecutive turns at the same intersection
// cannot disguise a U-turn as two cheaper 90-degree turns.
func searchRoadStates(w *World, xs, ys []float64, start, target roadState, allowUTurn bool) ([]roadState, bool) {
	numX, numY := len(xs), len(ys)
	dx := [4]int{1, 0, -1, 0}
	dy := [4]int{0, 1, 0, -1}
	type node struct {
		dist     float64
		turnCost int
		prev     roadState
		visited  bool
	}
	nodes := make([]node, numX*numY*4)
	index := func(s roadState) int { return (s.i*numY+s.j)*4 + s.dir }
	for i := range nodes {
		nodes[i] = node{dist: math.Inf(1), turnCost: math.MaxInt}
	}
	nodes[index(start)] = node{}
	best, bestTurnCost := math.Inf(1), math.MaxInt
	var finish roadState
	for {
		currentIndex := -1
		minD, minTurnCost := best, bestTurnCost
		for i, n := range nodes {
			if !n.visited && (n.dist < minD || (n.dist == minD && n.turnCost < minTurnCost)) {
				minD, minTurnCost, currentIndex = n.dist, n.turnCost, i
			}
		}
		if currentIndex < 0 {
			break
		}
		nodes[currentIndex].visited = true
		current := roadState{currentIndex / (numY * 4), currentIndex / 4 % numY, currentIndex % 4}
		if current == target {
			best, finish = minD, current
			break
		}
		for _, turn := range [...]struct {
			offset int
			cost   int
		}{{0, 0}, {1, 30}, {3, 60}, {2, 5000}} {
			if turn.offset == 2 && !allowUTurn {
				continue
			}
			dir := (current.dir + turn.offset) % 4
			next := roadState{current.i + dx[dir], current.j + dy[dir], dir}
			if next.i < 0 || next.i >= numX || next.j < 0 || next.j >= numY {
				continue
			}
			from := Vec2{X: xs[current.i], Y: ys[current.j]}
			to := Vec2{X: xs[next.i], Y: ys[next.j]}
			if !isRoadConnecting(from, to, w) {
				continue
			}
			turnCost := minTurnCost + turn.cost
			if current.i == target.i && current.j == target.j && dir == target.dir &&
				(minD < best || (minD == best && turnCost < bestTurnCost)) {
				best, bestTurnCost, finish = minD, turnCost, current
			}
			distance := minD + math.Abs(to.X-from.X) + math.Abs(to.Y-from.Y)
			n := &nodes[index(next)]
			if distance < n.dist || (distance == n.dist && turnCost < n.turnCost) {
				n.dist, n.turnCost, n.prev = distance, turnCost, current
			}
		}
	}
	if math.IsInf(best, 1) {
		return nil, false
	}
	states := []roadState{target}
	if finish != target {
		states = append(states, finish)
	}
	for current := finish; current != start; {
		prev := nodes[index(current)].prev
		if prev.dir != current.dir {
			states = append(states, roadState{prev.i, prev.j, current.dir})
		}
		states = append(states, prev)
		current = prev
	}
	for i, j := 0, len(states)-1; i < j; i, j = i+1, j-1 {
		states[i], states[j] = states[j], states[i]
	}
	return states, true
}
