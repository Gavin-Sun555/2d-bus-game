package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// BuildAutopilotDrivePath converts the orthogonal road route between two stops into a
// drivable polyline: instead of the sideways 90° jogs between bay and lane used for drawing,
// the bus merges diagonally out of the departure bay and pulls into the target bay along a
// tapered entry that ends in a straight, curb-aligned run-in.
func BuildAutopilotDrivePath(w *sim.World, from, to *sim.BusStop, lw float64) []sim.Vec2 {
	return buildAutopilotDrivePath(w, from, to, lw, false, false)
}

func buildAutopilotDrivePath(w *sim.World, from, to *sim.BusStop, lw float64, skipDeparture, skipArrival bool, options ...sim.RoadPathOptions) []sim.Vec2 {
	if to == nil {
		return nil
	}
	if !skipArrival {
		to = forwardDockStop(to)
	}
	long := len(options) > 0 && options[0].LongVehicle
	bayStraight := apBayStraight
	mergeLength, departureStraight := 160., 25.
	if long {
		bayStraight = 160
		mergeLength = 360
		departureStraight = 120
	}
	h2 := sim.UnitFromAngle(to.Heading)
	destination := to.Pos
	if skipArrival {
		destination = stopRoadPoint(to, lw, w)
	}
	bayEntry := destination.Sub(h2.Mul(bayStraight))
	if w == nil || from == nil || from.ID == to.ID {
		return []sim.Vec2{destination.Sub(h2.Mul(650)), bayEntry, destination}
	}

	raw := sim.ComputeRoadPathBetweenStops(w, from, to, lw, options...)
	if len(raw) < 4 {
		// Unreachable on the grid: best-effort direct approach into the bay
		origin := from.Pos
		if skipDeparture {
			origin = stopRoadPoint(from, lw, w)
		}
		return dedupePath([]sim.Vec2{origin, bayEntry, destination})
	}

	h1 := sim.UnitFromAngle(from.Heading)
	n := len(raw)
	out := make([]sim.Vec2, 0, n+4)

	// --- Departure: bay -> straight pull-out -> diagonal merge into the travel lane ---
	start := 1
	out = append(out, raw[0])
	lane1 := raw[1]
	if skipDeparture {
		out = []sim.Vec2{lane1}
		start = 2
	} else if departure, next := closeLeftDeparture(raw, h1, lw); next > 0 {
		out, start = departure, next
	} else if math.Abs(lane1.Sub(raw[0]).Dot(h1)) < 10.0 {
		seg := raw[2].Sub(lane1)
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h1) > 0.9 {
			mergeDist := math.Min(mergeLength, segLen*0.4)
			// A short approach may end before a long bus's nominal straight
			// pull-out. Keep the merge ahead of that straight, never behind it.
			out = append(out, raw[0].Add(h1.Mul(math.Min(departureStraight, mergeDist*.25))), lane1.Add(h1.Mul(mergeDist)))
			start = 2
		}
	}

	// --- Arrival: travel lane -> tapered entry -> straight run-in -> bay center ---
	end := n - 1 // exclusive index of the raw points copied verbatim
	lane2 := raw[n-2]
	tail := []sim.Vec2{to.Pos}
	if skipArrival {
		tail = []sim.Vec2{lane2}
		end = n - 2
	} else if math.Abs(lane2.Sub(to.Pos).Dot(h2)) < 10.0 && n-3 >= 0 {
		seg := lane2.Sub(raw[n-3])
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h2) > 0.9 {
			taperDist := math.Min(170.0, segLen*0.4)
			if long {
				taperDist = math.Min(340, segLen*.45)
			}
			tail = []sim.Vec2{lane2.Sub(h2.Mul(taperDist + bayStraight)), bayEntry, to.Pos}
			end = n - 2
		}
	}

	for i := start; i < end; i++ {
		out = append(out, raw[i])
	}
	out = append(out, tail...)
	return smoothRoundaboutDepartures(w, dedupePath(out), lw)
}

func dedupePath(pts []sim.Vec2) []sim.Vec2 {
	res := make([]sim.Vec2, 0, len(pts))
	for _, p := range pts {
		if len(res) == 0 || res[len(res)-1].Distance(p) > 3.0 {
			res = append(res, p)
		}
	}
	return res
}

// PrepareLeg (re)builds the cached drive path whenever the leg, world or lane width changes
func (ad *AutopilotDriver) PrepareLeg(w *sim.World, from, to *sim.BusStop, lw float64) {
	if ad.drivePath != nil {
		if ad.pathWorld == nil {
			ad.pathWorld = w
			if from != nil {
				ad.pathFromID = from.ID
			}
			if to != nil {
				ad.pathToID = to.ID
			}
			ad.pathLane = lw
			ad.pathOrdinaryBusLeft = ad.ordinaryBusLeft
			ad.pathSkipTarget = ad.CanSkipTargetStop
			return
		}
		if from != nil && to != nil && ad.pathWorld != nil &&
			(ad.pathWorld == w || ad.pathWorld.Theme == w.Theme) &&
			ad.pathFromID == from.ID && ad.pathToID == to.ID && ad.pathLane == lw && ad.pathSkipTarget == ad.CanSkipTargetStop && ad.pathOrdinaryBusLeft == ad.ordinaryBusLeft {
			ad.pathWorld = w
			return
		}
	}
	if ad.pathFromID != from.ID || ad.pathToID != to.ID {
		ad.dockDepartureRun = 0
		ad.smoothDockDeparture = false
		ad.midLaneRecovery = nil
		ad.approachMerge = nil
		ad.approachMergeJoined = false
		ad.parallelEntryCenter = nil
		ad.uTurn = nil
		ad.ordinaryBusLeft = false
		ad.busLaneJunction = nil
		ad.rightTurnJunction = nil
		ad.rightTurnWaiting = false
		ad.busJunctionEntered = false
	}
	if ad.stationDock != nil && ad.stationDock.stopID != to.ID {
		ad.departureFromDock = from != nil && from.ID == ad.stationDock.stopID
		ad.stationDock = nil
	}
	if ad.skippedDeparture != nil && from != nil && ad.skippedDeparture.stop.ID != from.ID {
		ad.skippedDeparture = nil
	}
	skipFrom := ad.skippedDeparture != nil && from != nil && ad.skippedDeparture.stop.ID == from.ID
	ad.directRoundaboutExit = nil
	ad.drivePath = buildAutopilotDrivePath(w, from, to, lw, skipFrom, ad.CanSkipTargetStop, sim.RoadPathOptions{OrdinaryBusLaneLeft: ad.ordinaryBusLeft, LongVehicle: ad.articulatedPath})
	ad.pathRevision = currentDrivePathRevision
	if skipFrom {
		ad.prependSkippedDeparture(lw, w)
	}
	ad.pathOrdinaryBusLeft = ad.ordinaryBusLeft
	ad.pathSkipTarget = ad.CanSkipTargetStop
	ad.bypassStopID = -1
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	ad.pathWorld = w
	ad.pathFromID = from.ID
	ad.pathToID = to.ID
	ad.pathLane = lw
	ad.WaypointIdx = 0
	ad.pathSynced = false
}

// projectToSegment returns the clamped parameter t, and distance of p to segment a-b
func projectToSegment(p, a, b sim.Vec2) (float64, float64) {
	ab := b.Sub(a)
	l2 := ab.Dot(ab)
	if l2 < 1e-9 {
		return 0, p.Distance(a)
	}
	t := sim.Clamp(p.Sub(a).Dot(ab)/l2, 0, 1)
	return t, p.Distance(a.Add(ab.Mul(t)))
}

// trackPath localizes the bus on the drive path. Progress is monotonic within a bounded
// arc-length window (so the bus can never "jump back" and turn around); a global search is
// only used on first sync or when the bus has strayed far from the path.
// Returns the arc-length position s and the lateral distance to the path.
func (ad *AutopilotDriver) trackPath(pos sim.Vec2) (float64, float64) {
	path := ad.drivePath
	n := len(path)
	if n < 2 {
		return 0, 0
	}
	best, bestT, bestD := -1, 0.0, math.Inf(1)
	consider := func(i int) {
		t, d := projectToSegment(pos, path[i], path[i+1])
		if d < bestD-0.5 {
			best, bestT, bestD = i, t, d
		}
	}

	if ad.pathSynced {
		for i := ad.WaypointIdx; i < n-1; i++ {
			if i > ad.WaypointIdx && ad.cumLen[i] > ad.LastPathS+apTrackWindow {
				break
			}
			consider(i)
		}
	}
	if best < 0 || bestD > apRelocalizeDist {
		best, bestT, bestD = -1, 0.0, math.Inf(1)
		for i := 0; i < n-1; i++ {
			consider(i)
		}
	}

	ad.WaypointIdx = best
	ad.pathSynced = true
	s := ad.cumLen[best] + bestT*(ad.cumLen[best+1]-ad.cumLen[best])
	ad.LastPathS = s
	return s, bestD
}

// pointAt returns the point at arc length s on the drive path, extrapolating straight past the end
func (ad *AutopilotDriver) pointAt(s float64) sim.Vec2 {
	path := ad.drivePath
	n := len(path)
	if n == 0 {
		return sim.Vec2{}
	}
	if n == 1 {
		return path[0]
	}
	total := ad.cumLen[n-1]
	if s >= total {
		dir := path[n-1].Sub(path[n-2]).Normalize()
		return path[n-1].Add(dir.Mul(s - total))
	}
	if s <= 0 {
		return path[0]
	}
	i := ad.WaypointIdx
	if i < 0 || i >= n-1 || ad.cumLen[i] > s {
		i = 0
	}
	for i < n-2 && ad.cumLen[i+1] < s {
		i++
	}
	segLen := ad.cumLen[i+1] - ad.cumLen[i]
	if segLen < 1e-9 {
		return path[i]
	}
	t := (s - ad.cumLen[i]) / segLen
	return path[i].Add(path[i+1].Sub(path[i]).Mul(t))
}

// minDistanceToPath calculates the minimum distance from pt to the active drivePath segments near currentWP
func minDistanceToPath(pt sim.Vec2, path []sim.Vec2, currentWP int) float64 {
	if len(path) < 2 {
		return 999.0
	}
	start := currentWP - 1
	if start < 0 {
		start = 0
	}
	end := currentWP + 10
	if end >= len(path) {
		end = len(path) - 1
	}
	minD := 999.0
	for i := start; i < end; i++ {
		d, _ := sim.DistToSegment(pt, path[i], path[i+1])
		if d < minD {
			minD = d
		}
	}
	return minD
}

// cornerSpeedLimit anticipates upcoming turns along the path and returns the speed the bus
// may currently drive so that it can comfortably slow down for each corner in time.
func (ad *AutopilotDriver) cornerSpeedLimit(s, speed float64) float64 {
	path := ad.drivePath
	n := len(path)
	limit := math.Inf(1)
	decel := apCornerDecel
	if ad.BerserkMode {
		decel = 160.0 // Firmer braking, while preserving the corner speed caps.
		if ad.GetBerserkCruiseSpeed() >= 550.0 {
			decel = 190.0
		}
	}
	horizon := speed*speed/(2*decel) + 100.0
	startK := ad.WaypointIdx
	if startK < 1 {
		startK = 1
	}
	for k := startK; k < n-1; k++ {
		d := ad.cumLen[k] - s
		if d < -35 {
			continue
		}
		if d > horizon {
			break
		}
		a := path[k].Sub(path[k-1])
		b := path[k+1].Sub(path[k])
		turn := math.Abs(sim.NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X)))

		// Also measure cumulative turn across multi-segment curved arcs (where segments are short connected links):
		if k+2 < n && path[k+1].Sub(path[k]).Length() < 120.0 {
			c := path[k+2].Sub(path[k+1])
			t2 := math.Abs(sim.NormalizeAngle(math.Atan2(c.Y, c.X) - math.Atan2(a.Y, a.X)))
			if t2 > turn {
				turn = t2
			}
		}
		if k+4 < n && path[k+3].Sub(path[k]).Length() < 240.0 {
			e := path[k+4].Sub(path[k+3])
			t4 := math.Abs(sim.NormalizeAngle(math.Atan2(e.Y, e.X) - math.Atan2(a.Y, a.X)))
			if t4 > turn {
				turn = t4
			}
		}

		vc := math.Inf(1)
		switch {
		case turn > 1.1:
			vc = 48.0
			if ad.BerserkMode {
				vc = 90.0 // Fast, smooth exhilarating cornering in Berserk mode (up from 75)
			}
		case turn > 0.6:
			vc = 52.0
			if ad.BerserkMode {
				vc = 110.0 // Smooth swift medium cornering in Berserk mode
			}
		case turn > 0.20:
			vc = 85.0
			if ad.BerserkMode {
				vc = math.Min(ad.GetBerserkCruiseSpeed(), math.Max(135.0, ad.GetBerserkCruiseSpeed()*0.42))
			}
		}
		if math.IsInf(vc, 1) {
			continue
		}
		allowed := math.Sqrt(vc*vc + 2*decel*math.Max(0, d-15.0))
		limit = math.Min(limit, allowed)
	}
	return limit
}

// basePathPosition removes the effective lane offset before projecting a world
// position onto the base path. The centerline clamp matches steering geometry.
func (ad *AutopilotDriver) basePathPosition(pos sim.Vec2, world *sim.World, heading, laneWidth float64) sim.Vec2 {
	if ad.LaneOffset != 0 && len(ad.drivePath) >= 2 {
		wIdx := ad.WaypointIdx
		if wIdx >= len(ad.drivePath)-1 {
			wIdx = len(ad.drivePath) - 2
		}
		seg := ad.drivePath[wIdx+1].Sub(ad.drivePath[wIdx])
		if segLen := seg.Length(); segLen > 1e-6 {
			segDir := seg.Mul(1.0 / segLen)
			segRight := sim.Vec2{X: -segDir.Y, Y: segDir.X}
			if ad.busLaneJunction != nil && !ad.busJunctionEntered && world != nil && !world.InsideJunction(pos, laneWidth, 0) {
				u := sim.UnitFromAngle(approachRoadHeading(heading))
				segRight = sim.Vec2{X: -u.Y, Y: u.X}
			}
			effOffset := ad.LaneOffset
			if world != nil && !world.InsideJunction(pos, laneWidth, 0) {
				wpt := ad.drivePath[wIdx]
				roadCenter, isEW, isMultiLane, is6Lane := sim.CurrentRoadInfo(world, wpt, heading, laneWidth)
				if isMultiLane || is6Lane {
					dWptCenter := math.Abs(wpt.Y - roadCenter)
					if !isEW {
						dWptCenter = math.Abs(wpt.X - roadCenter)
					}
					maxLeftShift := math.Max(0.0, dWptCenter-laneWidth*0.5)
					if effOffset < -maxLeftShift {
						effOffset = -maxLeftShift
					}
				}
			}
			return pos.Sub(segRight.Mul(effOffset))
		}
	}

	return pos
}

func forwardDockStop(stop *sim.BusStop) *sim.BusStop {
	if stop == nil {
		return nil
	}
	return stop.ForwardDockStop()
}
