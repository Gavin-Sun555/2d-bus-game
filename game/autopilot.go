package game

import (
	"math"
)

// AutopilotState represents the current behavioral state of the autonomous bus driver
type AutopilotState int

const (
	AutopilotStateCruising AutopilotState = iota
	AutopilotStateApproaching
	AutopilotStateDocked
	AutopilotStateDeparting
)

func (s AutopilotState) String() string {
	switch s {
	case AutopilotStateCruising:
		return "巡航行驶"
	case AutopilotStateApproaching:
		return "进站减速"
	case AutopilotStateDocked:
		return "停靠上下客"
	case AutopilotStateDeparting:
		return "出站汇入"
	default:
		return "巡航行驶"
	}
}

// Autopilot tuning constants (px, px/s, px/s²)
const (
	apCruiseSpeed        = 150.0 // Standard cruising speed (~40 km/h)
	apBerserkCruiseSpeed = 445.0 // Berserk mode straightaway top cruise speed (80 km/h)
	apRoundaboutSpeed    = 90.0  // Speed cap while circulating a roundabout
	apCornerDecel        = 120.0 // Planned deceleration used when anticipating corners
	apStopDecel          = 85.0  // Planned deceleration used when braking into a stop bay
	apApproachDist       = 220.0 // Remaining path length at which the bus is "approaching"
	apTrackWindow        = 260.0 // Arc-length window searched ahead of the current segment
	apRelocalizeDist     = 140.0 // Lateral error beyond which the tracker re-localizes globally
	apBayStraight        = 60.0  // Straight, curb-aligned run-in before the bay center
)

// VehicleObstacle represents any vehicle in the world (civilian car or another bus)
type VehicleObstacle struct {
	Pos     Vec2
	Heading float64
	Speed   float64
	Length  float64
	Width   float64
}

// Autopilot overtaking states (Berserk Mode on multi-lane roads)
const (
	OvertakeNone         = 0
	OvertakeChangingLane = 1
	OvertakePassing      = 2
	OvertakeReturning    = 3
)

// AutopilotDriver encapsulates autonomous driving intelligence for both the player's bus
// (when autopilot is enabled) and background AI route buses.
type AutopilotDriver struct {
	State         AutopilotState
	BerserkMode   bool // true if Berserk mode is active (player bus fast straightaways)
	DwellTimer    float64
	DwellTotal    float64
	DwellFinished bool
	DockedStopID  int
	LastTargetID  int
	WaypointIdx   int // Index of the drive-path segment currently being tracked (monotonic)

	// Overtaking state for Berserk mode on multi-lane roads
	OvertakeState        int     // OvertakeNone, OvertakeChangingLane, OvertakePassing, OvertakeReturning
	LaneOffset           float64 // Dynamic lateral offset from drivePath (-lw = left lane, +lw = right lane)
	TargetLaneOffset     float64 // Target lateral offset
	lastTargetLaneOffset float64 // Previous target lateral offset to detect target changes
	LaneOffsetStart      float64 // Starting lateral offset for smooth S-curve interpolation
	LaneOffsetProgress   float64 // Normalized progress [0.0, 1.0] along the transition S-curve
	LaneOffsetDuration   float64 // Duration in seconds for current transition S-curve
	OvertakeTimer        float64 // Duration in current overtake phase

	// Cached drive path for the active leg (prevStop -> targetStop)
	drivePath  []Vec2
	cumLen     []float64
	pathWorld  *World
	pathFromID int
	pathToID   int
	pathLane   float64
	pathSynced bool // false => next projection performs a global (non-monotonic) search

	// Last-frame tracking diagnostics (used by tests and debugging overlays)
	LastPathS       float64 // Arc-length position of the bus along the drive path
	LastRemaining   float64 // Remaining drive-path length to the target bay
	LastLateralErr  float64 // Perpendicular distance from the bus to the drive path
	LastPursuitPt   Vec2    // Pure-pursuit lookahead target point
	LastTargetSpeed float64 // Commanded target speed
}

// NewAutopilotDriver creates a fresh autonomous bus controller
func NewAutopilotDriver() *AutopilotDriver {
	return &AutopilotDriver{
		State:              AutopilotStateCruising,
		DwellTimer:         0,
		DwellTotal:         2.8,
		DwellFinished:      false,
		DockedStopID:       -1,
		LastTargetID:       -1,
		WaypointIdx:        0,
		LaneOffsetProgress: 1.0,
		LaneOffsetDuration: 1.0,
		pathFromID:         -1,
		pathToID:           -1,
	}
}

// Reset clears autonomous driving internal state
func (ad *AutopilotDriver) Reset() {
	ad.State = AutopilotStateCruising
	ad.DwellTimer = 0
	ad.DwellFinished = false
	ad.DockedStopID = -1
	ad.LastTargetID = -1
	ad.WaypointIdx = 0
	ad.OvertakeState = OvertakeNone
	ad.LaneOffset = 0.0
	ad.TargetLaneOffset = 0.0
	ad.lastTargetLaneOffset = 0.0
	ad.LaneOffsetStart = 0.0
	ad.LaneOffsetProgress = 1.0
	ad.LaneOffsetDuration = 1.0
	ad.OvertakeTimer = 0.0
	ad.drivePath = nil
	ad.cumLen = nil
	ad.pathWorld = nil
	ad.pathFromID = -1
	ad.pathToID = -1
	ad.pathSynced = false
}

// DrivePath returns the currently cached drive path (read-only; for HUD/debug/tests)
func (ad *AutopilotDriver) DrivePath() []Vec2 {
	return ad.drivePath
}

// BuildAutopilotDrivePath converts the orthogonal road route between two stops into a
// drivable polyline: instead of the sideways 90° jogs between bay and lane used for drawing,
// the bus merges diagonally out of the departure bay and pulls into the target bay along a
// tapered entry that ends in a straight, curb-aligned run-in.
func BuildAutopilotDrivePath(w *World, from, to *BusStop, lw float64) []Vec2 {
	if to == nil {
		return nil
	}
	h2 := Vec2{X: math.Cos(to.Heading), Y: math.Sin(to.Heading)}
	bayEntry := to.Pos.Sub(h2.Mul(apBayStraight))
	if w == nil || from == nil || from.ID == to.ID {
		return []Vec2{to.Pos.Sub(h2.Mul(650)), bayEntry, to.Pos}
	}

	raw := computeRoadPathBetweenStops(w, from, to, lw)
	if len(raw) < 4 {
		// Unreachable on the grid: best-effort direct approach into the bay
		return dedupePath([]Vec2{from.Pos, bayEntry, to.Pos})
	}

	h1 := Vec2{X: math.Cos(from.Heading), Y: math.Sin(from.Heading)}
	n := len(raw)
	out := make([]Vec2, 0, n+4)

	// --- Departure: bay -> straight pull-out -> diagonal merge into the travel lane ---
	start := 1
	out = append(out, raw[0])
	lane1 := raw[1]
	if math.Abs(lane1.Sub(raw[0]).Dot(h1)) < 10.0 {
		seg := raw[2].Sub(lane1)
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h1) > 0.9 {
			mergeDist := math.Min(160.0, segLen*0.4)
			out = append(out, raw[0].Add(h1.Mul(25.0)), lane1.Add(h1.Mul(mergeDist)))
			start = 2
		}
	}

	// --- Arrival: travel lane -> tapered entry -> straight run-in -> bay center ---
	end := n - 1 // exclusive index of the raw points copied verbatim
	lane2 := raw[n-2]
	tail := []Vec2{to.Pos}
	if math.Abs(lane2.Sub(to.Pos).Dot(h2)) < 10.0 && n-3 >= 0 {
		seg := lane2.Sub(raw[n-3])
		segLen := seg.Length()
		if segLen > 1e-6 && seg.Mul(1/segLen).Dot(h2) > 0.9 {
			taperDist := math.Min(170.0, segLen*0.4)
			tail = []Vec2{lane2.Sub(h2.Mul(taperDist + apBayStraight)), bayEntry, to.Pos}
			end = n - 2
		}
	}

	for i := start; i < end; i++ {
		out = append(out, raw[i])
	}
	out = append(out, tail...)
	return dedupePath(out)
}

func dedupePath(pts []Vec2) []Vec2 {
	res := make([]Vec2, 0, len(pts))
	for _, p := range pts {
		if len(res) == 0 || res[len(res)-1].Distance(p) > 3.0 {
			res = append(res, p)
		}
	}
	return res
}

// ensurePath (re)builds the cached drive path whenever the leg, world or lane width changes
func (ad *AutopilotDriver) ensurePath(w *World, from, to *BusStop, lw float64) {
	if ad.drivePath != nil && ad.pathWorld == w && ad.pathFromID == from.ID &&
		ad.pathToID == to.ID && ad.pathLane == lw {
		return
	}
	ad.drivePath = BuildAutopilotDrivePath(w, from, to, lw)
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
func projectToSegment(p, a, b Vec2) (float64, float64) {
	ab := b.Sub(a)
	l2 := ab.Dot(ab)
	if l2 < 1e-9 {
		return 0, p.Distance(a)
	}
	t := Clamp(p.Sub(a).Dot(ab)/l2, 0, 1)
	return t, p.Distance(a.Add(ab.Mul(t)))
}

// trackPath localizes the bus on the drive path. Progress is monotonic within a bounded
// arc-length window (so the bus can never "jump back" and turn around); a global search is
// only used on first sync or when the bus has strayed far from the path.
// Returns the arc-length position s and the lateral distance to the path.
func (ad *AutopilotDriver) trackPath(pos Vec2) (float64, float64) {
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
func (ad *AutopilotDriver) pointAt(s float64) Vec2 {
	path := ad.drivePath
	n := len(path)
	if n == 0 {
		return Vec2{}
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
func minDistanceToPath(pt Vec2, path []Vec2, currentWP int) float64 {
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
		d, _ := distToSegment(pt, path[i], path[i+1])
		if d < minD {
			minD = d
		}
	}
	return minD
}

// cornerSpeedLimit anticipates upcoming turns along the path and returns the speed the bus
// cornerSpeedLimit anticipates upcoming turns along the path and returns the speed the bus
// may currently drive so that it can comfortably slow down for each corner in time.
func (ad *AutopilotDriver) cornerSpeedLimit(s, speed float64) float64 {
	path := ad.drivePath
	n := len(path)
	limit := math.Inf(1)
	decel := apCornerDecel
	if ad.BerserkMode {
		decel = 140.0 // Planned deceleration when anticipating corners from high speed
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
		turn := math.Abs(NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X)))

		// Also measure cumulative turn across multi-segment curved arcs (where segments are short connected links):
		if k+2 < n && path[k+1].Sub(path[k]).Length() < 120.0 {
			c := path[k+2].Sub(path[k+1])
			t2 := math.Abs(NormalizeAngle(math.Atan2(c.Y, c.X) - math.Atan2(a.Y, a.X)))
			if t2 > turn {
				turn = t2
			}
		}
		if k+4 < n && path[k+3].Sub(path[k]).Length() < 240.0 {
			e := path[k+4].Sub(path[k+3])
			t4 := math.Abs(NormalizeAngle(math.Atan2(e.Y, e.X) - math.Atan2(a.Y, a.X)))
			if t4 > turn {
				turn = t4
			}
		}

		vc := math.Inf(1)
		switch {
		case turn > 1.1:
			vc = 48.0
			if ad.BerserkMode {
				vc = 75.0 // Fast, exhilarating right turn in Berserk mode
			}
		case turn > 0.6:
			vc = 52.0
			if ad.BerserkMode {
				vc = 110.0 // Smooth swift medium cornering in Berserk mode
			}
		case turn > 0.20:
			vc = 85.0
			if ad.BerserkMode {
				vc = 135.0 // High-speed, momentum-preserving lane change and mild curvature
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

// getCurrentRoadInfo detects the road segment the vehicle is currently on and whether it is a multi-lane or 6-lane avenue
func getCurrentRoadInfo(w *World, pos Vec2, heading float64, lw float64) (roadCenter float64, isEW bool, isMultiLane bool, is6Lane bool) {
	if w == nil {
		return 0, true, false, false
	}
	cosH := math.Abs(math.Cos(heading))
	sinH := math.Abs(math.Sin(heading))
	busEW := cosH > sinH

	for _, r := range w.Roads {
		roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		if roadIsEW != busEW {
			continue
		}
		c := r.P1.Y
		if !roadIsEW {
			c = r.P1.X
		}
		rWidth := r.GetWidth(lw)
		if roadIsEW {
			minX := math.Min(r.P1.X, r.P2.X) - 30.0
			maxX := math.Max(r.P1.X, r.P2.X) + 30.0
			if pos.X >= minX && pos.X <= maxX && math.Abs(pos.Y-c) <= rWidth*0.5+15.0 {
				is6 := (r.Lanes >= 6 || r.Type == RoadExpressway6Lane || w.IsRoad6Lane(c, true, pos))
				isMulti := (is6 || r.Lanes >= 4 || r.Type == RoadAvenue4Lane || w.IsRoad4Lane(c, true, pos))
				return c, true, isMulti, is6
			}
		} else {
			minY := math.Min(r.P1.Y, r.P2.Y) - 30.0
			maxY := math.Max(r.P1.Y, r.P2.Y) + 30.0
			if pos.Y >= minY && pos.Y <= maxY && math.Abs(pos.X-c) <= rWidth*0.5+15.0 {
				is6 := (r.Lanes >= 6 || r.Type == RoadExpressway6Lane || w.IsRoad6Lane(c, false, pos))
				isMulti := (is6 || r.Lanes >= 4 || r.Type == RoadAvenue4Lane || w.IsRoad4Lane(c, false, pos))
				return c, false, isMulti, is6
			}
		}
	}
	return 0, busEW, false, false
}

func (ad *AutopilotDriver) progressLaneOffset(dt, speedAbs float64) {
	if ad.TargetLaneOffset != ad.lastTargetLaneOffset {
		ad.lastTargetLaneOffset = ad.TargetLaneOffset
		ad.LaneOffsetStart = ad.LaneOffset
		ad.LaneOffsetProgress = 0.0
		totalDist := math.Abs(ad.TargetLaneOffset - ad.LaneOffsetStart)
		if totalDist < 0.1 {
			ad.LaneOffset = ad.TargetLaneOffset
			ad.LaneOffsetProgress = 1.0
			return
		}
		latSpeed := Clamp(speedAbs*0.32, 65.0, 95.0)
		minDuration := 0.65
		maxDuration := 1.0
		if ad.BerserkMode {
			latSpeed = Clamp(speedAbs*0.50, 95.0, 160.0)
			minDuration = 0.35
			maxDuration = 0.55
		}
		ad.LaneOffsetDuration = Clamp(totalDist/latSpeed, minDuration, maxDuration)
	}

	if ad.LaneOffsetProgress < 1.0 {
		if ad.LaneOffsetDuration <= 1e-4 {
			ad.LaneOffsetDuration = 0.7
		}
		ad.LaneOffsetProgress += dt / ad.LaneOffsetDuration
		if ad.LaneOffsetProgress >= 1.0 {
			ad.LaneOffsetProgress = 1.0
			ad.LaneOffset = ad.TargetLaneOffset
		} else {
			u := ad.LaneOffsetProgress
			// Ken Perlin's quintic smootherstep: 6u^5 - 15u^4 + 10u^3 (zero 1st & 2nd derivative at ends)
			w := u * u * u * (u*(u*6.0-15.0) + 10.0)
			ad.LaneOffset = ad.LaneOffsetStart + (ad.TargetLaneOffset-ad.LaneOffsetStart)*w
		}
	} else if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) > 0.05 {
		// Fallback for direct manual/test manipulation of LaneOffset
		ad.LaneOffsetStart = ad.LaneOffset
		ad.LaneOffsetProgress = 0.0
		totalDist := math.Abs(ad.TargetLaneOffset - ad.LaneOffsetStart)
		latSpeed := Clamp(speedAbs*0.32, 65.0, 95.0)
		ad.LaneOffsetDuration = Clamp(totalDist/latSpeed, 0.65, 1.0)
	}
}

// evaluate6LaneThroughTraffic analyzes vehicles ahead in the two through lanes of a 6-lane road:
// Outer through lane (offset 0.0 from base path) and Inner through lane (offset -lw from base path).
// Returns traffic penalties and blocked flags for each through lane.
func evaluate6LaneThroughTraffic(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, laneWidth, currentLaneOffset float64) (innerScore, outerScore float64, innerBlocked, outerBlocked bool) {
	corridor := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		delta := obs.Pos.Sub(bus.Pos)
		fwd := delta.X*cosH + delta.Y*sinH
		lat := -delta.X*sinH + delta.Y*cosH
		latPath := lat + currentLaneOffset
		corridorWithObs := corridor + obs.Width/2

		// Check outer through lane (offset 0.0)
		distOuter := math.Abs(latPath - 0.0)
		if distOuter < corridorWithObs {
			// If a vehicle is directly alongside or right ahead in outer lane, outer lane is blocked
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < 180.0 {
				outerBlocked = true
			}
			if fwd > -30.0 && fwd < 500.0 {
				w := 1.0
				if obs.Speed < 20.0 {
					w = 4.0 // Stopped at red light or in traffic queue
				} else if obs.Speed < apBerserkCruiseSpeed*0.75 {
					w = 2.0 // Slow vehicle
				}
				distWeight := math.Max(0.0, (500.0-fwd)/500.0)
				outerScore += w * (1.0 + distWeight)
				if fwd < 320.0 && (obs.Speed < 20.0 || fwd < 180.0) {
					outerBlocked = true
				}
			}
		}

		// Check inner through lane (offset -laneWidth)
		distInner := math.Abs(latPath - (-laneWidth))
		if distInner < corridorWithObs {
			// If a vehicle is directly alongside or right ahead in inner lane, inner lane is blocked
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < 180.0 {
				innerBlocked = true
			}
			if fwd > -30.0 && fwd < 500.0 {
				w := 1.0
				if obs.Speed < 20.0 {
					w = 4.0 // Stopped at red light or in traffic queue
				} else if obs.Speed < apBerserkCruiseSpeed*0.75 {
					w = 2.0 // Slow vehicle
				}
				distWeight := math.Max(0.0, (500.0-fwd)/500.0)
				innerScore += w * (1.0 + distWeight)
				if fwd < 320.0 && (obs.Speed < 20.0 || fwd < 180.0) {
					innerBlocked = true
				}
			}
		}
	}
	return
}

// isLaneClearToChange checks whether a target lane offset is clear of vehicles alongside, ahead within safety buffer,
// or approaching fast from behind.
func isLaneClearToChange(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset, targetOffset, laneWidth float64) bool {
	corridorBase := bus.Width/2 + 10.0
	for _, other := range obstacles {
		dOther := other.Pos.Sub(bus.Pos)
		fwdOther := dOther.X*cosH + dOther.Y*sinH
		latOther := -dOther.X*sinH + dOther.Y*cosH
		latPathOther := latOther + currentOffset
		distToTarget := math.Abs(latPathOther - targetOffset)
		corridor := corridorBase + other.Width/2
		if distToTarget < corridor {
			minSafeBehind := -(bus.Length/2 + other.Length/2 + 70.0)
			minSafeAhead := (bus.Length/2 + other.Length/2 + 75.0)
			if fwdOther > minSafeBehind && fwdOther < minSafeAhead {
				return false
			}
			if fwdOther <= minSafeBehind && fwdOther > -240.0 && other.Speed > bus.Speed+15.0 {
				return false
			}
		}
	}
	return true
}

// countQueuedCars counts the number of stopped or queued vehicles in a specific lane corridor
// between the bus and the upcoming stop line.
func countQueuedCars(obstacles []VehicleObstacle, bus *Bus, cosH, sinH, currentOffset, checkOffset, distToStopLine, laneWidth float64) int {
	count := 0
	corridor := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		corridorWithObs := corridor + obs.Width/2
		if math.Abs(latPath-checkOffset) < corridorWithObs {
			if fwd > -20.0 && fwd < distToStopLine+40.0 {
				if obs.Speed < 25.0 {
					count++
				}
			}
		}
	}
	return count
}

// isBaseLaneClearToReturn verifies whether the base lane (offset 0.0) is completely clear of any
// vehicles alongside, closely behind, or approaching fast from behind before initiating a return maneuver.
func isBaseLaneClearToReturn(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset float64, drivePath []Vec2, waypointIdx int) bool {
	corridorBase := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToBaseLane := math.Abs(latPath - 0.0)
		if len(drivePath) >= 2 {
			dPath := minDistanceToPath(obs.Pos, drivePath, waypointIdx)
			if dPath < distToBaseLane {
				distToBaseLane = dPath
			}
		}
		corridor := corridorBase + obs.Width/2
		if distToBaseLane < corridor {
			minSafeBehind := -(bus.Length/2 + obs.Length/2 + 75.0) // ~ -141px
			minSafeAhead := (bus.Length/2 + obs.Length/2 + 80.0)   // ~ +152px
			if fwd > minSafeBehind && fwd < minSafeAhead {
				return false
			}
			if fwd <= minSafeBehind && fwd > -240.0 && obs.Speed > bus.Speed+15.0 {
				return false
			}
		}
	}
	return true
}

func (ad *AutopilotDriver) updateOvertaking(
	bus *Bus,
	world *World,
	tls *TrafficLightSystem,
	obstacles []VehicleObstacle,
	laneWidth float64,
	s, speedAbs, remaining float64,
	dt float64,
) {
	// If Berserk mode is disabled, not cruising, or path invalid: smoothly return to base lane
	if !ad.BerserkMode || ad.State != AutopilotStateCruising || world == nil || len(ad.drivePath) < 2 {
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
	if !isMultiLane {
		// Not a multi-lane road in current travel direction (e.g. 2-lane road with 1 lane each way)
		// Overtaking is strictly prohibited on single-lane roads
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Check if upcoming corner prevents overtaking
	isUpcomingCorner := false
	for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
		d := ad.cumLen[k] - s
		if d < -5.0 {
			continue
		}
		if d > 450.0 {
			break
		}
		a := ad.drivePath[k].Sub(ad.drivePath[k-1])
		b := ad.drivePath[k+1].Sub(ad.drivePath[k])
		// Ignore pure lateral lane-merge jogs that do not change road heading
		if a.Length() < laneWidth*1.3 && b.Length() > 100.0 {
			continue
		}
		turn := math.Abs(NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X)))
		if turn > 0.35 {
			isUpcomingCorner = true
			break
		}
	}

	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	// When approaching destination stop bay to dock: must return to outer curb lane (offset 0.0)
	if remaining < 220.0 {
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Approaching an actual road corner/turn (left or right): abort overtake and return to base lane
	if isUpcomingCorner {
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Determine base lane: outer lane (~1.5*lw from center on 4-lane, ~2.5*lw on 6-lane) or inner lane (~0.5*lw from center)
	basePt := ad.pointAt(s)
	distBaseToCenter := math.Abs(basePt.Y - roadCenter)
	if !isEW {
		distBaseToCenter = math.Abs(basePt.X - roadCenter)
	}

	// =========================================================================
	// INTERSECTION APPROACH LANE SELECTION & RED LIGHT QUEUE BALANCING
	// =========================================================================
	var approachingInter *TrafficIntersection
	distToStopLine := 999.0
	isApproachingIntersection := false
	isUpcomingLeft := false
	isUpcomingRight := false

	if tls != nil {
		for _, inter := range tls.Intersections {
			stopLineOffset := inter.GetStopLineDistForHeading(bus.Heading, laneWidth)
			toInter := inter.Center.Sub(bus.Pos)
			forwardDist := toInter.X*cosH + toInter.Y*sinH
			lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

			if forwardDist > -stopLineOffset && forwardDist < stopLineOffset+500.0 && lateralDist < laneWidth*3.6 {
				approachingInter = inter
				distToStopLine = forwardDist - stopLineOffset
				isApproachingIntersection = (distToStopLine > 0 && distToStopLine < 450.0)

				hasRouteExit := false
				for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
					pt := ad.drivePath[idx]
					toPt := pt.Sub(inter.Center)
					fwdPt := toPt.X*cosH + toPt.Y*sinH
					latPt := -toPt.X*sinH + toPt.Y*cosH

					if fwdPt > 30.0 || math.Abs(latPt) > laneWidth*1.5 {
						hasRouteExit = true
						relAngle := NormalizeAngle(math.Atan2(latPt, fwdPt))
						if relAngle < -0.35 && relAngle > -2.80 {
							isUpcomingLeft = true
							isUpcomingRight = false
							break
						} else if relAngle > 0.35 && relAngle < 2.80 {
							isUpcomingRight = true
							isUpcomingLeft = false
							break
						} else if math.Abs(relAngle) <= 0.35 {
							isUpcomingLeft = false
							isUpcomingRight = false
							break
						}
					}
				}

				if !hasRouteExit {
					for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
						d := ad.cumLen[k] - s
						if d > 450.0 {
							break
						}
						segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
						segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
						if segPrev.Length() < laneWidth*1.3 && segNext.Length() > 100.0 {
							continue
						}
						turnAngle := NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X))
						if turnAngle < -0.35 || math.Abs(turnAngle) >= 2.8 {
							isUpcomingLeft = true
							break
						} else if turnAngle > 0.35 && math.Abs(turnAngle) < 2.8 {
							isUpcomingRight = true
							break
						}
					}
				}
				break
			}
		}
	}

	if isApproachingIntersection && approachingInter != nil {
		isStraight := !isUpcomingLeft && !isUpcomingRight
		hasDedicatedLeft := approachingInter.ApproachHasDedicatedLanesForHeading(bus.Heading)

		if isUpcomingRight {
			// Right turn must be from outer lane (0.0)
			if ad.TargetLaneOffset != 0.0 {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			}
		} else if isUpcomingLeft {
			// Left turn: check whether the route path (drivePath) already leads into the dedicated
			// left-turn lane (offset <= 1.15*laneWidth from road center).
			// If drivePath already enters the left-turn lane, the desired offset on drivePath is 0.0!
			// Commanding -laneWidth on an already-centered left-turn path would double-shift the vehicle
			// across the double yellow line into oncoming traffic.
			desiredOffset := 0.0
			if distBaseToCenter > laneWidth*1.15 {
				// Only if drivePath stayed in the outer lane (e.g. synthetic test fixtures)
				desiredOffset = -(distBaseToCenter - laneWidth*0.5)
				if desiredOffset < -laneWidth*2.0 {
					desiredOffset = -laneWidth * 2.0
				}
			}
			if ad.TargetLaneOffset != desiredOffset {
				if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, laneWidth) {
					ad.TargetLaneOffset = desiredOffset
					ad.OvertakeState = OvertakeChangingLane
				}
			} else if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
				ad.LaneOffset = ad.TargetLaneOffset
				ad.OvertakeState = OvertakeNone
			}
		} else if isStraight {
			// Going straight through intersection:
			if is6Lane {
				// 6-Lane Road: Two valid straight through lanes:
				// Middle through lane (-laneWidth) and Outer through lane (0.0).
				// (Innermost lane -2*lw is dedicated left-turn, strictly prohibited for straight)
				queuedMiddle := countQueuedCars(obstacles, bus, cosH, sinH, ad.LaneOffset, -laneWidth, distToStopLine, laneWidth)
				queuedOuter := countQueuedCars(obstacles, bus, cosH, sinH, ad.LaneOffset, 0.0, distToStopLine, laneWidth)

				var desiredOffset float64
				if queuedMiddle < queuedOuter {
					desiredOffset = -laneWidth
				} else if queuedOuter < queuedMiddle {
					desiredOffset = 0.0
				} else {
					if queuedMiddle == 0 && queuedOuter == 0 {
						innerScore, outerScore, innerBlocked, outerBlocked := evaluate6LaneThroughTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset)
						if innerBlocked && !outerBlocked {
							desiredOffset = 0.0
						} else if outerBlocked && !innerBlocked {
							desiredOffset = -laneWidth
						} else if outerScore > innerScore+0.6 {
							desiredOffset = -laneWidth
						} else if innerScore > outerScore+0.6 {
							desiredOffset = 0.0
						} else {
							if ad.LaneOffset < -laneWidth*0.5 {
								desiredOffset = -laneWidth
							} else {
								desiredOffset = 0.0
							}
						}
					} else {
						if ad.LaneOffset < -laneWidth*0.5 {
							desiredOffset = -laneWidth
						} else {
							desiredOffset = 0.0
						}
					}
				}

				if desiredOffset != ad.TargetLaneOffset {
					if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, laneWidth) {
						ad.TargetLaneOffset = desiredOffset
						ad.OvertakeState = OvertakeChangingLane
						ad.OvertakeTimer = 0.0
					}
				} else if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
					ad.LaneOffset = ad.TargetLaneOffset
					ad.OvertakeState = OvertakeNone
				}
			} else if hasDedicatedLeft {
				// 4-Lane Road with dedicated left-turn lane:
				// Left lane is STRICTLY dedicated to left turns.
				// STRAIGHT TRAFFIC MUST NEVER ENTER LEFT-TURN LANE!
				// When cars ahead are waiting for red light, queue in outer through lane!
				if ad.LaneOffset < -laneWidth*0.5 || ad.TargetLaneOffset < -laneWidth*0.5 {
					// Currently in left lane: must return to through lane as soon as safe
					if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
						ad.TargetLaneOffset = 0.0
						ad.OvertakeState = OvertakeReturning
					}
				} else {
					ad.TargetLaneOffset = 0.0
					if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset) < 1.0 {
						ad.LaneOffset = 0.0
						ad.OvertakeState = OvertakeNone
					}
				}
			} else {
				// Multi-lane road without dedicated left-turn lane: both lanes can go straight
				queuedInner := countQueuedCars(obstacles, bus, cosH, sinH, ad.LaneOffset, -laneWidth, distToStopLine, laneWidth)
				queuedOuter := countQueuedCars(obstacles, bus, cosH, sinH, ad.LaneOffset, 0.0, distToStopLine, laneWidth)
				var desiredOffset float64
				if queuedInner < queuedOuter {
					desiredOffset = -laneWidth
				} else if queuedOuter < queuedInner {
					desiredOffset = 0.0
				} else {
					if ad.LaneOffset < -laneWidth*0.5 {
						desiredOffset = -laneWidth
					} else {
						desiredOffset = 0.0
					}
				}
				if desiredOffset != ad.TargetLaneOffset {
					if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, laneWidth) {
						ad.TargetLaneOffset = desiredOffset
						ad.OvertakeState = OvertakeChangingLane
						ad.OvertakeTimer = 0.0
					}
				} else if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
					ad.LaneOffset = ad.TargetLaneOffset
					ad.OvertakeState = OvertakeNone
				}
			}
		}

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Mid-block overtaking
	var passOffset float64
	if distBaseToCenter > laneWidth*0.95 {
		passOffset = -laneWidth // Inner passing lane (left side in RHT)
	} else {
		// Base path is in inner lane. Check if approaching left turn
		hasUpcomingLeftTurn := false
		for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
			d := ad.cumLen[k] - s
			if d > 450.0 {
				break
			}
			a := ad.drivePath[k].Sub(ad.drivePath[k-1])
			b := ad.drivePath[k+1].Sub(ad.drivePath[k])
			turn := NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
			if turn < -0.35 || math.Abs(turn) >= 2.8 {
				hasUpcomingLeftTurn = true
				break
			}
		}
		if hasUpcomingLeftTurn {
			// Stay in left-turn lane, do not overtake
			if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			}
			ad.progressLaneOffset(dt, speedAbs)
			return
		}
		passOffset = laneWidth // Outer lane (right side)
	}

	// =========================================================================
	// 6-LANE ROAD STRAIGHT-THROUGH MID-BLOCK TRAFFIC SELECTION
	// =========================================================================
	is6LaneStraight := is6Lane && !isUpcomingCorner && remaining >= 220.0
	if is6LaneStraight {
		innerScore, outerScore, innerBlocked, outerBlocked := evaluate6LaneThroughTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset)

		var desiredOffset float64
		if innerBlocked && !outerBlocked {
			desiredOffset = 0.0 // Inner through lane blocked; use outer through lane
		} else if outerBlocked && !innerBlocked {
			desiredOffset = -laneWidth // Outer through lane blocked; use inner through lane
		} else if outerScore > innerScore+0.6 {
			desiredOffset = -laneWidth // Inner through lane has fewer cars
		} else if innerScore > outerScore+0.6 {
			desiredOffset = 0.0 // Outer through lane has fewer cars
		} else {
			// Traffic is comparable: maintain current lane (hysteresis to prevent zigzagging)
			if ad.LaneOffset < -laneWidth*0.5 {
				desiredOffset = -laneWidth
			} else {
				desiredOffset = 0.0
			}
		}

		if ad.OvertakeState == OvertakeNone {
			if desiredOffset != ad.TargetLaneOffset {
				if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, desiredOffset, laneWidth) {
					ad.OvertakeState = OvertakeChangingLane
					ad.TargetLaneOffset = desiredOffset
					ad.OvertakeTimer = 0.0
				}
			}
		}
	}

	switch ad.OvertakeState {
	case OvertakeNone:
		if !is6LaneStraight {
			// Scan for slow or stopped lead vehicle directly ahead in our current lane
			var blockerDist float64 = 999.0
			foundBlocker := false
			fwdMax := 180.0
			if ad.BerserkMode {
				// Increased lookahead (提前量) in Berserk mode: detect slow lead vehicles
				// up to 550px ahead so lane change initiates well in advance without close-range tailgating
				fwdMax = math.Max(480.0, speedAbs*1.8+100.0)
			}
			for _, obs := range obstacles {
				delta := obs.Pos.Sub(bus.Pos)
				fwd := delta.X*cosH + delta.Y*sinH
				lat := math.Abs(-delta.X*sinH + delta.Y*cosH)
				if fwd > (bus.Length/2+obs.Length/2+12.0) && fwd < fwdMax && lat < (bus.Width/2+obs.Width/2+8.0) {
					if math.Cos(obs.Heading-bus.Heading) > 0.4 {
						if obs.Speed < apBerserkCruiseSpeed*0.85 || obs.Speed < 10.0 {
							if fwd < blockerDist {
								blockerDist = fwd
								foundBlocker = true
							}
						}
					}
				}
			}

			if foundBlocker {
				// Check if passing lane is clear and safe
				passingClear := true
				for _, other := range obstacles {
					dOther := other.Pos.Sub(bus.Pos)
					fwdOther := dOther.X*cosH + dOther.Y*sinH
					latOther := -dOther.X*sinH + dOther.Y*cosH
					latPathOther := latOther + ad.LaneOffset
					distToPassLane := math.Abs(latPathOther - passOffset)
					corridor := bus.Width/2 + other.Width/2 + 10.0
					if distToPassLane < corridor {
						minSafeBehind := -(bus.Length/2 + other.Length/2 + 65.0)
						if fwdOther > minSafeBehind && fwdOther < math.Min(blockerDist+80.0, 480.0) {
							if other.Speed < apBerserkCruiseSpeed*0.70 || fwdOther < 220.0 {
								passingClear = false
								break
							}
						}
						if fwdOther <= minSafeBehind && fwdOther > -240.0 && other.Speed > bus.Speed+15.0 {
							passingClear = false
							break
						}
					}
				}

				if passingClear {
					// Initiate overtaking maneuver early
					ad.OvertakeState = OvertakeChangingLane
					ad.TargetLaneOffset = passOffset
					ad.OvertakeTimer = 0.0
				}
			}
		}

	case OvertakeChangingLane:
		ad.OvertakeTimer += dt
		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
			if is6LaneStraight {
				// Arrived in selected straight through lane on 6-lane road
				innerScore, outerScore, innerBlocked, outerBlocked := evaluate6LaneThroughTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset)
				if ad.TargetLaneOffset == -laneWidth {
					if outerBlocked || outerScore >= innerScore {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					}
				} else {
					if innerBlocked || innerScore >= outerScore {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					}
				}
			} else {
				ad.OvertakeState = OvertakePassing
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakePassing:
		ad.OvertakeTimer += dt

		// Check if we have passed all obstacles in the base lane
		// A vehicle is NOT safely cleared if:
		// 1. It is ahead of the bus (fwd >= 0 and fwd < 380.0)
		// 2. Or it is alongside/behind the bus without adequate clearance (fwd > minSafeBehind)
		hasObstacleNotCleared := false
		for _, obs := range obstacles {
			d := obs.Pos.Sub(bus.Pos)
			fwd := d.X*cosH + d.Y*sinH
			lat := -d.X*sinH + d.Y*cosH
			latPath := lat + ad.LaneOffset
			distToBaseLane := math.Abs(latPath - 0.0)
			if len(ad.drivePath) >= 2 {
				dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
				if dPath < distToBaseLane {
					distToBaseLane = dPath
				}
			}
			corridor := bus.Width/2 + obs.Width/2 + 10.0
			if distToBaseLane < corridor {
				// Obstacle is in the base lane!
				minSafeBehind := -(bus.Length/2 + obs.Length/2 + 75.0) // ~ -141px
				if fwd > minSafeBehind && fwd < 380.0 {
					hasObstacleNotCleared = true
					break
				}
			}
		}

		shouldReturn := false
		if !hasObstacleNotCleared && ad.OvertakeTimer > 0.6 {
			// On 6-lane roads going straight: if inner through lane has fewer/equal cars, DO NOT return to outer lane!
			if is6LaneStraight {
				innerScore, outerScore, _, outerBlocked := evaluate6LaneThroughTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset)
				if outerBlocked || outerScore >= innerScore {
					// Stay in inner through lane (-laneWidth)!
					ad.TargetLaneOffset = -laneWidth
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
					break
				}
			}
			shouldReturn = true
		}

		if shouldReturn {
			// Verify base lane is 100% clear before initiating return maneuver
			if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx) {
				ad.OvertakeState = OvertakeReturning
				ad.TargetLaneOffset = 0.0
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeReturning:
		ad.OvertakeTimer += dt

		// Active collision avoidance during return maneuver:
		// If base lane suddenly becomes obstructed while we are still transitioning, abort return!
		if math.Abs(ad.LaneOffset) > laneWidth*0.20 {
			for _, obs := range obstacles {
				d := obs.Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				lat := -d.X*sinH + d.Y*cosH
				latPath := lat + ad.LaneOffset
				distToBaseLane := math.Abs(latPath - 0.0)
				if len(ad.drivePath) >= 2 {
					dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
					if dPath < distToBaseLane {
						distToBaseLane = dPath
					}
				}
				if distToBaseLane < (bus.Width/2 + obs.Width/2 + 10.0) {
					dangerBehind := -(bus.Length/2 + obs.Length/2 + 45.0)
					dangerAhead := (bus.Length/2 + obs.Length/2 + 55.0)
					if fwd > dangerBehind && fwd < dangerAhead {
						// Imminent collision! Abort return and swing back into passing lane
						ad.TargetLaneOffset = passOffset
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
						break
					}
				}
			}
		}

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
		}
	}

	ad.progressLaneOffset(dt, speedAbs)
}

// ComputeControls calculates vehicle inputs (throttle, brake, steer, handbrake, toggleDoors)
// along with navigation guidance for any bus following a Route in World.
func (ad *AutopilotDriver) ComputeControls(
	bus *Bus,
	route *Route,
	world *World,
	tls *TrafficLightSystem,
	obstacles []VehicleObstacle,
	laneWidth float64,
	dt float64,
) (throttle, brake, steer float64, handbrake, toggleDoors bool, isDocked bool) {
	if route == nil || len(route.StopIDs) == 0 || world == nil || len(world.Stops) == 0 {
		return 0.0, 1.0, 0.0, true, false, false
	}

	targetStopID := route.GetCurrentTargetStopID()
	if targetStopID < 0 || targetStopID >= len(world.Stops) {
		return 0.0, 1.0, 0.0, true, false, false
	}
	targetStop := world.Stops[targetStopID]
	ad.LastTargetID = targetStopID

	// Find previous stop in sequence to obtain the driving lane path of the active leg
	prevIdx := (route.CurrentIndex - 1 + len(route.StopIDs)) % len(route.StopIDs)
	prevStopID := route.StopIDs[prevIdx]
	var prevStop *BusStop
	if prevStopID >= 0 && prevStopID < len(world.Stops) {
		prevStop = world.Stops[prevStopID]
	} else {
		prevStop = targetStop
	}

	distToStop := bus.Pos.Distance(targetStop.Pos)
	speedAbs := math.Abs(bus.Speed)

	// Ensure automatic transmission stays in Drive
	if bus.Gear != GearDrive {
		bus.ShiftTo(GearDrive)
	}

	// =========================================================================
	// 1. DOCKED STATE: Passenger Boarding, Dwell Timer & Door Cycling
	// =========================================================================
	if ad.State == AutopilotStateDocked {
		throttle = 0.0
		brake = 1.0
		steer = 0.0
		isDocked = true

		// If dwell is not yet finished:
		if !ad.DwellFinished {
			// Phase 1: Open doors if stopped and doors not yet opened
			openThreshold := 5.0
			if ad.BerserkMode {
				openThreshold = 15.0 // Trigger doors immediately as vehicle enters bay
			}
			if !bus.DoorsOpen && speedAbs < openThreshold && bus.DoorProgress <= 0.05 {
				toggleDoors = true
				return throttle, brake, steer, handbrake, toggleDoors, isDocked
			}

			// Phase 2: Dwell timer while doors are open
			if bus.DoorsOpen && ad.DwellTimer > 0 {
				ad.DwellTimer -= dt
				return throttle, brake, steer, handbrake, false, isDocked
			}

			// Phase 3: Dwell expired, close doors
			if bus.DoorsOpen && ad.DwellTimer <= 0 && bus.DoorProgress >= 0.80 {
				toggleDoors = true
				ad.DwellFinished = true
				return throttle, brake, steer, handbrake, toggleDoors, isDocked
			}
		}

		// Phase 4: Wait for doors to finish closing completely, then advance and depart
		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			if route.GetCurrentTargetStopID() == ad.DockedStopID {
				route.AdvanceToNextStop()
			}
			ad.LastTargetID = route.GetCurrentTargetStopID()
			ad.State = AutopilotStateDeparting
			ad.DockedStopID = -1
			ad.WaypointIdx = 0
			ad.pathSynced = false
			ad.DwellFinished = false
			return 1.0, 0.0, 0.0, false, false, false
		}
		return throttle, brake, steer, handbrake, false, isDocked
	}

	// =========================================================================
	// 2. PATH LOCALIZATION
	// =========================================================================
	ad.ensurePath(world, prevStop, targetStop, laneWidth)
	if !ad.pathSynced {
		ad.trackPath(bus.Pos)
	}

	remDist := ad.LastRemaining
	if remDist <= 0.0 && len(ad.cumLen) > 0 {
		remDist = ad.cumLen[len(ad.cumLen)-1] - ad.LastPathS
	}
	if remDist <= 0.0 {
		remDist = bus.Pos.Distance(targetStop.Pos)
	}

	// Update overtaking controller (Berserk Mode on multi-lane roads)
	ad.updateOvertaking(bus, world, tls, obstacles, laneWidth, ad.LastPathS, speedAbs, remDist, dt)

	posForTracking := bus.Pos
	if ad.LaneOffset != 0 && len(ad.drivePath) >= 2 {
		wIdx := ad.WaypointIdx
		if wIdx >= len(ad.drivePath)-1 {
			wIdx = len(ad.drivePath) - 2
		}
		seg := ad.drivePath[wIdx+1].Sub(ad.drivePath[wIdx])
		if segLen := seg.Length(); segLen > 1e-6 {
			segDir := seg.Mul(1.0 / segLen)
			segRight := Vec2{X: -segDir.Y, Y: segDir.X}
			posForTracking = bus.Pos.Sub(segRight.Mul(ad.LaneOffset))
		}
	}

	s, lateralErr := ad.trackPath(posForTracking)
	total := 0.0
	if n := len(ad.cumLen); n > 0 {
		total = ad.cumLen[n-1]
	}
	remaining := math.Max(0, total-s)

	// =========================================================================
	// 3. DOCKING TRIGGER (Only when arriving, never while departing)
	// =========================================================================
	alignedNearBay := distToStop < 45.0 && math.Abs(NormalizeAngle(bus.Heading-targetStop.Heading)) < 0.45
	// Fallback: the bus has come to rest at the very end of its path right at the stop
	restingAtStop := remaining < 10.0 && distToStop < 70.0
	maxDockSpeed := 8.0
	if ad.BerserkMode {
		maxDockSpeed = 22.0 // Snap into docked state immediately as bus enters the bay
	}
	if ad.State != AutopilotStateDeparting && speedAbs < maxDockSpeed &&
		(targetStop.ContainsBus(bus) || alignedNearBay || restingAtStop) {
		ad.State = AutopilotStateDocked
		ad.DockedStopID = targetStop.ID
		ad.DwellTimer = ad.DwellTotal
		if ad.BerserkMode {
			ad.DwellTimer = 0.6 // Rapid 0.6s dwell in Berserk mode for lightning stop service
		}
		ad.DwellFinished = false
		if !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			toggleDoors = true
		}
		return 0.0, 1.0, 0.0, false, toggleDoors, true
	}

	// If doors are open outside docked state (e.g. player parked then toggled autopilot),
	// command doors to close immediately so vehicle can start moving!
	if bus.DoorsOpen || bus.DoorProgress > 0.05 {
		closeCmd := false
		if bus.DoorsOpen && speedAbs < 5.0 {
			closeCmd = true
		}
		return 0.0, 1.0, 0.0, false, closeCmd, false
	}

	// Transition between Depart, Cruise, and Approach
	approachThreshold := apApproachDist
	if ad.BerserkMode {
		// Late aggressive braking in Berserk mode: cruise near full speed until closer to stop
		approachThreshold = math.Max(apApproachDist*0.65, speedAbs*speedAbs/(2*160.0)+35.0)
	}
	if remaining < approachThreshold {
		ad.State = AutopilotStateApproaching
	} else if ad.State == AutopilotStateDeparting {
		if bus.Pos.Distance(prevStop.Pos) > 120.0 {
			ad.State = AutopilotStateCruising
		}
	} else {
		ad.State = AutopilotStateCruising
	}

	// =========================================================================
	// 4. PURE PURSUIT STEERING (lookahead measured from the bus's own path position)
	// =========================================================================
	nearRoundabout := false
	if world != nil {
		for _, rb := range world.Roundabouts {
			if bus.Pos.Distance(rb.Center) <= rb.Radius+80.0 {
				nearRoundabout = true
				break
			}
		}
	}

	lookahead := Clamp(speedAbs*0.55+45.0, 55.0, 140.0)
	if nearRoundabout {
		lookahead = Clamp(speedAbs*0.35+30.0, 35.0, 52.0)
	} else if ad.BerserkMode {
		lookahead = Clamp(speedAbs*0.55+45.0, 55.0, 210.0)
	}

	// Clamp lookahead when approaching or executing sharp corners so Pure Pursuit does not cut across the corner
	// into oncoming lanes before or during the intersection turn (not for circular roundabout tracking):
	if !nearRoundabout {
		startClampK := ad.WaypointIdx
		if startClampK < 1 {
			startClampK = 1
		}
		for k := startClampK; k < len(ad.drivePath)-1; k++ {
			d := ad.cumLen[k] - s
			if d < -20.0 {
				continue
			}
			if d > lookahead+60.0 {
				break
			}
			segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
			segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
			turn := math.Abs(NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
			if k+2 < len(ad.drivePath) {
				seg2 := ad.drivePath[k+2].Sub(ad.drivePath[k+1])
				t2 := math.Abs(NormalizeAngle(math.Atan2(seg2.Y, seg2.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t2 > turn {
					turn = t2
				}
			}
			if k+4 < len(ad.drivePath) {
				seg4 := ad.drivePath[k+4].Sub(ad.drivePath[k+3])
				t4 := math.Abs(NormalizeAngle(math.Atan2(seg4.Y, seg4.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t4 > turn {
					turn = t4
				}
			}
			if k+6 < len(ad.drivePath) {
				seg6 := ad.drivePath[k+6].Sub(ad.drivePath[k+5])
				t6 := math.Abs(NormalizeAngle(math.Atan2(seg6.Y, seg6.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if t6 > turn {
					turn = t6
				}
			}

			if turn > 0.15 {
				maxLookahead := 65.0
				if turn > 0.35 {
					maxLookahead = 52.0
				}
				if turn > 0.8 {
					maxLookahead = 42.0
				}
				if lookahead > maxLookahead {
					lookahead = maxLookahead
				}
				break
			}
		}
	}
	baseTargetPt := ad.pointAt(s + lookahead)
	targetPt := baseTargetPt
	if ad.LaneOffset != 0 && len(ad.drivePath) >= 2 {
		nextPt := ad.pointAt(s + lookahead + 2.0)
		toNext := nextPt.Sub(baseTargetPt)
		if toNext.Length() > 1e-3 {
			uNext := toNext.Normalize()
			trgRight := Vec2{X: -uNext.Y, Y: uNext.X}
			targetPt = baseTargetPt.Add(trgRight.Mul(ad.LaneOffset))
		}
	}
	toTarget := targetPt.Sub(bus.Pos)
	alpha := NormalizeAngle(math.Atan2(toTarget.Y, toTarget.X) - bus.Heading)
	// A target behind the bus only happens when badly off-path: turn at full lock, never reverse
	alpha = Clamp(alpha, -math.Pi/2, math.Pi/2)
	ld := math.Max(toTarget.Length(), 1.0)
	delta := math.Atan2(2.0*bus.WheelBase*math.Sin(alpha), ld)
	steer = Clamp(delta/bus.MaxSteerAngle, -1.0, 1.0)

	// =========================================================================
	// 5. SPEED & BRAKING CONTROLLER
	// =========================================================================
	targetSpd := apCruiseSpeed
	isApproachingSharpTurn := false
	for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
		d := ad.cumLen[k] - s
		if d < -10.0 {
			continue
		}
		if d > 280.0 {
			break
		}
		segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
		segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
		turnAngle := math.Abs(NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
		if turnAngle > 0.6 {
			isApproachingSharpTurn = true
			break
		}
	}

	if ad.BerserkMode {
		if !isApproachingSharpTurn {
			// On straight roadways & mid-block lane changes in Berserk mode, maintain full cruise speed
			targetSpd = apBerserkCruiseSpeed
		} else {
			targetSpd = math.Max(apCruiseSpeed, ad.cornerSpeedLimit(s, speedAbs))
		}
	}

	// A. Anticipate corners / roundabout arcs ahead, and slow while circulating
	targetSpd = math.Min(targetSpd, ad.cornerSpeedLimit(s, speedAbs))
	if nearRoundabout {
		targetSpd = math.Min(targetSpd, apRoundaboutSpeed)
	}

	// B. Large heading error or lateral deviation: slow down to recover the lane
	// During active overtaking (lane change, passing, returning), lateral offset is intentional;
	// only clamp to 55 px/s when not maneuvering, or if heading error indicates a severe spinout.
	if ad.OvertakeState == OvertakeNone {
		// Probe heading alignment between the bus and the upcoming path direction:
		pNow := ad.pointAt(s)
		pAhead := ad.pointAt(s + 25.0)
		segTangent := pAhead.Sub(pNow)
		hErr := 0.0
		if segTangent.Length() > 0.5 {
			pathHeading := math.Atan2(segTangent.Y, segTangent.X)
			hErr = math.Abs(NormalizeAngle(pathHeading - bus.Heading))
		}
		if ad.BerserkMode {
			if isApproachingSharpTurn {
				// Near intersections/sharp corners, maintain responsive cornering speed and safe lane alignment
				if math.Abs(alpha) > 0.70 || hErr > 0.70 || lateralErr > 50.0 {
					targetSpd = math.Min(targetSpd, 75.0)
				} else if math.Abs(alpha) > 0.35 || hErr > 0.35 {
					targetSpd = math.Min(targetSpd, 85.0)
				}
			} else {
				// On straight roads, lane changes and gentle merges should fly through without sluggish braking
				if math.Abs(alpha) > 0.75 || hErr > 0.75 || lateralErr > 55.0 {
					targetSpd = math.Min(targetSpd, 100.0)
				} else if math.Abs(alpha) > 0.35 || hErr > 0.35 {
					targetSpd = math.Min(targetSpd, 130.0)
				}
			}
		} else {
			// Standard mode: smooth, safe cornering
			if math.Abs(alpha) > 0.5 || hErr > 0.5 || lateralErr > 40.0 {
				targetSpd = math.Min(targetSpd, 38.0)
			} else if math.Abs(alpha) > 0.25 || hErr > 0.25 {
				targetSpd = math.Min(targetSpd, 48.0)
			}
		}
	} else {
		if math.Abs(alpha) > 1.1 {
			targetSpd = math.Min(targetSpd, 70.0)
		}
	}

	// C. Station approach: brake along a constant-deceleration profile to stop at the bay center
	stopDecel := apStopDecel
	if ad.BerserkMode {
		stopDecel = 160.0 // Strong crisp deceleration in Berserk mode
	}
	stopSpd := math.Sqrt(2 * stopDecel * math.Max(0, remaining-2.0))
	if remaining > 6.0 {
		if ad.BerserkMode {
			stopSpd = math.Max(stopSpd, 36.0) // Maintain swift speed directly into the bay
		} else {
			stopSpd = math.Max(stopSpd, 14.0)
		}
	} else {
		stopSpd = 0
	}
	targetSpd = math.Min(targetSpd, stopSpd)

	// D. Red / Yellow Light Stopping
	if tls != nil {
		lookaheadTL := math.Max(140.0, speedAbs*speedAbs/(2*130.0)+60.0)

		isUpcomingLeft := false
		isUpcomingRight := false

		defaultStopLineDist := laneWidth*1.6 + 18.0
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)

		// 1. Direct intersection route analysis:
		// Check the intersection that the bus is approaching within lookahead distance
		for _, inter := range tls.Intersections {
			stopLineDist := inter.GetStopLineDistForHeading(bus.Heading, laneWidth)
			toInter := inter.Center.Sub(bus.Pos)
			forwardDist := toInter.X*cosH + toInter.Y*sinH
			lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

			if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookaheadTL+120.0 && lateralDist < laneWidth*3.6 {
				hasRouteExit := false

				// Check ad.drivePath points through and beyond this intersection
				for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
					pt := ad.drivePath[idx]
					toPt := pt.Sub(inter.Center)
					fwdPt := toPt.X*cosH + toPt.Y*sinH
					latPt := -toPt.X*sinH + toPt.Y*cosH

					// Exit point past the intersection center mouth
					if fwdPt > 30.0 || math.Abs(latPt) > laneWidth*1.5 {
						hasRouteExit = true
						relAngle := NormalizeAngle(math.Atan2(latPt, fwdPt))
						if relAngle < -0.35 && relAngle > -2.80 {
							isUpcomingLeft = true
							isUpcomingRight = false
							break
						} else if relAngle > 0.35 && relAngle < 2.80 {
							isUpcomingRight = true
							isUpcomingLeft = false
							break
						} else if math.Abs(relAngle) <= 0.35 {
							// Explicit straight path through intersection
							isUpcomingLeft = false
							isUpcomingRight = false
							break
						}
					}
				}

				// If no exit waypoint was found, check if vehicle is physically positioned in dedicated left-turn lane
				if !hasRouteExit && inter.ApproachHasDedicatedLanesForHeading(bus.Heading) {
					distFromCenter := 0.0
					if math.Abs(cosH) > math.Abs(sinH) {
						distFromCenter = math.Abs(bus.Pos.Y - inter.Center.Y)
					} else {
						distFromCenter = math.Abs(bus.Pos.X - inter.Center.X)
					}
					if distFromCenter > 5.0 && distFromCenter < laneWidth*0.95 {
						isUpcomingLeft = true
					}
				}
				break
			}
		}

		// 2. Fallback waypoint turn angle check (for test fixtures without full intersection objects)
		if !isUpcomingLeft && !isUpcomingRight {
			lookaheadTurn := math.Max(450.0, lookaheadTL+defaultStopLineDist+150.0)
			for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
				d := ad.cumLen[k] - s
				if d < -10.0 {
					continue
				}
				if d > lookaheadTurn {
					break
				}
				segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
				segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
				// Ignore pure lateral lane-merge jogs that do not change road heading
				if segPrev.Length() < laneWidth*1.3 && segNext.Length() > 100.0 {
					continue
				}
				turnAngle := NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X))
				if turnAngle > 0.35 && math.Abs(turnAngle) < 2.8 {
					isUpcomingRight = true
				} else if turnAngle < -0.35 || math.Abs(turnAngle) >= 2.8 {
					isUpcomingLeft = true
				}
			}
		}

		// Mutual exclusion: left turn cannot be right turn
		if isUpcomingLeft {
			isUpcomingRight = false
		}

		bumperPos := bus.Pos.Add(Vec2{X: cosH * (bus.Length * 0.5), Y: sinH * (bus.Length * 0.5)})

		shouldStop, distToLine := tls.CheckStopForVehicle(bumperPos, bus.Heading, lookaheadTL, laneWidth, isUpcomingLeft, isUpcomingRight, ad.BerserkMode)
		if shouldStop {
			if distToLine <= 14.0 {
				targetSpd = 0.0
			} else {
				stopProfile := math.Sqrt(2 * 135.0 * math.Max(0.0, distToLine-14.0))
				targetSpd = math.Min(targetSpd, stopProfile)
			}
		}
	}

	// E. Forward Vehicle Obstacle Avoidance
	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)
	fwdLimit := 140.0
	if ad.BerserkMode {
		fwdLimit = math.Max(140.0, speedAbs*speedAbs/(2*130.0)+80.0)
	}
	for _, obs := range obstacles {
		// 1. If bus is near a roundabout, vehicles that are stopped well outside
		// the roundabout curb and away from the bus's path should not trigger false braking
		if world != nil {
			isSafelyWaitingOutside := false
			for _, rb := range world.Roundabouts {
				if bus.Pos.Distance(rb.Center) < rb.Radius+10.0 {
					// Obstacle is truly outside the circular roadway, essentially stopped, and safely distanced from driving path
					if obs.Pos.Distance(rb.Center) > rb.Radius+15.0 && obs.Speed < 5.0 {
						if len(ad.drivePath) >= 2 {
							dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
							if dPath > (bus.Width/2 + obs.Width/2 + 6.0) {
								isSafelyWaitingOutside = true
								break
							}
						} else if obs.Pos.Distance(bus.Pos) > (bus.Length/2 + obs.Length/2 + 25.0) {
							isSafelyWaitingOutside = true
							break
						}
					}
				}
			}
			if isSafelyWaitingOutside {
				continue
			}
		}

		delta := obs.Pos.Sub(bus.Pos)
		fwdDist := delta.X*cosH + delta.Y*sinH
		latDist := math.Abs(-delta.X*sinH + delta.Y*cosH)

		// Immediate physical proximity / collision containment:
		// If obstacle directly overlaps with or is right against the bus front envelope:
		directDist := obs.Pos.Distance(bus.Pos)
		collisionDist := bus.Length/2 + obs.Length/2 + 8.0
		if directDist < collisionDist && fwdDist > -10.0 && latDist < (bus.Width/2+obs.Width/2+8.0) {
			targetSpd = 0.0
			continue
		}

		if fwdDist > 2.0 && fwdDist < fwdLimit && latDist < (bus.Width/2+obs.Width/2+12.0) {
			// For vehicles where we have a valid tracked path: verify obstacle is within our path corridor
			if len(ad.drivePath) >= 2 {
				obsPos := obs.Pos
				if ad.LaneOffset != 0 {
					wIdx := ad.WaypointIdx
					if wIdx >= len(ad.drivePath)-1 {
						wIdx = len(ad.drivePath) - 2
					}
					seg := ad.drivePath[wIdx+1].Sub(ad.drivePath[wIdx])
					if segLen := seg.Length(); segLen > 1e-6 {
						segDir := seg.Mul(1.0 / segLen)
						segRight := Vec2{X: -segDir.Y, Y: segDir.X}
						obsPos = obs.Pos.Sub(segRight.Mul(ad.LaneOffset))
					}
				}
				dPath := minDistanceToPath(obsPos, ad.drivePath, ad.WaypointIdx)
				if dPath > (bus.Width/2 + obs.Width/2 + 10.0) {
					// Obstacle is outside our tracked path corridor.
					// If actively changing lanes, only ignore the obstacle if we have PHYSICALLY cleared it laterally.
					if ad.OvertakeState == OvertakeChangingLane {
						minClearLat := bus.Width/2 + obs.Width/2 + 5.0
						if latDist < minClearLat {
							// Not laterally clear yet! Maintain safe buffer to avoid clipping bumper
							safeDist := bus.Length/2 + obs.Length/2 + 30.0
							if ad.BerserkMode {
								safeDist = bus.Length/2 + obs.Length/2 + 16.0
							}
							if fwdDist < safeDist {
								if ad.BerserkMode {
									targetSpd = math.Min(targetSpd, math.Max(obs.Speed+35.0, 120.0))
								} else {
									targetSpd = math.Min(targetSpd, math.Max(0.0, obs.Speed*0.9))
								}
							}
						}
					}
					continue
				}
			}

			// For oncoming vehicles (facing opposite direction), ignore if safely separated laterally
			cosHeadDiff := math.Cos(obs.Heading - bus.Heading)
			if cosHeadDiff < -0.2 && latDist > (bus.Width/2+obs.Width/2+2.0) {
				continue
			}

			safeStopDist := bus.Length/2 + obs.Length/2 + 25.0
			if ad.BerserkMode && ad.OvertakeState == OvertakeChangingLane {
				safeStopDist = bus.Length/2 + obs.Length/2 + 16.0
			}
			if fwdDist < safeStopDist {
				targetSpd = 0.0
			} else {
				leadSpd := math.Max(0.0, obs.Speed)
				decelRate := 135.0
				if ad.BerserkMode {
					decelRate = 180.0
				}
				decelSpd := leadSpd + math.Sqrt(2*decelRate*math.Max(0, fwdDist-safeStopDist))
				if ad.BerserkMode && ad.OvertakeState == OvertakeChangingLane {
					decelSpd = math.Max(decelSpd, leadSpd+35.0)
				}
				targetSpd = math.Min(targetSpd, decelSpd)
			}
		}
	}

	// F. Acceleration & Foot Braking
	switch {
	case targetSpd <= 0.5:
		throttle = 0.0
		if speedAbs < 10.0 {
			brake = 1.0
		} else {
			brake = Clamp(speedAbs/50.0, 0.5, 1.0)
		}
	case bus.Speed < targetSpd-2.5:
		if ad.BerserkMode {
			throttle = Clamp((targetSpd-bus.Speed)/22.0, 0.45, 1.0)
		} else {
			throttle = Clamp((targetSpd-bus.Speed)/35.0, 0.3, 1.0)
		}
		brake = 0.0
	case bus.Speed > targetSpd+3.5:
		throttle = 0.0
		if ad.BerserkMode {
			brake = Clamp((bus.Speed-targetSpd)/25.0, 0.2, 1.0)
		} else {
			brake = Clamp((bus.Speed-targetSpd)/40.0, 0.08, 1.0)
		}
	case bus.Speed < targetSpd:
		throttle = 0.15
		brake = 0.0
	default:
		throttle = 0.0 // coast (rolling resistance) inside the hold band
		brake = 0.0
	}

	ad.LastPathS = s
	ad.LastRemaining = remaining
	ad.LastLateralErr = lateralErr
	ad.LastPursuitPt = targetPt
	ad.LastTargetSpeed = targetSpd

	return throttle, brake, steer, false, false, false
}
