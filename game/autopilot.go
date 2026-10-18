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
	State           AutopilotState
	BerserkMode     bool    // true if Berserk mode is active (player bus fast straightaways)
	BerserkMaxSpeed float64 // Configurable Berserk mode cruise speed in px/s (default: 445.0 / 80 km/h)
	DwellTimer      float64
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
		BerserkMaxSpeed:    apBerserkCruiseSpeed,
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

// GetBerserkCruiseSpeed returns the configured top cruise speed for Berserk mode in px/s
func (ad *AutopilotDriver) GetBerserkCruiseSpeed() float64 {
	if ad != nil && ad.BerserkMaxSpeed > 50.0 {
		return ad.BerserkMaxSpeed
	}
	return apBerserkCruiseSpeed
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
			return
		}
		if from != nil && to != nil && ad.pathWorld == w && ad.pathFromID == from.ID &&
			ad.pathToID == to.ID && ad.pathLane == lw {
			return
		}
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
		if ad.GetBerserkCruiseSpeed() >= 550.0 {
			decel = 165.0
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

// getCurrentRoadInfo detects the road segment the vehicle is currently on and whether it is a multi-lane or 6-lane avenue
func getCurrentRoadInfo(w *World, pos Vec2, heading float64, lw float64) (roadCenter float64, isEW bool, isMultiLane bool, is6Lane bool) {
	if w == nil {
		return 0, true, false, false
	}
	for _, rb := range w.Roundabouts {
		if pos.Distance(rb.Center) <= rb.Radius+15.0 {
			return 0, false, false, false
		}
	}
	cosH := math.Abs(math.Cos(heading))
	sinH := math.Abs(math.Sin(heading))
	busEW := cosH > sinH

	for pass := 0; pass < 2; pass++ {
		targetIsEW := busEW
		if pass == 1 {
			targetIsEW = !busEW
		}
		for _, r := range w.Roads {
			roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
			if roadIsEW != targetIsEW {
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
	}
	return 0, busEW, false, false
}

func (ad *AutopilotDriver) progressLaneOffset(dt, speedAbs float64) {
	targetDiff := math.Abs(ad.TargetLaneOffset - ad.lastTargetLaneOffset)
	if targetDiff > 0.0 {
		// Only restart the transition curve if the target changed significantly (> 2.0px)
		// or if we were already done with the previous transition (LaneOffsetProgress >= 1.0).
		// Continuous dynamic adjustments (such as tracking a tapering lane into an intersection)
		// must preserve transition progress so the vehicle does not stall in its lane!
		if targetDiff > 2.0 || ad.LaneOffsetProgress >= 1.0 {
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
			if ad.TargetLaneOffset < ad.LaneOffsetStart && ad.TargetLaneOffset <= -80.0 {
				// Shifting left towards innermost left-turn lane on 6-lane road (right next to double yellow line):
				// Must transition smoothly with generous duration so yaw angle stays small
				latSpeed = Clamp(speedAbs*0.22, 35.0, 60.0)
				minDuration = 0.95
				maxDuration = 1.45
			}
			ad.LaneOffsetDuration = Clamp(totalDist/latSpeed, minDuration, maxDuration)
		} else {
			ad.lastTargetLaneOffset = ad.TargetLaneOffset
		}
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

// evaluate6LaneTraffic evaluates traffic across all 3 lanes of a 6-lane road in the driving direction:
// 1. Leftmost lane (offset -2*lw): passing / fast lane
// 2. Middle lane (offset -lw): center through lane
// 3. Rightmost lane (offset 0.0): outer through / slow lane
func evaluate6LaneTraffic(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, laneWidth, currentLaneOffset float64, berserkCruiseSpeedOptional ...float64) (leftScore, midScore, rightScore float64, leftBlocked, midBlocked, rightBlocked bool) {
	berserkCruise := apBerserkCruiseSpeed
	if len(berserkCruiseSpeedOptional) > 0 && berserkCruiseSpeedOptional[0] > 50.0 {
		berserkCruise = berserkCruiseSpeedOptional[0]
	}
	corridor := bus.Width/2 + 10.0

	// Dynamic lookahead horizon based on cruise speed
	scanHorizon := math.Max(600.0, berserkCruise*1.6+100.0)

	for _, obs := range obstacles {
		delta := obs.Pos.Sub(bus.Pos)
		fwd := delta.X*cosH + delta.Y*sinH
		lat := -delta.X*sinH + delta.Y*cosH
		latPath := lat + currentLaneOffset
		corridorWithObs := corridor + obs.Width/2

		closingSpeed := math.Max(0.0, berserkCruise-obs.Speed)
		dynAdvance := closingSpeed*2.2 + (bus.Length/2 + obs.Length/2 + 50.0)
		blockedLookahead := math.Max(380.0, dynAdvance)

		// 1. Right lane (offset 0.0)
		distRight := math.Abs(latPath - 0.0)
		if distRight < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				rightBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0 // Stopped at red light or in traffic queue
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5 // Significantly slower than cruise
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				rightScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					rightBlocked = true
				}
			}
		}

		// 2. Middle lane (offset -laneWidth)
		distMid := math.Abs(latPath - (-laneWidth))
		if distMid < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				midBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				midScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					midBlocked = true
				}
			}
		}

		// 3. Leftmost lane (offset -2.0*laneWidth)
		distLeft := math.Abs(latPath - (-2.0*laneWidth))
		if distLeft < corridorWithObs {
			if fwd > -(bus.Length/2+obs.Length/2+25.0) && fwd < (bus.Length/2+obs.Length/2+35.0) {
				leftBlocked = true
			}
			if fwd > -30.0 && fwd < scanHorizon {
				w := 1.0
				if obs.Speed < 25.0 {
					w = 4.0
				} else if obs.Speed < berserkCruise*0.85 {
					w = 2.5
				}
				distWeight := math.Max(0.0, (scanHorizon-fwd)/scanHorizon)
				leftScore += w * (1.0 + distWeight)
				if (obs.Speed < berserkCruise*0.85 || obs.Speed < 25.0) && fwd < blockedLookahead {
					leftBlocked = true
				}
			}
		}
	}
	return
}

// evaluate6LaneThroughTraffic analyzes vehicles ahead in the two through lanes of a 6-lane road
// for backward compatibility with existing 2-lane intersection approach callers.
func evaluate6LaneThroughTraffic(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, laneWidth, currentLaneOffset float64, berserkCruiseSpeedOptional ...float64) (innerScore, outerScore float64, innerBlocked, outerBlocked bool) {
	_, midScore, rightScore, _, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, currentLaneOffset, berserkCruiseSpeedOptional...)
	return midScore, rightScore, midBlocked, rightBlocked
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
			closingAhead := math.Max(minSafeAhead, (bus.Speed-other.Speed)*1.6+65.0)
			if fwdOther > minSafeBehind && fwdOther < closingAhead {
				return false
			}
			if fwdOther <= minSafeBehind && fwdOther > -280.0 && other.Speed > bus.Speed+12.0 {
				return false
			}
		}

		// Also check intermediate lane if this is a multi-lane change
		if math.Abs(targetOffset-currentOffset) > laneWidth*1.3 {
			midOffset := (currentOffset + targetOffset) * 0.5
			distToMid := math.Abs(latPathOther - midOffset)
			if distToMid < corridor {
				alongsideBehind := -(bus.Length/2 + other.Length/2 + 25.0)
				alongsideAhead := (bus.Length/2 + other.Length/2 + 35.0)
				if fwdOther > alongsideBehind && fwdOther < alongsideAhead {
					return false
				}
			}
		}
	}
	return true
}

// isOncomingLaneClear checks whether the oncoming lane on a 2-way 2-lane road
// (typically at targetOffset = -laneWidth) is completely free of oncoming traffic
// and any obstacles within safe sight distance before borrowing the lane to overtake.
func isOncomingLaneClear(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset, targetOffset, laneWidth, speedAbs float64) bool {
	corridorBase := bus.Width/2 + 12.0
	for _, obs := range obstacles {
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToTarget := math.Abs(latPath - targetOffset)
		corridor := corridorBase + obs.Width/2
		if distToTarget < corridor {
			cosDiff := math.Cos(obs.Heading - bus.Heading)
			// 1. Vehicle is alongside the bus in the target oncoming lane
			sideDist := bus.Length/2 + obs.Length/2 + 40.0
			if fwd > -sideDist && fwd < sideDist {
				return false
			}

			// 2. Oncoming vehicle heading towards the bus (closing speed = bus.Speed + obs.Speed)
			if cosDiff < -0.2 {
				// Must have ample clear sight distance ahead
				reqClearDist := math.Max(700.0, (speedAbs+200.0)*2.4)
				if fwd >= -40.0 && fwd < reqClearDist {
					return false
				}
			} else {
				// 3. Same-direction or stopped vehicle occupying the target lane ahead
				sameDirClearDist := math.Max(420.0, speedAbs*1.8+90.0)
				if fwd >= -40.0 && fwd < sameDirClearDist {
					return false
				}
				// 4. Same-direction vehicle approaching fast from behind
				if fwd < -sideDist && fwd > -260.0 && obs.Speed > speedAbs+12.0 {
					return false
				}
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

// checkStationApproachLeadVehicle checks whether another vehicle (e.g. lead AI bus) is currently
// ahead in the curb lane, alongside in the adjacent lane, or occupying the destination stop bay.
func checkStationApproachLeadVehicle(
	bus *Bus,
	obstacles []VehicleObstacle,
	cosH, sinH float64,
	currentOffset float64,
	drivePath []Vec2,
	waypointIdx int,
	toStop *BusStop,
) (hasLeadAhead bool, hasLeadAlongside bool, hasPassedLead bool, leadObs *VehicleObstacle) {
	for i := range obstacles {
		obs := &obstacles[i]
		d := obs.Pos.Sub(bus.Pos)
		fwd := d.X*cosH + d.Y*sinH
		lat := -d.X*sinH + d.Y*cosH
		latPath := lat + currentOffset
		distToBaseLane := math.Abs(latPath - 0.0)

		// Check distance to base drive path or destination bay
		if len(drivePath) >= 2 {
			dPath := minDistanceToPath(obs.Pos, drivePath, waypointIdx)
			if dPath < distToBaseLane {
				distToBaseLane = dPath
			}
			if len(drivePath) > 5 {
				endWP := len(drivePath) - 3
				dEnd := minDistanceToPath(obs.Pos, drivePath, endWP)
				if dEnd < distToBaseLane {
					distToBaseLane = dEnd
				}
			}
		}
		if toStop != nil && math.Cos(obs.Heading-toStop.Heading) > 0.4 {
			dBay := obs.Pos.Distance(toStop.Pos)
			if dBay < (toStop.Length*0.6 + obs.Length*0.5 + 20.0) {
				// Obstacle is inside or at the entrance of the destination stop bay
				distToBaseLane = math.Min(distToBaseLane, 5.0)
			}
		}

		corridor := bus.Width/2 + obs.Width/2 + 16.0
		if distToBaseLane < corridor {
			safeClearBehind := -(bus.Length/2 + obs.Length/2 + 25.0) // ~ -73px
			safeClearAhead := (bus.Length/2 + obs.Length/2 + 25.0)   // ~ +73px

			if fwd > safeClearAhead {
				hasLeadAhead = true
				leadObs = obs
			} else if fwd >= safeClearBehind && fwd <= safeClearAhead {
				hasLeadAlongside = true
				leadObs = obs
			} else if fwd < safeClearBehind && fwd > -250.0 {
				hasPassedLead = true
			}
		}
	}
	return
}

// isBaseLaneClearToReturn verifies whether the base lane (offset 0.0) is completely clear of any
// vehicles alongside, closely behind, or approaching fast from behind before initiating a return maneuver.
// If toStop is provided, it also verifies that no vehicle is occupying the destination stop bay or
// blocking the approach in the base lane ahead of our vehicle.
func isBaseLaneClearToReturn(bus *Bus, obstacles []VehicleObstacle, cosH, sinH, currentOffset float64, drivePath []Vec2, waypointIdx int, toStop *BusStop) bool {
	corridorBase := bus.Width/2 + 10.0
	for _, obs := range obstacles {
		if toStop != nil && math.Cos(obs.Heading-toStop.Heading) > 0.4 {
			dBay := obs.Pos.Distance(toStop.Pos)
			if dBay < (toStop.Length*0.6 + obs.Length*0.5 + 35.0) {
				return false
			}
		}

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
			if len(drivePath) > 5 {
				endWP := len(drivePath) - 3
				dEnd := minDistanceToPath(obs.Pos, drivePath, endWP)
				if dEnd < distToBaseLane {
					distToBaseLane = dEnd
				}
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
	targetStop *BusStop,
) {
	// If Berserk mode is disabled, not in driving state, or path invalid: smoothly return to base lane
	isDrivingState := ad.State == AutopilotStateCruising || ad.State == AutopilotStateApproaching
	if !ad.BerserkMode || !isDrivingState || world == nil || len(ad.drivePath) < 2 {
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// =========================================================================
	// ROUNDABOUT DUAL-LANE OVERTAKING (BERSERK MODE)
	// =========================================================================
	var currentRb *Roundabout
	for i := range world.Roundabouts {
		if bus.Pos.Distance(world.Roundabouts[i].Center) <= world.Roundabouts[i].Radius+35.0 {
			currentRb = &world.Roundabouts[i]
			break
		}
	}

	if currentRb != nil {
		rbCenter := currentRb.Center
		rApron := currentRb.InnerRadius + 16.0
		rOuter := currentRb.Radius
		midR := (rApron + rOuter) * 0.5
		outerLaneR := (midR + rOuter) * 0.5
		innerLaneR := (rApron + midR) * 0.5
		passOffset := -(outerLaneR - innerLaneR)
		if math.Abs(passOffset) < laneWidth*0.5 {
			passOffset = -laneWidth
		}

		// 1. Check distance to route's exit from roundabout
		isApproachingExit := false
		for k := ad.WaypointIdx + 1; k < len(ad.drivePath); k++ {
			if ad.drivePath[k].Distance(rbCenter) > currentRb.Radius+15.0 {
				dToExit := ad.cumLen[k] - s
				if dToExit > 0 && dToExit < math.Max(180.0, speedAbs*1.5) {
					isApproachingExit = true
				}
				break
			}
		}

		if isApproachingExit {
			// Approaching exit: must return to base outer lane
			if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeReturning
			}
			if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
				ad.LaneOffset = 0.0
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
			ad.progressLaneOffset(dt, speedAbs)
			return
		}

		// 2. Roundabout overtaking state machine
		thetaBus := math.Atan2(bus.Pos.Y-rbCenter.Y, bus.Pos.X-rbCenter.X)

		switch ad.OvertakeState {
		case OvertakeNone:
			foundBlocker := false
			var nearestBlockerDist float64 = 9999.0
			for i := range obstacles {
				obs := &obstacles[i]
				dObs := obs.Pos.Distance(rbCenter)
				if dObs >= midR-12.0 && dObs <= rOuter+25.0 {
					thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
					dAngle := NormalizeAngle(thetaBus - thetaObs)
					if dAngle > 0.05 && dAngle < math.Pi*0.65 {
						arcDist := dAngle * outerLaneR
						if arcDist > 15.0 && arcDist < 260.0 {
							if obs.Speed < apRoundaboutSpeed*0.85 || obs.Speed < 25.0 {
								if arcDist < nearestBlockerDist {
									nearestBlockerDist = arcDist
									foundBlocker = true
								}
							}
						}
					}
				}
			}

			if foundBlocker {
				// Verify inner passing lane is clear
				innerLaneClear := true
				for i := range obstacles {
					obs := &obstacles[i]
					dObs := obs.Pos.Distance(rbCenter)
					if dObs < midR+10.0 && dObs >= currentRb.InnerRadius-10.0 {
						thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
						dAngle := NormalizeAngle(thetaBus - thetaObs)
						arcDist := dAngle * innerLaneR
						if arcDist > -60.0 && arcDist < 160.0 {
							innerLaneClear = false
							break
						}
					}
				}

				if innerLaneClear {
					ad.OvertakeState = OvertakeChangingLane
					ad.TargetLaneOffset = passOffset
					ad.OvertakeTimer = 0.0
				}
			}

		case OvertakeChangingLane:
			ad.OvertakeTimer += dt
			if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.5 {
				ad.LaneOffset = ad.TargetLaneOffset
				ad.OvertakeState = OvertakePassing
				ad.OvertakeTimer = 0.0
			}

		case OvertakePassing:
			ad.OvertakeTimer += dt
			hasOuterBlockerAhead := false
			for i := range obstacles {
				obs := &obstacles[i]
				dObs := obs.Pos.Distance(rbCenter)
				if dObs >= midR-12.0 && dObs <= rOuter+25.0 {
					thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
					dAngle := NormalizeAngle(thetaBus - thetaObs)
					arcDist := dAngle * outerLaneR
					if arcDist > -20.0 && arcDist < 140.0 {
						hasOuterBlockerAhead = true
						break
					}
				}
			}

			if !hasOuterBlockerAhead && ad.OvertakeTimer > 0.5 {
				outerClear := true
				for i := range obstacles {
					obs := &obstacles[i]
					dObs := obs.Pos.Distance(rbCenter)
					if dObs >= midR-12.0 && dObs <= rOuter+25.0 {
						thetaObs := math.Atan2(obs.Pos.Y-rbCenter.Y, obs.Pos.X-rbCenter.X)
						dAngle := NormalizeAngle(thetaBus - thetaObs)
						arcDist := dAngle * outerLaneR
						if arcDist > -50.0 && arcDist < 80.0 {
							outerClear = false
							break
						}
					}
				}
				if outerClear {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
					ad.OvertakeTimer = 0.0
				}
			}

		case OvertakeReturning:
			ad.OvertakeTimer += dt
			ad.TargetLaneOffset = 0.0
			if math.Abs(ad.LaneOffset) < 1.0 {
				ad.LaneOffset = 0.0
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
		}

		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
	is2Lane := !isMultiLane && !is6Lane
	if !isMultiLane && (!ad.BerserkMode || !is2Lane) {
		// Not a multi-lane road in current travel direction (e.g. 2-lane road with 1 lane each way)
		// In Standard Mode, overtaking is strictly prohibited on single-lane/2-lane roads
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeReturning
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Check if upcoming corner prevents overtaking, and whether the route turns left along this road
	isUpcomingCorner := false
	isUpcomingCornerLeft := false
	hasNextLeftTurn := false

	maxCornerDist := math.Max(450.0, speedAbs*1.2)
	if is6Lane {
		maxCornerDist = math.Max(650.0, speedAbs*1.6)
	} else if isMultiLane {
		maxCornerDist = math.Max(500.0, speedAbs*1.3)
	}

	for k := ad.WaypointIdx + 1; k < len(ad.drivePath)-1; k++ {
		d := ad.cumLen[k] - s
		if d < -5.0 {
			continue
		}
		if d > 1800.0 {
			break
		}
		a := ad.drivePath[k].Sub(ad.drivePath[k-1])
		b := ad.drivePath[k+1].Sub(ad.drivePath[k])
		// A segment b continuing straight along the current road heading is not a corner turn
		// (e.g. straightening out from a bus stop bay exit or lane taper)
		angB := NormalizeAngle(math.Atan2(b.Y, b.X) - bus.Heading)
		if math.Abs(angB) < 0.25 {
			continue
		}
		turn := NormalizeAngle(math.Atan2(b.Y, b.X) - math.Atan2(a.Y, a.X))
		if (turn < -0.35 && turn > -2.80) || math.Abs(turn) >= 2.80 {
			if !hasNextLeftTurn {
				hasNextLeftTurn = true
			}
			if d <= maxCornerDist && !isUpcomingCorner {
				isUpcomingCorner = true
				isUpcomingCornerLeft = true
			}
			break
		} else if turn > 0.35 && turn < 2.80 {
			if d <= maxCornerDist && !isUpcomingCorner {
				isUpcomingCorner = true
			}
			break
		}
	}

	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	// =========================================================================
	// STATION APPROACH & BAY ENTRY CONTROLLER
	// =========================================================================
	// When approaching destination stop bay:
	// 1. If there is enough runway and we have cleanly passed the lead vehicle (or bay is clear),
	//    merge ahead in front of it into the bay.
	// 2. If there is NOT enough runway to overtake, wait behind in the curb lane,
	//    or wait alongside in the adjacent passing lane without colliding.
	stationApproachThreshold := math.Max(420.0, speedAbs*2.2+150.0)
	if remaining < stationApproachThreshold {
		hasLeadAhead, hasLeadAlongside, hasPassedLead, leadObs := checkStationApproachLeadVehicle(
			bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, targetStop,
		)

		// Case A: Bus is in the base curb lane (OvertakeNone)
		if ad.OvertakeState == OvertakeNone && math.Abs(ad.LaneOffset) < 1.0 {
			if hasLeadAhead || hasLeadAlongside {
				// Check whether there is sufficient runway to overtake cleanly in front before the stop:
				// 1. We must have sufficient remaining runway along the approach path
				// 2. The lead obstacle itself must not already be in/entering the station bay
				// 3. The adjacent passing lane must be physically clear to change into
				minOvertakeRunway := math.Max(460.0, speedAbs*2.2+160.0)
				leadDistToStop := 9999.0
				if targetStop != nil && leadObs != nil {
					leadDistToStop = leadObs.Pos.Distance(targetStop.Pos)
				}
				passClear := isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth)
				hasSpaceToOvertake := remaining >= minOvertakeRunway && leadDistToStop >= 280.0 && passClear && (isMultiLane || is6Lane)

				if hasSpaceToOvertake {
					// There IS space to overtake: initiate pass into adjacent lane!
					targetOffset := -laneWidth
					if is6Lane && ad.LaneOffset <= -laneWidth*0.5 {
						targetOffset = -2.0 * laneWidth
					}
					ad.TargetLaneOffset = targetOffset
					ad.OvertakeState = OvertakeChangingLane
					ad.OvertakeTimer = 0.0
				} else {
					// No space to overtake cleanly: hold base lane, queue/wait behind the lead vehicle!
					ad.LaneOffset = 0.0
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
				ad.progressLaneOffset(dt, speedAbs)
				return
			}
		}

		// Case B: Bus is in the adjacent/passing lane
		if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
			baseClear := isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, targetStop)
			isBayOccupied := false
			if targetStop != nil {
				for i := range obstacles {
					if math.Cos(obstacles[i].Heading-targetStop.Heading) > 0.4 &&
						obstacles[i].Pos.Distance(targetStop.Pos) < (targetStop.Length*0.6+obstacles[i].Length*0.5+35.0) {
						isBayOccupied = true
						break
					}
				}
			}

			if !isBayOccupied {
				// Destination stop bay is open: actively return to base curb lane to dock!
				if !hasLeadAlongside && baseClear {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				} else {
					targetWaitOffset := -laneWidth
					if is6Lane && ad.LaneOffset <= -laneWidth*1.5 {
						targetWaitOffset = -2.0 * laneWidth
					}
					ad.TargetLaneOffset = targetWaitOffset
					ad.OvertakeState = OvertakePassing
				}
			} else {
				// Destination bay is occupied by another vehicle:
				// If we have cleanly passed the lead vehicle and bay approach is clear, merge in front;
				// otherwise, wait alongside in adjacent lane.
				if hasPassedLead && remaining >= 90.0 && !hasLeadAlongside && !hasLeadAhead && baseClear {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				} else {
					targetWaitOffset := -laneWidth
					if is6Lane && ad.LaneOffset <= -laneWidth*1.5 {
						targetWaitOffset = -2.0 * laneWidth
					}
					ad.TargetLaneOffset = targetWaitOffset
					ad.OvertakeState = OvertakePassing
				}
			}
		}

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
			ad.LaneOffset = 0.0
			ad.TargetLaneOffset = 0.0
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
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

	// Approaching an actual road corner/turn (left or right):
	if isUpcomingCorner {
		if isUpcomingCornerLeft && (isMultiLane || is6Lane) {
			// Upcoming corner is a LEFT turn on a multi-lane road:
			if ad.LaneOffset < -laneWidth*0.3 || ad.OvertakeState != OvertakeNone {
				// Bus was overtaking on the left.
				// Do not return to the outer right lane; stay in the left lane which leads directly into the left-turn lane.
				targetLeftOffset := -(distBaseToCenter - laneWidth*0.5)
				if targetLeftOffset > 0.0 {
					targetLeftOffset = 0.0
				}
				if is6Lane {
					if targetLeftOffset < -laneWidth*2.0 {
						targetLeftOffset = -laneWidth * 2.0
					}
				} else {
					if targetLeftOffset < -laneWidth {
						targetLeftOffset = -laneWidth
					}
				}
				ad.TargetLaneOffset = targetLeftOffset
				if math.Abs(ad.LaneOffset-targetLeftOffset) < 1.5 || distBaseToCenter <= laneWidth*1.15 {
					ad.LaneOffset = targetLeftOffset
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
			} else {
				// Bus was not overtaking: keep offset 0.0, drivePath will naturally merge into left-turn lane
				ad.TargetLaneOffset = 0.0
				ad.LaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
		} else {
			// Right turn or 2-lane road: abort overtake and return to base outer lane
			if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			}
			if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
				ad.LaneOffset = 0.0
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
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

			maxInterDist := 450.0
			if is6Lane {
				maxInterDist = 900.0
			} else if isMultiLane {
				maxInterDist = 550.0
			}
			maxForwardDist := stopLineOffset + maxInterDist + 80.0
			maxLatDist := laneWidth * 3.6
			if is6Lane {
				maxLatDist = laneWidth * 4.8
			}

			if forwardDist > -stopLineOffset && forwardDist < maxForwardDist && lateralDist < maxLatDist {
				approachingInter = inter
				distToStopLine = forwardDist - stopLineOffset
				isApproachingIntersection = (distToStopLine > 0 && distToStopLine < maxInterDist)

				hasRouteExit := false
				exitLatThreshold := laneWidth * 1.6
				if is6Lane {
					exitLatThreshold = laneWidth * 2.8
				}
				for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
					pt := ad.drivePath[idx]
					toPt := pt.Sub(inter.Center)
					fwdPt := toPt.X*cosH + toPt.Y*sinH
					latPt := -toPt.X*sinH + toPt.Y*cosH

					// Only evaluate waypoints near or beyond intersection mouth;
					// approach waypoints before the intersection stop line must never be mistaken for exit turns
					if fwdPt < -stopLineOffset {
						continue
					}

					if fwdPt > 30.0 || math.Abs(latPt) > exitLatThreshold {
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
						if d > maxInterDist {
							break
						}
						segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
						segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
						angPrev := NormalizeAngle(math.Atan2(segPrev.Y, segPrev.X) - bus.Heading)
						angNext := NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - bus.Heading)
						if math.Abs(angPrev) < 0.45 && math.Abs(angNext) < 0.45 {
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
		if is2Lane {
			// On 2-lane roads, borrowing oncoming lane through or near intersections is strictly prohibited!
			if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			}
			if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset) < 1.0 {
				ad.LaneOffset = 0.0
				ad.TargetLaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
			}
			ad.progressLaneOffset(dt, speedAbs)
			return
		}

		isStraight := !isUpcomingLeft && !isUpcomingRight
		hasDedicatedLeft := approachingInter.ApproachHasDedicatedLanesForHeading(bus.Heading)

		if isUpcomingRight {
			// Right turn must be from outer lane (0.0)
			if ad.TargetLaneOffset != 0.0 {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
				}
			}
		} else if isUpcomingLeft {
			// Left turn: check whether we are currently overtaking on the left or have LaneOffset
			if ad.LaneOffset < -laneWidth*0.3 || ad.OvertakeState != OvertakeNone {
				// Bus was overtaking on the left.
				// The left passing lane aligns directly with the dedicated left-turn lane!
				// DO NOT return to the outer right lane; seamlessly transition into the left-turn lane.
				targetLeftOffset := -(distBaseToCenter - laneWidth*0.5)
				if targetLeftOffset > 0.0 {
					targetLeftOffset = 0.0
				}
				if is6Lane {
					if targetLeftOffset < -laneWidth*2.0 {
						targetLeftOffset = -laneWidth * 2.0
					}
				} else {
					if targetLeftOffset < -laneWidth {
						targetLeftOffset = -laneWidth
					}
				}

				if math.Abs(ad.LaneOffset-targetLeftOffset) < 1.5 || distBaseToCenter <= laneWidth*1.15 {
					// In left-turn corridor: synchronize LaneOffset with targetLeftOffset
					// as drivePath tapers into left-turn lane, so physical lateral position stays rock-solid
					ad.TargetLaneOffset = targetLeftOffset
					ad.LaneOffset = targetLeftOffset
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				} else {
					ad.TargetLaneOffset = targetLeftOffset
					if ad.OvertakeState == OvertakeChangingLane && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
						ad.LaneOffset = ad.TargetLaneOffset
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					}
				}
			} else {
				// Not overtaking: drivePath naturally leads into the dedicated left-turn lane
				ad.TargetLaneOffset = 0.0
				ad.LaneOffset = 0.0
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
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
						innerScore, outerScore, innerBlocked, outerBlocked := evaluate6LaneThroughTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
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
					} else if ad.LaneOffset < -laneWidth*1.5 {
						// In innermost left-turn lane, must exit to through lanes (middle or outer) before stop line
						altOffset := 0.0
						if desiredOffset == 0.0 {
							altOffset = -laneWidth
						}
						if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, altOffset, laneWidth) {
							ad.TargetLaneOffset = altOffset
							ad.OvertakeState = OvertakeChangingLane
							ad.OvertakeTimer = 0.0
						} else {
							ad.TargetLaneOffset = -laneWidth
							ad.OvertakeState = OvertakeReturning
							ad.OvertakeTimer = 0.0
						}
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
					if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
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

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
			ad.OvertakeState = OvertakeNone
		}
		ad.progressLaneOffset(dt, speedAbs)
		return
	}

	// Mid-block overtaking
	var passOffset float64
	if is2Lane {
		passOffset = -laneWidth // On 2-lane road: borrow oncoming lane to the left in RHT
	} else if distBaseToCenter > laneWidth*0.95 {
		passOffset = -laneWidth // Inner passing lane (left side in RHT)
	} else {
		passOffset = laneWidth // Outer lane (right side)
	}

	// =========================================================================
	// 6-LANE ROAD STRAIGHT-THROUGH MID-BLOCK TRAFFIC SELECTION
	// =========================================================================
	minOvertakeRunway := math.Max(450.0, speedAbs*2.4+160.0)
	is6LaneStraight := is6Lane && !isUpcomingCorner && !isUpcomingLeft && !isUpcomingCornerLeft && !hasNextLeftTurn && !isApproachingIntersection && remaining >= minOvertakeRunway
	if is6LaneStraight {
		leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())

		var desiredOffset float64
		if ad.BerserkMode {
			// In Berserk Mode on a 3-lane-per-direction road (Right: 0.0, Mid: -lw, Left: -2*lw):
			if rightBlocked && midBlocked {
				// Both right two lanes have cars/blockers:
				if !leftBlocked {
					desiredOffset = -2.0 * laneWidth // Overtake via the clear leftmost lane!
				} else {
					// All 3 lanes blocked, stay in current lane
					if ad.LaneOffset <= -laneWidth*1.5 {
						desiredOffset = -2.0 * laneWidth
					} else if ad.LaneOffset <= -laneWidth*0.5 {
						desiredOffset = -laneWidth
					} else {
						desiredOffset = 0.0
					}
				}
			} else if rightBlocked && !midBlocked {
				// Right lane blocked, middle lane open
				if !leftBlocked && leftScore < midScore-0.5 {
					desiredOffset = -2.0 * laneWidth
				} else {
					desiredOffset = -laneWidth
				}
			} else if midBlocked && !rightBlocked {
				// Middle lane blocked
				if !leftBlocked {
					desiredOffset = -2.0 * laneWidth // Pass on the left
				} else {
					desiredOffset = 0.0 // Pass on the right
				}
			} else if leftBlocked && midBlocked && !rightBlocked {
				desiredOffset = 0.0
			} else if !rightBlocked && !midBlocked {
				// Both right lanes clear: if in leftmost lane, return to middle lane once safe
				if ad.LaneOffset <= -laneWidth*1.5 {
					desiredOffset = -laneWidth
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
			} else {
				if ad.LaneOffset <= -laneWidth*1.5 {
					desiredOffset = -2.0 * laneWidth
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
			}
		} else {
			// Standard mode: middle through lane (-lw) or outer lane (0.0)
			if midBlocked && !rightBlocked {
				desiredOffset = 0.0
			} else if rightBlocked && !midBlocked {
				desiredOffset = -laneWidth
			} else if rightScore > midScore+0.6 {
				desiredOffset = -laneWidth
			} else if midScore > rightScore+0.6 {
				desiredOffset = 0.0
			} else {
				if ad.LaneOffset < -laneWidth*0.5 {
					desiredOffset = -laneWidth
				} else {
					desiredOffset = 0.0
				}
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
		// Don't initiate overtaking if approaching destination stop without sufficient runway
		// to complete the pass and cleanly pull into the bay in front
		minOvertakeRunway := math.Max(450.0, speedAbs*2.4+160.0)
		if remaining < minOvertakeRunway {
			break
		}

		// On 2-lane roads (borrowing oncoming lane):
		// Must ONLY initiate borrowing after the turn has fully completed, vehicle has settled
		// in the travel lane, and heading/steering are straight. Never borrow lane while turning
		// or right at the corner exit, which would swing into the opposite shoulder!
		if is2Lane {
			roadHeading := 0.0
			if isEW {
				if math.Cos(bus.Heading) < 0 {
					roadHeading = math.Pi
				}
			} else {
				if math.Sin(bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
				} else {
					roadHeading = -math.Pi / 2.0
				}
			}
			headingErr := math.Abs(NormalizeAngle(bus.Heading - roadHeading))
			if headingErr > 0.10 { // > ~5.7 deg: still turning or settling!
				break
			}

			if math.Abs(bus.SteeringAngle) > 0.06 {
				break
			}

			if math.Abs(ad.LastLateralErr) > 6.0 {
				break
			}

			hasCurvatureNearby := false
			for k := 1; k < len(ad.drivePath)-1; k++ {
				d := ad.cumLen[k] - s
				if d < -180.0 {
					continue
				}
				if d > 180.0 {
					break
				}
				segPrev := ad.drivePath[k].Sub(ad.drivePath[k-1])
				segNext := ad.drivePath[k+1].Sub(ad.drivePath[k])
				turnAngle := math.Abs(NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - math.Atan2(segPrev.Y, segPrev.X)))
				if turnAngle > 0.18 {
					hasCurvatureNearby = true
					break
				}
			}
			if hasCurvatureNearby {
				break
			}

			if world != nil {
				interTurnClearance := math.Max(260.0, laneWidth*4.2+55.0)
				inIntersection := false
				for _, ipt := range world.Intersections {
					if bus.Pos.Distance(ipt) < interTurnClearance {
						inIntersection = true
						break
					}
				}
				if inIntersection {
					break
				}
			}
		}

		// Dynamic lookahead (提前量) for slow/stopped blocker detection:
		// Ensures slow lead vehicles are detected with ample margin so the lane change
		// initiates well before the bus enters the braking distance, preventing collisions
		// before completing the lateral transition.
		fwdMax := math.Max(480.0, speedAbs*2.4+160.0)

		var blockerDist float64 = 9999.0
		foundBlocker := false

		for i := range obstacles {
			obs := &obstacles[i]
			if targetStop != nil && obs.Pos.Distance(targetStop.Pos) < 260.0 {
				continue // Vehicle already at/entering destination stop bay
			}
			delta := obs.Pos.Sub(bus.Pos)
			fwd := delta.X*cosH + delta.Y*sinH
			lat := math.Abs(-delta.X*sinH + delta.Y*cosH)
			if fwd > (bus.Length/2+obs.Length/2+12.0) && fwd < fwdMax && lat < (bus.Width/2+obs.Width/2+10.0) {
				if math.Cos(obs.Heading-bus.Heading) > 0.4 {
					cruise := ad.GetBerserkCruiseSpeed()
					if !ad.BerserkMode {
						cruise = apCruiseSpeed
					}
					if obs.Speed < cruise*0.85 || obs.Speed < 20.0 {
						closingSpeed := math.Max(0.0, speedAbs-obs.Speed)
						// Dynamic lookahead: lateral transition time (~0.85-1.2s) * closing speed + cushion
						reqAdvance := closingSpeed*2.2 + (bus.Length/2 + obs.Length/2 + 80.0)
						if fwd < math.Max(reqAdvance, 400.0) {
							if fwd < blockerDist {
								blockerDist = fwd
								foundBlocker = true
							}
						}
					}
				}
			}
		}

		if foundBlocker {
			targetPassOffset := passOffset
			if (is6LaneStraight || (is6Lane && hasNextLeftTurn)) && ad.BerserkMode {
				leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				_ = rightScore
				_ = midScore
				_ = leftScore

				if ad.LaneOffset > -laneWidth*0.5 {
					// Currently in outer lane (0.0):
					// Right lane is blocked (foundBlocker).
					if (midBlocked || midScore > 0.5) && !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth // Both right lanes have cars, left lane is clear!
					} else if !midBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
						targetPassOffset = -laneWidth
					} else if !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth
					} else {
						targetPassOffset = -laneWidth
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 && ad.LaneOffset > -laneWidth*1.5 {
					// Currently in middle lane (-laneWidth):
					// Middle lane is blocked (foundBlocker).
					if !leftBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth // Pass on clear left lane!
					} else if !rightBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
						targetPassOffset = 0.0
					} else if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -2.0*laneWidth, laneWidth) {
						targetPassOffset = -2.0 * laneWidth
					} else {
						targetPassOffset = 0.0
					}
				} else {
					// Currently in leftmost lane (-2.0*laneWidth):
					if !midBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
						targetPassOffset = -laneWidth
					} else if !rightBlocked && isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
						targetPassOffset = 0.0
					} else {
						targetPassOffset = -laneWidth
					}
				}
			} else if is6LaneStraight {
				if ad.LaneOffset < -laneWidth*0.5 {
					targetPassOffset = 0.0
				} else {
					targetPassOffset = -laneWidth
				}
			}

			// Verify passing lane is safe and clear to change into
			clearToChange := false
			if is2Lane {
				clearToChange = isOncomingLaneClear(bus, obstacles, cosH, sinH, ad.LaneOffset, targetPassOffset, laneWidth, speedAbs)
			} else {
				clearToChange = isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, targetPassOffset, laneWidth)
			}

			if clearToChange {
				ad.OvertakeState = OvertakeChangingLane
				ad.TargetLaneOffset = targetPassOffset
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeChangingLane:
		ad.OvertakeTimer += dt

		// Active oncoming threat check for 2-lane roads:
		// If an oncoming car approaches while changing into oncoming lane, abort immediately!
		if is2Lane {
			if !isOncomingLaneClear(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.TargetLaneOffset, laneWidth, speedAbs) {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
					ad.OvertakeTimer = 0.0
					break
				}
			}
		}

		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
			if is6LaneStraight {
				// Arrived in selected straight through / passing lane on 6-lane road
				_, midScore, rightScore, _, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				if ad.TargetLaneOffset <= -laneWidth*1.5 {
					// Arrived in leftmost lane (-2.0*laneWidth)
					if midBlocked || rightBlocked || midScore > 0 || rightScore > 0 {
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					}
				} else if ad.TargetLaneOffset <= -laneWidth*0.5 {
					// Arrived in middle lane (-laneWidth)
					if rightBlocked || rightScore >= midScore {
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
					} else {
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
					}
				} else {
					// Arrived in outer lane (0.0)
					ad.OvertakeState = OvertakeNone
					ad.OvertakeTimer = 0.0
				}
			} else {
				ad.OvertakeState = OvertakePassing
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakePassing:
		ad.OvertakeTimer += dt

		// Active oncoming threat check for 2-lane roads
		if is2Lane {
			hasImminentOncoming := false
			for _, obs := range obstacles {
				d := obs.Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				lat := -d.X*sinH + d.Y*cosH
				latPath := lat + ad.LaneOffset
				distToOncoming := math.Abs(latPath - (-laneWidth))
				corridor := bus.Width/2 + obs.Width/2 + 14.0
				if distToOncoming < corridor {
					cosDiff := math.Cos(obs.Heading - bus.Heading)
					if cosDiff < -0.2 && fwd > 0 && fwd < math.Max(600.0, speedAbs*2.0+120.0) {
						hasImminentOncoming = true
						break
					}
				}
			}
			if hasImminentOncoming {
				if isBaseLaneClearToReturn(bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, nil) {
					ad.TargetLaneOffset = 0.0
					ad.OvertakeState = OvertakeReturning
					ad.OvertakeTimer = 0.0
					break
				}
			}
		}

		// Check if we have passed all obstacles in the lanes being overtaken
		hasObstacleNotCleared := false
		for _, obs := range obstacles {
			d := obs.Pos.Sub(bus.Pos)
			fwd := d.X*cosH + d.Y*sinH
			lat := -d.X*sinH + d.Y*cosH
			latPath := lat + ad.LaneOffset

			isOvertakingObs := false
			corridor := bus.Width/2 + obs.Width/2 + 10.0

			if is6LaneStraight {
				if ad.LaneOffset <= -laneWidth*1.5 {
					// In leftmost lane: overtaking middle lane (-lw) and outer lane (0.0)
					if math.Abs(latPath-(-laneWidth)) < corridor || math.Abs(latPath-0.0) < corridor {
						isOvertakingObs = true
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					// In middle lane: overtaking outer lane (0.0)
					if math.Abs(latPath-0.0) < corridor {
						isOvertakingObs = true
					}
				} else {
					// In outer lane: overtaking middle lane (-lw)
					if math.Abs(latPath-(-laneWidth)) < corridor {
						isOvertakingObs = true
					}
				}
			} else {
				distToBaseLane := math.Abs(latPath - 0.0)
				if len(ad.drivePath) >= 2 {
					dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
					if dPath < distToBaseLane {
						distToBaseLane = dPath
					}
				}
				if distToBaseLane < corridor {
					isOvertakingObs = true
				}
			}

			if isOvertakingObs {
				minSafeBehind := -(bus.Length/2 + obs.Length/2 + 75.0) // ~ -141px
				// 1. Vehicle is alongside or closely behind us: not safe to cut back yet!
				if fwd > minSafeBehind && fwd < 150.0 {
					hasObstacleNotCleared = true
					break
				}
				// 2. Vehicle is ahead of us in the lane being overtaken and is slow: stay in passing lane!
				cruise := ad.GetBerserkCruiseSpeed()
				if !ad.BerserkMode {
					cruise = apCruiseSpeed
				}
				aheadBuffer := math.Max(480.0, speedAbs*2.0+100.0)
				if targetStop != nil && remaining < 650.0 {
					// Approaching station: prioritize returning to base lane to dock over continuing to pass distant vehicles
					aheadBuffer = math.Max(80.0, remaining-100.0)
				}
				if fwd >= 150.0 && fwd < aheadBuffer && (obs.Speed < cruise*0.88 || obs.Speed < 25.0) {
					hasObstacleNotCleared = true
					break
				}
			}
		}

		shouldReturn := false
		if !hasObstacleNotCleared && ad.OvertakeTimer > 0.6 {
			if isUpcomingLeft || isUpcomingCornerLeft || hasNextLeftTurn {
				targetLeftOffset := -(distBaseToCenter - laneWidth*0.5)
				if targetLeftOffset > 0.0 {
					targetLeftOffset = 0.0
				}
				if is6Lane {
					if targetLeftOffset < -laneWidth*2.0 {
						targetLeftOffset = -laneWidth * 2.0
					}
				} else {
					if targetLeftOffset < -laneWidth {
						targetLeftOffset = -laneWidth
					}
				}
				ad.TargetLaneOffset = targetLeftOffset
				ad.OvertakeState = OvertakeNone
				ad.OvertakeTimer = 0.0
				break
			}
			if is6LaneStraight {
				leftScore, midScore, rightScore, leftBlocked, midBlocked, rightBlocked := evaluate6LaneTraffic(bus, obstacles, cosH, sinH, laneWidth, ad.LaneOffset, ad.GetBerserkCruiseSpeed())
				_ = leftScore
				_ = leftBlocked
				if ad.LaneOffset <= -laneWidth*1.5 {
					// Currently in leftmost lane (-2*lw)
					if midBlocked {
						if rightBlocked {
							// Both right lanes still blocked; stay in leftmost lane!
							ad.TargetLaneOffset = -2.0 * laneWidth
							ad.OvertakeState = OvertakeNone
							ad.OvertakeTimer = 0.0
							break
						} else {
							// Middle lane blocked, but outer lane clear: return to outer lane (0.0)
							if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, 0.0, laneWidth) {
								ad.TargetLaneOffset = 0.0
								ad.OvertakeState = OvertakeReturning
								ad.OvertakeTimer = 0.0
								break
							}
						}
					} else {
						// Middle lane is clear! Return to middle lane (-laneWidth)
						if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, -laneWidth, laneWidth) {
							ad.TargetLaneOffset = -laneWidth
							ad.OvertakeState = OvertakeReturning
							ad.OvertakeTimer = 0.0
							break
						}
					}
				} else if ad.LaneOffset <= -laneWidth*0.5 {
					// Currently in middle lane (-laneWidth)
					if rightBlocked || rightScore >= midScore {
						// Stay in middle through lane!
						ad.TargetLaneOffset = -laneWidth
						ad.OvertakeState = OvertakeNone
						ad.OvertakeTimer = 0.0
						break
					}
				}
			}
			shouldReturn = true
		}

		if shouldReturn {
			targetRetOffset := 0.0
			if is6LaneStraight && ad.LaneOffset <= -laneWidth*1.5 {
				targetRetOffset = -laneWidth
			}
			if isLaneClearToChange(bus, obstacles, cosH, sinH, ad.LaneOffset, targetRetOffset, laneWidth) {
				ad.OvertakeState = OvertakeReturning
				ad.TargetLaneOffset = targetRetOffset
				ad.OvertakeTimer = 0.0
			}
		}

	case OvertakeReturning:
		ad.OvertakeTimer += dt

		if isUpcomingLeft || isUpcomingCornerLeft {
			targetLeftOffset := -(distBaseToCenter - laneWidth*0.5)
			if targetLeftOffset > 0.0 {
				targetLeftOffset = 0.0
			}
			if is6Lane {
				if targetLeftOffset < -laneWidth*2.0 {
					targetLeftOffset = -laneWidth * 2.0
				}
			} else {
				if targetLeftOffset < -laneWidth {
					targetLeftOffset = -laneWidth
				}
			}
			ad.TargetLaneOffset = targetLeftOffset
			ad.OvertakeState = OvertakeNone
			ad.OvertakeTimer = 0.0
			break
		}

		// Active collision avoidance during return maneuver:
		// If return lane suddenly becomes obstructed while we are still transitioning, abort return!
		if math.Abs(ad.LaneOffset-ad.TargetLaneOffset) > laneWidth*0.15 {
			for _, obs := range obstacles {
				d := obs.Pos.Sub(bus.Pos)
				fwd := d.X*cosH + d.Y*sinH
				lat := -d.X*sinH + d.Y*cosH
				latPath := lat + ad.LaneOffset
				distToRetLane := math.Abs(latPath - ad.TargetLaneOffset)
				if ad.TargetLaneOffset == 0.0 && len(ad.drivePath) >= 2 {
					dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
					if dPath < distToRetLane {
						distToRetLane = dPath
					}
				}
				if distToRetLane < (bus.Width/2 + obs.Width/2 + 10.0) {
					dangerBehind := -(bus.Length/2 + obs.Length/2 + 45.0)
					dangerAhead := (bus.Length/2 + obs.Length/2 + 55.0)
					if fwd > dangerBehind && fwd < dangerAhead {
						// Imminent collision! Abort return and swing back into passing lane
						if is6LaneStraight && ad.TargetLaneOffset <= -laneWidth*0.5 {
							ad.TargetLaneOffset = -2.0 * laneWidth
						} else {
							ad.TargetLaneOffset = passOffset
						}
						ad.OvertakeState = OvertakePassing
						ad.OvertakeTimer = 0.0
						break
					}
				}
			}
		}

		if ad.OvertakeState == OvertakeReturning && math.Abs(ad.LaneOffset-ad.TargetLaneOffset) < 1.0 {
			ad.LaneOffset = ad.TargetLaneOffset
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
			// Phase 1: Open doors if vehicle has slowed to crawl (< 15 px/s) and doors not yet opened
			openThreshold := 15.0
			if !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
				if speedAbs < openThreshold {
					toggleDoors = true
					return throttle, brake, steer, handbrake, toggleDoors, isDocked
				}
				// Still braking to crawl speed; hold full brake and wait
				return throttle, brake, steer, handbrake, false, isDocked
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

		// Phase 4: Wait for dwell to finish AND doors to finish closing completely, then advance and depart
		if ad.DwellFinished && !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
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
	ad.updateOvertaking(bus, world, tls, obstacles, laneWidth, ad.LastPathS, speedAbs, remDist, dt, targetStop)

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
			effOffset := ad.LaneOffset
			if world != nil {
				wpt := ad.drivePath[wIdx]
				roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, wpt, bus.Heading, laneWidth)
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
			posForTracking = bus.Pos.Sub(segRight.Mul(effOffset))
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
	alignedNearBay := distToStop < 55.0 && math.Abs(NormalizeAngle(bus.Heading-targetStop.Heading)) < 0.45
	// Fallback: the bus has come to rest at the very end of its path right at the stop
	restingAtStop := remaining < 15.0 && distToStop < 70.0
	maxDockSpeed := 10.0
	if ad.BerserkMode {
		maxDockSpeed = 26.0 // Snap into docked state smoothly as bus enters the bay
	}
	inBaseLane := math.Abs(ad.LaneOffset) < 10.0
	if ad.State != AutopilotStateDeparting && speedAbs < maxDockSpeed &&
		(targetStop.ContainsBus(bus) || (inBaseLane && (alignedNearBay || restingAtStop))) {
		ad.State = AutopilotStateDocked
		ad.DockedStopID = targetStop.ID
		ad.DwellTimer = ad.DwellTotal
		if ad.BerserkMode {
			ad.DwellTimer = 1.0 // 1.0s dwell in Berserk mode: fast, responsive, yet reliably boards passengers
		}
		ad.DwellFinished = false
		if speedAbs < 15.0 && !bus.DoorsOpen && bus.DoorProgress <= 0.05 {
			toggleDoors = true
		}
		return 0.0, 1.0, 0.0, false, toggleDoors, true
	}

	// If doors are open outside docked state (e.g. player parked then toggled autopilot),
	// command doors to close immediately so vehicle can start moving!
	if bus.DoorsOpen || bus.DoorProgress > 0.05 {
		closeCmd := false
		if bus.DoorsOpen && speedAbs < 15.0 {
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
	var nearRb *Roundabout
	if world != nil {
		for i := range world.Roundabouts {
			if bus.Pos.Distance(world.Roundabouts[i].Center) <= world.Roundabouts[i].Radius+80.0 {
				nearRoundabout = true
				nearRb = &world.Roundabouts[i]
				break
			}
		}
	}

	isTransitioningToLeftTurn := false
	is2LaneRoad := false
	if world != nil {
		_, _, isMulti, is6 := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
		if (isMulti || is6) && ad.TargetLaneOffset <= -laneWidth*0.8 {
			isTransitioningToLeftTurn = true
		}
		if !isMulti && !is6 {
			is2LaneRoad = true
		}
	}

	lookahead := Clamp(speedAbs*0.55+45.0, 55.0, 140.0)
	if nearRoundabout {
		lookahead = Clamp(speedAbs*0.35+30.0, 35.0, 52.0)
	} else if is2LaneRoad && (ad.OvertakeState == OvertakeChangingLane || (ad.LaneOffset != 0 && math.Abs(lateralErr) > 6.0)) {
		// On 2-lane roads borrowing oncoming lane: smooth, longer lookahead prevents steep yaw into opposite shoulder
		lookahead = Clamp(speedAbs*0.42+50.0, 65.0, 105.0)
	} else if !isTransitioningToLeftTurn && (ad.OvertakeState == OvertakeChangingLane || (ad.LaneOffset != 0 && math.Abs(lateralErr) > 6.0)) {
		// Responsive lookahead during mid-block lane transition so bus steers decisively into new lane
		lookahead = Clamp(speedAbs*0.28+36.0, 48.0, 92.0)
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
			effOffset := ad.LaneOffset
			if nearRb != nil {
				// In a roundabout, only negative offset (inner passing lane) or 0 (outer base lane) is allowed.
				// Positive offset (shifting outwards into curb or exits) must never be permitted.
				if effOffset > 0.0 {
					effOffset = 0.0
				}
				maxInnerShift := -(nearRb.Radius - nearRb.InnerRadius - 28.0)
				if effOffset < maxInnerShift {
					effOffset = maxInnerShift
				}
			} else if world != nil {
				roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
				if !isMultiLane && !is6Lane {
					roadCenter, isEW, isMultiLane, is6Lane = getCurrentRoadInfo(world, baseTargetPt, bus.Heading, laneWidth)
				}
				if isMultiLane || is6Lane {
					dBaseCenter := math.Abs(baseTargetPt.Y - roadCenter)
					if !isEW {
						dBaseCenter = math.Abs(baseTargetPt.X - roadCenter)
					}
					// Innermost lane center is at 0.5*laneWidth from road center.
					// Vehicle must never cross the double-yellow line into oncoming traffic on multi-lane roads.
					maxLeftShift := math.Max(0.0, dBaseCenter-laneWidth*0.5)
					if effOffset < -maxLeftShift {
						effOffset = -maxLeftShift
					}
				} else {
					// On 2-lane roads: clamp effOffset so targetPt stays at or inside oncoming lane center
					maxBorrowShift := laneWidth * 0.88
					if effOffset < -maxBorrowShift {
						effOffset = -maxBorrowShift
					}
				}
			}

			targetPt = baseTargetPt.Add(trgRight.Mul(effOffset))

			if nearRb != nil {
				// While baseTargetPt is circulating inside the roundabout, keep targetPt strictly
				// within the circular annular roadway of the roundabout
				if baseTargetPt.Distance(nearRb.Center) <= nearRb.Radius+10.0 {
					toTrg := targetPt.Sub(nearRb.Center)
					distTrg := toTrg.Length()
					if distTrg > 1e-3 {
						uTrg := toTrg.Mul(1.0 / distTrg)
						minAllowedR := nearRb.InnerRadius + 18.0
						maxAllowedR := nearRb.Radius - 10.0
						clampedR := Clamp(distTrg, minAllowedR, maxAllowedR)
						targetPt = nearRb.Center.Add(uTrg.Mul(clampedR))
					}
				}
			} else if world != nil {
				roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
				if isMultiLane || is6Lane {
					// Guard targetPt so it never crosses the double yellow line (0.5*laneWidth from roadCenter)
					if isEW {
						if math.Cos(bus.Heading) > 0 {
							if targetPt.Y < roadCenter+laneWidth*0.5 {
								targetPt.Y = roadCenter + laneWidth*0.5
							}
						} else {
							if targetPt.Y > roadCenter-laneWidth*0.5 {
								targetPt.Y = roadCenter - laneWidth*0.5
							}
						}
					} else {
						if math.Sin(bus.Heading) > 0 {
							if targetPt.X > roadCenter-laneWidth*0.5 {
								targetPt.X = roadCenter - laneWidth*0.5
							}
						} else {
							if targetPt.X < roadCenter+laneWidth*0.5 {
								targetPt.X = roadCenter + laneWidth*0.5
							}
						}
					}
				} else {
					// 2-lane road opposite shoulder guard:
					// Opposite shoulder is at roadCenter ± laneWidth.
					// Guard targetPt so it never penetrates closer than 0.52*laneWidth to the opposite shoulder!
					marginFromShoulder := laneWidth * 0.52
					if isEW {
						if math.Cos(bus.Heading) > 0 {
							minAllowedY := roadCenter - laneWidth + marginFromShoulder
							if targetPt.Y < minAllowedY {
								targetPt.Y = minAllowedY
							}
						} else {
							maxAllowedY := roadCenter + laneWidth - marginFromShoulder
							if targetPt.Y > maxAllowedY {
								targetPt.Y = maxAllowedY
							}
						}
					} else {
						if math.Sin(bus.Heading) > 0 {
							// Traveling South: oncoming lane is East (X > roadCenter), opposite shoulder is East (max X)
							maxAllowedX := roadCenter + laneWidth - marginFromShoulder
							if targetPt.X > maxAllowedX {
								targetPt.X = maxAllowedX
							}
						} else {
							// Traveling North: oncoming lane is West (X < roadCenter), opposite shoulder is West (min X)
							minAllowedX := roadCenter - laneWidth + marginFromShoulder
							if targetPt.X < minAllowedX {
								targetPt.X = minAllowedX
							}
						}
					}
				}
			}
		}
	}

	// Station approach lateral protection when waiting in adjacent lane:
	// When bus is waiting alongside in adjacent lane approaching destination stop,
	// keep targetPt centered in the adjacent travel lane, preventing it from steering into the curb bay.
	if nearRb == nil && world != nil && remaining < 350.0 && ad.TargetLaneOffset < -laneWidth*0.3 {
		roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
		if isMultiLane || is6Lane {
			distAdj := laneWidth * 0.5 // Inner lane on 4-lane
			if is6Lane {
				if ad.TargetLaneOffset <= -laneWidth*1.5 {
					distAdj = laneWidth * 0.5 // Leftmost lane
				} else {
					distAdj = laneWidth * 1.5 // Middle lane
				}
			}
			if isEW {
				if math.Cos(bus.Heading) > 0 {
					targetPt.Y = roadCenter + distAdj
				} else {
					targetPt.Y = roadCenter - distAdj
				}
			} else {
				if math.Sin(bus.Heading) > 0 {
					targetPt.X = roadCenter - distAdj
				} else {
					targetPt.X = roadCenter + distAdj
				}
			}
		}
	}
	toTarget := targetPt.Sub(bus.Pos)
	alpha := NormalizeAngle(math.Atan2(toTarget.Y, toTarget.X) - bus.Heading)
	// A target behind the bus only happens when badly off-path: turn at full lock, never reverse
	alpha = Clamp(alpha, -math.Pi/2, math.Pi/2)
	ld := math.Max(toTarget.Length(), 1.0)
	delta := math.Atan2(2.0*bus.WheelBase*math.Sin(alpha), ld)
	steer = Clamp(delta/bus.MaxSteerAngle, -1.0, 1.0)


	// Double-yellow line collision protection on multi-lane and 6-lane roads:
	// When running parallel to the road centerline on an intersection approach or road segment,
	// enforce that vehicle's front corner clearance to the double-yellow line >= margin.
	if world != nil {
		rCenter, rEW, isMulti, is6 := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
		if isTransitioningToLeftTurn && (isMulti || is6) {
			roadHeading := 0.0
			dCenter := bus.Pos.Y - rCenter
			if rEW {
				if math.Cos(bus.Heading) < 0 {
					roadHeading = math.Pi
					dCenter = rCenter - bus.Pos.Y
				}
			} else {
				if math.Sin(bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
					dCenter = rCenter - bus.Pos.X
				} else {
					roadHeading = -math.Pi / 2.0
					dCenter = bus.Pos.X - rCenter
				}
			}

			// Enforce when bus is on our side of the road and in or near inner/middle lane corridor (<= 1.9*lw)
			if dCenter > 0 && dCenter <= laneWidth*1.9 {
				isBeforeInterMouth := true
				if tls != nil {
					for _, inter := range tls.Intersections {
						cosH := math.Cos(bus.Heading)
						sinH := math.Sin(bus.Heading)
						toInter := inter.Center.Sub(bus.Pos)
						fwdDist := toInter.X*cosH + toInter.Y*sinH
						stopLineOffset := inter.GetStopLineDistForHeading(bus.Heading, laneWidth)
						if fwdDist > -30.0 && fwdDist < stopLineOffset*0.35 {
							isBeforeInterMouth = false
							break
						}
					}
				}

				if isBeforeInterMouth {
					psi := NormalizeAngle(roadHeading - bus.Heading)
					halfL := bus.Length * 0.5
					halfW := bus.Width * 0.5
					margin := 8.0

					// Compute maximum safe yaw angle towards oncoming traffic:
					// While transitioning towards the innermost left-turn lane, yaw should not exceed ~18.3 deg (0.32 rad)
					// and must dynamically tighten to ensure front-left corner never violates the double-yellow line.
					avail := (dCenter - halfW - margin) / halfL
					maxPsi := 0.32
					if avail <= 0.0 {
						maxPsi = 0.0
					} else if avail < 1.0 {
						geomPsi := math.Asin(avail)
						if geomPsi < maxPsi {
							maxPsi = geomPsi
						}
					}

					if psi > maxPsi {
						// Heading is too steep towards double-yellow line!
						excess := psi - maxPsi
						correctiveSteer := Clamp(excess*6.0+0.25, 0.30, 1.0)
						if steer < correctiveSteer {
							steer = correctiveSteer
						}
					} else if psi > maxPsi*0.75 {
						if steer < 0 {
							steer = 0.0
						}
					}
					if dCenter <= halfW+margin {
						if steer < 0.40 {
							steer = 0.40
						}
					}
				}
			}
		}

		// Opposite shoulder collision protection on 2-lane roads when borrowing oncoming lane:
		// When borrowing oncoming lane (LaneOffset < 0) or vehicle position is near oncoming half, enforce clearance to opposite shoulder.
		if !isMulti && !is6 {
			roadHeading := 0.0
			dOppositeShoulder := 0.0

			if rEW {
				if math.Cos(bus.Heading) > 0 {
					roadHeading = 0.0
					dOppositeShoulder = bus.Pos.Y - (rCenter - laneWidth)
				} else {
					roadHeading = math.Pi
					dOppositeShoulder = (rCenter + laneWidth) - bus.Pos.Y
				}
			} else {
				if math.Sin(bus.Heading) > 0 {
					roadHeading = math.Pi / 2.0
					dOppositeShoulder = (rCenter + laneWidth) - bus.Pos.X
				} else {
					roadHeading = -math.Pi / 2.0
					dOppositeShoulder = bus.Pos.X - (rCenter - laneWidth)
				}
			}

			if ad.LaneOffset < -laneWidth*0.25 || ad.TargetLaneOffset < -laneWidth*0.25 {
				psiTowardsShoulder := -NormalizeAngle(bus.Heading - roadHeading)
				halfL := bus.Length * 0.5
				halfW := bus.Width * 0.5
				margin := 6.0 // Preserve at least 6px daylight from opposite shoulder

				avail := (dOppositeShoulder - halfW - margin) / halfL
				maxPsi := 0.16 // ~9 degrees max safe yaw towards opposite shoulder
				if avail <= 0.0 {
					maxPsi = 0.0
				} else if avail < 1.0 {
					geomPsi := math.Asin(avail)
					if geomPsi < maxPsi {
						maxPsi = geomPsi
					}
				}

				if psiTowardsShoulder > maxPsi {
					excess := psiTowardsShoulder - maxPsi
					correctiveSteer := Clamp(excess*4.5+0.15, 0.25, 1.0)
					// Steer right away from opposite shoulder
					if steer < correctiveSteer {
						steer = correctiveSteer
					}
				} else if psiTowardsShoulder > maxPsi*0.70 {
					if steer < 0 {
						steer = 0.0
					}
				}
				if dOppositeShoulder <= halfW+margin {
					if steer < 0.35 {
						steer = 0.35 // Firm corrective steer back towards center
					}
				}
			}
		}
	}

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
			targetSpd = ad.GetBerserkCruiseSpeed()
		} else {
			targetSpd = math.Max(apCruiseSpeed, ad.cornerSpeedLimit(s, speedAbs))
		}
	}

	// A. Anticipate corners / roundabout arcs ahead, and slow while circulating
	targetSpd = math.Min(targetSpd, ad.cornerSpeedLimit(s, speedAbs))
	if nearRoundabout {
		rbSpeed := apRoundaboutSpeed
		if ad.BerserkMode {
			rbSpeed = 120.0
		}
		targetSpd = math.Min(targetSpd, rbSpeed)
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
	if remaining > 2.0 || (math.Abs(ad.LaneOffset) > 10.0 && distToStop > 20.0) {
		creepFloor := math.Min(16.0, math.Max(remaining, distToStop)*1.5)
		if ad.BerserkMode {
			creepFloor = math.Min(22.0, math.Max(remaining, distToStop)*1.8)
		}
		creepFloor = math.Max(creepFloor, 14.0)
		stopSpd = math.Max(stopSpd, creepFloor)
	} else {
		stopSpd = 0.0
	}
	targetSpd = math.Min(targetSpd, stopSpd)

	// If approaching destination stop while offset from base lane, cap speed smoothly to allow lateral entry into bay
	if remaining < 450.0 && (math.Abs(ad.LaneOffset) > 5.0 || math.Abs(ad.TargetLaneOffset) > 5.0) {
		entrySpd := math.Max(60.0, math.Sqrt(2*140.0*math.Max(0, remaining-30.0)))
		targetSpd = math.Min(targetSpd, entrySpd)
	}

	// Station approach speed control when waiting alongside in adjacent lane:
	if remaining < 350.0 && (ad.LaneOffset < -laneWidth*0.3 || ad.TargetLaneOffset < -laneWidth*0.3) {
		cosH := math.Cos(bus.Heading)
		sinH := math.Sin(bus.Heading)
		hasAhead, hasSide, _, leadObs := checkStationApproachLeadVehicle(
			bus, obstacles, cosH, sinH, ad.LaneOffset, ad.drivePath, ad.WaypointIdx, targetStop,
		)
		if (hasAhead || hasSide) && leadObs != nil && targetStop != nil && math.Cos(leadObs.Heading-targetStop.Heading) > 0.4 {
			dBay := leadObs.Pos.Distance(targetStop.Pos)
			if dBay < (targetStop.Length*0.6 + leadObs.Length*0.5 + 35.0) {
				dObs := leadObs.Pos.Sub(bus.Pos)
				fwdObs := dObs.X*cosH + dObs.Y*sinH
				leadSpd := math.Max(0.0, leadObs.Speed)
				if fwdObs < 35.0 {
					targetSpd = math.Min(targetSpd, leadSpd)
				}
				if leadSpd < 5.0 && fwdObs > -25.0 {
					targetSpd = 0.0
				}
			}
		}
	}

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

			if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookaheadTL+120.0 && lateralDist < laneWidth*4.8 {
				hasRouteExit := false

				exitLatThreshold := laneWidth * 1.6
				if world != nil {
					_, _, _, is6 := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
					if is6 {
						exitLatThreshold = laneWidth * 2.8
					}
				}

				// Check ad.drivePath points through and beyond this intersection
				for idx := ad.WaypointIdx; idx < len(ad.drivePath); idx++ {
					pt := ad.drivePath[idx]
					toPt := pt.Sub(inter.Center)
					fwdPt := toPt.X*cosH + toPt.Y*sinH
					latPt := -toPt.X*sinH + toPt.Y*cosH

					if fwdPt < -stopLineDist {
						continue
					}

					// Exit point past the intersection center mouth
					if fwdPt > 30.0 || math.Abs(latPt) > exitLatThreshold {
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
				// Ignore pure lateral lane-merge jogs or tapers that stay along current road heading
				angPrev := NormalizeAngle(math.Atan2(segPrev.Y, segPrev.X) - bus.Heading)
				angNext := NormalizeAngle(math.Atan2(segNext.Y, segNext.X) - bus.Heading)
				if math.Abs(angPrev) < 0.45 && math.Abs(angNext) < 0.45 {
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

		// In a roundabout, vehicles safely separated in adjacent circular lanes (e.g. inner passing lane vs outer lane)
		if nearRoundabout && world != nil {
			isRoundaboutSeparated := false
			for _, rb := range world.Roundabouts {
				dBus := bus.Pos.Distance(rb.Center)
				dObs := obs.Pos.Distance(rb.Center)
				if dBus <= rb.Radius+15.0 && dObs <= rb.Radius+15.0 {
					// Both vehicles in roundabout: if separated radially into different lanes
					if math.Abs(dBus-dObs) >= (bus.Width/2 + obs.Width/2 + 8.0) {
						isRoundaboutSeparated = true
						break
					}
				}
			}
			if isRoundaboutSeparated {
				continue
			}
		}

		delta := obs.Pos.Sub(bus.Pos)
		fwdDist := delta.X*cosH + delta.Y*sinH
		latDist := math.Abs(-delta.X*sinH + delta.Y*cosH)

		// For oncoming vehicles (facing opposite direction), ignore if safely separated:
		cosHeadDiff := math.Cos(obs.Heading - bus.Heading)
		if cosHeadDiff < -0.2 {
			// 1. Safely separated laterally in local vehicle frame (standard straight-road case)
			if latDist > (bus.Width/2 + obs.Width/2 + 2.0) {
				continue
			}
			// 2. Safely separated by road centerline / double yellow line on multi-lane roads
			if world != nil {
				separated := false
				for _, r := range world.Roads {
					roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
					c := r.P1.Y
					if !roadIsEW {
						c = r.P1.X
					}
					is6 := (r.Lanes >= 6 || r.Type == RoadExpressway6Lane || world.IsRoad6Lane(c, roadIsEW, bus.Pos))
					isMulti := (is6 || r.Lanes >= 4 || r.Type == RoadAvenue4Lane || world.IsRoad4Lane(c, roadIsEW, bus.Pos))
					if !isMulti {
						continue
					}
					rWidth := r.GetWidth(laneWidth)
					if roadIsEW {
						minX := math.Min(r.P1.X, r.P2.X) - 30.0
						maxX := math.Max(r.P1.X, r.P2.X) + 30.0
						if bus.Pos.X >= minX && bus.Pos.X <= maxX && math.Abs(bus.Pos.Y-c) <= rWidth*0.5+15.0 &&
							obs.Pos.X >= minX && obs.Pos.X <= maxX && math.Abs(obs.Pos.Y-c) <= rWidth*0.5+15.0 {
							dBus := bus.Pos.Y - c
							dObs := obs.Pos.Y - c
							if (dBus <= 2.0 && dObs >= -2.0) || (dBus >= -2.0 && dObs <= 2.0) {
								if math.Abs(bus.Pos.Y-obs.Pos.Y) >= (bus.Width/2 + obs.Width/2) {
									separated = true
									break
								}
							}
						}
					} else {
						minY := math.Min(r.P1.Y, r.P2.Y) - 30.0
						maxY := math.Max(r.P1.Y, r.P2.Y) + 30.0
						if bus.Pos.Y >= minY && bus.Pos.Y <= maxY && math.Abs(bus.Pos.X-c) <= rWidth*0.5+15.0 &&
							obs.Pos.Y >= minY && obs.Pos.Y <= maxY && math.Abs(obs.Pos.X-c) <= rWidth*0.5+15.0 {
							dBus := bus.Pos.X - c
							dObs := obs.Pos.X - c
							if (dBus <= 2.0 && dObs >= -2.0) || (dBus >= -2.0 && dObs <= 2.0) {
								if math.Abs(bus.Pos.X-obs.Pos.X) >= (bus.Width/2 + obs.Width/2) {
									separated = true
									break
								}
							}
						}
					}
				}
				if separated {
					continue
				}
			}
			// 3. Safely outside our tracked drive path corridor (e.g. turning across intersection while oncoming car waits in its lane)
			if len(ad.drivePath) >= 2 {
				dPath := minDistanceToPath(obs.Pos, ad.drivePath, ad.WaypointIdx)
				if dPath > (bus.Width/2 + obs.Width/2 + 6.0) {
					continue
				}
			}
		}

		// Immediate physical proximity / collision containment:
		// If obstacle directly overlaps with or is right against the bus front envelope:
		directDist := obs.Pos.Distance(bus.Pos)
		collisionDist := bus.Length/2 + obs.Length/2 + 8.0
		if directDist < collisionDist && fwdDist > -(bus.Length/2) {
			isLaneSeparated := (math.Abs(ad.LaneOffset) > 8.0 || ad.OvertakeState != OvertakeNone) && latDist >= (bus.Width/2+obs.Width/2-2.0)
			if !isLaneSeparated && latDist < (bus.Width/2+obs.Width/2+8.0) {
				targetSpd = 0.0
				continue
			}
		}

		if fwdDist > 2.0 && fwdDist < fwdLimit && latDist < (bus.Width/2+obs.Width/2+18.0) {
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
						effOffset := ad.LaneOffset
						if world != nil {
							wpt := ad.drivePath[wIdx]
							roadCenter, isEW, isMultiLane, is6Lane := getCurrentRoadInfo(world, wpt, bus.Heading, laneWidth)
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
						obsPos = obs.Pos.Sub(segRight.Mul(effOffset))
					}
				}
				dPath := minDistanceToPath(obsPos, ad.drivePath, ad.WaypointIdx)
				if dPath > (bus.Width/2 + obs.Width/2 + 10.0) {
					// Obstacle is in adjacent lane corridor.
					minClearLat := bus.Width/2 + obs.Width/2 + 6.0
					actualLat := latDist
					if world != nil {
						_, isEW, _, _ := getCurrentRoadInfo(world, bus.Pos, bus.Heading, laneWidth)
						if isEW {
							actualLat = math.Abs(bus.Pos.Y - obs.Pos.Y)
						} else {
							actualLat = math.Abs(bus.Pos.X - obs.Pos.X)
						}
					}
					if actualLat < minClearLat && fwdDist > 0 {
						// Still laterally overlapping lead vehicle: decelerate smoothly behind until laterally clear!
						safeDist := bus.Length/2 + obs.Length/2 + 25.0
						if fwdDist < safeDist {
							targetSpd = math.Min(targetSpd, math.Max(0.0, obs.Speed-5.0))
						} else {
							decelRate := 160.0
							decelSpd := obs.Speed + math.Sqrt(2*decelRate*math.Max(0, fwdDist-safeDist))
							targetSpd = math.Min(targetSpd, decelSpd)
						}
					}
					continue
				}
			}

			safeStopDist := bus.Length/2 + obs.Length/2 + 30.0
			if ad.BerserkMode && ad.OvertakeState == OvertakeChangingLane {
				safeStopDist = bus.Length/2 + obs.Length/2 + 25.0
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
				// If closing in at close quarters while still overlapping laterally:
				// Clamp closing rate so bus cannot close faster than available buffer distance
				gap := fwdDist - safeStopDist
				if gap < 50.0 {
					maxClosing := math.Max(0.0, gap*0.75)
					decelSpd = math.Min(decelSpd, leadSpd+maxClosing)
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
