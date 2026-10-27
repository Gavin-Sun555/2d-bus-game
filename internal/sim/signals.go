package sim

import (
	"math"
)

type LightSignal int

const (
	SignalRed LightSignal = iota
	SignalYellow
	SignalGreen
)

type LightPhase int

const (
	PhaseEWGreen LightPhase = iota
	PhaseEWYellow
	PhaseEWLeftGreen
	PhaseEWLeftYellow
	PhaseNSGreen
	PhaseNSYellow
	PhaseNSLeftGreen
	PhaseNSLeftYellow
)

// TrafficIntersection manages signal phases and stop lines for an intersection
type TrafficIntersection struct {
	approachDimensions approachDimensionsCache
	ID                 int
	Center             Vec2
	Phase              LightPhase
	Timer              float64
	Cooldown           float64 // Cooldown for player violation detection
	IsMajor            bool    // true for major avenue intersections with dedicated left-turn lanes & signals

	// Per-approach road configurations (0: West, 1: East, 2: North, 3: South)
	ApproachHasRoad [4]bool
	ApproachBusLane [4]bool
	ApproachLanes   [4]int  // Total lanes of the road (2, 4, 6 or 8)
	ApproachIs4Lane [4]bool // True if the road entering this approach has 4 lanes (dual carriageway / 2 incoming lanes)

	// Durations
	GreenDuration     float64
	YellowDuration    float64
	LeftGreenDuration float64

	// Player intersection traversal tracking
	PlayerInside              bool
	PlayerCrossedBeforeYellow bool
	PlayerCrossedOnRed        bool
	PlayerTurnedRight         bool // true if executed a lawful right turn (exempt from red light fines)
	PlayerTurnedLeft          bool // true if executed a lawful left turn (exempt from through red light fines if left arrow is green)
	PlayerEntryBusLane        bool
	PlayerEntryBusLeftOnly    bool
	PlayerEntryRightTurnLane  bool    // Wide-junction right turn may start well after the rear crosses the line.
	PlayerEntryLane           int     // 0 = inner/left-turn lane, 1 = outer/straight-right lane
	PlayerEntryHeading        float64 // Fixed approach road axis; lane-change yaw must not rotate the stop line.
	PlayerEntryPos            Vec2
	PlayerEntryIsEW           bool // true if vehicle entered along East-West corridor
	PassageCompleted          bool // true once vehicle reached exit/completed traversal, prevents cross-street re-entry
	FinedThisPassage          bool
}

// RoadCamera represents a traffic surveillance camera on straight mid-block road segments
type RoadCamera struct {
	ID         int
	Pos        Vec2
	RoadName   string
	IsEW       bool    // true for East-West road, false for North-South road
	RoadCoord  float64 // Centerline coordinate: Y for EW road, X for NS road
	Lanes      int     // Total road lanes (2, 4, 6)
	Cooldown   float64 // Fine cooldown
	FlashTimer float64 // Camera strobe flash visual timer
}

// TrafficLightSystem coordinates all city intersections and surveillance cameras
type TrafficLightSystem struct {
	junctionIndex *junctionIndex
	Intersections []*TrafficIntersection
	RoadCameras   []*RoadCamera
}

func NewTrafficLightSystem() *TrafficLightSystem {
	tls := &TrafficLightSystem{
		Intersections: make([]*TrafficIntersection, 0),
		RoadCameras: []*RoadCamera{
			{ID: 0, Pos: Vec2{X: 1950 * MapGridScale, Y: 0}, RoadName: "中央主干大道 (东段)", IsEW: true, RoadCoord: 0, Lanes: 4},
			{ID: 1, Pos: Vec2{X: -1950 * MapGridScale, Y: 0}, RoadName: "中央主干大道 (西段)", IsEW: true, RoadCoord: 0, Lanes: 4},
			{ID: 2, Pos: Vec2{X: 0, Y: -300 * MapGridScale}, RoadName: "城市中轴大道 (北段)", IsEW: false, RoadCoord: 0, Lanes: 4},
			{ID: 3, Pos: Vec2{X: 0, Y: 300 * MapGridScale}, RoadName: "城市中轴大道 (南段)", IsEW: false, RoadCoord: 0, Lanes: 4},
			{ID: 4, Pos: Vec2{X: 650 * MapGridScale, Y: -1200 * MapGridScale}, RoadName: "北环六车道快速路", IsEW: true, RoadCoord: -1200 * MapGridScale, Lanes: 6},
			{ID: 5, Pos: Vec2{X: -650 * MapGridScale, Y: 1200 * MapGridScale}, RoadName: "南环快速路", IsEW: true, RoadCoord: 1200 * MapGridScale, Lanes: 4},
		},
	}

	xs := []float64{-2600 * MapGridScale, -1300 * MapGridScale, 0, 1300 * MapGridScale, 2600 * MapGridScale}
	ys := []float64{-1200 * MapGridScale, -600 * MapGridScale, 0, 600 * MapGridScale, 1200 * MapGridScale}

	id := 0
	for iy, y := range ys {
		for ix, x := range xs {
			// Major intersections are along 4-lane avenues: North Ring, Central Blvd, South Ring, and Central Ave
			isEW4Lane := (y == -1200 || y == 0 || y == 1200)
			isNS4Lane := (x == 0)
			isMajor := (isEW4Lane || isNS4Lane)

			ewLanes := 2
			if isEW4Lane {
				ewLanes = 4
			}
			nsLanes := 2
			if isNS4Lane {
				nsLanes = 4
			}

			inter := &TrafficIntersection{
				ID:                id,
				Center:            Vec2{X: x, Y: y},
				IsMajor:           isMajor,
				ApproachHasRoad:   [4]bool{true, true, true, true},
				ApproachLanes:     [4]int{ewLanes, ewLanes, nsLanes, nsLanes},
				ApproachIs4Lane:   [4]bool{isEW4Lane, isEW4Lane, isNS4Lane, isNS4Lane},
				GreenDuration:     8.0,
				YellowDuration:    2.5,
				LeftGreenDuration: 5.5,
				Cooldown:          0,
			}

			if isMajor {
				// 8-phase cycle for major intersections: (8.0 + 2.5 + 5.5 + 2.5) * 2 = 37.0s
				offsetTimer := math.Mod(float64(ix*6+iy*7), 37.0)
				t := offsetTimer
				switch {
				case t < 8.0:
					inter.Phase = PhaseEWGreen
					inter.Timer = t
				case t < 10.5:
					inter.Phase = PhaseEWYellow
					inter.Timer = t - 8.0
				case t < 16.0:
					inter.Phase = PhaseEWLeftGreen
					inter.Timer = t - 10.5
				case t < 18.5:
					inter.Phase = PhaseEWLeftYellow
					inter.Timer = t - 16.0
				case t < 26.5:
					inter.Phase = PhaseNSGreen
					inter.Timer = t - 18.5
				case t < 29.0:
					inter.Phase = PhaseNSYellow
					inter.Timer = t - 26.5
				case t < 34.5:
					inter.Phase = PhaseNSLeftGreen
					inter.Timer = t - 29.0
				default:
					inter.Phase = PhaseNSLeftYellow
					inter.Timer = t - 34.5
				}
			} else {
				// Standard 4-phase cycle for minor streets
				offsetTimer := math.Mod(float64(ix*5+iy*7), 26.0)
				t := offsetTimer
				switch {
				case t < 10.0:
					inter.Phase = PhaseEWGreen
					inter.Timer = t
				case t < 13.0:
					inter.Phase = PhaseEWYellow
					inter.Timer = t - 10.0
				case t < 23.0:
					inter.Phase = PhaseNSGreen
					inter.Timer = t - 13.0
				default:
					inter.Phase = PhaseNSYellow
					inter.Timer = t - 23.0
				}
			}

			tls.Intersections = append(tls.Intersections, inter)
			id++
		}
	}

	return tls
}

// BuildForWorld adapts traffic lights and speed cameras to the active city map's road topology
func (tls *TrafficLightSystem) BuildForWorld(w *World) {
	tls.junctionIndex = nil
	if w == nil {
		return
	}
	tls.Intersections = make([]*TrafficIntersection, 0)
	tls.RoadCameras = make([]*RoadCamera, 0)

	if w.Theme == MapThemeMetropolis {
		tls.RoadCameras = []*RoadCamera{
			{ID: 0, Pos: Vec2{X: 1950 * MapGridScale, Y: 0}, RoadName: "中央主干大道 (东段)", IsEW: true, RoadCoord: 0, Lanes: 4},
			{ID: 1, Pos: Vec2{X: -1950 * MapGridScale, Y: 0}, RoadName: "中央主干大道 (西段)", IsEW: true, RoadCoord: 0, Lanes: 4},
			{ID: 2, Pos: Vec2{X: 0, Y: -300 * MapGridScale}, RoadName: "城市中轴大道 (北段)", IsEW: false, RoadCoord: 0, Lanes: 4},
			{ID: 3, Pos: Vec2{X: 0, Y: 300 * MapGridScale}, RoadName: "城市中轴大道 (南段)", IsEW: false, RoadCoord: 0, Lanes: 4},
			{ID: 4, Pos: Vec2{X: 650 * MapGridScale, Y: -1200 * MapGridScale}, RoadName: "北环六车道快速路", IsEW: true, RoadCoord: -1200 * MapGridScale, Lanes: 6},
			{ID: 5, Pos: Vec2{X: -650 * MapGridScale, Y: 1200 * MapGridScale}, RoadName: "南环快速路", IsEW: true, RoadCoord: 1200 * MapGridScale, Lanes: 4},
		}
	} else {
		camID := 0
		for _, r := range w.Roads {
			if r.Lanes == 4 {
				mid := r.P1.Add(r.P2).Mul(0.5)
				tooClose := false
				for _, ipt := range w.Intersections {
					if mid.Distance(ipt) < 250 {
						tooClose = true
						break
					}
				}
				if !tooClose && camID < 6 {
					isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
					coord := mid.Y
					if !isEW {
						coord = mid.X
					}
					tls.RoadCameras = append(tls.RoadCameras, &RoadCamera{
						ID:        camID,
						Pos:       mid,
						RoadName:  r.Name,
						IsEW:      isEW,
						RoadCoord: coord,
					})
					camID++
				}
			}
		}
	}

	for id, pt := range w.Intersections {
		// Roundabouts do NOT have traditional traffic lights; they are unsignalized circular junctions
		if w.IsAtRoundabout(pt) {
			continue
		}

		inter := &TrafficIntersection{
			ID:                id,
			Center:            pt,
			GreenDuration:     8.0,
			YellowDuration:    2.5,
			LeftGreenDuration: 5.5,
			Cooldown:          0,
		}
		inter.SetupApproachesFromWorld(w)

		offsetTimer := math.Mod(float64(id*7), 37.0)
		t := offsetTimer
		if inter.IsMajor {
			switch {
			case t < 8.0:
				inter.Phase = PhaseEWGreen
				inter.Timer = t
			case t < 10.5:
				inter.Phase = PhaseEWYellow
				inter.Timer = t - 8.0
			case t < 16.0:
				inter.Phase = PhaseEWLeftGreen
				inter.Timer = t - 10.5
			case t < 18.5:
				inter.Phase = PhaseEWLeftYellow
				inter.Timer = t - 16.0
			case t < 26.5:
				inter.Phase = PhaseNSGreen
				inter.Timer = t - 18.5
			case t < 29.0:
				inter.Phase = PhaseNSYellow
				inter.Timer = t - 26.5
			case t < 34.5:
				inter.Phase = PhaseNSLeftGreen
				inter.Timer = t - 29.0
			default:
				inter.Phase = PhaseNSLeftYellow
				inter.Timer = t - 34.5
			}
		} else {
			tSub := math.Mod(float64(id*7), 21.0)
			switch {
			case tSub < 8.0:
				inter.Phase = PhaseEWGreen
				inter.Timer = tSub
			case tSub < 10.5:
				inter.Phase = PhaseEWYellow
				inter.Timer = tSub - 8.0
			case tSub < 18.5:
				inter.Phase = PhaseNSGreen
				inter.Timer = tSub - 10.5
			default:
				inter.Phase = PhaseNSYellow
				inter.Timer = tSub - 18.5
			}
		}
		tls.Intersections = append(tls.Intersections, inter)
	}
}
