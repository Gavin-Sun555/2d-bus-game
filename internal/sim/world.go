package sim

import (
	"image/color"
	"math"
)

const (
	MapGridScale  = 3.0  // Multiplier lengthening road distances between intersections (~3x) for overtaking & high-speed cruising
	ShoulderWidth = 40.0 // Width of shoulder outside road curb where bus can drive but slows down
)

type RoadType int

const (
	RoadAvenue4Lane     RoadType = iota // 4-lane major avenue / ring highway
	RoadStreet2Lane                     // 2-lane city street
	RoadExpressway6Lane                 // 6-lane express avenue / trunk highway
	RoadBoulevard8Lane
	RoadBusBoulevard8Lane
)

type RoadZone int

const (
	ZoneRoad        RoadZone = iota // Asphalt road surface
	ZoneShoulder                    // Roadside shoulder / sidewalk (slow down)
	ZoneHardBarrier                 // Off-limits / hard barrier (collision & damage & fine)
)

type BoundaryCheckResult struct {
	Zone         RoadZone
	Pushback     Vec2
	Penetration  float64
	ObstacleName string
}

// RoadSegment represents an interconnected road segment
type RoadSegment struct {
	P1       Vec2
	P2       Vec2
	Lanes    int
	Type     RoadType
	HasLines bool
	Name     string
}

func (r RoadSegment) GetWidth(laneWidth float64) float64 {
	lanes := r.LaneCount()
	median := 12.0
	if lanes >= 4 || r.Type == RoadAvenue4Lane {
		median = 24
	}
	return float64(lanes)*laneWidth + median
}

// Building represents decorative scenery blocks
type Building struct {
	Pos   Vec2
	W     float64
	H     float64
	Color color.RGBA
	Roof  color.RGBA
	Name  string
}

// Tree represents roadside greenery
type Tree struct {
	Pos    Vec2
	Radius float64
	Color  color.RGBA
}

// MapTheme represents the visual, geographic, and architectural style of the map
type MapTheme int

const (
	MapThemeMetropolis  MapTheme = iota // 1. 都会核心 (Metropolis CBD) - 环城快速路、核心高层、都会中央运河
	MapThemeCoastalBay                  // 2. 海滨港湾 (Coastal Bay) - 滨海海景大道、跨海特大桥、离岸深水港区
	MapThemeRiverValley                 // 3. 溪谷江城 (River Valley) - 宽阔长江双岸、北岸山地台地、南岸高铁新城、跨江斜拉双桥
	MapThemeEmeraldLake                 // 4. 湖岛水乡 (Emerald Lake) - 天然翠湖水系、湖心仙岛、四方长堤石拱桥、江南水乡古镇
	MapThemeTwinCities                  // 5. 高新双城 (Twin Districts) - 临空智造西城、硅谷科技东城、中央高架快速路、立体枢纽立交桥
	MapThemeCustom                      // 6. 自定义地图 (Custom Map) - 玩家自建路网、独立客运站与水系、本地JSON即时载入
)

func (m MapTheme) Name() string {
	switch m {
	case MapThemeMetropolis:
		return "都会核心 (Metropolis CBD)"
	case MapThemeCoastalBay:
		return "海滨港湾 (Coastal Bay)"
	case MapThemeRiverValley:
		return "溪谷江城 (River Valley)"
	case MapThemeEmeraldLake:
		return "湖岛水乡 (Emerald Lake)"
	case MapThemeTwinCities:
		return "高新双城 (Twin Districts)"
	case MapThemeCustom:
		return "自定义地图 (Custom Map)"
	default:
		return "都会核心"
	}
}

func (m MapTheme) DisplayName() string {
	switch m {
	case MapThemeMetropolis:
		return "现代都会 (Metropolis)"
	case MapThemeCoastalBay:
		return "海滨港湾 (Coastal Bay)"
	case MapThemeRiverValley:
		return "溪谷江城 (River Valley)"
	case MapThemeEmeraldLake:
		return "湖畔水乡 (Emerald Lake)"
	case MapThemeTwinCities:
		return "高新双城 (Twin Districts)"
	case MapThemeCustom:
		return "自定义地图 (Custom Map)"
	default:
		return m.Name()
	}
}

func (m MapTheme) ShortName() string {
	switch m {
	case MapThemeMetropolis:
		return "现代都会"
	case MapThemeCoastalBay:
		return "海滨港湾"
	case MapThemeRiverValley:
		return "溪谷江城"
	case MapThemeEmeraldLake:
		return "湖畔水乡"
	case MapThemeTwinCities:
		return "高新双城"
	case MapThemeCustom:
		return "自定义"
	default:
		return "现代都会"
	}
}

func (m MapTheme) Subtitle() string {
	switch m {
	case MapThemeMetropolis:
		return "大都会环线快速路 · 核心商务高层群 · 运河双子桥"
	case MapThemeCoastalBay:
		return "沿海景观风情大道 · 跨海湾长堤大桥 · 国际深水港"
	case MapThemeRiverValley:
		return "宽阔天然长江大峡谷 · 北岸山城台地 · 跨江斜拉双桥"
	case MapThemeEmeraldLake:
		return "天然翠湖环湖公路 · 湖心绿岛古镇 · 四方长堤石拱桥"
	case MapThemeTwinCities:
		return "临空智造与软件双城 · 中央跨区高架 · 立体互通立交"
	case MapThemeCustom:
		return "玩家自由定制路网 · 独立客运站与水系 · 实时载入 custom_map.json"
	default:
		return ""
	}
}

func (m MapTheme) Next() MapTheme {
	return (m + 1) % 6
}

type BridgeStyle int

const (
	BridgeStyleStandard BridgeStyle = iota
	BridgeStyleSuspension
	BridgeStyleStoneArch
	BridgeStyleOverpass
)

// Bridge represents a river or ocean crossing bridge structure
type Bridge struct {
	Pos        Vec2    // Center of bridge
	Heading    float64 // Direction of road across bridge (0 for EW, Pi/2 for NS)
	Length     float64 // Length along heading
	Width      float64 // Width of bridge deck
	Name       string
	IsArch     bool // Classic stone arch bridge
	IsCable    bool // Modern suspension cable bridge
	IsOverpass bool // Grade-separated highway overpass
}

func (b Bridge) Style() BridgeStyle {
	if b.IsArch {
		return BridgeStyleStoneArch
	}
	if b.IsCable {
		return BridgeStyleSuspension
	}
	if b.IsOverpass {
		return BridgeStyleOverpass
	}
	return BridgeStyleStandard
}

// Roundabout represents a circular rotary traffic plaza
type Roundabout struct {
	Center      Vec2
	Radius      float64 // Outer road radius
	InnerRadius float64 // Inner plaza/island radius
	Name        string
	Type        RoundaboutType
}

func (r Roundabout) OuterRadius() float64 {
	return r.Radius
}

// IsAtRoundabout checks if a given coordinate is at or inside any traffic roundabout
func (w *World) IsAtRoundabout(pt Vec2) bool {
	for _, rb := range w.Roundabouts {
		if pt.Distance(rb.Center) <= rb.Radius+40.0 {
			return true
		}
	}
	return false
}

// GetRoadLanes returns the number of lanes for a road at coordinate (and optional position)
func (w *World) GetRoadLanes(coord float64, isEW bool, posOptional ...Vec2) int {
	if w == nil {
		return 2
	}
	hasPos := len(posOptional) > 0
	var pos Vec2
	if hasPos {
		pos = posOptional[0]
	}

	for _, r := range w.Roads {
		roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		if roadIsEW == isEW {
			c := r.P1.Y
			if !isEW {
				c = r.P1.X
			}
			if math.Abs(c-coord) < 25.0 {
				if hasPos {
					if isEW {
						minX := math.Min(r.P1.X, r.P2.X) - 40.0
						maxX := math.Max(r.P1.X, r.P2.X) + 40.0
						if pos.X < minX || pos.X > maxX {
							continue
						}
					} else {
						minY := math.Min(r.P1.Y, r.P2.Y) - 40.0
						maxY := math.Max(r.P1.Y, r.P2.Y) + 40.0
						if pos.Y < minY || pos.Y > maxY {
							continue
						}
					}
				}
				return r.LaneCount()
			}
		}
	}
	return 2
}

// IsRoad6Lane checks whether a road at coordinate (and optional position) is a 6-lane express avenue
func (w *World) IsRoad6Lane(coord float64, isEW bool, posOptional ...Vec2) bool {
	return w.GetRoadLanes(coord, isEW, posOptional...) >= 6
}

// IsRoad4Lane checks whether a road at coordinate (and optional position) is a 4-lane or multi-lane avenue
func (w *World) IsRoad4Lane(coord float64, isEW bool, posOptional ...Vec2) bool {
	return w.GetRoadLanes(coord, isEW, posOptional...) >= 4
}

type RoundaboutType int

const (
	RoundaboutCivic RoundaboutType = iota
	RoundaboutLighthouse
	RoundaboutIsland
)

// WaterBody represents expansive lakes, coastal oceans, and river canals
type WaterBody struct {
	IsOcean  bool
	HasBeach bool
	IsLake   bool
	Center   Vec2
	Width    float64
	Height   float64
	Color    color.RGBA
	Shimmer  color.RGBA
	Name     string
}

// River represents a scenic waterway
type River struct {
	Y      float64
	Height float64
}

// World holds the city roads, buildings, trees, water, and stops
type World struct {
	Theme         MapTheme
	Roads         []RoadSegment
	Buildings     []Building
	Trees         []Tree
	Stops         []*BusStop
	Rivers        []River
	Waters        []WaterBody
	Bridges       []Bridge
	Roundabouts   []Roundabout
	Intersections []Vec2
	CustomMap     *CustomMapData
}

// ActiveWorldForRoute stores a global reference to the currently active world for route pathfinding
var ActiveWorldForRoute *World

func NewWorld(initialLaneWidth float64, theme ...MapTheme) *World {
	t := MapThemeMetropolis
	if len(theme) > 0 {
		t = theme[0]
	}
	w := &World{
		Theme:         t,
		Roads:         make([]RoadSegment, 0),
		Buildings:     make([]Building, 0),
		Trees:         make([]Tree, 0),
		Stops:         make([]*BusStop, 0),
		Rivers:        make([]River, 0),
		Waters:        make([]WaterBody, 0),
		Bridges:       make([]Bridge, 0),
		Roundabouts:   make([]Roundabout, 0),
		Intersections: make([]Vec2, 0),
	}

	w.BuildMap(t, initialLaneWidth)
	return w
}

func (w *World) BuildMap(theme MapTheme, laneWidth float64) {
	w.Theme = theme
	w.Roads = make([]RoadSegment, 0)
	w.Buildings = make([]Building, 0)
	w.Trees = make([]Tree, 0)
	w.Stops = make([]*BusStop, 0)
	w.Rivers = make([]River, 0)
	w.Waters = make([]WaterBody, 0)
	w.Bridges = make([]Bridge, 0)
	w.Roundabouts = make([]Roundabout, 0)
	w.Intersections = make([]Vec2, 0)

	switch theme {
	case MapThemeMetropolis:
		w.buildMap1Metropolis(laneWidth)
	case MapThemeCoastalBay:
		w.buildMap2CoastalBay(laneWidth)
	case MapThemeRiverValley:
		w.buildMap3RiverValley(laneWidth)
	case MapThemeEmeraldLake:
		w.buildMap4EmeraldLake(laneWidth)
	case MapThemeTwinCities:
		w.buildMap5TwinCities(laneWidth)
	case MapThemeCustom:
		w.buildMap6Custom(laneWidth)
	default:
		w.buildMap1Metropolis(laneWidth)
	}

	w.applyMapScale(MapGridScale, laneWidth)
	w.MatchStopRoadLanes(laneWidth)
	w.RebuildIntersections()
	ActiveWorldForRoute = w
	ClearRoutePathCache()
}

// MatchStopRoadLanes automatically detects the actual adjacent road segment for each bus stop
// and aligns s.RoadLanes to the road's true lane count, guaranteeing that all bus stop bays
// are indented safely outside the curb rather than sitting inside the travel roadway.
func (w *World) MatchStopRoadLanes(laneWidth float64) {
	for _, s := range w.Stops {
		isEW := math.Abs(math.Cos(s.Heading)) > math.Abs(math.Sin(s.Heading))
		for _, r := range w.Roads {
			rIsEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
			if isEW != rIsEW {
				continue
			}
			if isEW {
				if math.Abs(r.P1.Y-s.RoadCenter.Y) < 10.0 {
					minX := math.Min(r.P1.X, r.P2.X) - 50.0
					maxX := math.Max(r.P1.X, r.P2.X) + 50.0
					if s.RoadCenter.X >= minX && s.RoadCenter.X <= maxX {
						if s.RoadLanes != r.Lanes {
							s.RoadLanes = r.Lanes
							s.UpdateGeometry(laneWidth)
						}
						break
					}
				}
			} else {
				if math.Abs(r.P1.X-s.RoadCenter.X) < 10.0 {
					minY := math.Min(r.P1.Y, r.P2.Y) - 50.0
					maxY := math.Max(r.P1.Y, r.P2.Y) + 50.0
					if s.RoadCenter.Y >= minY && s.RoadCenter.Y <= maxY {
						if s.RoadLanes != r.Lanes {
							s.RoadLanes = r.Lanes
							s.UpdateGeometry(laneWidth)
						}
						break
					}
				}
			}
		}
	}
}

// applyMapScale uniformly scales road lengths, stop positions, water, bridges, and buildings
// to lengthen intersection-to-intersection road blocks by ~3x while preserving standard lane widths.
func (w *World) applyMapScale(scale float64, laneWidth float64) {
	if scale == 1.0 {
		return
	}
	for i := range w.Roads {
		w.Roads[i].P1 = w.Roads[i].P1.Mul(scale)
		w.Roads[i].P2 = w.Roads[i].P2.Mul(scale)
	}
	for _, st := range w.Stops {
		st.RoadCenter = st.RoadCenter.Mul(scale)
		st.UpdateGeometry(laneWidth)
	}
	for i := range w.Bridges {
		w.Bridges[i].Pos = w.Bridges[i].Pos.Mul(scale)
		w.Bridges[i].Length *= scale
	}
	for i := range w.Roundabouts {
		w.Roundabouts[i].Center = w.Roundabouts[i].Center.Mul(scale)
	}
	for i := range w.Waters {
		w.Waters[i].Center = w.Waters[i].Center.Mul(scale)
		w.Waters[i].Width *= scale
		w.Waters[i].Height *= scale
	}
	for i := range w.Rivers {
		w.Rivers[i].Y *= scale
		w.Rivers[i].Height *= scale
	}
	for i := range w.Buildings {
		w.Buildings[i].Pos = w.Buildings[i].Pos.Mul(scale)
		w.Buildings[i].W *= 1.8
		w.Buildings[i].H *= 1.8
	}
	for i := range w.Trees {
		w.Trees[i].Pos = w.Trees[i].Pos.Mul(scale)
	}
}

func (w *World) IsOverpassAt(pt Vec2) bool {
	for _, br := range w.Bridges {
		if br.IsOverpass {
			cosH := math.Cos(br.Heading)
			sinH := math.Sin(br.Heading)
			diff := pt.Sub(br.Pos)
			longDist := math.Abs(diff.X*cosH + diff.Y*sinH)
			latDist := math.Abs(-diff.X*sinH + diff.Y*cosH)
			if longDist <= br.Length*0.5+15.0 && latDist <= br.Width*0.5+15.0 {
				return true
			}
		}
	}
	return false
}

func (w *World) GetIntersectionRoadDimensions(c Vec2, laneWidth float64) (ewHalf, nsHalf float64) {
	maxEW := 2
	maxNS := 2
	for _, r := range w.Roads {
		if math.Abs(r.P1.Y-r.P2.Y) < 1.0 && math.Abs(r.P1.Y-c.Y) < 10.0 {
			minX := math.Min(r.P1.X, r.P2.X)
			maxX := math.Max(r.P1.X, r.P2.X)
			if c.X >= minX-15.0 && c.X <= maxX+15.0 {
				if r.Lanes > maxEW {
					maxEW = r.Lanes
				}
			}
		}
		if math.Abs(r.P1.X-r.P2.X) < 1.0 && math.Abs(r.P1.X-c.X) < 10.0 {
			minY := math.Min(r.P1.Y, r.P2.Y)
			maxY := math.Max(r.P1.Y, r.P2.Y)
			if c.Y >= minY-15.0 && c.Y <= maxY+15.0 {
				if r.Lanes > maxNS {
					maxNS = r.Lanes
				}
			}
		}
	}
	ewHalf = laneWidth*1.0 + 6.0
	if maxEW >= 6 {
		ewHalf = laneWidth*float64(maxEW)/2 + 12.0
	} else if maxEW >= 4 {
		ewHalf = laneWidth*2.0 + 12.0
	}
	nsHalf = laneWidth*1.0 + 6.0
	if maxNS >= 6 {
		nsHalf = laneWidth*float64(maxNS)/2 + 12.0
	} else if maxNS >= 4 {
		nsHalf = laneWidth*2.0 + 12.0
	}
	return ewHalf, nsHalf
}

func (w *World) RebuildIntersections() {
	w.Intersections = make([]Vec2, 0)
	var ewRoads []RoadSegment
	var nsRoads []RoadSegment

	for _, r := range w.Roads {
		if math.Abs(r.P1.Y-r.P2.Y) < 1.0 {
			ewRoads = append(ewRoads, r)
		} else if math.Abs(r.P1.X-r.P2.X) < 1.0 {
			nsRoads = append(nsRoads, r)
		}
	}

	seen := make(map[[2]int]bool)
	for _, ew := range ewRoads {
		minX := math.Min(ew.P1.X, ew.P2.X)
		maxX := math.Max(ew.P1.X, ew.P2.X)
		y := ew.P1.Y

		for _, ns := range nsRoads {
			minY := math.Min(ns.P1.Y, ns.P2.Y)
			maxY := math.Max(ns.P1.Y, ns.P2.Y)
			x := ns.P1.X

			if x >= minX-5 && x <= maxX+5 && y >= minY-5 && y <= maxY+5 {
				pt := Vec2{X: x, Y: y}
				if w.IsOverpassAt(pt) {
					continue
				}
				key := [2]int{int(math.Round(x)), int(math.Round(y))}
				if !seen[key] {
					seen[key] = true
					w.Intersections = append(w.Intersections, pt)
				}
			}
		}
	}
}

func (w *World) WaterBodies() []WaterBody {
	return w.Waters
}

// UpdateLaneWidth updates all bus stop positions dynamically when lane width changes
func (w *World) UpdateLaneWidth(laneWidth float64) {
	w.MatchStopRoadLanes(laneWidth)
	for _, s := range w.Stops {
		s.UpdateGeometry(laneWidth)
	}
}

func (w *World) Update(dt float64, route ...*Route) {
	var r *Route
	if len(route) > 0 {
		r = route[0]
	}
	for _, s := range w.Stops {
		// Only bus stops on the planned route generate passengers.
		// Stops not on the route must not spawn passengers and remain at 0 waiting.
		if r != nil && !r.ContainsStop(s.ID) {
			s.WaitingPassengers = 0
			s.SpawnTimer = 0
			continue
		}
		s.Update(dt)
	}
}
