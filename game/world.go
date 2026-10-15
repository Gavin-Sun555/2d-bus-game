package game

import (
	"image/color"
	"math"
	"sort"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
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
	if r.Lanes >= 6 || r.Type == RoadExpressway6Lane {
		return float64(r.Lanes)*laneWidth + 24.0 // 6 lanes + central median
	}
	if r.Lanes == 4 || r.Type == RoadAvenue4Lane {
		return float64(r.Lanes)*laneWidth + 24.0 // 4 lanes + central median
	}
	return float64(r.Lanes)*laneWidth + 12.0 // 2 lanes + median divider
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
				if r.Lanes > 0 {
					return r.Lanes
				}
				if r.Type == RoadExpressway6Lane {
					return 6
				}
				if r.Type == RoadAvenue4Lane {
					return 4
				}
				return 2
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
	w.computeIntersections()
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
		ewHalf = laneWidth*3.0 + 12.0
	} else if maxEW >= 4 {
		ewHalf = laneWidth*2.0 + 12.0
	}
	nsHalf = laneWidth*1.0 + 6.0
	if maxNS >= 6 {
		nsHalf = laneWidth*3.0 + 12.0
	} else if maxNS >= 4 {
		nsHalf = laneWidth*2.0 + 12.0
	}
	return ewHalf, nsHalf
}

func (w *World) computeIntersections() {
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

// 1. 都会核心 (Metropolis CBD)
func (w *World) buildMap1Metropolis(laneWidth float64) {
	// Avenues: 4-lane Ring & Grid, with 6-lane North Ring Expressway
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: 2600, Y: -1200}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "北环六车道快速路"},
		RoadSegment{P1: Vec2{X: -2600, Y: -600}, P2: Vec2{X: 2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北二路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 0}, P2: Vec2{X: 2600, Y: 0}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "中央主干大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南二路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "南环快速路"},

		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "西客运大道"},
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西公园大道"},
		RoadSegment{P1: Vec2{X: 0, Y: -1200}, P2: Vec2{X: 0, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "城市中轴大道"},
		RoadSegment{P1: Vec2{X: 1300, Y: -1200}, P2: Vec2{X: 1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "东市民大道"},
		RoadSegment{P1: Vec2{X: 2600, Y: -1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "东高新大道"},
	)

	// Central Canal & Bridges
	w.Rivers = append(w.Rivers, River{Y: -300, Height: 60})
	w.Waters = append(w.Waters, WaterBody{
		Center:  Vec2{X: 0, Y: -300},
		Width:   8000,
		Height:  60,
		Color:   RGBA(14, 116, 144, 210),
		Shimmer: RGBA(103, 232, 249, 180),
		Name:    "都会中央运河",
	})
	w.Bridges = append(w.Bridges,
		Bridge{Pos: Vec2{X: -1300, Y: -300}, Heading: math.Pi / 2, Length: 120, Width: 80, Name: "西运河大桥"},
		Bridge{Pos: Vec2{X: 0, Y: -300}, Heading: math.Pi / 2, Length: 140, Width: 130, Name: "中轴运河双子桥"},
		Bridge{Pos: Vec2{X: 1300, Y: -300}, Heading: math.Pi / 2, Length: 120, Width: 80, Name: "东市民运河桥"},
	)

	// Central Civic Roundabout
	w.Roundabouts = append(w.Roundabouts, Roundabout{
		Center:      Vec2{X: 0, Y: 0},
		Radius:      240.0,
		InnerRadius: 120.0,
		Name:        "CBD中央市政环岛广场",
		Type:        RoundaboutCivic,
	})

	w.buildMetropolisStops(laneWidth)
	w.buildMetropolisBuildings()
	w.buildMetropolisGreenery()
}

// 2. 海滨港湾 (Coastal Bay)
func (w *World) buildMap2CoastalBay(laneWidth float64) {
	// Ocean Bay & Beach
	w.Waters = append(w.Waters, WaterBody{
		IsOcean:  true,
		HasBeach: true,
		Center:   Vec2{X: 1800, Y: 0},
		Width:    3200,
		Height:   3200,
		Color:    RGBA(3, 105, 161, 230),
		Shimmer:  RGBA(186, 230, 253, 180),
		Name:     "蔚蓝海湾 (Coastal Ocean Bay)",
	})

	// Mainland Grid (X in [-2600, 0])
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: 0, Y: -1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北海滨大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: -600}, P2: Vec2{X: 0, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "渔港观光大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: 0}, P2: Vec2{X: 0, Y: 0}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "港城中央六车道大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: 0, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "滨海度假路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 1200}, P2: Vec2{X: 0, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "南湾快速大道"},

		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -2600, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西渔港路"},
		RoadSegment{P1: Vec2{X: -1950, Y: -1200}, P2: Vec2{X: -1950, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西海岸中路"},
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "港城中路"},
		RoadSegment{P1: Vec2{X: -650, Y: -1200}, P2: Vec2{X: -650, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南湾滨海辅道"},
		RoadSegment{P1: Vec2{X: 0, Y: -1200}, P2: Vec2{X: 0, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "沿海海景大道"},
	)

	// Ocean Causeway & Offshore Deepwater Port Island (X in [0, 2600])
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: 0, Y: 0}, P2: Vec2{X: 2600, Y: 0}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "跨海湾长堤六车道大桥大道"},
		RoadSegment{P1: Vec2{X: 2600, Y: -600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "深水码头港区大道"},
		RoadSegment{P1: Vec2{X: 1800, Y: -600}, P2: Vec2{X: 2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "港口北防波堤路"},
		RoadSegment{P1: Vec2{X: 1800, Y: 600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "港口南集装箱路"},
		RoadSegment{P1: Vec2{X: 1800, Y: -600}, P2: Vec2{X: 1800, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "离岸保税环线"},

		// South Cape Connecting Lines
		RoadSegment{P1: Vec2{X: 0, Y: 1200}, P2: Vec2{X: 1800, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南湾海角栈道"},
	)

	// Grand Sea Bridge
	w.Bridges = append(w.Bridges,
		Bridge{Pos: Vec2{X: 900, Y: 0}, Heading: 0, Length: 1200, Width: 140, Name: "跨海湾特大悬索桥", IsCable: true},
	)

	// Lighthouse Roundabout at (-1300, 600)
	w.Roundabouts = append(w.Roundabouts, Roundabout{
		Center:      Vec2{X: -1300, Y: 600},
		Radius:      225.0,
		InnerRadius: 110.0,
		Name:        "海滨灯塔环岛",
		Type:        RoundaboutLighthouse,
	})

	w.buildCoastalBayStops(laneWidth)
	w.buildCoastalBayBuildings()
	w.buildCoastalBayGreenery()
}

// 3. 溪谷江城 (River Valley)
func (w *World) buildMap3RiverValley(laneWidth float64) {
	// Grand Natural River (360px wide, Center Y = 0)
	w.Rivers = append(w.Rivers, River{Y: 0, Height: 360})
	w.Waters = append(w.Waters, WaterBody{
		Center:  Vec2{X: 0, Y: 0},
		Width:   8000,
		Height:  360,
		Color:   RGBA(15, 118, 110, 225),
		Shimmer: RGBA(153, 246, 228, 170),
		Name:    "长江大峡谷水系",
	})

	// North Bank Roads (Y <= -600, spacious 600px grid)
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: 2600, Y: -1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "北山生态大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: -600}, P2: Vec2{X: 2600, Y: -600}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "北滨江观景大道"},

		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "翠微山路"},
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: -1300, Y: -600}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "西大桥北引道"},
		RoadSegment{P1: Vec2{X: 0, Y: -1200}, P2: Vec2{X: 0, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "江北中轴大道"},
		RoadSegment{P1: Vec2{X: 1300, Y: -1200}, P2: Vec2{X: 1300, Y: -600}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "东大桥北引道"},
		RoadSegment{P1: Vec2{X: 2600, Y: -1200}, P2: Vec2{X: 2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "观音山大道"},
	)

	// River Crossing Bridges (Spanning across the river corridor between Y=-600 and Y=600)
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -1300, Y: -600}, P2: Vec2{X: -1300, Y: 600}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "溪谷斜拉长江大桥"},
		RoadSegment{P1: Vec2{X: 1300, Y: -600}, P2: Vec2{X: 1300, Y: 600}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "世纪悬索长江六车道大桥"},
	)
	w.Bridges = append(w.Bridges,
		Bridge{Pos: Vec2{X: -1300, Y: 0}, Heading: math.Pi / 2, Length: 600, Width: 95, Name: "溪谷斜拉长江大桥", IsCable: true},
		Bridge{Pos: Vec2{X: 1300, Y: 0}, Heading: math.Pi / 2, Length: 600, Width: 140, Name: "世纪悬索长江大桥", IsCable: true},
	)

	// South Bank Roads (Y >= 600, spacious 600px grid)
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "南滨江商务大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: 1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "高铁新城六车道大道"},

		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: -2600, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "创智南路"},
		RoadSegment{P1: Vec2{X: -1300, Y: 600}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "西大桥南引道"},
		RoadSegment{P1: Vec2{X: 0, Y: 600}, P2: Vec2{X: 0, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "江南中轴大道"},
		RoadSegment{P1: Vec2{X: 1300, Y: 600}, P2: Vec2{X: 1300, Y: 1200}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "东大桥南引道"},
		RoadSegment{P1: Vec2{X: 2600, Y: 600}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "产业园区路"},
	)

	w.buildRiverValleyStops(laneWidth)
	w.buildRiverValleyBuildings()
	w.buildRiverValleyGreenery()
}

// 4. 湖岛水乡 (Emerald Lake)
func (w *World) buildMap4EmeraldLake(laneWidth float64) {
	// Massive Emerald Lake
	w.Waters = append(w.Waters, WaterBody{
		IsLake:  true,
		Center:  Vec2{X: 0, Y: 0},
		Width:   3200,
		Height:  1800,
		Color:   RGBA(4, 120, 87, 230),
		Shimmer: RGBA(167, 243, 208, 160),
		Name:    "天然翠湖 (Emerald Lake)",
	})

	// Outer Lake Ring Highway (Spacious 600px grid)
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: 2600, Y: -1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "翠湖北环大道"},
		RoadSegment{P1: Vec2{X: -2600, Y: -600}, P2: Vec2{X: 2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北岸滨湖观景路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南岸滨湖观景路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "翠湖南环大道"},

		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "环湖西大道"},
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西岸水乡景观路"},
		RoadSegment{P1: Vec2{X: 1300, Y: -1200}, P2: Vec2{X: 1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "东岸名胜景观路"},
		RoadSegment{P1: Vec2{X: 2600, Y: -1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "环湖东大道"},
	)

	// Four Directional Causeway Bridges into the Central Lake Island
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: 0, Y: -1200}, P2: Vec2{X: 0, Y: 0}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "翠湖北堤公路"},
		RoadSegment{P1: Vec2{X: 0, Y: 0}, P2: Vec2{X: 0, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "翠湖南堤公路"},
		RoadSegment{P1: Vec2{X: -2600, Y: 0}, P2: Vec2{X: 0, Y: 0}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "翠湖西堤公路"},
		RoadSegment{P1: Vec2{X: 0, Y: 0}, P2: Vec2{X: 2600, Y: 0}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "翠湖东堤公路"},
	)

	// Stone Arch Bridges
	w.Bridges = append(w.Bridges,
		Bridge{Pos: Vec2{X: 0, Y: -750}, Heading: math.Pi / 2, Length: 500, Width: 80, Name: "翠湖北堤长桥", IsArch: true},
		Bridge{Pos: Vec2{X: 0, Y: 750}, Heading: math.Pi / 2, Length: 500, Width: 80, Name: "翠湖南堤长桥", IsArch: true},
		Bridge{Pos: Vec2{X: -1550, Y: 0}, Heading: 0, Length: 600, Width: 80, Name: "翠湖西堤长桥", IsArch: true},
		Bridge{Pos: Vec2{X: 1550, Y: 0}, Heading: 0, Length: 600, Width: 80, Name: "翠湖东堤长桥", IsArch: true},
	)

	// Lake Island Roundabout at (0, 0)
	w.Roundabouts = append(w.Roundabouts, Roundabout{
		Center:      Vec2{X: 0, Y: 0},
		Radius:      220.0,
		InnerRadius: 105.0,
		Name:        "湖心仙岛古风环岛",
		Type:        RoundaboutIsland,
	})

	w.buildEmeraldLakeStops(laneWidth)
	w.buildEmeraldLakeBuildings()
	w.buildEmeraldLakeGreenery()
}

// 5. 高新双城 (Twin Districts & Interchange Expressway)
func (w *World) buildMap5TwinCities(laneWidth float64) {
	// Central Ecological Buffer Greenbelt & Canal
	w.Waters = append(w.Waters, WaterBody{
		Center:  Vec2{X: 0, Y: 0},
		Width:   180,
		Height:  3000,
		Color:   RGBA(14, 116, 144, 210),
		Shimmer: RGBA(103, 232, 249, 180),
		Name:    "双城生态隔离水道",
	})

	// West District (Logistics & Airport: X in [-2600, -1300])
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -1300, Y: -1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "临空货运北线"},
		RoadSegment{P1: Vec2{X: -2600, Y: -600}, P2: Vec2{X: -1300, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "保税物流北街"},
		RoadSegment{P1: Vec2{X: -2600, Y: 600}, P2: Vec2{X: -1300, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "智造研发南街"},
		RoadSegment{P1: Vec2{X: -2600, Y: 1200}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "智造货运南线"},

		RoadSegment{P1: Vec2{X: -2600, Y: -1200}, P2: Vec2{X: -2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "临空国际大道"},
		RoadSegment{P1: Vec2{X: -1950, Y: -1200}, P2: Vec2{X: -1950, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "航站楼环线中路"},
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: -1300, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "智能制造中路"},
	)

	// East District (Silicon Valley & Software Park: X in [1300, 2600])
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: 1300, Y: -1200}, P2: Vec2{X: 2600, Y: -1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "硅谷创智北街"},
		RoadSegment{P1: Vec2{X: 1300, Y: -600}, P2: Vec2{X: 2600, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "人工智能研发街"},
		RoadSegment{P1: Vec2{X: 1300, Y: 600}, P2: Vec2{X: 2600, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "独角兽总部基地街"},
		RoadSegment{P1: Vec2{X: 1300, Y: 1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "大学科技南街"},

		RoadSegment{P1: Vec2{X: 1300, Y: -1200}, P2: Vec2{X: 1300, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "软件大道"},
		RoadSegment{P1: Vec2{X: 1950, Y: -1200}, P2: Vec2{X: 1950, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "未来科技中轴线"},
		RoadSegment{P1: Vec2{X: 2600, Y: -1200}, P2: Vec2{X: 2600, Y: 1200}, Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "未来科技大道"},
	)

	// Central Connectors & Interchange
	w.Roads = append(w.Roads,
		RoadSegment{P1: Vec2{X: -1300, Y: -1200}, P2: Vec2{X: 1300, Y: -1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北双城干道"},
		RoadSegment{P1: Vec2{X: -1300, Y: -600}, P2: Vec2{X: 1300, Y: -600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北立交联络线"},
		RoadSegment{P1: Vec2{X: -1300, Y: 600}, P2: Vec2{X: 1300, Y: 600}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南立交联络线"},
		RoadSegment{P1: Vec2{X: -1300, Y: 1200}, P2: Vec2{X: 1300, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南双城干道"},

		// Central Cross-District Expressway Viaduct (Continuously spans across the entire map at Y = 0!)
		RoadSegment{P1: Vec2{X: -2600, Y: 0}, P2: Vec2{X: 2600, Y: 0}, Lanes: 6, Type: RoadExpressway6Lane, HasLines: true, Name: "中央跨区高架六车道快速路"},

		// Ground-Level Central Interchange Connectors flanking the central canal park
		RoadSegment{P1: Vec2{X: -650, Y: -1200}, P2: Vec2{X: -650, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西互通立体匝道"},
		RoadSegment{P1: Vec2{X: 650, Y: -1200}, P2: Vec2{X: 650, Y: 1200}, Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "东互通立体匝道"},
	)

	// Grade-Separated Highway Overpass Viaduct at (0, 0) spanning across the canal
	w.Bridges = append(w.Bridges,
		Bridge{Pos: Vec2{X: 0, Y: 0}, Heading: 0, Length: 360, Width: 352, Name: "双城枢纽跨线立交桥", IsOverpass: true},
	)

	w.buildTwinCitiesStops(laneWidth)
	w.buildTwinCitiesBuildings()
	w.buildTwinCitiesGreenery()
}

// 6. 自定义地图 (Custom Map)
func (w *World) buildMap6Custom(laneWidth float64) {
	if w.CustomMap != nil {
		ApplyCustomMapToWorld(w, w.CustomMap, laneWidth)
		return
	}
	data, err := LoadCustomMapFromFile(DefaultCustomMapFileName)
	if err != nil || data == nil {
		data = GetDefaultCustomMapData()
	}
	w.CustomMap = data
	ApplyCustomMapToWorld(w, data, laneWidth)
}

// --- Map-Specific Stops Builders ---

func (w *World) buildMetropolisStops(laneWidth float64) {
	names := [12]string{
		"中央枢纽 (Central Terminal)",
		"国际金融中心 (IFC Tower)",
		"市民环岛广场 (Civic Plaza)",
		"科技创新园区 (Tech Campus)",
		"奥体中心 (Olympic Stadium)",
		"高铁北站 (North Railway Hub)",
		"西运河客运港 (Canal Port)",
		"大学城园区 (University Park)",
		"市立第一医院 (City Hospital)",
		"运河生态湿地 (Eco Wetlands)",
		"文创艺术街区 (Creative District)",
		"国际会展中心 (Expo Center)",
	}
	accents := [12]color.RGBA{
		RGBA(59, 130, 246, 255), RGBA(16, 185, 129, 255), RGBA(168, 85, 247, 255),
		RGBA(14, 165, 233, 255), RGBA(236, 72, 153, 255), RGBA(245, 158, 11, 255),
		RGBA(6, 182, 212, 255), RGBA(139, 92, 246, 255), RGBA(239, 68, 68, 255),
		RGBA(34, 197, 94, 255), RGBA(249, 115, 22, 255), RGBA(217, 70, 239, 255),
	}
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -650, Y: 0}, 0, 4, accents[0], laneWidth, 12),
		NewBusStop(1, names[1], Vec2{X: 650, Y: 0}, math.Pi, 4, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: 1300, Y: -300}, math.Pi/2, 2, accents[2], laneWidth, 0),
		NewBusStop(3, names[3], Vec2{X: 2600, Y: -900}, -math.Pi/2, 4, accents[3], laneWidth, 0),
		NewBusStop(4, names[4], Vec2{X: 1950, Y: -1200}, math.Pi, 6, accents[4], laneWidth, 8),
		NewBusStop(5, names[5], Vec2{X: -650, Y: -1200}, 0, 6, accents[5], laneWidth, 16),
		NewBusStop(6, names[6], Vec2{X: -2600, Y: -900}, math.Pi/2, 4, accents[6], laneWidth, 6),
		NewBusStop(7, names[7], Vec2{X: -2600, Y: 900}, -math.Pi/2, 4, accents[7], laneWidth, 10),
		NewBusStop(8, names[8], Vec2{X: -1300, Y: 300}, -math.Pi/2, 2, accents[8], laneWidth, 0),
		NewBusStop(9, names[9], Vec2{X: -1950, Y: 1200}, 0, 4, accents[9], laneWidth, 0),
		NewBusStop(10, names[10], Vec2{X: 650, Y: 1200}, math.Pi, 4, accents[10], laneWidth, 0),
		NewBusStop(11, names[11], Vec2{X: 2600, Y: 900}, math.Pi/2, 4, accents[11], laneWidth, 4),
	)
}

func (w *World) buildCoastalBayStops(laneWidth float64) {
	names := [12]string{
		"西渔港客运站 (West Harbor)",
		"港城中央广场 (Civic Plaza)",
		"北海滨浴场站 (North Beach)",
		"金色沙滩度假区 (Golden Beach)",
		"跨海大桥西岸站 (Bay Bridge West)",
		"东海深水码头总站 (Deepwater Port)",
		"保税港集装箱区 (Bonded Port)",
		"游艇国际俱乐部 (Yacht Club)",
		"渔人码头海鲜街 (Fisherman's Wharf)",
		"滨海帆船游艇港 (Marina Yacht Docks)",
		"深水港智能集装箱总站 (Container Hub)",
		"海角海景灯塔观景台 (Cape Lighthouse)",
	}
	accents := [12]color.RGBA{
		RGBA(14, 165, 233, 255), RGBA(59, 130, 246, 255), RGBA(6, 182, 212, 255),
		RGBA(245, 158, 11, 255), RGBA(239, 68, 68, 255), RGBA(168, 85, 247, 255),
		RGBA(16, 185, 129, 255), RGBA(236, 72, 153, 255),
		RGBA(245, 158, 11, 255), RGBA(14, 165, 233, 255), RGBA(168, 85, 247, 255), RGBA(239, 68, 68, 255),
	}
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -1625, Y: 0}, 0, 6, accents[0], laneWidth, 10),
		NewBusStop(1, names[1], Vec2{X: -325, Y: 0}, 0, 6, accents[1], laneWidth, 12),
		NewBusStop(2, names[2], Vec2{X: -1625, Y: -1200}, 0, 2, accents[2], laneWidth, 6),
		NewBusStop(3, names[3], Vec2{X: 0, Y: -300}, math.Pi/2, 4, accents[3], laneWidth, 8),
		NewBusStop(4, names[4], Vec2{X: 0, Y: 300}, -math.Pi/2, 4, accents[4], laneWidth, 10),
		NewBusStop(5, names[5], Vec2{X: 2600, Y: -300}, -math.Pi/2, 4, accents[5], laneWidth, 15),
		NewBusStop(6, names[6], Vec2{X: 2600, Y: 300}, math.Pi/2, 4, accents[6], laneWidth, 8),
		NewBusStop(7, names[7], Vec2{X: -975, Y: 600}, 0, 2, accents[7], laneWidth, 6),
		NewBusStop(8, names[8], Vec2{X: -1625, Y: -600}, 0, 2, accents[8], laneWidth, 10),
		NewBusStop(9, names[9], Vec2{X: -1300, Y: -300}, math.Pi/2, 2, accents[9], laneWidth, 8),
		NewBusStop(10, names[10], Vec2{X: -1950, Y: 300}, -math.Pi/2, 2, accents[10], laneWidth, 12),
		NewBusStop(11, names[11], Vec2{X: 900, Y: 1200}, 0, 2, accents[11], laneWidth, 8),
	)
}

func (w *World) buildRiverValleyStops(laneWidth float64) {
	names := [12]string{
		"北山森林公园站 (North Forest Park)",
		"溪谷江北观景台 (North Valley View)",
		"北滨江生态街 (Riverfront North)",
		"溪谷斜拉大桥北站 (Cable Bridge North)",
		"高铁新城枢纽总站 (High-Speed Rail Hub)",
		"南滨江金融中心 (Financial Riverfront)",
		"世纪悬索大桥南站 (Suspension Bridge South)",
		"江南智造产业园 (South Industrial Park)",
		"江北中心大道站 (North Center Ave)",
		"北岸半山森林观景台 (Mountain Lookout)",
		"南滨江观澜大道站 (Riverfront Vista)",
		"高铁总部大道站 (HSR Headquarters)",
	}
	accents := [12]color.RGBA{
		RGBA(16, 185, 129, 255), RGBA(14, 165, 233, 255), RGBA(6, 182, 212, 255),
		RGBA(245, 158, 11, 255), RGBA(59, 130, 246, 255), RGBA(168, 85, 247, 255),
		RGBA(239, 68, 68, 255), RGBA(234, 179, 8, 255),
		RGBA(34, 197, 94, 255), RGBA(16, 185, 129, 255), RGBA(14, 165, 233, 255), RGBA(168, 85, 247, 255),
	}
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -650, Y: -1200}, 0, 4, accents[0], laneWidth, 12),
		NewBusStop(1, names[1], Vec2{X: 650, Y: -1200}, math.Pi, 4, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: -1950, Y: -600}, 0, 4, accents[2], laneWidth, 6),
		NewBusStop(3, names[3], Vec2{X: -1300, Y: -300}, math.Pi/2, 4, accents[3], laneWidth, 10),
		NewBusStop(4, names[4], Vec2{X: -650, Y: 1200}, math.Pi, 6, accents[4], laneWidth, 16),
		NewBusStop(5, names[5], Vec2{X: -650, Y: 600}, 0, 4, accents[5], laneWidth, 10),
		NewBusStop(6, names[6], Vec2{X: 1300, Y: -150}, -math.Pi/2, 6, accents[6], laneWidth, 10),
		NewBusStop(7, names[7], Vec2{X: 650, Y: 600}, 0, 4, accents[7], laneWidth, 8),
		NewBusStop(8, names[8], Vec2{X: 0, Y: -900}, math.Pi/2, 2, accents[8], laneWidth, 10),
		NewBusStop(9, names[9], Vec2{X: 650, Y: -600}, math.Pi, 4, accents[9], laneWidth, 8),
		NewBusStop(10, names[10], Vec2{X: 1950, Y: 600}, math.Pi, 4, accents[10], laneWidth, 10),
		NewBusStop(11, names[11], Vec2{X: 650, Y: 1200}, 0, 6, accents[11], laneWidth, 10),
	)
}

func (w *World) buildEmeraldLakeStops(laneWidth float64) {
	names := [12]string{
		"湖心仙岛古风总站 (Lake Island Hub)",
		"翠湖北堤石桥站 (North Causeway Bridge)",
		"环湖北岸商业街 (North Lake Commercial)",
		"环湖东路古镇站 (East Ancient Town)",
		"翠湖南堤石桥站 (South Causeway Bridge)",
		"水岸非遗博览园 (Heritage Expo)",
		"环湖西岸湿地公园 (West Wetlands)",
		"西岸水乡景观站 (West Waters Town)",
		"北岸滨湖观景路 (North Lake Scenic)",
		"翠湖东堤石桥站 (East Causeway Bridge)",
		"南岸滨湖观景站 (South Lake Vista)",
		"东岸名胜景观站 (East Scenic Way)",
	}
	accents := [12]color.RGBA{
		RGBA(34, 197, 94, 255), RGBA(14, 165, 233, 255), RGBA(245, 158, 11, 255),
		RGBA(168, 85, 247, 255), RGBA(59, 130, 246, 255), RGBA(236, 72, 153, 255),
		RGBA(16, 185, 129, 255), RGBA(239, 68, 68, 255),
		RGBA(34, 197, 94, 255), RGBA(245, 158, 11, 255), RGBA(16, 185, 129, 255), RGBA(239, 68, 68, 255),
	}
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -975, Y: 0}, 0, 2, accents[0], laneWidth, 14),
		NewBusStop(1, names[1], Vec2{X: 0, Y: -900}, math.Pi/2, 2, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: 650, Y: -1200}, math.Pi, 4, accents[2], laneWidth, 10),
		NewBusStop(3, names[3], Vec2{X: 2600, Y: -300}, math.Pi/2, 4, accents[3], laneWidth, 12),
		NewBusStop(4, names[4], Vec2{X: 0, Y: 900}, -math.Pi/2, 2, accents[4], laneWidth, 8),
		NewBusStop(5, names[5], Vec2{X: -650, Y: 1200}, 0, 4, accents[5], laneWidth, 10),
		NewBusStop(6, names[6], Vec2{X: -2600, Y: 300}, -math.Pi/2, 4, accents[6], laneWidth, 8),
		NewBusStop(7, names[7], Vec2{X: -1950, Y: -1200}, 0, 4, accents[7], laneWidth, 6),
		NewBusStop(8, names[8], Vec2{X: -1950, Y: -600}, 0, 2, accents[8], laneWidth, 8),
		NewBusStop(9, names[9], Vec2{X: 1950, Y: 0}, 0, 2, accents[9], laneWidth, 10),
		NewBusStop(10, names[10], Vec2{X: -1950, Y: 600}, math.Pi, 2, accents[10], laneWidth, 8),
		NewBusStop(11, names[11], Vec2{X: 1300, Y: -300}, -math.Pi/2, 2, accents[11], laneWidth, 10),
	)
}

func (w *World) buildTwinCitiesStops(laneWidth float64) {
	names := [12]string{
		"临空国际物流港总站 (Airport Logistics Hub)",
		"智能制造产业园站 (Smart Factory Campus)",
		"工业中心西站 (West Industrial Center)",
		"枢纽立交西互通站 (Interchange West Gate)",
		"软件研发创新中心 (Software Innovation Hub)",
		"硅谷大学科技园站 (Silicon Valley University)",
		"智慧生态新城站 (Smart City Center)",
		"枢纽立交东互通站 (Interchange East Gate)",
		"临空国际机场 T1/T2航站楼 (Airport Terminal)",
		"新能源智造超级工厂 (Giga Factory Campus)",
		"人工智能未来科学城 (AI Science City)",
		"独角兽企业总部基地 (Unicorn Headquarters)",
	}
	accents := [12]color.RGBA{
		RGBA(139, 92, 246, 255), RGBA(59, 130, 246, 255), RGBA(14, 165, 233, 255),
		RGBA(245, 158, 11, 255), RGBA(168, 85, 247, 255), RGBA(16, 185, 129, 255),
		RGBA(236, 72, 153, 255), RGBA(239, 68, 68, 255),
		RGBA(139, 92, 246, 255), RGBA(59, 130, 246, 255), RGBA(14, 165, 233, 255), RGBA(236, 72, 153, 255),
	}
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -2275, Y: 0}, 0, 6, accents[0], laneWidth, 12),
		NewBusStop(1, names[1], Vec2{X: -2275, Y: -1200}, 0, 2, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: -1300, Y: 300}, math.Pi/2, 4, accents[2], laneWidth, 10),
		NewBusStop(3, names[3], Vec2{X: -975, Y: 0}, 0, 6, accents[3], laneWidth, 10),
		NewBusStop(4, names[4], Vec2{X: 975, Y: 0}, 0, 6, accents[4], laneWidth, 10),
		NewBusStop(5, names[5], Vec2{X: 2275, Y: -1200}, 0, 2, accents[5], laneWidth, 8),
		NewBusStop(6, names[6], Vec2{X: 2275, Y: 0}, 0, 6, accents[6], laneWidth, 14),
		NewBusStop(7, names[7], Vec2{X: 1300, Y: 300}, -math.Pi/2, 4, accents[7], laneWidth, 8),
		NewBusStop(8, names[8], Vec2{X: -1625, Y: -600}, 0, 2, accents[8], laneWidth, 14),
		NewBusStop(9, names[9], Vec2{X: -1625, Y: 600}, math.Pi, 2, accents[9], laneWidth, 10),
		NewBusStop(10, names[10], Vec2{X: 1625, Y: -600}, 0, 2, accents[10], laneWidth, 12),
		NewBusStop(11, names[11], Vec2{X: 1625, Y: 600}, math.Pi, 2, accents[11], laneWidth, 10),
	)
}

// --- Map-Specific Buildings & Greenery Builders ---

func (w *World) buildMetropolisBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		{cx: 650, cy: 300, w: 340, h: 160, col: RGBA(15, 23, 42, 255), roof: RGBA(30, 41, 59, 255)},
		{cx: -650, cy: 300, w: 320, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1720, cy: -810, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: 650, cy: -820, w: 360, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(236, 72, 153, 255)},
		{cx: -650, cy: -820, w: 360, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -1950, cy: -820, w: 360, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: -1950, cy: 850, w: 380, h: 160, col: RGBA(39, 39, 42, 255), roof: RGBA(63, 63, 70, 255)},
		{cx: -1950, cy: 300, w: 340, h: 150, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: -650, cy: 850, w: 340, h: 160, col: RGBA(248, 250, 252, 255), roof: RGBA(239, 68, 68, 255)},
		{cx: 1950, cy: 850, w: 380, h: 160, col: RGBA(24, 24, 27, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1950, cy: 300, w: 360, h: 150, col: RGBA(15, 23, 42, 255), roof: RGBA(217, 70, 239, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildMetropolisGreenery() {
	treeColor := RGBA(22, 101, 52, 255)
	for x := -1150.0; x <= -150.0; x += 100.0 {
		for y := 720.0; y <= 1080.0; y += 90.0 {
			w.Trees = append(w.Trees, Tree{Pos: Vec2{X: x, Y: y}, Radius: 16, Color: treeColor})
		}
	}
	for _, x := range []float64{-1625, -975, 325, 975, 1625} {
		for _, y := range []float64{-1050, -450, 450, 1050} {
			w.Trees = append(w.Trees,
				Tree{Pos: Vec2{X: x - 120, Y: y}, Radius: 14, Color: treeColor},
				Tree{Pos: Vec2{X: x + 120, Y: y}, Radius: 14, Color: treeColor},
			)
		}
	}
}

func (w *World) buildCoastalBayBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// Mainland Coast Blocks (Centered in 600px tall blocks)
		{cx: -1625, cy: -900, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: -900, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: -325, cy: -900, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(100, 116, 139, 255)},
		{cx: -1625, cy: -300, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(217, 119, 6, 255)},
		{cx: -975, cy: -300, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(6, 182, 212, 255)},
		{cx: -325, cy: -300, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: -1625, cy: 300, w: 280, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: 300, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(6, 182, 212, 255)},
		{cx: -325, cy: 300, w: 260, h: 140, col: RGBA(241, 245, 249, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -1625, cy: 900, w: 260, h: 140, col: RGBA(254, 243, 199, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -975, cy: 900, w: 260, h: 140, col: RGBA(224, 242, 254, 255), roof: RGBA(16, 185, 129, 255)},
		// Island Port Buildings
		{cx: 2200, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: 2200, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(245, 158, 11, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildCoastalBayGreenery() {
	palmColor := RGBA(16, 185, 129, 255)
	for _, x := range []float64{-1625, -975, -325} {
		for _, y := range []float64{-900, -300, 300, 900} {
			w.Trees = append(w.Trees,
				Tree{Pos: Vec2{X: x - 180, Y: y}, Radius: 15, Color: palmColor},
				Tree{Pos: Vec2{X: x + 180, Y: y}, Radius: 15, Color: palmColor},
			)
		}
	}
}

func (w *World) buildRiverValleyBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// North Bank Foothill Town (Centered in 1300px x 600px blocks at Y = -900)
		{cx: -1950, cy: -950, w: 260, h: 120, col: RGBA(241, 245, 249, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: -650, cy: -850, w: 240, h: 120, col: RGBA(248, 250, 252, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: 650, cy: -900, w: 240, h: 120, col: RGBA(241, 245, 249, 255), roof: RGBA(51, 65, 85, 255)},
		{cx: 1950, cy: -900, w: 260, h: 120, col: RGBA(248, 250, 252, 255), roof: RGBA(71, 85, 105, 255)},
		// South Bank Railway City (Centered in 1300px x 600px blocks at Y = 900)
		{cx: -1950, cy: 900, w: 260, h: 120, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -650, cy: 900, w: 240, h: 120, col: RGBA(15, 23, 42, 255), roof: RGBA(30, 41, 59, 255)},
		{cx: 650, cy: 900, w: 240, h: 120, col: RGBA(30, 41, 59, 255), roof: RGBA(168, 85, 247, 255)},
		{cx: 1950, cy: 900, w: 260, h: 120, col: RGBA(15, 23, 42, 255), roof: RGBA(234, 179, 8, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildRiverValleyGreenery() {
	pineColor := RGBA(5, 150, 105, 255)
	for _, x := range []float64{-1950, -650, 650, 1950} {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: x - 180, Y: -900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x + 180, Y: -900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x - 180, Y: 900}, Radius: 16, Color: pineColor},
			Tree{Pos: Vec2{X: x + 180, Y: 900}, Radius: 16, Color: pineColor},
		)
	}
}

func (w *World) buildEmeraldLakeBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// Outer Shore Courtyards
		{cx: -1950, cy: -900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: 1950, cy: -900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: -1950, cy: 900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		{cx: 1950, cy: 900, w: 280, h: 100, col: RGBA(248, 250, 252, 255), roof: RGBA(39, 39, 42, 255)},
		// Lake Island Pavilions on Shore
		{cx: -320, cy: -180, w: 140, h: 80, col: RGBA(254, 242, 242, 255), roof: RGBA(180, 83, 9, 255)},
		{cx: 320, cy: 180, w: 140, h: 80, col: RGBA(254, 242, 242, 255), roof: RGBA(180, 83, 9, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildEmeraldLakeGreenery() {
	willowColor := RGBA(34, 197, 94, 255)
	for _, x := range []float64{-320, 320} {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: x, Y: -180}, Radius: 14, Color: willowColor},
			Tree{Pos: Vec2{X: x, Y: 180}, Radius: 14, Color: willowColor},
		)
	}
}

func (w *World) buildTwinCitiesBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// West District: Logistics Hangars & Factories
		{cx: -1625, cy: -300, w: 260, h: 140, col: RGBA(51, 65, 85, 255), roof: RGBA(71, 85, 105, 255)},
		{cx: -2275, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: -1625, cy: 300, w: 260, h: 140, col: RGBA(51, 65, 85, 255), roof: RGBA(100, 116, 139, 255)},
		{cx: -2275, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(245, 158, 11, 255)},
		// East District: Software Towers & Campuses
		{cx: 1625, cy: -300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(139, 92, 246, 255)},
		{cx: 2275, cy: -300, w: 260, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(14, 165, 233, 255)},
		{cx: 1625, cy: 300, w: 260, h: 140, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},
		{cx: 2275, cy: 300, w: 260, h: 140, col: RGBA(15, 23, 42, 255), roof: RGBA(236, 72, 153, 255)},
	}
	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos: Vec2{X: b.cx, Y: b.cy}, W: b.w, H: b.h, Color: b.col, Roof: b.roof,
		})
	}
}

func (w *World) buildTwinCitiesGreenery() {
	parkColor := RGBA(16, 185, 129, 255)
	for y := -900.0; y <= 900.0; y += 180.0 {
		w.Trees = append(w.Trees,
			Tree{Pos: Vec2{X: -250, Y: y}, Radius: 15, Color: parkColor},
			Tree{Pos: Vec2{X: 250, Y: y}, Radius: 15, Color: parkColor},
		)
	}
}

// CheckBoundaries checks if a position is on the road, on the shoulder, or hitting a hard barrier
func (w *World) CheckBoundaries(pos Vec2, busRadius float64, laneWidth float64) BoundaryCheckResult {
	// 1. Check decorative Building collisions (Hard Barrier)
	for _, b := range w.Buildings {
		hw := b.W/2 + busRadius
		hh := b.H/2 + busRadius
		if math.Abs(pos.X-b.Pos.X) < hw && math.Abs(pos.Y-b.Pos.Y) < hh {
			// Collision with building
			dx := pos.X - b.Pos.X
			dy := pos.Y - b.Pos.Y
			var push Vec2
			if math.Abs(dx)/hw > math.Abs(dy)/hh {
				if dx > 0 {
					push = Vec2{X: 1, Y: 0}
				} else {
					push = Vec2{X: -1, Y: 0}
				}
			} else {
				if dy > 0 {
					push = Vec2{X: 0, Y: 1}
				} else {
					push = Vec2{X: 0, Y: -1}
				}
			}
			return BoundaryCheckResult{
				Zone:         ZoneHardBarrier,
				Pushback:     push,
				Penetration:  10.0,
				ObstacleName: "建筑墙体硬阻隔",
			}
		}
	}

	// 2. Check Outer Metropolitan Perimeter Wall (Hard Barrier)
	boundX := 3200.0 * MapGridScale
	boundY := 1480.0 * MapGridScale
	if pos.X > boundX {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: -1, Y: 0}, Penetration: pos.X - boundX, ObstacleName: "东侧边界隔离墙"}
	}
	if pos.X < -boundX {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 1, Y: 0}, Penetration: -boundX - pos.X, ObstacleName: "西侧边界隔离墙"}
	}
	if pos.Y > boundY {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: -1}, Penetration: pos.Y - boundY, ObstacleName: "南侧边界隔离墙"}
	}
	if pos.Y < -boundY {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: 1}, Penetration: -boundY - pos.Y, ObstacleName: "北侧边界隔离墙"}
	}

	// 2.5 Check Roundabout Road Corridors and Central Islands FIRST
	for _, rb := range w.Roundabouts {
		d := pos.Distance(rb.Center)
		if d < rb.InnerRadius-15.0 {
			dir := pos.Sub(rb.Center).Normalize()
			if dir.Length() == 0 {
				dir = Vec2{X: 1, Y: 0}
			}
			return BoundaryCheckResult{
				Zone:         ZoneHardBarrier,
				Pushback:     dir,
				Penetration:  (rb.InnerRadius - 15.0) - d + 5.0,
				ObstacleName: "环岛中央景观安全岛",
			}
		}
		if d <= rb.Radius+15.0 {
			return BoundaryCheckResult{Zone: ZoneRoad}
		}
	}

	// 2.7 Check Intersection Turning Clearances:
	// Provide generous turning clearance at all intersections of the active map so long buses can comfortably turn
	interTurnClearance := math.Max(260.0, laneWidth*4.2+55.0)
	for _, ipt := range w.Intersections {
		dx := pos.X - ipt.X
		dy := pos.Y - ipt.Y
		if dx*dx+dy*dy < interTurnClearance*interTurnClearance {
			return BoundaryCheckResult{Zone: ZoneRoad}
		}
	}

	// 2.8 Check Bus Stop Bays, Platforms, and Approach/Exit Transition Tapers:
	// Bus stops feature indented turnout bays with lead-in deceleration and lead-out merge tapers.
	// Ensure that vehicles pulling into or out of bus bays navigate smoothly without hitting phantom roadside barriers.
	for _, st := range w.Stops {
		cosH := math.Cos(st.Heading)
		sinH := math.Sin(st.Heading)
		dx := pos.X - st.Pos.X
		dy := pos.Y - st.Pos.Y

		// Longitudinal offset along direction of travel
		dLong := dx*cosH + dy*sinH
		// Lateral offset perpendicular to travel (positive towards sidewalk/shelter, negative towards road)
		dLat := -dx*sinH + dy*cosH

		// 1. Shelter physical structure collision check (located on sidewalk at dLat = 36.0, length 68, width 18)
		if dLat > st.Width*0.5 {
			shelterHW := 34.0 + 8.0 // half-length 34 + buffer
			if math.Abs(dLong) < shelterHW && dLat >= 24.0 && dLat <= 48.0 {
				pushX := sinH
				pushY := -cosH
				return BoundaryCheckResult{
					Zone:         ZoneHardBarrier,
					Pushback:     Vec2{X: pushX, Y: pushY},
					Penetration:  dLat - 24.0,
					ObstacleName: "公交候车亭立柱玻璃顶棚",
				}
			}
		}

		// 2. Bus Bay & Transit Corridor Envelope:
		// Bay box length is st.Length (140 px, dLong in [-70, +70]).
		// Approach queue corridor: dLong in [-1200, -70] (safely accommodates multi-bus fleet departure queue up to 8 buses).
		// Departure taper: dLong in [+70, +180] (110 px smooth exit corridor).
		if dLong >= -1200.0 && dLong <= 180.0 {
			curbDist := (float64(st.RoadLanes)/2.0)*laneWidth + st.Width*0.48
			// Only apply to the bus stop side of the road
			if dLat >= -curbDist-15.0 {
				mainRoadCurbLat := -(st.Width * 0.48)
				bayOuterCurbLat := st.Width*0.5 + 12.0

				var curbLat float64
				if dLong < -70.0 {
					// Approach queuing corridor: maintain full outer bay envelope clearance for queued AI fleet
					curbLat = bayOuterCurbLat
				} else if dLong > 70.0 {
					// Departure transition taper
					t := (180.0 - dLong) / 110.0
					curbLat = mainRoadCurbLat + t*(bayOuterCurbLat-mainRoadCurbLat)
				} else {
					// Directly inside bus stop bay box
					curbLat = bayOuterCurbLat
				}

				if dLat <= curbLat {
					return BoundaryCheckResult{Zone: ZoneRoad}
				}
			}
		}
	}

	// 3. Check Road and Shoulder distances across all straight, curved, and rotary road segments
	minDistBeyondCurb := math.MaxFloat64
	var closestPush Vec2

	for _, r := range w.Roads {
		roadW := r.GetWidth(laneWidth)
		roadHW := roadW / 2

		d, push := distToSegment(pos, r.P1, r.P2)
		dBeyond := d - roadHW
		if dBeyond < minDistBeyondCurb {
			minDistBeyondCurb = dBeyond
			closestPush = push
		}
	}

	// Decision based on minimum distance beyond curb
	if minDistBeyondCurb <= 0 {
		// Fully on asphalt road
		return BoundaryCheckResult{Zone: ZoneRoad}
	} else if minDistBeyondCurb <= ShoulderWidth {
		// On roadside shoulder/sidewalk (can drive, but slows down)
		return BoundaryCheckResult{
			Zone:        ZoneShoulder,
			Penetration: minDistBeyondCurb,
		}
	}

	// Beyond shoulder -> Hard barrier impact!
	return BoundaryCheckResult{
		Zone:         ZoneHardBarrier,
		Pushback:     closestPush,
		Penetration:  minDistBeyondCurb - ShoulderWidth,
		ObstacleName: "路侧硬阻隔防撞护栏",
	}
}

// distToSegment calculates the minimum Euclidean distance from point p to line segment ab,
// returning the distance and a unit pushback vector pointing from p towards the nearest point on ab.
func distToSegment(p, a, b Vec2) (float64, Vec2) {
	ab := b.Sub(a)
	l2 := ab.X*ab.X + ab.Y*ab.Y
	if l2 == 0 {
		d := p.Distance(a)
		if d == 0 {
			return 0, Vec2{X: 0, Y: 0}
		}
		return d, a.Sub(p).Mul(1.0 / d)
	}
	t := (p.Sub(a)).Dot(ab) / l2
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	proj := a.Add(ab.Mul(t))
	d := p.Distance(proj)
	if d == 0 {
		return 0, Vec2{X: 0, Y: 0}
	}
	push := proj.Sub(p).Mul(1.0 / d)
	return d, push
}

// UpdateLaneWidth updates all bus stop positions dynamically when lane width changes
func (w *World) UpdateLaneWidth(laneWidth float64) {
	w.MatchStopRoadLanes(laneWidth)
	for _, s := range w.Stops {
		s.UpdateGeometry(laneWidth)
	}
}

func (w *World) Update(dt float64) {
	for _, s := range w.Stops {
		s.Update(dt)
	}
}

func (w *World) Draw(screen *ebiten.Image, cam *Camera, targetStopID int, laneWidth float64, isPlanMode ...bool) {
	planMode := len(isPlanMode) > 0 && isPlanMode[0]

	// 1. Draw base ground with theme-tailored palette
	switch w.Theme {
	case MapThemeCoastalBay:
		screen.Fill(RGBA(24, 32, 47, 255)) // Warm maritime navy-slate
	case MapThemeEmeraldLake:
		screen.Fill(RGBA(17, 29, 24, 255)) // Verdant parkland slate-green
	default:
		screen.Fill(RGBA(15, 23, 42, 255)) // Dark metropolitan slate
	}

	// 2. Draw Scenic Waterbodies, Oceans, Lakes & Canals
	drawWaterBodies(screen, cam, w)
	for _, riv := range w.Rivers {
		drawRiver(screen, cam, riv)
	}

	// 3. Draw Road Network, Soft Shoulders & Hard Crash Barriers
	drawIntersectionPads(screen, cam, w, laneWidth, planMode)
	for _, r := range w.Roads {
		drawRoad(screen, cam, w, r, laneWidth, planMode)
	}
	drawIntersectionAsphalt(screen, cam, w, laneWidth)

	// 4. Draw Roundabouts (Rotary plazas and central monuments)
	drawRoundabouts(screen, cam, w)

	// 5. Draw Bridges (Railings, cable stays, stone arches, overpass viaducts)
	drawBridges(screen, cam, w, laneWidth)

	// 6. Draw Intersections & Crosswalks (skip in plan mode or when zoomed out)
	if !planMode && cam.Zoom >= 0.35 {
		drawCrosswalks(screen, cam, w, laneWidth)
	}

	// 7. Draw Bus Stops
	for _, s := range w.Stops {
		s.Draw(screen, cam, s.ID == targetStopID, false, planMode)
	}

	// 8. Draw Buildings & Shadows
	for _, b := range w.Buildings {
		drawBuilding(screen, cam, b, planMode)
	}

	// 9. Draw Trees
	for _, t := range w.Trees {
		drawTree(screen, cam, t, planMode)
	}
}

func drawWaterBodies(screen *ebiten.Image, cam *Camera, w *World) {
	for _, wb := range w.Waters {
		halfW := wb.Width / 2
		halfH := wb.Height / 2
		if !cam.IsBoxVisible(wb.Center.X-halfW, wb.Center.Y-halfH, wb.Center.X+halfW, wb.Center.Y+halfH) {
			continue
		}

		if wb.IsOcean {
			// Coastal Golden Beach Strip along west shore of ocean
			beachW := 90.0
			beachCenter := Vec2{X: wb.Center.X - halfW + beachW/2, Y: wb.Center.Y}
			drawRotatedBox(screen, cam, beachCenter, 1, 0, beachW, wb.Height, RGBA(217, 119, 6, 190))
			beachEdge := beachCenter.Add(Vec2{X: beachW / 2, Y: 0})
			drawRotatedBox(screen, cam, beachEdge, 1, 0, 16, wb.Height, RGBA(254, 240, 138, 160))
		}

		// Main water surface
		drawRotatedBox(screen, cam, wb.Center, 1, 0, wb.Width, wb.Height, wb.Color)

		// Water shimmer / wave lines
		if cam.Zoom >= 0.25 {
			s := cam.WorldToScreen(wb.Center)
			shimmerLen := float32(120 * cam.Zoom)
			vector.StrokeLine(screen, float32(s.X-60), float32(s.Y-40), float32(s.X-60)+shimmerLen, float32(s.Y-40), float32(1.8*cam.Zoom), wb.Shimmer, true)
			vector.StrokeLine(screen, float32(s.X+80), float32(s.Y+50), float32(s.X+80)+shimmerLen, float32(s.Y+50), float32(1.8*cam.Zoom), wb.Shimmer, true)
		}
	}
}

func drawRoundabouts(screen *ebiten.Image, cam *Camera, w *World) {
	for _, rb := range w.Roundabouts {
		if !cam.IsPointVisible(rb.Center, rb.Radius+80) {
			continue
		}

		cScreen := cam.WorldToScreen(rb.Center)
		rOuter := float32(rb.Radius * cam.Zoom)
		rInner := float32(rb.InnerRadius * cam.Zoom)
		if rInner < 3 {
			rInner = 3
		}

		// 1. Outer Slate Shoulder / Sidewalk Buffer Ring
		rShoulder := float32((rb.Radius + 18.0) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rShoulder, RGBA(51, 65, 85, 255), true)

		// 2. Circular Asphalt Rotary Carriageway
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rOuter, RGBA(30, 41, 59, 255), true)
		vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rOuter, float32(2.5*cam.Zoom), RGBA(148, 163, 184, 200), true)

		// 3. Mountable Truck Apron Ring (环岛卡车围裙) around the central island
		rApron := float32((rb.InnerRadius + 16.0) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rApron, RGBA(71, 85, 105, 240), true)
		vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rApron, float32(1.8*cam.Zoom), RGBA(100, 116, 139, 180), true)

		// 4. Circulating Lane Dividing Dashed Circle (in middle of carriageway)
		if cam.Zoom >= 0.25 {
			midR := (rb.InnerRadius + 16.0 + rb.Radius) * 0.5
			numDashes := 24
			dashStep := (2 * math.Pi) / float64(numDashes)
			dashLen := dashStep * 0.55
			dashW := float32(2.2 * cam.Zoom)
			dashCol := RGBA(248, 250, 252, 215)
			for i := 0; i < numDashes; i++ {
				th1 := float64(i) * dashStep
				th2 := th1 + dashLen
				p1 := rb.Center.Add(Vec2{X: math.Cos(th1) * midR, Y: math.Sin(th1) * midR})
				p2 := rb.Center.Add(Vec2{X: math.Cos(th2) * midR, Y: math.Sin(th2) * midR})
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), dashW, dashCol, true)
			}
		}

		// 5. Entry Yield Markings (让行虚线 & 让行倒三角) on incoming approaches
		if cam.Zoom >= 0.32 {
			drawRoundaboutYieldEntrances(screen, cam, rb)
		}

		// 6. Central Island & Themed Monuments
		switch rb.Type {
		case RoundaboutCivic:
			// Concentric plaza paving
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, RGBA(30, 41, 59, 255), true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.0*cam.Zoom), RGBA(203, 213, 225, 230), true)
			if rInner > 15 {
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner*0.75, float32(1.5*cam.Zoom), RGBA(71, 85, 105, 180), true)
				// Central water fountain
				fountainR := rInner * 0.45
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR, RGBA(14, 116, 144, 240), true)
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR, float32(2*cam.Zoom), RGBA(56, 189, 248, 220), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), fountainR*0.35, RGBA(255, 255, 255, 255), true)
			}

		case RoundaboutLighthouse:
			// Coastal green lawn island
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, RGBA(22, 101, 52, 255), true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.0*cam.Zoom), RGBA(203, 213, 225, 230), true)
			if rInner > 15 {
				// Historic Nautical Lighthouse tower
				towerR := rInner * 0.4
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR, RGBA(248, 250, 252, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.7, RGBA(239, 68, 68, 255), true)
				// Beacon lamp glow
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.35, RGBA(250, 204, 21, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), towerR*0.8, RGBA(254, 240, 138, 90), true)
			}

		case RoundaboutIsland:
			// Emerald water moat buffer & green island
			vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, RGBA(22, 101, 52, 255), true)
			vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), rInner, float32(3.5*cam.Zoom), RGBA(4, 120, 87, 230), true)
			if rInner > 15 {
				// Classical Chinese octagonal gazebo/pavilion roof
				pavR := rInner * 0.42
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR, RGBA(217, 119, 6, 255), true)
				vector.StrokeCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR, float32(2*cam.Zoom), RGBA(180, 83, 9, 255), true)
				vector.DrawFilledCircle(screen, float32(cScreen.X), float32(cScreen.Y), pavR*0.28, RGBA(250, 204, 21, 255), true)
			}
		}
	}
}

func drawRoundaboutYieldEntrances(screen *ebiten.Image, cam *Camera, rb Roundabout) {
	entryOffset := (rb.InnerRadius + rb.Radius) * 0.28
	entryWidth := (rb.Radius - rb.InnerRadius) * 0.72
	entries := []struct {
		pos   Vec2
		dir   Vec2
		width float64
	}{
		// West entry: incoming from -X into roundabout (+X direction)
		{pos: Vec2{X: rb.Center.X - rb.Radius, Y: rb.Center.Y + entryOffset}, dir: Vec2{X: 1, Y: 0}, width: entryWidth},
		// East entry: incoming from +X into roundabout (-X direction)
		{pos: Vec2{X: rb.Center.X + rb.Radius, Y: rb.Center.Y - entryOffset}, dir: Vec2{X: -1, Y: 0}, width: entryWidth},
		// North entry: incoming from -Y into roundabout (+Y direction)
		{pos: Vec2{X: rb.Center.X - entryOffset, Y: rb.Center.Y - rb.Radius}, dir: Vec2{X: 0, Y: 1}, width: entryWidth},
		// South entry: incoming from +Y into roundabout (-Y direction)
		{pos: Vec2{X: rb.Center.X + entryOffset, Y: rb.Center.Y + rb.Radius}, dir: Vec2{X: 0, Y: -1}, width: entryWidth},
	}

	for _, e := range entries {
		perp := Vec2{X: -e.dir.Y, Y: e.dir.X}
		halfW := e.width * 0.5
		pA := e.pos.Add(perp.Mul(halfW))
		pB := e.pos.Sub(perp.Mul(halfW))

		// Draw dash segments across entry
		for i := 0; i < 4; i++ {
			t1 := float64(i) / 4.0
			t2 := t1 + 0.16
			pt1 := pA.Add(pB.Sub(pA).Mul(t1))
			pt2 := pA.Add(pB.Sub(pA).Mul(t2))
			s1 := cam.WorldToScreen(pt1)
			s2 := cam.WorldToScreen(pt2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
				float32(3.0*cam.Zoom), RGBA(248, 250, 252, 220), true)
		}

		// Yield Triangle (▽) pointing towards the roundabout
		triTip := e.pos.Sub(e.dir.Mul(14.0))
		triBaseCenter := e.pos.Sub(e.dir.Mul(34.0))
		triL := triBaseCenter.Add(perp.Mul(9.0))
		triR := triBaseCenter.Sub(perp.Mul(9.0))

		sTip := cam.WorldToScreen(triTip)
		sL := cam.WorldToScreen(triL)
		sR := cam.WorldToScreen(triR)
		vector.StrokeLine(screen, float32(sTip.X), float32(sTip.Y), float32(sL.X), float32(sL.Y), float32(2.0*cam.Zoom), RGBA(248, 250, 252, 210), true)
		vector.StrokeLine(screen, float32(sTip.X), float32(sTip.Y), float32(sR.X), float32(sR.Y), float32(2.0*cam.Zoom), RGBA(248, 250, 252, 210), true)
		vector.StrokeLine(screen, float32(sL.X), float32(sL.Y), float32(sR.X), float32(sR.Y), float32(2.0*cam.Zoom), RGBA(248, 250, 252, 210), true)
	}
}

func drawBridges(screen *ebiten.Image, cam *Camera, w *World, laneWidth float64) {
	for _, br := range w.Bridges {
		rad := math.Max(br.Length, br.Width) * 0.65
		if !cam.IsPointVisible(br.Pos, rad) {
			continue
		}

		cosH := math.Cos(br.Heading)
		sinH := math.Sin(br.Heading)

		// 1. Heavy reinforced bridge deck borders
		offset := br.Width / 2
		drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, RGBA(15, 23, 42, 255), 4)
		drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, RGBA(15, 23, 42, 255), 4)

		if br.IsOverpass {
			// Grade-separated Highway Overpass Viaduct (立体跨线立交桥)
			// 1. Sleek drop shadow along outer bottom edge outside the bridge deck
			drawParallelLine(screen, cam, br.Pos.Add(Vec2{X: 6, Y: 10}), cosH, sinH, br.Length, offset+8, RGBA(15, 23, 42, 150), 10)
			// 2. Sturdy reinforced support pier pillars on outer sides at canal banks
			pierOffset := br.Width/2 + 8
			pierW := 22.0
			pierL := 40.0
			drawRotatedBox(screen, cam, br.Pos.Add(Vec2{X: -br.Length * 0.35, Y: pierOffset}), cosH, sinH, pierL, pierW, RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(Vec2{X: -br.Length * 0.35, Y: -pierOffset}), cosH, sinH, pierL, pierW, RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(Vec2{X: br.Length * 0.35, Y: pierOffset}), cosH, sinH, pierL, pierW, RGBA(100, 116, 139, 255))
			drawRotatedBox(screen, cam, br.Pos.Add(Vec2{X: br.Length * 0.35, Y: -pierOffset}), cosH, sinH, pierL, pierW, RGBA(100, 116, 139, 255))
			// 3. Heavy reinforced concrete crash barriers with warning stripes
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, RGBA(226, 232, 240, 255), 4.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, RGBA(226, 232, 240, 255), 4.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset-4, RGBA(245, 158, 11, 230), 2.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset+4, RGBA(245, 158, 11, 230), 2.5)
		} else if br.IsCable {
			// Grand Bay Suspension Cable Bridge
			p1 := cam.WorldToScreen(br.Pos.Add(Vec2{X: -br.Length * 0.25, Y: 0}))
			p2 := cam.WorldToScreen(br.Pos.Add(Vec2{X: br.Length * 0.25, Y: 0}))
			towerH := float32(65 * cam.Zoom)
			towerW := float32(7 * cam.Zoom)

			// Pylon 1
			vector.StrokeLine(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(p1.X), float32(p1.Y+float64(towerH*0.3)), towerW, RGBA(148, 163, 184, 255), true)
			vector.DrawFilledCircle(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(4*cam.Zoom), RGBA(239, 68, 68, 255), true)

			// Pylon 2
			vector.StrokeLine(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(p2.X), float32(p2.Y+float64(towerH*0.3)), towerW, RGBA(148, 163, 184, 255), true)
			vector.DrawFilledCircle(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(4*cam.Zoom), RGBA(239, 68, 68, 255), true)

			// Suspension stay cables
			cableLw := float32(math.Max(1.0, 1.6*cam.Zoom))
			cableCol := RGBA(226, 232, 240, 170)
			for d := -br.Length * 0.45; d <= br.Length*0.45; d += br.Length * 0.15 {
				ptDeck := cam.WorldToScreen(br.Pos.Add(Vec2{X: d, Y: 0}))
				vector.StrokeLine(screen, float32(p1.X), float32(p1.Y-float64(towerH)), float32(ptDeck.X), float32(ptDeck.Y), cableLw, cableCol, true)
				vector.StrokeLine(screen, float32(p2.X), float32(p2.Y-float64(towerH)), float32(ptDeck.X), float32(ptDeck.Y), cableLw, cableCol, true)
			}
		} else if br.IsArch {
			// Classical Jiangnan Stone Arch Bridge Balustrades
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, RGBA(241, 245, 249, 230), 3)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, RGBA(241, 245, 249, 230), 3)
		} else {
			// Modern Twin Canal Bridges with yellow hazard warning markers
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, offset, RGBA(234, 179, 8, 230), 2.5)
			drawParallelLine(screen, cam, br.Pos, cosH, sinH, br.Length, -offset, RGBA(234, 179, 8, 230), 2.5)
		}
	}
}

func drawIntersectionPads(screen *ebiten.Image, cam *Camera, w *World, laneWidth float64, planMode bool) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		shoulderHalfX := nsHalf + ShoulderWidth + 14.0
		shoulderHalfY := ewHalf + ShoulderWidth + 14.0
		if !cam.IsPointVisible(c, math.Max(shoulderHalfX, shoulderHalfY)*1.5) {
			continue
		}
		// In plan mode, skip shoulder backing apron at intersections
		if !planMode {
			drawRotatedBox(screen, cam, c, 1, 0, shoulderHalfX*2, shoulderHalfY*2, RGBA(51, 65, 85, 255))
		}
	}
}

func drawIntersectionAsphalt(screen *ebiten.Image, cam *Camera, w *World, laneWidth float64) {
	for _, c := range w.Intersections {
		if w.IsAtRoundabout(c) {
			continue // Roundabouts render their own circular roadway, apron, and center plaza
		}
		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(c, laneWidth)
		padHalfX := nsHalf + 18.0
		padHalfY := ewHalf + 18.0
		if !cam.IsPointVisible(c, math.Max(padHalfX, padHalfY)*1.5) {
			continue
		}
		drawRotatedBox(screen, cam, c, 1, 0, padHalfX*2, padHalfY*2, RGBA(30, 41, 59, 255))

		// Draw smooth curved curb return fillets at all 4 corners of the intersection
		// so turning vehicles have broad, sweeping asphalt connecting the perpendicular roads
		curbR := 36.0 * cam.Zoom
		if curbR >= 2.0 {
			quads := [4][2]float64{
				{-padHalfX + 16.0, -padHalfY + 16.0},
				{padHalfX - 16.0, -padHalfY + 16.0},
				{-padHalfX + 16.0, padHalfY - 16.0},
				{padHalfX - 16.0, padHalfY - 16.0},
			}
			for _, q := range quads {
				cornerPt := cam.WorldToScreen(c.Add(Vec2{X: q[0], Y: q[1]}))
				vector.DrawFilledCircle(screen, float32(cornerPt.X), float32(cornerPt.Y), float32(curbR), RGBA(30, 41, 59, 255), true)
			}
		}
	}
}

func drawRiver(screen *ebiten.Image, cam *Camera, riv River) {
	if !cam.IsBoxVisible(-16000, riv.Y-riv.Height/2, 16000, riv.Y+riv.Height/2) {
		return
	}
	drawRotatedBox(screen, cam, Vec2{X: 0, Y: riv.Y}, 1, 0, 32000, riv.Height, RGBA(14, 116, 144, 210)) // Cyan-blue canal water
}

func drawRoad(screen *ebiten.Image, cam *Camera, w *World, r RoadSegment, laneWidth float64, planMode bool) {
	dir := r.P2.Sub(r.P1)
	len := dir.Length()
	if len == 0 {
		return
	}
	center := r.P1.Add(r.P2).Mul(0.5)
	roadW := r.GetWidth(laneWidth)

	// Frustum culling check for road segment
	boundRadius := len*0.5 + roadW*1.5 + ShoulderWidth*2
	if !cam.IsPointVisible(center, boundRadius) {
		return
	}

	heading := math.Atan2(dir.Y, dir.X)
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	// 1. Soft Shoulder (drivable buffer with deceleration, ShoulderWidth = 40px)
	if !planMode && cam.Zoom >= 0.35 {
		shoulderTotalW := roadW + ShoulderWidth*2
		drawRotatedBox(screen, cam, center, cosH, sinH, len, shoulderTotalW, RGBA(51, 65, 85, 255)) // Slate sidewalk / shoulder
	}

	// 2. Asphalt Road Surface (#1e293b)
	drawRotatedBox(screen, cam, center, cosH, sinH, len, roadW, RGBA(30, 41, 59, 255))

	// 3. Road Curb dividing lines & Hard Crash Barrier along outer edge of shoulder (skip in plan mode / low zoom)
	if !planMode && cam.Zoom >= 0.35 {
		curbOffset := roadW / 2
		drawParallelLine(screen, cam, center, cosH, sinH, len, curbOffset, RGBA(148, 163, 184, 180), 2)
		drawParallelLine(screen, cam, center, cosH, sinH, len, -curbOffset, RGBA(148, 163, 184, 180), 2)

		var splits []float64
		isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
		minX, maxX := math.Min(r.P1.X, r.P2.X), math.Max(r.P1.X, r.P2.X)
		minY, maxY := math.Min(r.P1.Y, r.P2.Y), math.Max(r.P1.Y, r.P2.Y)
		for _, pt := range w.Intersections {
			if isEW {
				if math.Abs(pt.Y-r.P1.Y) < 5.0 && pt.X > minX+5.0 && pt.X < maxX-5.0 {
					splits = append(splits, pt.X)
				}
			} else {
				if math.Abs(pt.X-r.P1.X) < 5.0 && pt.Y > minY+5.0 && pt.Y < maxY-5.0 {
					splits = append(splits, pt.Y)
				}
			}
		}
		sort.Float64s(splits)

		barrierOffset := roadW/2 + ShoulderWidth
		drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, barrierOffset, laneWidth, splits)
		drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, -barrierOffset, laneWidth, splits)
	}

	// 5. Center Dividers & Lane Markings
	if r.HasLines {
		if r.Lanes >= 6 || r.Type == RoadExpressway6Lane {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6.0, RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6.0, RGBA(250, 204, 21, 240))

			laneOffset1 := laneWidth * 1.0
			laneOffset2 := laneWidth * 2.0
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset2, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset2, planMode)
		} else if r.Lanes == 4 || r.Type == RoadAvenue4Lane {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6.0, RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6.0, RGBA(250, 204, 21, 240))

			laneOffset1 := laneWidth
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset1, planMode)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset1, planMode)
		} else {
			drawDashedYellowLine(screen, cam, center, cosH, sinH, len, planMode)
		}
	}
}

func drawParallelLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, offset float64, c color.RGBA, stroke float32) {
	p1 := center.Add(Vec2{X: -cosH*len/2 - sinH*offset, Y: -sinH*len/2 + cosH*offset})
	p2 := center.Add(Vec2{X: cosH*len/2 - sinH*offset, Y: sinH*len/2 + cosH*offset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), stroke*float32(cam.Zoom), c, true)
}

func drawHazardBarrier(screen *ebiten.Image, cam *Camera, r RoadSegment, center Vec2, cosH, sinH, len, offset, laneWidth float64, splits []float64) {
	isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0
	gapRadius := math.Max(245.0, laneWidth*4.2+50.0) // generous opening at all road intersections for comfortable turning

	if isEW {
		minX := math.Min(r.P1.X, r.P2.X)
		maxX := math.Max(r.P1.X, r.P2.X)
		prevX := minX
		for _, ix := range splits {
			segStart := prevX + gapRadius
			if prevX == minX {
				segStart = minX
			}
			segEnd := ix - gapRadius
			if segEnd > segStart {
				p1 := Vec2{X: segStart, Y: r.P1.Y + offset}
				p2 := Vec2{X: segEnd, Y: r.P1.Y + offset}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
			}
			prevX = ix
		}
		// Final segment to road end
		segStart := prevX + gapRadius
		if prevX == minX {
			segStart = minX
		}
		segEnd := maxX
		if segEnd > segStart {
			p1 := Vec2{X: segStart, Y: r.P1.Y + offset}
			p2 := Vec2{X: segEnd, Y: r.P1.Y + offset}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
		}
	} else {
		minY := math.Min(r.P1.Y, r.P2.Y)
		maxY := math.Max(r.P1.Y, r.P2.Y)
		prevY := minY
		for _, iy := range splits {
			segStart := prevY + gapRadius
			if prevY == minY {
				segStart = minY
			}
			segEnd := iy - gapRadius
			if segEnd > segStart {
				p1 := Vec2{X: r.P1.X - offset, Y: segStart}
				p2 := Vec2{X: r.P1.X - offset, Y: segEnd}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
			}
			prevY = iy
		}
		// Final segment to road end
		segStart := prevY + gapRadius
		if prevY == minY {
			segStart = minY
		}
		segEnd := maxY
		if segEnd > segStart {
			p1 := Vec2{X: r.P1.X - offset, Y: segStart}
			p2 := Vec2{X: r.P1.X - offset, Y: segEnd}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
		}
	}
}

func drawCenterDividers(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, perpOffset float64, c color.RGBA) {
	p1 := center.Add(Vec2{X: -cosH*len/2 - sinH*perpOffset, Y: -sinH*len/2 + cosH*perpOffset})
	p2 := center.Add(Vec2{X: cosH*len/2 - sinH*perpOffset, Y: sinH*len/2 + cosH*perpOffset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), c, true)
}

func drawDashedYellowLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len float64, planMode bool) {
	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Draw single continuous line to eliminate ~110 vector calls per road
		drawCenterDividers(screen, cam, center, cosH, sinH, len, 0, RGBA(250, 204, 21, 200))
		return
	}
	dashLen := 26.0
	gapLen := 20.0
	step := dashLen + gapLen
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(Vec2{X: cosH * dStart, Y: sinH * dStart})
		db := center.Add(Vec2{X: cosH * dEnd, Y: sinH * dEnd})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(3*cam.Zoom), RGBA(250, 204, 21, 230), true)
	}
}

func drawDashedWhiteLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, perpOffset float64, planMode bool) {
	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Draw single continuous line to eliminate ~220 vector calls per road
		drawCenterDividers(screen, cam, center, cosH, sinH, len, perpOffset, RGBA(241, 245, 249, 130))
		return
	}
	dashLen := 24.0
	gapLen := 22.0
	step := dashLen + gapLen
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(Vec2{X: cosH*dStart - sinH*perpOffset, Y: sinH*dStart + cosH*perpOffset})
		db := center.Add(Vec2{X: cosH*dEnd - sinH*perpOffset, Y: sinH*dEnd + cosH*perpOffset})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(2*cam.Zoom), RGBA(241, 245, 249, 180), true)
	}
}

func drawCrosswalks(screen *ebiten.Image, cam *Camera, w *World, laneWidth float64) {
	for _, pt := range w.Intersections {
		if w.IsAtRoundabout(pt) {
			continue // Roundabouts feature self-regulating circular rotary flow, not straight 4-way crosswalks or lane arrows
		}

		ewHalf, nsHalf := w.GetIntersectionRoadDimensions(pt, laneWidth)
		ewStopLineDist := nsHalf + 43.0
		ewCrosswalkDist := ewStopLineDist - 25.0
		nsStopLineDist := ewHalf + 43.0
		nsCrosswalkDist := nsStopLineDist - 25.0

		maxBound := math.Max(ewStopLineDist, nsStopLineDist) + 60.0
		if !cam.IsPointVisible(pt, maxBound) {
			continue
		}

		hasWest, westLanes := false, 0
		hasEast, eastLanes := false, 0
		hasNorth, northLanes := false, 0
		hasSouth, southLanes := false, 0

		for _, r := range w.Roads {
			if math.Abs(r.P1.Y-r.P2.Y) < 1.0 && math.Abs(r.P1.Y-pt.Y) < 10.0 {
				minX := math.Min(r.P1.X, r.P2.X)
				maxX := math.Max(r.P1.X, r.P2.X)
				if minX < pt.X-15.0 && maxX >= pt.X-15.0 {
					hasWest = true
					if r.Lanes > westLanes {
						westLanes = r.Lanes
					}
				}
				if minX <= pt.X+15.0 && maxX > pt.X+15.0 {
					hasEast = true
					if r.Lanes > eastLanes {
						eastLanes = r.Lanes
					}
				}
			}
			if math.Abs(r.P1.X-r.P2.X) < 1.0 && math.Abs(r.P1.X-pt.X) < 10.0 {
				minY := math.Min(r.P1.Y, r.P2.Y)
				maxY := math.Max(r.P1.Y, r.P2.Y)
				if minY < pt.Y-15.0 && maxY >= pt.Y-15.0 {
					hasNorth = true
					if r.Lanes > northLanes {
						northLanes = r.Lanes
					}
				}
				if minY <= pt.Y+15.0 && maxY > pt.Y+15.0 {
					hasSouth = true
					if r.Lanes > southLanes {
						southLanes = r.Lanes
					}
				}
			}
		}

		// 1. Draw Zebra Crossings around intersection for connected branches only
		if hasNorth {
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: 0, Y: -nsCrosswalkDist}), true, northLanes, laneWidth)
		}
		if hasSouth {
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: 0, Y: nsCrosswalkDist}), true, southLanes, laneWidth)
		}
		if hasWest {
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: -ewCrosswalkDist, Y: 0}), false, westLanes, laneWidth)
		}
		if hasEast {
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: ewCrosswalkDist, Y: 0}), false, eastLanes, laneWidth)
		}

		// 2. Draw dedicated turning lane markings & pavement arrows for multi-lane avenues
		if westLanes >= 6 {
			// West approach: 6-lane express avenue (Lane 0 left-turn, Lane 1 through, Lane 2 through/turn)
			wCenter := pt.X - ewStopLineDist
			wLane0Y := pt.Y + 4 + laneWidth*0.5
			wLane1Y := pt.Y + 4 + laneWidth*1.5
			wLane2Y := pt.Y + 4 + laneWidth*2.5
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wLane0Y}, 0, true)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wLane0Y}, 0, true)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wLane1Y}, 0, false)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wLane1Y}, 0, false)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wLane2Y}, 0, false)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wLane2Y}, 0, false)
			drawSolidWhiteDivider(screen, cam, Vec2{X: wCenter - 60, Y: pt.Y + 4 + laneWidth*1.0}, 1, 0, 120)
			drawSolidWhiteDivider(screen, cam, Vec2{X: wCenter - 60, Y: pt.Y + 4 + laneWidth*2.0}, 1, 0, 120)
		} else if westLanes >= 4 {
			wCenter := pt.X - ewStopLineDist
			wInnerY := pt.Y + 4 + laneWidth*0.5
			wOuterY := pt.Y + 4 + laneWidth*1.5
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wInnerY}, 0, true)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wInnerY}, 0, true)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wOuterY}, 0, false)
			drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wOuterY}, 0, false)
			wLineY := pt.Y + 4 + laneWidth
			drawSolidWhiteDivider(screen, cam, Vec2{X: wCenter - 60, Y: wLineY}, 1, 0, 120)
		}

		if eastLanes >= 6 {
			// East approach: 6-lane express avenue
			eCenter := pt.X + ewStopLineDist
			eLane0Y := pt.Y - 4 - laneWidth*0.5
			eLane1Y := pt.Y - 4 - laneWidth*1.5
			eLane2Y := pt.Y - 4 - laneWidth*2.5
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eLane0Y}, math.Pi, true)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eLane0Y}, math.Pi, true)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eLane1Y}, math.Pi, false)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eLane1Y}, math.Pi, false)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eLane2Y}, math.Pi, false)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eLane2Y}, math.Pi, false)
			drawSolidWhiteDivider(screen, cam, Vec2{X: eCenter + 60, Y: pt.Y - 4 - laneWidth*1.0}, -1, 0, 120)
			drawSolidWhiteDivider(screen, cam, Vec2{X: eCenter + 60, Y: pt.Y - 4 - laneWidth*2.0}, -1, 0, 120)
		} else if eastLanes >= 4 {
			eCenter := pt.X + ewStopLineDist
			eInnerY := pt.Y - 4 - laneWidth*0.5
			eOuterY := pt.Y - 4 - laneWidth*1.5
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eInnerY}, math.Pi, true)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eInnerY}, math.Pi, true)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eOuterY}, math.Pi, false)
			drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eOuterY}, math.Pi, false)
			eLineY := pt.Y - 4 - laneWidth
			drawSolidWhiteDivider(screen, cam, Vec2{X: eCenter + 60, Y: eLineY}, -1, 0, 120)
		}

		if northLanes >= 6 {
			// North approach: 6-lane express avenue
			nCenter := pt.Y - nsStopLineDist
			nLane0X := pt.X - 4 - laneWidth*0.5
			nLane1X := pt.X - 4 - laneWidth*1.5
			nLane2X := pt.X - 4 - laneWidth*2.5
			drawGroundArrow(screen, cam, Vec2{X: nLane0X, Y: nCenter - 45}, math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: nLane0X, Y: nCenter - 110}, math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: nLane1X, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: nLane1X, Y: nCenter - 110}, math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: nLane2X, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: nLane2X, Y: nCenter - 110}, math.Pi/2, false)
			drawSolidWhiteDivider(screen, cam, Vec2{X: pt.X - 4 - laneWidth*1.0, Y: nCenter - 60}, 0, 1, 120)
			drawSolidWhiteDivider(screen, cam, Vec2{X: pt.X - 4 - laneWidth*2.0, Y: nCenter - 60}, 0, 1, 120)
		} else if northLanes >= 4 {
			nCenter := pt.Y - nsStopLineDist
			nInnerX := pt.X - 4 - laneWidth*0.5
			nOuterX := pt.X - 4 - laneWidth*1.5
			drawGroundArrow(screen, cam, Vec2{X: nInnerX, Y: nCenter - 45}, math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: nInnerX, Y: nCenter - 110}, math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: nOuterX, Y: nCenter - 45}, math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: nOuterX, Y: nCenter - 110}, math.Pi/2, false)
			nLineX := pt.X - 4 - laneWidth
			drawSolidWhiteDivider(screen, cam, Vec2{X: nLineX, Y: nCenter - 60}, 0, 1, 120)
		}

		if southLanes >= 6 {
			// South approach: 6-lane express avenue
			sCenter := pt.Y + nsStopLineDist
			sLane0X := pt.X + 4 + laneWidth*0.5
			sLane1X := pt.X + 4 + laneWidth*1.5
			sLane2X := pt.X + 4 + laneWidth*2.5
			drawGroundArrow(screen, cam, Vec2{X: sLane0X, Y: sCenter + 45}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: sLane0X, Y: sCenter + 110}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: sLane1X, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: sLane1X, Y: sCenter + 110}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: sLane2X, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: sLane2X, Y: sCenter + 110}, -math.Pi/2, false)
			drawSolidWhiteDivider(screen, cam, Vec2{X: pt.X + 4 + laneWidth*1.0, Y: sCenter + 60}, 0, -1, 120)
			drawSolidWhiteDivider(screen, cam, Vec2{X: pt.X + 4 + laneWidth*2.0, Y: sCenter + 60}, 0, -1, 120)
		} else if southLanes >= 4 {
			sCenter := pt.Y + nsStopLineDist
			sInnerX := pt.X + 4 + laneWidth*0.5
			sOuterX := pt.X + 4 + laneWidth*1.5
			drawGroundArrow(screen, cam, Vec2{X: sInnerX, Y: sCenter + 45}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: sInnerX, Y: sCenter + 110}, -math.Pi/2, true)
			drawGroundArrow(screen, cam, Vec2{X: sOuterX, Y: sCenter + 45}, -math.Pi/2, false)
			drawGroundArrow(screen, cam, Vec2{X: sOuterX, Y: sCenter + 110}, -math.Pi/2, false)
			sLineX := pt.X + 4 + laneWidth
			drawSolidWhiteDivider(screen, cam, Vec2{X: sLineX, Y: sCenter + 60}, 0, -1, 120)
		}
	}
}

func drawZebraBranch(screen *ebiten.Image, cam *Camera, center Vec2, horizontal bool, lanes int, laneWidth float64) {
	numStripes := 6
	step := 15.0
	if lanes >= 6 {
		numStripes = 18
	} else if lanes >= 4 {
		numStripes = 12
	}
	for i := -numStripes / 2; i <= numStripes/2; i++ {
		var p1, p2 Vec2
		if horizontal {
			offset := float64(i) * step
			p1 = center.Add(Vec2{X: offset, Y: -10})
			p2 = center.Add(Vec2{X: offset, Y: 10})
		} else {
			offset := float64(i) * step
			p1 = center.Add(Vec2{X: -10, Y: offset})
			p2 = center.Add(Vec2{X: 10, Y: offset})
		}
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(7.5*cam.Zoom), RGBA(241, 245, 249, 215), true)
	}
}

func drawSolidWhiteDivider(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, length float64) {
	halfLen := length * 0.5
	p1 := center.Add(Vec2{X: -cosH * halfLen, Y: -sinH * halfLen})
	p2 := center.Add(Vec2{X: cosH * halfLen, Y: sinH * halfLen})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), RGBA(248, 250, 252, 220), true)
}

func drawGroundArrow(screen *ebiten.Image, cam *Camera, pos Vec2, heading float64, isLeftTurn bool) {
	scale := float32(cam.Zoom)
	if scale < 0.28 {
		return
	}
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	fwd := Vec2{X: cosH, Y: sinH}
	left := Vec2{X: sinH, Y: -cosH} // In screen coords (+Y is down), driver's left is (sinH, -cosH)

	col := RGBA(248, 250, 252, 225)
	lw := float32(2.8 * scale)

	if !isLeftTurn {
		// Straight Arrow (↑)
		p1 := pos.Sub(fwd.Mul(14))
		p2 := pos.Add(fwd.Mul(14))
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lw, col, true)

		pl := pos.Add(fwd.Mul(6)).Add(left.Mul(6))
		pr := pos.Add(fwd.Mul(6)).Sub(left.Mul(6))
		sl := cam.WorldToScreen(pl)
		sr := cam.WorldToScreen(pr)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sl.X), float32(sl.Y), lw, col, true)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sr.X), float32(sr.Y), lw, col, true)
	} else {
		// Left-Turn Arrow (↰)
		p1 := pos.Sub(fwd.Mul(14))
		pCurve := pos.Add(fwd.Mul(4))
		s1 := cam.WorldToScreen(p1)
		sc := cam.WorldToScreen(pCurve)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(sc.X), float32(sc.Y), lw, col, true)

		pTip := pCurve.Add(left.Mul(15))
		stip := cam.WorldToScreen(pTip)
		vector.StrokeLine(screen, float32(sc.X), float32(sc.Y), float32(stip.X), float32(stip.Y), lw, col, true)

		pBarbT := pTip.Sub(left.Mul(5)).Add(fwd.Mul(4))
		pBarbB := pTip.Sub(left.Mul(5)).Sub(fwd.Mul(4))
		sbt := cam.WorldToScreen(pBarbT)
		sbb := cam.WorldToScreen(pBarbB)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbt.X), float32(sbt.Y), lw, col, true)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbb.X), float32(sbb.Y), lw, col, true)
	}
}

func drawBuilding(screen *ebiten.Image, cam *Camera, b Building, planMode bool) {
	if !cam.IsBoxVisible(b.Pos.X-b.W/2-20, b.Pos.Y-b.H/2-20, b.Pos.X+b.W/2+20, b.Pos.Y+b.H/2+20) {
		return
	}

	if !planMode {
		shadowOffset := Vec2{X: 12, Y: 16}
		drawRotatedBox(screen, cam, b.Pos.Add(shadowOffset), 1, 0, b.W, b.H, RGBA(2, 6, 23, 110))
	}
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, b.Color)

	roofPadding := 10.0
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-roofPadding, b.H-roofPadding, b.Roof)

	// Window lights: skip in plan mode or when zoomed out to eliminate ~400 vector circles
	if !planMode && cam.Zoom >= 0.45 {
		for wx := -b.W/2 + 25; wx < b.W/2-25; wx += 32 {
			for wy := -b.H/2 + 25; wy < b.H/2-25; wy += 32 {
				wp := b.Pos.Add(Vec2{X: wx, Y: wy})
				ws := cam.WorldToScreen(wp)
				vector.DrawFilledCircle(screen, float32(ws.X), float32(ws.Y), float32(3*cam.Zoom),
					RGBA(254, 240, 138, 160), true)
			}
		}
	}
}

func drawTree(screen *ebiten.Image, cam *Camera, t Tree, planMode bool) {
	if !cam.IsPointVisible(t.Pos, t.Radius+15) {
		return
	}

	ts := cam.WorldToScreen(t.Pos)
	r := float32(t.Radius * cam.Zoom)
	if r < 1.5 {
		r = 1.5
	}

	if planMode || cam.Zoom < 0.38 {
		// Low zoom / Plan mode: Single canopy circle eliminates 500+ vector calls
		vector.DrawFilledCircle(screen, float32(ts.X), float32(ts.Y), r, t.Color, true)
		return
	}

	vector.DrawFilledCircle(screen, float32(ts.X+float64(r*0.4)), float32(ts.Y+float64(r*0.4)), r, RGBA(2, 6, 23, 70), true)
	vector.DrawFilledCircle(screen, float32(ts.X), float32(ts.Y), r, t.Color, true)
	vector.DrawFilledCircle(screen, float32(ts.X-float64(r*0.2)), float32(ts.Y-float64(r*0.2)), r*0.5,
		RGBA(74, 222, 128, 140), true)
}
