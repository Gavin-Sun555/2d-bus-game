package game

import (
	"encoding/json"
	"fmt"
	"image/color"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// CustomRoadData represents a serializable road segment in JSON
type CustomRoadData struct {
	P1    [2]float64 `json:"p1"`
	P2    [2]float64 `json:"p2"`
	Lanes int        `json:"lanes"` // 2 or 4
	Name  string     `json:"name"`
}

// CustomStopData represents a serializable bus stop in JSON
type CustomStopData struct {
	ID       int        `json:"id"`
	Name     string     `json:"name"`
	Pos      [2]float64 `json:"pos"`
	Heading  float64    `json:"heading"` // in radians (e.g. 0=East, 1.57=South, 3.14=West, -1.57=North)
	Lanes    int        `json:"lanes"`   // 2 or 4
	Color    string     `json:"color"`   // Hex string like "#3B82F6"
	Capacity int        `json:"capacity"`
}

// CustomBridgeData represents a serializable bridge structure in JSON
type CustomBridgeData struct {
	Pos     [2]float64 `json:"pos"`
	Heading float64    `json:"heading"`
	Length  float64    `json:"length"`
	Width   float64    `json:"width"`
	Name    string     `json:"name"`
	Style   string     `json:"style"` // "standard", "cable", "arch", "overpass"
}

// CustomWaterData represents a serializable water body in JSON
type CustomWaterData struct {
	Name     string     `json:"name"`
	Center   [2]float64 `json:"center"`
	Width    float64    `json:"width"`
	Height   float64    `json:"height"`
	Color    string     `json:"color"`   // Hex string
	Shimmer  string     `json:"shimmer"` // Hex string
	IsOcean  bool       `json:"is_ocean,omitempty"`
	HasBeach bool       `json:"has_beach,omitempty"`
	IsLake   bool       `json:"is_lake,omitempty"`
}

// CustomRiverData represents a serializable river waterway in JSON
type CustomRiverData struct {
	Y      float64 `json:"y"`
	Height float64 `json:"height"`
}

// CustomRoundaboutData represents a serializable traffic roundabout in JSON
type CustomRoundaboutData struct {
	Name        string     `json:"name"`
	Center      [2]float64 `json:"center"`
	Radius      float64    `json:"radius"`
	InnerRadius float64    `json:"inner_radius"`
	Type        string     `json:"type"` // "civic", "lighthouse", "island"
}

// CustomBuildingData represents a decorative building in JSON
type CustomBuildingData struct {
	Name   string     `json:"name,omitempty"`
	Pos    [2]float64 `json:"pos"`
	Width  float64    `json:"width"`
	Height float64    `json:"height"`
	Color  string     `json:"color"`
	Roof   string     `json:"roof"`
}

// CustomTreeData represents greenery in JSON
type CustomTreeData struct {
	Pos    [2]float64 `json:"pos"`
	Radius float64    `json:"radius"`
	Color  string     `json:"color"`
}

// CustomMapData is the root structure for custom map files
type CustomMapData struct {
	Name        string                 `json:"name"`
	Subtitle    string                 `json:"subtitle"`
	Style       string                 `json:"style"` // "metropolis", "coastal", "river", "lake", "twincities"
	Roads       []CustomRoadData       `json:"roads"`
	Stops       []CustomStopData       `json:"stops"`
	Bridges     []CustomBridgeData     `json:"bridges,omitempty"`
	Waters      []CustomWaterData      `json:"waters,omitempty"`
	Rivers      []CustomRiverData      `json:"rivers,omitempty"`
	Roundabouts []CustomRoundaboutData `json:"roundabouts,omitempty"`
	Buildings   []CustomBuildingData   `json:"buildings,omitempty"`
	Trees       []CustomTreeData       `json:"trees,omitempty"`
}

const DefaultCustomMapFileName = "custom_map.json"

// parseHexColor converts a hex string (e.g. "#3B82F6", "#3B82F6FF", "3B82F6") to color.RGBA
func parseHexColor(s string, defaultColor color.RGBA) color.RGBA {
	s = strings.TrimPrefix(strings.TrimSpace(s), "#")
	if len(s) == 6 {
		v, err := strconv.ParseUint(s, 16, 32)
		if err == nil {
			return RGBA(uint8(v>>16), uint8((v>>8)&0xFF), uint8(v&0xFF), 255)
		}
	} else if len(s) == 8 {
		v, err := strconv.ParseUint(s, 16, 32)
		if err == nil {
			return RGBA(uint8(v>>24), uint8((v>>16)&0xFF), uint8((v>>8)&0xFF), uint8(v&0xFF))
		}
	}
	return defaultColor
}

// colorToHex converts a color.RGBA to hex string
func colorToHex(c color.RGBA) string {
	if c.A == 255 {
		return fmt.Sprintf("#%02X%02X%02X", c.R, c.G, c.B)
	}
	return fmt.Sprintf("#%02X%02X%02X%02X", c.R, c.G, c.B, c.A)
}

// GetDefaultCustomMapData returns the default starter custom map: "星空未来城 (Starfield Future City)"
func GetDefaultCustomMapData() *CustomMapData {
	return &CustomMapData{
		Name:     "星空未来城 (Starfield Future City)",
		Subtitle: "玩家自定义自建都市 · 星光高架与双子湖生态客运网",
		Style:    "metropolis",
		Roads: []CustomRoadData{
			// Major 4-lane Ring & Grid
			{P1: [2]float64{-2600, -1200}, P2: [2]float64{2600, -1200}, Lanes: 4, Name: "星光北环大道"},
			{P1: [2]float64{-2600, -600}, P2: [2]float64{2600, -600}, Lanes: 2, Name: "未来二路"},
			{P1: [2]float64{-2600, 0}, P2: [2]float64{2600, 0}, Lanes: 4, Name: "天际中央主轴"},
			{P1: [2]float64{-2600, 600}, P2: [2]float64{2600, 600}, Lanes: 2, Name: "创新二路"},
			{P1: [2]float64{-2600, 1200}, P2: [2]float64{2600, 1200}, Lanes: 4, Name: "星湖南环快速路"},

			{P1: [2]float64{-2600, -1200}, P2: [2]float64{-2600, 1200}, Lanes: 4, Name: "西空港大道"},
			{P1: [2]float64{-1300, -1200}, P2: [2]float64{-1300, 1200}, Lanes: 2, Name: "星云西路"},
			{P1: [2]float64{0, -1200}, P2: [2]float64{0, 1200}, Lanes: 4, Name: "核心中轴大道"},
			{P1: [2]float64{1300, -1200}, P2: [2]float64{1300, 1200}, Lanes: 2, Name: "量子东路"},
			{P1: [2]float64{2600, -1200}, P2: [2]float64{2600, 1200}, Lanes: 4, Name: "东超级港大道"},

			// Cross Diagonal Bypass Connectors
			{P1: [2]float64{-650, -1200}, P2: [2]float64{-650, 1200}, Lanes: 2, Name: "智慧生活西辅道"},
			{P1: [2]float64{650, -1200}, P2: [2]float64{650, 1200}, Lanes: 2, Name: "数字产业东辅道"},
		},
		Stops: []CustomStopData{
			{ID: 0, Name: "星光中央总站 (Starlight Hub)", Pos: [2]float64{-325, 0}, Heading: 0, Lanes: 4, Color: "#3B82F6", Capacity: 14},
			{ID: 1, Name: "未来科技塔 (Future Tower)", Pos: [2]float64{325, 0}, Heading: math.Pi, Lanes: 4, Color: "#10B981", Capacity: 10},
			{ID: 2, Name: "天际广场 (Skyline Plaza)", Pos: [2]float64{1300, -300}, Heading: math.Pi / 2, Lanes: 2, Color: "#A855F7", Capacity: 8},
			{ID: 3, Name: "量子实验室 (Quantum Lab)", Pos: [2]float64{2600, -900}, Heading: -math.Pi / 2, Lanes: 2, Color: "#06B6D4", Capacity: 8},
			{ID: 4, Name: "星海体育馆 (Star Arena)", Pos: [2]float64{1950, -1200}, Heading: math.Pi, Lanes: 4, Color: "#EC4899", Capacity: 10},
			{ID: 5, Name: "空港北枢纽 (North Airport)", Pos: [2]float64{-325, -1200}, Heading: 0, Lanes: 4, Color: "#F59E0B", Capacity: 14},
			{ID: 6, Name: "运河客运港 (Canal Port)", Pos: [2]float64{-2600, -900}, Heading: math.Pi / 2, Lanes: 2, Color: "#0EA5E9", Capacity: 8},
			{ID: 7, Name: "大学城东校区 (Uni Campus)", Pos: [2]float64{-2600, 900}, Heading: -math.Pi / 2, Lanes: 2, Color: "#8B5CF6", Capacity: 8},
			{ID: 8, Name: "第一市民医院 (General Hospital)", Pos: [2]float64{-1300, 300}, Heading: -math.Pi / 2, Lanes: 2, Color: "#EF4444", Capacity: 10},
			{ID: 9, Name: "星湖生态公园 (Star Lake Park)", Pos: [2]float64{-1950, 1200}, Heading: 0, Lanes: 4, Color: "#22C55E", Capacity: 8},
			{ID: 10, Name: "文创生活街区 (Creative Zone)", Pos: [2]float64{325, 1200}, Heading: math.Pi, Lanes: 4, Color: "#F97316", Capacity: 10},
			{ID: 11, Name: "国际会展总站 (Expo Center)", Pos: [2]float64{2600, 900}, Heading: math.Pi / 2, Lanes: 2, Color: "#D946EF", Capacity: 8},
		},
		Rivers: []CustomRiverData{
			{Y: -300, Height: 70},
		},
		Waters: []CustomWaterData{
			{
				Name:    "未来中央运河",
				Center:  [2]float64{0, -300},
				Width:   8000,
				Height:  70,
				Color:   "#0E7490D2",
				Shimmer: "#67E8F9B4",
			},
			{
				Name:    "翡翠生态明珠湖",
				Center:  [2]float64{-1950, 600},
				Width:   1100,
				Height:  600,
				Color:   "#047857E6",
				Shimmer: "#A7F3D0A0",
				IsLake:  true,
			},
		},
		Bridges: []CustomBridgeData{
			{Pos: [2]float64{-1300, -300}, Heading: math.Pi / 2, Length: 130, Width: 80, Name: "西运河大桥", Style: "standard"},
			{Pos: [2]float64{0, -300}, Heading: math.Pi / 2, Length: 150, Width: 130, Name: "天际星光悬索大桥", Style: "cable"},
			{Pos: [2]float64{1300, -300}, Heading: math.Pi / 2, Length: 130, Width: 80, Name: "东星云大桥", Style: "standard"},
		},
		Roundabouts: []CustomRoundaboutData{
			{Name: "星城中央市政环岛广场", Center: [2]float64{0, 0}, Radius: 220.0, InnerRadius: 105.0, Type: "civic"},
		},
		Buildings: []CustomBuildingData{
			{Name: "未来科技总部", Pos: [2]float64{975, 300}, Width: 340, Height: 160, Color: "#0F172A", Roof: "#1E293B"},
			{Name: "星空金融中心", Pos: [2]float64{-975, 300}, Width: 320, Height: 160, Color: "#1E293B", Roof: "#334155"},
			{Name: "量子云谷大厦", Pos: [2]float64{1720, -850}, Width: 260, Height: 140, Color: "#1E293B", Roof: "#3B82F6"},
			{Name: "超级智慧城", Pos: [2]float64{975, -900}, Width: 360, Height: 160, Color: "#1E293B", Roof: "#EC4899"},
			{Name: "创新梦工场", Pos: [2]float64{-975, -900}, Width: 360, Height: 160, Color: "#0F172A", Roof: "#F59E0B"},
			{Name: "星云科研园区", Pos: [2]float64{-1950, -900}, Width: 360, Height: 160, Color: "#1E293B", Roof: "#475569"},
			{Name: "市民体育中心", Pos: [2]float64{-975, 850}, Width: 340, Height: 160, Color: "#F8FAFC", Roof: "#EF4444"},
			{Name: "国际博览城", Pos: [2]float64{1950, 850}, Width: 380, Height: 160, Color: "#18181B", Roof: "#334155"},
		},
		Trees: []CustomTreeData{
			{Pos: [2]float64{-1150, 750}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{-950, 850}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{-750, 950}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{-550, 1050}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{1150, 750}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{1350, 850}, Radius: 16, Color: "#166534"},
			{Pos: [2]float64{1550, 950}, Radius: 16, Color: "#166534"},
		},
	}
}

// GetCustomMapPresets provides 3 alternative preset maps users can instantly switch between
func GetCustomMapPresets() []*CustomMapData {
	p1 := GetDefaultCustomMapData()

	p2 := &CustomMapData{
		Name:     "云海高山城 (Cloud Mountain Valley)",
		Subtitle: "半山观光盘山公路 · 峡谷飞渡特大桥 · 溪谷生态客运线",
		Style:    "river",
		Roads: []CustomRoadData{
			{P1: [2]float64{-2600, -1200}, P2: [2]float64{2600, -1200}, Lanes: 4, Name: "云峰北路"},
			{P1: [2]float64{-2600, -600}, P2: [2]float64{2600, -600}, Lanes: 2, Name: "半山云海大道"},
			{P1: [2]float64{-2600, 600}, P2: [2]float64{2600, 600}, Lanes: 2, Name: "南峡谷滨水路"},
			{P1: [2]float64{-2600, 1200}, P2: [2]float64{2600, 1200}, Lanes: 4, Name: "翠微南山大道"},

			{P1: [2]float64{-1800, -1200}, P2: [2]float64{-1800, 1200}, Lanes: 4, Name: "西大峡谷跨江大道"},
			{P1: [2]float64{0, -1200}, P2: [2]float64{0, 1200}, Lanes: 4, Name: "中央峡谷彩虹大道"},
			{P1: [2]float64{1800, -1200}, P2: [2]float64{1800, 1200}, Lanes: 4, Name: "东高山观光大道"},
		},
		Stops: []CustomStopData{
			{ID: 0, Name: "云顶山庄总站 (Cloud Mountain Peak)", Pos: [2]float64{-650, -1200}, Heading: 0, Lanes: 4, Color: "#10B981", Capacity: 12},
			{ID: 1, Name: "半山观云台 (Mountain Cloud Vista)", Pos: [2]float64{650, -600}, Heading: math.Pi, Lanes: 2, Color: "#06B6D4", Capacity: 8},
			{ID: 2, Name: "西峡谷跨江大桥站 (West Gorge Bridge)", Pos: [2]float64{-1800, -300}, Heading: math.Pi / 2, Lanes: 4, Color: "#3B82F6", Capacity: 10},
			{ID: 3, Name: "彩虹峡谷中央站 (Rainbow Canyon Hub)", Pos: [2]float64{0, 300}, Heading: math.Pi / 2, Lanes: 4, Color: "#F59E0B", Capacity: 12},
			{ID: 4, Name: "南岸瀑布胜境 (South Falls Resort)", Pos: [2]float64{650, 600}, Heading: 0, Lanes: 2, Color: "#EC4899", Capacity: 8},
			{ID: 5, Name: "翠微古镇南站 (Green Mountain Town)", Pos: [2]float64{-650, 1200}, Heading: math.Pi, Lanes: 4, Color: "#A855F7", Capacity: 10},
		},
		Rivers: []CustomRiverData{
			{Y: 0, Height: 320},
		},
		Waters: []CustomWaterData{
			{
				Name:    "翡翠大峡谷奔流",
				Center:  [2]float64{0, 0},
				Width:   8000,
				Height:  320,
				Color:   "#0F766EE1",
				Shimmer: "#99F6E4AA",
			},
		},
		Bridges: []CustomBridgeData{
			{Pos: [2]float64{-1800, 0}, Heading: math.Pi / 2, Length: 520, Width: 100, Name: "西峡谷斜拉特大桥", Style: "cable"},
			{Pos: [2]float64{0, 0}, Heading: math.Pi / 2, Length: 520, Width: 110, Name: "中央峡谷彩虹特大桥", Style: "cable"},
			{Pos: [2]float64{1800, 0}, Heading: math.Pi / 2, Length: 520, Width: 100, Name: "东峡谷高架大桥", Style: "cable"},
		},
		Roundabouts: []CustomRoundaboutData{
			{Name: "云峰观景环岛", Center: [2]float64{0, -1200}, Radius: 220.0, InnerRadius: 105.0, Type: "civic"},
		},
	}

	p3 := &CustomMapData{
		Name:     "黄金海岸半岛 (Golden Coast Peninsula)",
		Subtitle: "海角灯塔大道 · 帆船游艇码头 · 离岸深水国际集装箱港",
		Style:    "coastal",
		Roads: []CustomRoadData{
			{P1: [2]float64{-2600, -1200}, P2: [2]float64{0, -1200}, Lanes: 2, Name: "北渔港路"},
			{P1: [2]float64{-2600, 0}, P2: [2]float64{2600, 0}, Lanes: 4, Name: "跨海黄金海景大道"},
			{P1: [2]float64{-2600, 1200}, P2: [2]float64{0, 1200}, Lanes: 4, Name: "海滩南环快速路"},

			{P1: [2]float64{-2600, -1200}, P2: [2]float64{-2600, 1200}, Lanes: 2, Name: "西海岸公路"},
			{P1: [2]float64{-1300, -1200}, P2: [2]float64{-1300, 1200}, Lanes: 2, Name: "滨海度假大道"},
			{P1: [2]float64{0, -1200}, P2: [2]float64{0, 1200}, Lanes: 4, Name: "半岛黄金海岸线"},

			{P1: [2]float64{2600, -800}, P2: [2]float64{2600, 800}, Lanes: 4, Name: "深水港离岸环线"},
			{P1: [2]float64{1800, -800}, P2: [2]float64{2600, -800}, Lanes: 2, Name: "北防波堤公路"},
			{P1: [2]float64{1800, 800}, P2: [2]float64{2600, 800}, Lanes: 2, Name: "南集装箱货运路"},
			{P1: [2]float64{1800, -800}, P2: [2]float64{1800, 800}, Lanes: 2, Name: "离岸保税大道"},
		},
		Stops: []CustomStopData{
			{ID: 0, Name: "黄金沙滩总站 (Golden Beach Terminal)", Pos: [2]float64{-1950, 0}, Heading: 0, Lanes: 4, Color: "#F59E0B", Capacity: 12},
			{ID: 1, Name: "海滨灯塔名胜站 (Lighthouse Cape)", Pos: [2]float64{0, 600}, Heading: -math.Pi / 2, Lanes: 4, Color: "#06B6D4", Capacity: 10},
			{ID: 2, Name: "帆船游艇俱乐部 (Yacht Club)", Pos: [2]float64{0, -600}, Heading: math.Pi / 2, Lanes: 4, Color: "#3B82F6", Capacity: 10},
			{ID: 3, Name: "跨海特大桥西站 (Bay Bridge West)", Pos: [2]float64{-650, 0}, Heading: 0, Lanes: 4, Color: "#EF4444", Capacity: 8},
			{ID: 4, Name: "深水码头总站 (Deepwater Port)", Pos: [2]float64{2200, 0}, Heading: 0, Lanes: 4, Color: "#A855F7", Capacity: 14},
			{ID: 5, Name: "保税物流中心 (Bonded Logistics)", Pos: [2]float64{2600, 400}, Heading: math.Pi / 2, Lanes: 4, Color: "#10B981", Capacity: 8},
		},
		Waters: []CustomWaterData{
			{
				Name:     "蔚蓝太平洋海湾",
				Center:   [2]float64{1800, 0},
				Width:    3400,
				Height:   3400,
				Color:    "#0369A1E6",
				Shimmer:  "#BAE6FDB4",
				IsOcean:  true,
				HasBeach: true,
			},
		},
		Bridges: []CustomBridgeData{
			{Pos: [2]float64{900, 0}, Heading: 0, Length: 1200, Width: 100, Name: "跨海湾特大悬索桥", Style: "cable"},
		},
		Roundabouts: []CustomRoundaboutData{
			{Name: "海滨灯塔环岛广场", Center: [2]float64{-1300, 600}, Radius: 220.0, InnerRadius: 105.0, Type: "lighthouse"},
		},
	}

	return []*CustomMapData{p1, p2, p3}
}

// EnsureDefaultCustomMapFile checks if path exists, and if not writes default starter JSON
func EnsureDefaultCustomMapFile(path string) error {
	if _, err := os.Stat(path); err == nil {
		return nil // File already exists
	}
	defaultData := GetDefaultCustomMapData()
	return SaveCustomMapToFile(path, defaultData)
}

// ValidateCustomMapData checks the integrity, orthogonality, and constraints of custom map data
func ValidateCustomMapData(data *CustomMapData) error {
	if data == nil {
		return fmt.Errorf("地图数据不能为空")
	}
	if strings.TrimSpace(data.Name) == "" {
		return fmt.Errorf("地图名称不能为空")
	}
	if len(data.Roads) < 2 {
		return fmt.Errorf("自定义地图至少需要 2 条道路 (当前: %d 条)", len(data.Roads))
	}
	if len(data.Stops) < 2 {
		return fmt.Errorf("自定义地图至少需要 2 座车站 (当前: %d 座)", len(data.Stops))
	}

	// Validate road orthogonality
	for i, r := range data.Roads {
		isEW := math.Abs(r.P1[1]-r.P2[1]) < 1.0
		isNS := math.Abs(r.P1[0]-r.P2[0]) < 1.0
		if !isEW && !isNS {
			return fmt.Errorf("道路 #%d (%s) 必须与坐标轴正交 (水平或垂直方向, non-orthogonal road)", i, r.Name)
		}
		length := math.Hypot(r.P2[0]-r.P1[0], r.P2[1]-r.P1[1])
		if length < 10.0 {
			return fmt.Errorf("道路 #%d (%s) 长度过短: %.1f px", i, r.Name, length)
		}
	}

	// Validate parallel road spacing: Ensure two parallel two-way roads are not too close (must be >= 400px apart)
	const minParallelRoadSpacing = 400.0
	for i := 0; i < len(data.Roads); i++ {
		for j := i + 1; j < len(data.Roads); j++ {
			r1 := data.Roads[i]
			r2 := data.Roads[j]
			isR1EW := math.Abs(r1.P1[1]-r1.P2[1]) < 1.0
			isR2EW := math.Abs(r2.P1[1]-r2.P2[1]) < 1.0
			isR1NS := math.Abs(r1.P1[0]-r1.P2[0]) < 1.0
			isR2NS := math.Abs(r2.P1[0]-r2.P2[0]) < 1.0

			// Parallel East-West roads
			if isR1EW && isR2EW {
				y1 := (r1.P1[1] + r1.P2[1]) / 2.0
				y2 := (r2.P1[1] + r2.P2[1]) / 2.0
				dist := math.Abs(y1 - y2)
				minX1 := math.Min(r1.P1[0], r1.P2[0])
				maxX1 := math.Max(r1.P1[0], r1.P2[0])
				minX2 := math.Min(r2.P1[0], r2.P2[0])
				maxX2 := math.Max(r2.P1[0], r2.P2[0])
				overlap := math.Min(maxX1, maxX2) - math.Max(minX1, minX2)
				if overlap > 30.0 && dist > 1.0 && dist < minParallelRoadSpacing {
					return fmt.Errorf("道路 #%d (%s) 与 道路 #%d (%s) 为平行道路且间距过小 (%.1f px < %.0f px): 平行双向道路间距过小不合常理，必须保持至少 400 px 以上街区距离",
						i, r1.Name, j, r2.Name, dist, minParallelRoadSpacing)
				}
			}

			// Parallel North-South roads
			if isR1NS && isR2NS {
				x1 := (r1.P1[0] + r1.P2[0]) / 2.0
				x2 := (r2.P1[0] + r2.P2[0]) / 2.0
				dist := math.Abs(x1 - x2)
				minY1 := math.Min(r1.P1[1], r1.P2[1])
				maxY1 := math.Max(r1.P1[1], r1.P2[1])
				minY2 := math.Min(r2.P1[1], r2.P2[1])
				maxY2 := math.Max(r2.P1[1], r2.P2[1])
				overlap := math.Min(maxY1, maxY2) - math.Max(minY1, minY2)
				if overlap > 30.0 && dist > 1.0 && dist < minParallelRoadSpacing {
					return fmt.Errorf("道路 #%d (%s) 与 道路 #%d (%s) 为平行道路且间距过小 (%.1f px < %.0f px): 平行双向道路间距过小不合常理，必须保持至少 400 px 以上街区距离",
						i, r1.Name, j, r2.Name, dist, minParallelRoadSpacing)
				}
			}
		}
	}

	// Validate stop IDs are unique
	seenIDs := make(map[int]bool)
	for i, s := range data.Stops {
		if seenIDs[s.ID] {
			return fmt.Errorf("站点 #%d 存在重复的站点 ID (%d): duplicate stop id", i, s.ID)
		}
		seenIDs[s.ID] = true
	}

	// Compute intersections of custom map roads to ensure bus stops are not placed at or near intersections
	var inters []Vec2
	for i := 0; i < len(data.Roads); i++ {
		for j := i + 1; j < len(data.Roads); j++ {
			r1 := data.Roads[i]
			r2 := data.Roads[j]
			isR1EW := math.Abs(r1.P1[1]-r1.P2[1]) < 1.0
			isR2EW := math.Abs(r2.P1[1]-r2.P2[1]) < 1.0
			if isR1EW != isR2EW {
				var ew, ns CustomRoadData
				if isR1EW {
					ew, ns = r1, r2
				} else {
					ew, ns = r2, r1
				}
				x := ns.P1[0]
				y := ew.P1[1]
				minEWX := math.Min(ew.P1[0], ew.P2[0])
				maxEWX := math.Max(ew.P1[0], ew.P2[0])
				minNSY := math.Min(ns.P1[1], ns.P2[1])
				maxNSY := math.Max(ns.P1[1], ns.P2[1])
				if x >= minEWX-1.0 && x <= maxEWX+1.0 && y >= minNSY-1.0 && y <= maxNSY+1.0 {
					inters = append(inters, Vec2{X: x, Y: y})
				}
			}
		}
	}

	// Validate stop distance from road intersections (must be >= 150px)
	for i, s := range data.Stops {
		stopPos := Vec2{X: s.Pos[0], Y: s.Pos[1]}
		for _, it := range inters {
			dist := stopPos.Distance(it)
			if dist < 150.0 {
				return fmt.Errorf("站点 #%d (%s) 距离路口过近 (%.1f px < 150 px): 公交站点必须与交叉路口保持安全距离", i, s.Name, dist)
			}
		}
	}

	// Validate roundabouts: Ensure circular roadway is wide enough (carriageway >= 80px)
	for i, rb := range data.Roundabouts {
		carriagewayW := rb.Radius - rb.InnerRadius
		if carriagewayW < 80.0 {
			return fmt.Errorf("环岛 #%d (%s) 环形车道宽度过窄 (%.1f px < 80 px): 环岛必须具备足够通行的环形车道宽度", i, rb.Name, carriagewayW)
		}
		if rb.InnerRadius < 60.0 {
			return fmt.Errorf("环岛 #%d (%s) 内部安全岛过小 (%.1f px < 60 px)", i, rb.Name, rb.InnerRadius)
		}
	}

	// Validate buildings do not overlap with travel roadways
	for i, b := range data.Buildings {
		bCenter := Vec2{X: b.Pos[0], Y: b.Pos[1]}
		hw := b.Width / 2.0
		hh := b.Height / 2.0
		for _, r := range data.Roads {
			isEW := math.Abs(r.P1[1]-r.P2[1]) < 1.0
			roadW := float64(r.Lanes) * 52.0
			roadHW := roadW / 2.0
			if isEW {
				roadY := r.P1[1]
				minX := math.Min(r.P1[0], r.P2[0])
				maxX := math.Max(r.P1[0], r.P2[0])
				if bCenter.X+hw > minX && bCenter.X-hw < maxX && math.Abs(bCenter.Y-roadY) < (hh+roadHW) {
					return fmt.Errorf("建筑 #%d (%s) 与道路 (%s) 重叠侵限: 建筑不得侵占机动车行车道", i, b.Name, r.Name)
				}
			} else {
				roadX := r.P1[0]
				minY := math.Min(r.P1[1], r.P2[1])
				maxY := math.Max(r.P1[1], r.P2[1])
				if bCenter.Y+hh > minY && bCenter.Y-hh < maxY && math.Abs(bCenter.X-roadX) < (hw+roadHW) {
					return fmt.Errorf("建筑 #%d (%s) 与道路 (%s) 重叠侵限: 建筑不得侵占机动车行车道", i, b.Name, r.Name)
				}
			}
		}
	}

	return nil
}

// LoadCustomMapFromFile parses custom map json from disk
func LoadCustomMapFromFile(path string) (*CustomMapData, error) {
	targetPath := path
	bytes, err := os.ReadFile(targetPath)
	if err != nil {
		// Try parent directory fallback (e.g. when executing from game/ subdirectory)
		parentPath := filepath.Join("..", path)
		if pBytes, pErr := os.ReadFile(parentPath); pErr == nil {
			bytes = pBytes
			targetPath = parentPath
		} else {
			// If file does not exist and it matches the default filename, automatically scaffold starter map
			if filepath.Base(path) == DefaultCustomMapFileName {
				if ensureErr := EnsureDefaultCustomMapFile(path); ensureErr == nil {
					if cBytes, cErr := os.ReadFile(path); cErr == nil {
						bytes = cBytes
						targetPath = path
					} else {
						return nil, fmt.Errorf("读取自定义地图文件失败: %w", err)
					}
				} else {
					return nil, fmt.Errorf("读取自定义地图文件失败: %w", err)
				}
			} else {
				return nil, fmt.Errorf("读取自定义地图文件失败: %w", err)
			}
		}
	}

	var data CustomMapData
	if err := json.Unmarshal(bytes, &data); err != nil {
		return nil, fmt.Errorf("自定义地图 JSON 格式错误: %w", err)
	}

	if data.Name == "" {
		data.Name = "未命名自定义地图"
	}
	if data.Subtitle == "" {
		data.Subtitle = "玩家自建路网与客运线路"
	}

	if err := ValidateCustomMapData(&data); err != nil {
		return nil, fmt.Errorf("自定义地图数据校验未通过: %w", err)
	}

	return &data, nil
}

// SaveCustomMapToFile writes custom map json to disk with pretty indenting
func SaveCustomMapToFile(path string, data *CustomMapData) error {
	bytes, err := json.MarshalIndent(data, "", "  ")
	if err != nil {
		return fmt.Errorf("序列化地图数据失败: %w", err)
	}

	dir := filepath.Dir(path)
	if dir != "" && dir != "." {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return fmt.Errorf("创建目录失败: %w", err)
		}
	}

	return os.WriteFile(path, bytes, 0644)
}

// ApplyCustomMapToWorld converts CustomMapData into World entities
func ApplyCustomMapToWorld(w *World, data *CustomMapData, laneWidth float64) {
	if w == nil || data == nil {
		return
	}

	w.Theme = MapThemeCustom
	w.Roads = make([]RoadSegment, 0, len(data.Roads))
	w.Buildings = make([]Building, 0, len(data.Buildings))
	w.Trees = make([]Tree, 0, len(data.Trees))
	w.Stops = make([]*BusStop, 0, len(data.Stops))
	w.Rivers = make([]River, 0, len(data.Rivers))
	w.Waters = make([]WaterBody, 0, len(data.Waters))
	w.Bridges = make([]Bridge, 0, len(data.Bridges))
	w.Roundabouts = make([]Roundabout, 0, len(data.Roundabouts))

	// 1. Waters
	for _, wd := range data.Waters {
		col := parseHexColor(wd.Color, RGBA(14, 116, 144, 210))
		shim := parseHexColor(wd.Shimmer, RGBA(103, 232, 249, 180))
		w.Waters = append(w.Waters, WaterBody{
			IsOcean:  wd.IsOcean,
			HasBeach: wd.HasBeach,
			IsLake:   wd.IsLake,
			Center:   Vec2{X: wd.Center[0], Y: wd.Center[1]},
			Width:    wd.Width,
			Height:   wd.Height,
			Color:    col,
			Shimmer:  shim,
			Name:     wd.Name,
		})
	}

	// 2. Rivers
	for _, rd := range data.Rivers {
		w.Rivers = append(w.Rivers, River{
			Y:      rd.Y,
			Height: rd.Height,
		})
	}

	// 3. Roads
	for _, r := range data.Roads {
		lanes := r.Lanes
		if lanes != 2 && lanes != 4 {
			lanes = 4
		}
		rType := RoadAvenue4Lane
		if lanes == 2 {
			rType = RoadStreet2Lane
		}
		w.Roads = append(w.Roads, RoadSegment{
			P1:       Vec2{X: r.P1[0], Y: r.P1[1]},
			P2:       Vec2{X: r.P2[0], Y: r.P2[1]},
			Lanes:    lanes,
			Type:     rType,
			HasLines: true,
			Name:     r.Name,
		})
	}

	// 4. Bridges
	for _, b := range data.Bridges {
		isCable := (strings.ToLower(b.Style) == "cable" || strings.ToLower(b.Style) == "suspension")
		isArch := (strings.ToLower(b.Style) == "arch")
		isOverpass := (strings.ToLower(b.Style) == "overpass")
		w.Bridges = append(w.Bridges, Bridge{
			Pos:        Vec2{X: b.Pos[0], Y: b.Pos[1]},
			Heading:    b.Heading,
			Length:     b.Length,
			Width:      b.Width,
			Name:       b.Name,
			IsCable:    isCable,
			IsArch:     isArch,
			IsOverpass: isOverpass,
		})
	}

	// 5. Roundabouts
	for _, rb := range data.Roundabouts {
		rbType := RoundaboutCivic
		switch strings.ToLower(rb.Type) {
		case "lighthouse":
			rbType = RoundaboutLighthouse
		case "island":
			rbType = RoundaboutIsland
		}
		w.Roundabouts = append(w.Roundabouts, Roundabout{
			Center:      Vec2{X: rb.Center[0], Y: rb.Center[1]},
			Radius:      rb.Radius,
			InnerRadius: rb.InnerRadius,
			Name:        rb.Name,
			Type:        rbType,
		})
	}

	// 6. Bus Stops
	for i, s := range data.Stops {
		stopID := s.ID
		if stopID < 0 {
			stopID = i
		}
		lanes := s.Lanes
		if lanes != 2 && lanes != 4 {
			lanes = 4
		}
		accent := parseHexColor(s.Color, RGBA(59, 130, 246, 255))
		cap := s.Capacity
		if cap <= 0 {
			cap = 10
		}
		w.Stops = append(w.Stops, NewBusStop(
			stopID,
			s.Name,
			Vec2{X: s.Pos[0], Y: s.Pos[1]},
			s.Heading,
			lanes,
			accent,
			laneWidth,
			cap,
		))
	}

	// 7. Buildings
	for _, b := range data.Buildings {
		col := parseHexColor(b.Color, RGBA(30, 41, 59, 255))
		roof := parseHexColor(b.Roof, RGBA(51, 65, 85, 255))
		w.Buildings = append(w.Buildings, Building{
			Pos:   Vec2{X: b.Pos[0], Y: b.Pos[1]},
			W:     b.Width,
			H:     b.Height,
			Color: col,
			Roof:  roof,
			Name:  b.Name,
		})
	}

	// 8. Trees
	for _, tr := range data.Trees {
		col := parseHexColor(tr.Color, RGBA(22, 101, 52, 255))
		rad := tr.Radius
		if rad <= 0 {
			rad = 15
		}
		w.Trees = append(w.Trees, Tree{
			Pos:    Vec2{X: tr.Pos[0], Y: tr.Pos[1]},
			Radius: rad,
			Color:  col,
		})
	}

	// Compute intersections dynamically
	w.computeIntersections()
}
