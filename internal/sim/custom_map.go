package sim

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
	BusLane bool       `json:"bus_lane,omitempty"` // Outermost lane reserved for buses, shared with right turns at junctions.
	P1      [2]float64 `json:"p1"`
	P2      [2]float64 `json:"p2"`
	Lanes   int        `json:"lanes"` // 2, 4, 6, or 8
	Name    string     `json:"name"`
}

// CustomStopData represents a serializable bus stop in JSON
type CustomStopData struct {
	ID       int        `json:"id"`
	Name     string     `json:"name"`
	Pos      [2]float64 `json:"pos"`
	Heading  float64    `json:"heading"` // in radians (e.g. 0=East, 1.57=South, 3.14=West, -1.57=North)
	Lanes    int        `json:"lanes"`   // 2, 4, 6, or 8
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
		if r.BusLane && r.Lanes != 8 {
			return fmt.Errorf("道路 #%d: 公交专用道需要双向八车道道路 (lanes: 8)", i)
		}
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
