package sim

import (
	"math"
)

// GetDefaultCustomMapData returns the default starter custom map: "星空未来城 (Starfield Future City)"
func GetDefaultCustomMapData() *CustomMapData {
	return &CustomMapData{
		Name:     "星空未来城 (Starfield Future City)",
		Subtitle: "玩家自定义自建都市 · 星光高架与双子湖生态客运网",
		Style:    "metropolis",
		Roads: []CustomRoadData{
			// Major 4-lane Ring & Grid, with 6-lane North Ring Avenue
			{P1: [2]float64{-2600, -1200}, P2: [2]float64{2600, -1200}, Lanes: 6, Name: "星光北环六车道大道"},
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
			{ID: 3, Name: "量子实验室 (Quantum Lab)", Pos: [2]float64{2600, -900}, Heading: -math.Pi / 2, Lanes: 4, Color: "#06B6D4", Capacity: 8},
			{ID: 4, Name: "星海体育馆 (Star Arena)", Pos: [2]float64{1950, -1200}, Heading: math.Pi, Lanes: 6, Color: "#EC4899", Capacity: 10},
			{ID: 5, Name: "空港北枢纽 (North Airport)", Pos: [2]float64{-325, -1200}, Heading: 0, Lanes: 6, Color: "#F59E0B", Capacity: 14},
			{ID: 6, Name: "运河客运港 (Canal Port)", Pos: [2]float64{-2600, -900}, Heading: math.Pi / 2, Lanes: 4, Color: "#0EA5E9", Capacity: 8},
			{ID: 7, Name: "大学城东校区 (Uni Campus)", Pos: [2]float64{-2600, 900}, Heading: -math.Pi / 2, Lanes: 4, Color: "#8B5CF6", Capacity: 8},
			{ID: 8, Name: "第一市民医院 (General Hospital)", Pos: [2]float64{-1300, 300}, Heading: -math.Pi / 2, Lanes: 2, Color: "#EF4444", Capacity: 10},
			{ID: 9, Name: "星湖生态公园 (Star Lake Park)", Pos: [2]float64{-1950, 1200}, Heading: 0, Lanes: 4, Color: "#22C55E", Capacity: 8},
			{ID: 10, Name: "文创生活街区 (Creative Zone)", Pos: [2]float64{325, 1200}, Heading: math.Pi, Lanes: 4, Color: "#F97316", Capacity: 10},
			{ID: 11, Name: "国际会展总站 (Expo Center)", Pos: [2]float64{2600, 900}, Heading: math.Pi / 2, Lanes: 4, Color: "#D946EF", Capacity: 8},
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
