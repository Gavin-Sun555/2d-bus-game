package sim

import (
	"math"
)

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
