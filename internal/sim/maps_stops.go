package sim

import (
	"image/color"
	"math"
)

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
		NewBusStop(1, names[1], Vec2{X: 650, Y: 0}, 0, 4, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: 1300, Y: -300}, -math.Pi/2, 2, accents[2], laneWidth, 0),
		NewBusStop(3, names[3], Vec2{X: 2600, Y: -900}, math.Pi/2, 4, accents[3], laneWidth, 0),
		NewBusStop(4, names[4], Vec2{X: 1950, Y: -1200}, 0, 6, accents[4], laneWidth, 8),
		NewBusStop(5, names[5], Vec2{X: -650, Y: -1200}, math.Pi, 6, accents[5], laneWidth, 16),
		NewBusStop(6, names[6], Vec2{X: -2600, Y: -900}, math.Pi/2, 4, accents[6], laneWidth, 6),
		// Keep a full block for departure and the multi-lane left merge at the south junction.
		NewBusStop(7, names[7], Vec2{X: -2600, Y: 300}, math.Pi/2, 4, accents[7], laneWidth, 10),
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
		NewBusStop(1, names[1], Vec2{X: -325, Y: 0}, math.Pi, 6, accents[1], laneWidth, 12),
		NewBusStop(2, names[2], Vec2{X: -1625, Y: -1200}, 0, 2, accents[2], laneWidth, 6),
		NewBusStop(3, names[3], Vec2{X: 0, Y: -300}, math.Pi/2, 4, accents[3], laneWidth, 8),
		NewBusStop(4, names[4], Vec2{X: 0, Y: 300}, -math.Pi/2, 4, accents[4], laneWidth, 10),
		NewBusStop(5, names[5], Vec2{X: 2600, Y: -300}, math.Pi/2, 4, accents[5], laneWidth, 15),
		NewBusStop(6, names[6], Vec2{X: 2600, Y: 300}, math.Pi/2, 4, accents[6], laneWidth, 8),
		NewBusStop(7, names[7], Vec2{X: -975, Y: 600}, math.Pi, 2, accents[7], laneWidth, 6),
		NewBusStop(8, names[8], Vec2{X: -1625, Y: -600}, math.Pi, 2, accents[8], laneWidth, 10),
		NewBusStop(9, names[9], Vec2{X: -1300, Y: -300}, -math.Pi/2, 2, accents[9], laneWidth, 8),
		NewBusStop(10, names[10], Vec2{X: -1950, Y: 300}, -math.Pi/2, 2, accents[10], laneWidth, 12),
		NewBusStop(11, names[11], Vec2{X: 900, Y: 1200}, math.Pi, 2, accents[11], laneWidth, 8),
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
	// Place bridge stops on the banks, away from the central river span.
	w.Stops = append(w.Stops,
		NewBusStop(0, names[0], Vec2{X: -650, Y: -1200}, 0, 4, accents[0], laneWidth, 12),
		NewBusStop(1, names[1], Vec2{X: 650, Y: -1200}, math.Pi, 4, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: -1950, Y: -600}, math.Pi, 4, accents[2], laneWidth, 6),
		NewBusStop(3, names[3], Vec2{X: -1300, Y: -350}, math.Pi/2, 4, accents[3], laneWidth, 10),
		NewBusStop(4, names[4], Vec2{X: -650, Y: 1200}, 0, 6, accents[4], laneWidth, 16),
		NewBusStop(5, names[5], Vec2{X: -650, Y: 600}, math.Pi, 4, accents[5], laneWidth, 10),
		NewBusStop(6, names[6], Vec2{X: 1300, Y: 350}, -math.Pi/2, 6, accents[6], laneWidth, 10),
		NewBusStop(7, names[7], Vec2{X: 650, Y: 600}, math.Pi, 4, accents[7], laneWidth, 8),
		NewBusStop(8, names[8], Vec2{X: 0, Y: -900}, math.Pi/2, 2, accents[8], laneWidth, 10),
		NewBusStop(9, names[9], Vec2{X: 650, Y: -600}, 0, 4, accents[9], laneWidth, 8),
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
		NewBusStop(4, names[4], Vec2{X: 0, Y: 900}, math.Pi/2, 2, accents[4], laneWidth, 8),
		NewBusStop(5, names[5], Vec2{X: -650, Y: 1200}, math.Pi, 4, accents[5], laneWidth, 10),
		NewBusStop(6, names[6], Vec2{X: -2600, Y: 300}, -math.Pi/2, 4, accents[6], laneWidth, 8),
		NewBusStop(7, names[7], Vec2{X: -1950, Y: -1200}, 0, 4, accents[7], laneWidth, 6),
		NewBusStop(8, names[8], Vec2{X: -1950, Y: -600}, math.Pi, 2, accents[8], laneWidth, 8),
		NewBusStop(9, names[9], Vec2{X: 1950, Y: 0}, math.Pi, 2, accents[9], laneWidth, 10),
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
		NewBusStop(1, names[1], Vec2{X: -2275, Y: -1200}, math.Pi, 2, accents[1], laneWidth, 8),
		NewBusStop(2, names[2], Vec2{X: -1300, Y: 300}, -math.Pi/2, 4, accents[2], laneWidth, 10),
		NewBusStop(3, names[3], Vec2{X: -975, Y: 0}, 0, 6, accents[3], laneWidth, 10),
		NewBusStop(4, names[4], Vec2{X: 975, Y: 0}, 0, 6, accents[4], laneWidth, 10),
		NewBusStop(5, names[5], Vec2{X: 2275, Y: -1200}, math.Pi, 2, accents[5], laneWidth, 8),
		NewBusStop(6, names[6], Vec2{X: 2275, Y: 0}, 0, 6, accents[6], laneWidth, 14),
		NewBusStop(7, names[7], Vec2{X: 1300, Y: 300}, math.Pi/2, 4, accents[7], laneWidth, 8),
		NewBusStop(8, names[8], Vec2{X: -1625, Y: -600}, math.Pi, 2, accents[8], laneWidth, 14),
		NewBusStop(9, names[9], Vec2{X: -1625, Y: 600}, 0, 2, accents[9], laneWidth, 10),
		NewBusStop(10, names[10], Vec2{X: 1625, Y: -600}, math.Pi, 2, accents[10], laneWidth, 12),
		NewBusStop(11, names[11], Vec2{X: 1625, Y: 600}, 0, 2, accents[11], laneWidth, 10),
	)
}
