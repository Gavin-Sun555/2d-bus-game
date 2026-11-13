package view

import (
	"image/color"
	"math"
)

// valleyRailHub renders the Grand High-Speed Rail Hub (高铁新城枢纽总站).
func (a metroCanvas) valleyRailHub() {
	// Granite passenger station plaza
	a.softPlaza(0, 0.22, 0.94, 0.46, 0.04, 0.04, rgb(204, 208, 212))
	// Grand curved station concourse roof with skylights
	a.archBlock(0, 0.18, 0.88, 0.34, 6, rgb(52, 68, 82), rgb(240, 244, 248), 10, true)
	a.skylight(0, 0.18, 0.74, 0.18, 8, 2)
	// Entrance canopy & taxi loop
	a.rect(0, 0.38, 0.44, 0.06, rgb(224, 142, 54))
	if a.detail {
		a.miniBus(-0.16, 0.42, 0, rgb(56, 142, 224), true)
		a.miniCar(0.12, 0.42, 0, rgb(240, 196, 38), true)
		a.miniCar(0.24, 0.42, 0, rgb(245, 245, 245), false)
	}
	a.railPlatform(-.22, rgb(96, 130, 150))
	a.sign(0, 0.44, "高铁新城枢纽总站", "HIGH-SPEED RAIL HUB")
}

// valleyHSRHQ renders the High-Speed Rail Corporate Headquarters (高铁总部大道站).
func (a metroCanvas) valleyHSRHQ() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(212, 216, 220))
	// High-rise glass corporate tower
	a.archBlock(-0.16, -0.06, 0.54, 0.48, 6, rgb(44, 60, 76), rgb(232, 238, 246), 11, true)
	a.skylight(-0.16, -0.06, 0.38, 0.28, 4, 3)
	// Aerodynamic train scale model monument in plaza
	a.oval(0.26, 0.04, 0.28, 0.44, rgb(194, 202, 208))
	if a.detail {
		pos := a.point(0.26, 0.04)
		drawVehicleShell(a.screen, a.cam, pos, a.lot.heading+math.Pi/2, 42, 16, rgb(246, 248, 252), 6, 6)
		a.line(0.18, -0.14, 0.18, 0.22, 2.0, rgb(38, 118, 214))
	}
	// Entrance concourse & greenery
	a.rect(-0.16, 0.26, 0.32, 0.06, rgb(228, 142, 54))
	if a.detail {
		a.treeCluster(0.32, -0.28, 14)
		a.treeCluster(0.32, 0.30, 14)
		a.treeCluster(-0.40, 0.28, 13)
	}
	a.sign(0, 0.42, "高铁总部大道站", "HSR HEADQUARTERS")
}

// valleyFinancialTowers renders the Riverfront Financial Center (南滨江金融中心).
func (a metroCanvas) valleyFinancialTowers() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(208, 212, 216))
	// Twin financial skyscrapers with connecting skybridge
	a.archBlock(-0.24, -0.08, 0.34, 0.44, 6, rgb(42, 58, 74), rgb(234, 240, 248), 12, true)
	a.archBlock(0.24, -0.08, 0.34, 0.44, 6, rgb(42, 58, 74), rgb(234, 240, 248), 12, true)
	a.skylight(-0.24, -0.08, 0.24, 0.24, 3, 3)
	a.skylight(0.24, -0.08, 0.24, 0.24, 3, 3)
	// Connecting high-level skybridge
	a.rect(0, -0.16, 0.18, 0.10, rgb(68, 92, 114))
	a.line(-0.08, -0.16, 0.08, -0.16, 2.0, rgb(56, 172, 224))
	// Grand reflecting pool plaza
	a.oval(0, 0.22, 0.52, 0.24, rgb(36, 118, 146))
	a.oval(0, 0.22, 0.44, 0.18, rgb(68, 178, 204))
	a.circlePx(0, 0.22, 6.0, rgb(240, 238, 228))
	if a.detail {
		a.treeCluster(-0.38, 0.26, 14)
		a.treeCluster(0.38, 0.26, 14)
	}
	a.sign(0, 0.42, "南滨江金融中心", "FINANCIAL RIVERFRONT TOWERS")
}

// valleyCultureVista renders the Riverfront Cultural Center & Vista Plaza (南滨江观澜大道站).
func (a metroCanvas) valleyCultureVista() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(212, 208, 196))
	// Organic curved cultural arts center
	a.oval(-0.12, -0.06, 0.62, 0.44, rgb(62, 82, 98))
	a.oval(-0.12, -0.06, 0.54, 0.36, rgb(242, 244, 248))
	a.oval(-0.12, -0.06, 0.36, 0.22, rgb(52, 148, 186))
	// Stepped riverside amphitheater facing north
	a.oval(0.28, -0.06, 0.32, 0.36, rgb(184, 176, 162))
	a.oval(0.28, -0.06, 0.24, 0.26, rgb(204, 196, 182))
	a.circlePx(0.28, -0.06, 5.0, rgb(238, 148, 54))
	// Vista terrace & promenade
	a.rect(0, 0.26, 0.84, 0.14, rgb(194, 186, 170))
	if a.detail {
		a.treeCluster(-0.38, 0.24, 14)
		a.treeCluster(0.38, 0.24, 14)
		a.flowerBed(0, 0.34, 0.42, 0.06, rgb(224, 88, 108))
	}
	a.sign(0, 0.42, "南滨江观澜大道站", "RIVERFRONT VISTA CENTER")
}

// valleyNorthForest renders the North Forest Park Gate & Visitor Pavilion (北山森林公园站).
func (a metroCanvas) valleyNorthForest() {
	// Mountain stone paving & terraced gardens
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(186, 182, 168))
	// Mountain gateway pavilion (traditional timber & stone)
	a.archBlock(0, 0.16, 0.72, 0.24, 5, rgb(118, 76, 52), rgb(92, 96, 98), 8, false)
	a.rect(0, 0.16, 0.58, 0.06, rgb(196, 68, 52))
	// Terraced botanical pine garden
	a.softLawn(0, -0.18, 0.88, 0.38)
	if a.detail {
		coniferCols := []color.RGBA{rgb(22, 88, 52), rgb(32, 106, 64), rgb(18, 74, 44)}
		for i, x := range []float64{-0.34, -0.18, 0, 0.18, 0.34} {
			for j, y := range []float64{-0.28, -0.10} {
				c := coniferCols[(i+j)%len(coniferCols)]
				a.circlePx(x, y, 7.5, c)
				a.circlePx(x, y, 3.5, tint(c, rgb(255, 255, 255), 0.25))
			}
		}
		// Stone lanterns
		a.circlePx(-0.24, 0.28, 3.5, rgb(238, 228, 196))
		a.circlePx(0.24, 0.28, 3.5, rgb(238, 228, 196))
	}
	a.sign(0, 0.40, "北山森林公园站", "NORTH FOREST PARK")
}

// valleyLookout renders the Valley Panoramic Lookout Deck (溪谷江北观景台).
func (a metroCanvas) valleyLookout() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(192, 186, 172))
	// Hillside stone plinth
	a.rect(0, -0.12, 0.84, 0.48, rgb(168, 160, 146))
	// Multi-tier wooden and glass panoramic lookout deck
	a.rect(0, -0.14, 0.74, 0.38, rgb(186, 148, 108))
	if a.detail {
		for x := -0.34; x <= 0.34; x += 0.04 {
			a.line(x, -0.32, x, 0.04, 1.0, rgb(148, 114, 82))
		}
		// Panoramic telescope viewpoints along scenic edge
		for _, x := range []float64{-0.28, -0.10, 0.10, 0.28} {
			a.circlePx(x, -0.32, 3.5, rgb(68, 64, 58))
			a.circlePx(x, -0.32, 1.5, rgb(240, 196, 52))
		}
	}
	// Visitor teahouse & administration
	a.archBlock(0, 0.24, 0.64, 0.22, 4, rgb(128, 88, 62), rgb(242, 238, 228), 7, true)
	if a.detail {
		a.treeCluster(-0.36, 0.22, 13)
		a.treeCluster(0.36, 0.22, 13)
	}
	a.sign(0, 0.42, "溪谷江北观景台", "NORTH VALLEY VIEW LOOKOUT")
}

// valleyEcoNorth renders the Riverfront North Eco-Center (北滨江生态街).
func (a metroCanvas) valleyEcoNorth() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(206, 202, 188))
	// Biophilic eco-center with green sod roof
	a.archBlock(-0.14, 0.04, 0.58, 0.42, 5, rgb(58, 86, 68), rgb(74, 128, 88), 9, true)
	a.oval(-0.14, 0.04, 0.42, 0.26, rgb(54, 108, 72))
	a.skylight(-0.14, 0.04, 0.28, 0.14, 3, 2)
	// Elevated wooden wetland boardwalk & reed pond
	a.rect(0.28, 0.04, 0.30, 0.48, rgb(48, 126, 128))
	a.rect(0.28, 0.04, 0.12, 0.44, rgb(184, 152, 114))
	if a.detail {
		a.treeCluster(0.36, -0.24, 12)
		a.treeCluster(0.36, 0.24, 12)
		a.treeCluster(-0.38, 0.28, 13)
	}
	a.sign(0, 0.42, "北滨江生态街", "RIVERFRONT ECO-CENTER")
}

// valleyBridgeheadNorth renders Cable Bridge North Bridgehead Plaza (溪谷斜拉大桥北站).
func (a metroCanvas) valleyBridgeheadNorth() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(204, 208, 212))
	// Massive diamond-shaped concrete bridge pylon monument
	pylonCol := rgb(238, 242, 246)
	a.archBlock(0, -0.16, 0.26, 0.38, 6, rgb(68, 82, 94), pylonCol, 10, false)
	// Stylized bridge stay cables fan
	if a.detail {
		for i := -4; i <= 4; i++ {
			x := float64(i) * 0.09
			a.line(0, -0.28, x, 0.02, 1.8, rgb(214, 148, 54))
		}
	}
	// Bridge administration & visitor plaza
	a.archBlock(-0.24, 0.22, 0.38, 0.24, 4, rgb(56, 72, 86), rgb(232, 236, 242), 8, true)
	a.rect(0.24, 0.22, 0.36, 0.22, rgb(188, 194, 200))
	if a.detail {
		a.circlePx(0.24, 0.22, 6.0, rgb(48, 128, 168))
		a.treeCluster(0.38, -0.24, 12)
	}
	a.sign(0, 0.42, "溪谷斜拉大桥北站", "CABLE BRIDGE NORTH PLAZA")
}

// valleyBridgeheadSouth renders Suspension Bridge South Bridgehead Plaza (世纪悬索大桥南站).
func (a metroCanvas) valleyBridgeheadSouth() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(206, 210, 214))
	// Massive concrete suspension cable anchorage block
	a.archBlock(0, -0.16, 0.62, 0.36, 6, rgb(58, 72, 84), rgb(186, 192, 198), 11, false)
	a.rect(0, -0.16, 0.50, 0.24, rgb(118, 128, 136))
	// Suspension main cables anchor lines
	if a.detail {
		a.line(-0.28, -0.32, -0.18, 0.02, 3.0, rgb(224, 148, 52))
		a.line(0.28, -0.32, 0.18, 0.02, 3.0, rgb(224, 148, 52))
	}
	// Bridge museum & visitor plaza
	a.archBlock(-0.18, 0.24, 0.52, 0.22, 4, rgb(52, 68, 82), rgb(236, 240, 246), 8, true)
	if a.detail {
		a.treeCluster(0.32, 0.22, 14)
		a.treeCluster(-0.38, -0.22, 13)
	}
	a.sign(0, 0.42, "世纪悬索大桥南站", "SUSPENSION BRIDGE SOUTH PLAZA")
}

// valleySmartCampus renders South Industrial Manufacturing Park (江南智造产业园).
func (a metroCanvas) valleySmartCampus() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(212, 216, 220))
	// Modern R&D manufacturing wings
	a.archBlock(-0.18, -0.12, 0.54, 0.38, 5, rgb(48, 64, 78), rgb(234, 238, 244), 9, true)
	a.solarBank(-0.18, -0.12, 0.42, 0.24, 4, 3)
	// Tech laboratory & office
	a.archBlock(0.24, 0.12, 0.36, 0.44, 4, rgb(48, 64, 78), rgb(234, 238, 244), 8, true)
	a.skylight(0.24, 0.12, 0.24, 0.24, 3, 2)
	// Courtyard park
	a.softLawn(-0.22, 0.24, 0.38, 0.24)
	if a.detail {
		a.treeCluster(-0.22, 0.24, 13)
		a.miniCar(0.24, -0.26, 0, rgb(48, 142, 214), false)
		a.miniCar(0.34, -0.26, 0, rgb(240, 240, 240), false)
	}
	a.sign(0, 0.42, "江南智造产业园", "SMART INDUSTRIAL PARK")
}

// valleyCivicCenter renders Jiangbei Center Ave Civic Center (江北中心大道站).
func (a metroCanvas) valleyCivicCenter() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(214, 210, 198))
	// Classical symmetric civic building
	a.archBlock(0, -0.06, 0.78, 0.44, 6, rgb(98, 88, 76), rgb(242, 238, 228), 10, true)
	a.rect(0, -0.06, 0.42, 0.12, rgb(168, 128, 82))
	a.circlePx(0, -0.06, 8.0, rgb(238, 204, 78))
	// Stepped granite plaza & fountain
	a.oval(0, 0.26, 0.48, 0.22, rgb(42, 128, 156))
	a.circlePx(0, 0.26, 5.0, rgb(240, 240, 240))
	if a.detail {
		a.treeCluster(-0.36, 0.24, 13)
		a.treeCluster(0.36, 0.24, 13)
	}
	a.sign(0, 0.42, "江北中心大道站", "JIANGBEI CIVIC CENTER")
}

// valleyMountainLookout renders Mountain Lookout Teahouse (北岸半山森林观景台).
func (a metroCanvas) valleyMountainLookout() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(194, 188, 174))
	// Traditional hillside teahouse pavilion
	a.archBlock(0, 0.04, 0.68, 0.38, 6, rgb(138, 82, 54), rgb(118, 122, 126), 9, false)
	a.rect(0, 0.04, 0.52, 0.10, rgb(198, 72, 54))
	// Viewing courtyard overlooking the valley
	a.rect(0, -0.28, 0.82, 0.20, rgb(178, 172, 158))
	if a.detail {
		for i := 0; i < 10; i++ {
			x := -0.36 + float64(i)*0.08
			a.circlePx(x, -0.36, 2.5, rgb(68, 64, 58))
		}
		a.treeCluster(-0.36, 0.26, 14)
		a.treeCluster(0.36, 0.26, 14)
	}
	a.sign(0, 0.42, "北岸半山森林观景台", "MOUNTAIN LOOKOUT TEAHOUSE")
}
