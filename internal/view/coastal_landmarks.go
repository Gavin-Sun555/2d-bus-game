package view

import (
	"image/color"
	"math"
)

// coastalPort renders the West Harbor Ferry Terminal (西渔港客运站).
func (a metroCanvas) coastalPort() {
	// Basin / quay slipway
	a.rect(0, -0.22, 0.94, 0.44, rgb(28, 96, 126))
	a.rect(0, -0.01, 0.96, 0.04, rgb(168, 162, 146))
	// 2 Finger piers extending into the basin
	for _, x := range []float64{-0.28, 0.28} {
		a.rect(x, -0.24, 0.12, 0.42, rgb(184, 178, 162))
		a.rect(x, -0.24, 0.08, 0.38, rgb(138, 132, 118))
		if a.detail {
			for y := -0.42; y <= -0.06; y += 0.09 {
				a.circlePx(x-0.05, y, 2.5, rgb(68, 64, 58))
				a.circlePx(x+0.05, y, 2.5, rgb(68, 64, 58))
			}
		}
	}
	// Docked passenger catamarans / ferries
	if a.detail {
		a.passengerFerry(-0.16, -0.26, rgb(38, 142, 172))
		a.passengerFerry(0.16, -0.22, rgb(224, 118, 54))
	}
	// Terminal concourse hall
	a.softPlaza(0, 0.22, 0.92, 0.48, 0.04, 0.04, rgb(206, 198, 176))
	a.archBlock(0, 0.20, 0.74, 0.32, 6, rgb(48, 76, 96), rgb(236, 240, 246), 10, true)
	a.rect(0, 0.18, 0.58, 0.08, rgb(52, 138, 178))
	a.skylight(0, 0.20, 0.42, 0.16, 5, 2)
	// Passenger entrance canopy & taxi pickup
	a.rect(0, 0.38, 0.36, 0.06, rgb(228, 148, 58))
	if a.detail {
		a.miniBus(-0.12, 0.42, 0, rgb(56, 142, 224), false)
		a.miniCar(0.12, 0.42, 0, rgb(240, 196, 38), true)
		a.treeCluster(-0.40, 0.22, 14)
		a.treeCluster(0.40, 0.22, 14)
	}
	a.sign(0, 0.44, "西渔港客运站", "WEST HARBOR FERRY TERMINAL")
}

// coastalCivic renders the Seaside Civic Plaza (港城中央广场).
func (a metroCanvas) coastalCivic() {
	// Grand circular civic plaza
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(212, 204, 184))
	a.oval(0, 0.02, 0.82, 0.76, rgb(194, 186, 168))
	a.oval(0, 0.02, 0.68, 0.62, rgb(226, 218, 202))
	// Tiered fountain
	a.oval(0, 0.02, 0.44, 0.40, rgb(168, 162, 146))
	a.oval(0, 0.02, 0.38, 0.34, rgb(38, 126, 154))
	a.oval(0, 0.02, 0.24, 0.20, rgb(72, 178, 198))
	a.circlePx(0, 0.02, 7, rgb(228, 216, 186))
	a.circlePx(0, 0.02, 3.5, rgb(238, 178, 56))
	// Flanking civic hall pavilions
	a.archBlock(-0.36, -0.06, 0.18, 0.54, 5, rgb(136, 118, 98), rgb(238, 234, 222), 9, true)
	a.archBlock(0.36, -0.06, 0.18, 0.54, 5, rgb(136, 118, 98), rgb(238, 234, 222), 9, true)
	// Colonnade pergola along north edge
	a.rect(0, -0.34, 0.56, 0.08, rgb(184, 166, 142))
	if a.detail {
		for i := 0; i < 8; i++ {
			ang := float64(i) * math.Pi / 4
			px, py := 0.32*math.Cos(ang), 0.02+0.28*math.Sin(ang)
			a.circlePx(px, py, 4.5, rgb(24, 118, 82))
		}
		a.treeCluster(-0.38, 0.32, 14)
		a.treeCluster(0.38, 0.32, 14)
	}
	a.sign(0, 0.42, "港城中央广场", "CIVIC PLAZA")
}

// coastalBeach renders the North Beach Resort (北海滨浴场站).
func (a metroCanvas) coastalBeach() {
	// Golden sandy ground & sun terraces
	a.rect(0, 0, 0.94, 0.90, rgb(228, 214, 172))
	a.rect(0, -0.22, 0.90, 0.42, rgb(238, 224, 184))
	// Wooden boardwalk promenade
	a.rect(0, 0.12, 0.92, 0.12, rgb(186, 148, 108))
	if a.detail {
		for x := -0.44; x <= 0.44; x += 0.04 {
			a.line(x, 0.06, x, 0.18, 1.0, rgb(148, 112, 78))
		}
	}
	// Beach resort bathhouse & club pavilion
	a.archBlock(0, 0.28, 0.68, 0.22, 4, rgb(168, 126, 88), rgb(246, 242, 232), 8, true)
	a.rect(0, 0.34, 0.52, 0.06, rgb(52, 146, 182))
	// Beach volleyball court
	a.rect(-0.30, -0.22, 0.24, 0.16, rgb(216, 196, 148))
	a.line(-0.30, -0.30, -0.30, -0.14, 2.0, rgb(245, 245, 245))
	// Rows of beach parasols & sun loungers
	if a.detail {
		umbrellaCols := []color.RGBA{rgb(38, 146, 214), rgb(236, 142, 42), rgb(224, 58, 62), rgb(38, 172, 118)}
		for i, x := range []float64{0.05, 0.18, 0.31, 0.42} {
			for j, y := range []float64{-0.32, -0.16} {
				col := umbrellaCols[(i+j)%len(umbrellaCols)]
				a.rect(x-0.02, y, 0.03, 0.05, rgb(240, 238, 228))
				a.circlePx(x, y, 6.0, col)
				a.circlePx(x, y, 2.0, rgb(255, 255, 255))
			}
		}
		a.treeCluster(-0.40, 0.30, 13)
		a.treeCluster(0.40, 0.30, 13)
	}
	a.sign(0, 0.42, "北海滨浴场站", "NORTH BEACH RESORT")
}

// coastalResort renders Golden Beach Luxury Resort & Pool (金色沙滩度假区).
func (a metroCanvas) coastalResort() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(222, 212, 188))
	// Luxury hotel wings (staggered crescent)
	a.archBlock(-0.16, -0.18, 0.58, 0.28, 6, rgb(148, 112, 82), rgb(244, 240, 232), 10, true)
	a.archBlock(0.28, -0.12, 0.32, 0.34, 5, rgb(148, 112, 82), rgb(244, 240, 232), 9, true)
	a.skylight(-0.16, -0.18, 0.32, 0.12, 4, 2)
	// Grand curved turquoise pool & sun deck
	a.rect(-0.08, 0.16, 0.46, 0.26, rgb(234, 226, 206))
	a.oval(-0.08, 0.16, 0.40, 0.22, rgb(42, 156, 178))
	a.oval(-0.08, 0.16, 0.32, 0.16, rgb(68, 192, 212))
	// Private cabanas & sun deck loungers
	if a.detail {
		for _, x := range []float64{-0.26, -0.21, 0.06, 0.11} {
			a.circlePx(x, 0.26, 4.5, rgb(240, 142, 54))
		}
		a.rect(0.30, 0.18, 0.22, 0.18, rgb(188, 152, 114))
		a.treeCluster(0.32, 0.18, 15)
		a.treeCluster(-0.38, -0.18, 14)
	}
	// Entrance porte-cochère
	a.rect(0, 0.38, 0.42, 0.07, rgb(206, 142, 68))
	a.sign(0, 0.44, "金色沙滩度假区", "GOLDEN BEACH RESORT")
}

// coastalBridgeWest renders the Bay Bridge West Observation Visitor Center (跨海大桥西岸站).
func (a metroCanvas) coastalBridgeWest() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(202, 206, 210))
	// Modern cantilevered visitor center
	a.archBlock(-0.10, 0.05, 0.62, 0.38, 6, rgb(58, 72, 84), rgb(234, 238, 244), 10, true)
	a.rect(-0.10, 0.05, 0.48, 0.12, rgb(52, 138, 182))
	a.skylight(-0.10, 0.05, 0.36, 0.08, 4, 1)
	// Cantilevered observation deck overlooking the bridge
	a.rect(0.32, 0.05, 0.26, 0.32, rgb(168, 176, 184))
	a.rect(0.32, 0.05, 0.22, 0.28, rgb(74, 92, 106))
	// Suspension cable anchor monument
	a.rect(-0.36, -0.24, 0.14, 0.22, rgb(122, 132, 140))
	a.rect(0.36, -0.24, 0.14, 0.22, rgb(122, 132, 140))
	a.line(-0.36, -0.24, 0.36, -0.24, 3.0, rgb(214, 148, 54))
	if a.detail {
		a.circlePx(0.34, -0.04, 3.5, rgb(240, 204, 62))
		a.circlePx(0.34, 0.14, 3.5, rgb(240, 204, 62))
		a.treeCluster(-0.36, 0.26, 13)
	}
	a.sign(0, 0.42, "跨海大桥西岸站", "BAY BRIDGE WEST VISITOR CENTER")
}

// coastalDeepwaterPort renders Deepwater Container Terminal (东海深水码头总站).
func (a metroCanvas) coastalDeepwaterPort() {
	// Concrete apron quay
	a.rect(0, 0, 0.96, 0.92, rgb(128, 136, 142))
	a.rect(0, -0.38, 0.96, 0.14, rgb(98, 106, 112))
	// Water channel edge
	a.rect(0, -0.46, 0.96, 0.04, rgb(24, 88, 122))
	// Ship-to-Shore (STS) container gantry cranes
	stsCol := rgb(236, 92, 38)
	for _, x := range []float64{-0.32, 0.04, 0.36} {
		a.rect(x, -0.34, 0.18, 0.14, stsCol)
		a.rect(x, -0.34, 0.12, 0.08, rgb(68, 76, 84))
		a.line(x-0.08, -0.44, x-0.08, -0.24, 2.5, stsCol)
		a.line(x+0.08, -0.44, x+0.08, -0.24, 2.5, stsCol)
		a.line(x-0.10, -0.42, x+0.10, -0.42, 2.0, rgb(245, 245, 245))
	}
	// Colored shipping container stacks (Maersk cyan, CMA red, Evergreen green, Hapag orange)
	cCols := []color.RGBA{rgb(42, 156, 196), rgb(212, 54, 48), rgb(38, 138, 78), rgb(234, 138, 42), rgb(224, 186, 38)}
	for row := 0; row < 3; row++ {
		y := -0.12 + float64(row)*0.16
		for col := 0; col < 6; col++ {
			x := -0.38 + float64(col)*0.15
			cc := cCols[(row*3+col)%len(cCols)]
			a.rect(x, y, 0.12, 0.08, cc)
			a.rectPx(x, y, a.width*0.10, a.depth*0.06, tint(cc, rgb(255, 255, 255), 0.2))
		}
	}
	// Port operations building
	a.archBlock(-0.24, 0.36, 0.38, 0.16, 4, rgb(54, 68, 78), rgb(228, 234, 240), 7, true)
	if a.detail {
		a.miniCar(0.18, 0.36, 0, rgb(234, 142, 42), false)
		a.miniCar(0.30, 0.36, 0, rgb(240, 240, 240), false)
	}
	a.sign(0, 0.44, "东海深水码头总站", "DEEPWATER CONTAINER TERMINAL")
}

// coastalBondedPort renders the Bonded Port Logistics Hub (保税港集装箱区).
func (a metroCanvas) coastalBondedPort() {
	a.rect(0, 0, 0.94, 0.90, rgb(134, 142, 148))
	// Logistics warehouse with sawtooth roofs
	a.archBlock(0, -0.12, 0.86, 0.44, 4, rgb(48, 62, 74), rgb(218, 226, 234), 9, false)
	a.skylight(0, -0.12, 0.72, 0.28, 6, 2)
	// Loading dock bays with trailers
	a.rect(0, 0.14, 0.82, 0.06, rgb(88, 96, 102))
	if a.detail {
		for _, x := range []float64{-0.30, -0.10, 0.10, 0.30} {
			a.rect(x, 0.22, 0.07, 0.12, rgb(224, 232, 238))
			a.rect(x, 0.27, 0.06, 0.04, rgb(214, 68, 54))
		}
	}
	// Administration office & security gate
	a.archBlock(-0.24, 0.36, 0.38, 0.14, 3, rgb(52, 66, 78), rgb(236, 240, 246), 6, true)
	a.rect(0.24, 0.36, 0.34, 0.10, rgb(184, 148, 48))
	a.sign(0, 0.44, "保税港集装箱区", "BONDED PORT LOGISTICS")
}

// coastalYachtClub renders the International Yacht Club (游艇国际俱乐部).
func (a metroCanvas) coastalYachtClub() {
	// Marina water slip
	a.rect(0, -0.20, 0.94, 0.48, rgb(26, 102, 134))
	// Floating finger pier
	a.rect(0, -0.22, 0.88, 0.05, rgb(182, 146, 106))
	for _, x := range []float64{-0.32, -0.08, 0.16, 0.36} {
		a.rect(x, -0.28, 0.03, 0.24, rgb(168, 134, 98))
	}
	// Docked luxury motor yachts
	if a.detail {
		for _, x := range []float64{-0.20, 0.04, 0.26} {
			pos := a.point(x, -0.26)
			drawVehicleShell(a.screen, a.cam, pos, a.lot.heading-math.Pi/2, 44, 17, rgb(244, 248, 252), 6, 4)
			drawVehicleShell(a.screen, a.cam, pos, a.lot.heading-math.Pi/2, 26, 11, rgb(38, 76, 98), 4, 3)
		}
	}
	// Clubhouse with sail-inspired curved canopy
	a.softPlaza(0, 0.22, 0.92, 0.46, 0.04, 0.04, rgb(218, 212, 194))
	a.archBlock(0, 0.18, 0.66, 0.28, 6, rgb(138, 108, 78), rgb(246, 242, 234), 9, true)
	a.oval(0, 0.18, 0.44, 0.18, rgb(248, 248, 252))
	// Lawn terrace with umbrellas
	if a.detail {
		for _, x := range []float64{-0.28, 0.28} {
			a.circlePx(x, 0.20, 5, rgb(48, 148, 214))
			a.circlePx(x, 0.28, 5, rgb(234, 138, 48))
		}
		a.treeCluster(-0.38, 0.32, 14)
		a.treeCluster(0.38, 0.32, 14)
	}
	a.sign(0, 0.42, "游艇国际俱乐部", "INTERNATIONAL YACHT CLUB")
}

// coastalFishermanWharf renders Fisherman's Wharf & Seafood Market (渔人码头海鲜街).
func (a metroCanvas) coastalFishermanWharf() {
	// Timber deck over water
	a.rect(0, -0.16, 0.94, 0.52, rgb(176, 138, 98))
	a.rect(0, -0.44, 0.94, 0.08, rgb(26, 98, 128))
	// Planks
	if a.detail {
		for x := -0.44; x <= 0.44; x += 0.05 {
			a.line(x, -0.40, x, 0.08, 1.0, rgb(142, 108, 74))
		}
	}
	// Seafood market arcade stalls with colorful peaked awnings
	awningCols := []color.RGBA{rgb(214, 62, 54), rgb(42, 148, 196), rgb(234, 156, 42), rgb(38, 162, 108), rgb(168, 72, 122)}
	for i, x := range []float64{-0.34, -0.17, 0, 0.17, 0.34} {
		col := awningCols[i%len(awningCols)]
		a.archBlock(x, -0.12, 0.14, 0.22, 3, rgb(98, 72, 52), col, 6, false)
		a.rect(x, 0.02, 0.12, 0.05, rgb(236, 228, 212))
	}
	// Main seafood restaurant hall
	a.archBlock(0, 0.26, 0.78, 0.26, 5, rgb(118, 86, 62), rgb(244, 238, 226), 8, true)
	a.rect(0, 0.34, 0.58, 0.06, rgb(218, 118, 54))
	if a.detail {
		a.treeCluster(-0.38, 0.32, 12)
		a.treeCluster(0.38, 0.32, 12)
	}
	a.sign(0, 0.42, "渔人码头海鲜街", "FISHERMAN'S WHARF")
}

// coastalSailingMarina renders Sailing Yacht Marina (滨海帆船游艇港).
func (a metroCanvas) coastalSailingMarina() {
	// Basin water
	a.rect(0, -0.18, 0.94, 0.52, rgb(24, 102, 134))
	// Wooden pontoons
	a.rect(0, -0.20, 0.88, 0.04, rgb(186, 152, 112))
	for _, x := range []float64{-0.30, 0, 0.30} {
		a.rect(x, -0.28, 0.03, 0.22, rgb(168, 136, 98))
	}
	// Moored sailing yachts (slender white hulls and silver masts)
	if a.detail {
		for _, x := range []float64{-0.38, -0.15, 0.15, 0.38} {
			pos := a.point(x, -0.26)
			drawVehicleShell(a.screen, a.cam, pos, a.lot.heading-math.Pi/2, 36, 12, rgb(246, 248, 250), 5, 3)
			a.circlePx(x, -0.26, 2.5, rgb(188, 196, 204))
		}
	}
	// Sailing center clubhouse
	a.softPlaza(0, 0.24, 0.92, 0.44, 0.04, 0.04, rgb(214, 208, 192))
	a.archBlock(0, 0.20, 0.70, 0.26, 5, rgb(48, 76, 96), rgb(238, 242, 248), 8, true)
	a.skylight(0, 0.20, 0.48, 0.12, 4, 1)
	if a.detail {
		a.treeCluster(-0.38, 0.28, 13)
		a.treeCluster(0.38, 0.28, 13)
	}
	a.sign(0, 0.42, "滨海帆船游艇港", "MARINA YACHT DOCKS")
}

// coastalContainerHub renders Automated Smart Container Hub (深水港智能集装箱总站).
func (a metroCanvas) coastalContainerHub() {
	a.rect(0, 0, 0.96, 0.92, rgb(126, 134, 140))
	// Intermodal rail transfer tracks
	for _, y := range []float64{-0.34, -0.26} {
		a.line(-0.46, y, 0.46, y, 2.0, rgb(188, 196, 202))
		if a.detail {
			for x := -0.44; x <= 0.44; x += 0.04 {
				a.line(x, y-0.02, x, y+0.02, 1.2, rgb(96, 88, 78))
			}
		}
	}
	// Automated Stacking Crane (ASC) rail gantry
	ascCol := rgb(48, 142, 214)
	a.rect(0, -0.06, 0.88, 0.04, ascCol)
	a.line(-0.42, -0.18, -0.42, 0.08, 3.0, ascCol)
	a.line(0.42, -0.18, 0.42, 0.08, 3.0, ascCol)
	// Matrix of container stacks
	cCols := []color.RGBA{rgb(42, 156, 196), rgb(212, 54, 48), rgb(38, 138, 78), rgb(234, 138, 42)}
	for row := 0; row < 2; row++ {
		y := 0.04 + float64(row)*0.14
		for col := 0; col < 6; col++ {
			x := -0.38 + float64(col)*0.15
			cc := cCols[(row*2+col)%len(cCols)]
			a.rect(x, y, 0.12, 0.08, cc)
		}
	}
	// Operations control tower
	a.archBlock(-0.28, 0.36, 0.30, 0.16, 4, rgb(52, 66, 78), rgb(236, 240, 246), 7, true)
	a.sign(0, 0.44, "深水港智能集装箱总站", "SMART CONTAINER HUB")
}

// coastalLighthouse renders the Cape Lighthouse Lookout (海角海景灯塔观景台).
func (a metroCanvas) coastalLighthouse() {
	// Rocky promontory foundation
	a.oval(0, -0.02, 0.88, 0.84, rgb(154, 146, 132))
	a.oval(0, -0.02, 0.74, 0.70, rgb(178, 170, 154))
	// Circular observation deck & safety balustrade
	a.oval(0, -0.02, 0.58, 0.54, rgb(216, 210, 194))
	if a.detail {
		for i := 0; i < 16; i++ {
			ang := float64(i) * math.Pi / 8
			px, py := 0.28*math.Cos(ang), -0.02+0.26*math.Sin(ang)
			a.circlePx(px, py, 2.0, rgb(68, 64, 58))
		}
	}
	// Iconic tapered lighthouse tower (red & white alternating bands)
	a.circlePx(0, -0.02, 38.0, rgb(214, 54, 46))
	a.circlePx(0, -0.02, 32.0, rgb(248, 248, 252))
	a.circlePx(0, -0.02, 26.0, rgb(214, 54, 46))
	a.circlePx(0, -0.02, 20.0, rgb(248, 248, 252))
	// Black lantern gallery room & golden beacon
	a.circlePx(0, -0.02, 14.0, rgb(38, 42, 48))
	a.circlePx(0, -0.02, 8.5, rgb(248, 204, 54))
	a.circlePx(0, -0.02, 4.0, rgb(255, 255, 255))
	// Keeper's cottage & weather radar dome
	a.archBlock(-0.28, 0.22, 0.24, 0.24, 3, rgb(138, 108, 82), rgb(244, 240, 230), 6, true)
	a.oval(0.28, 0.22, 0.18, 0.18, rgb(238, 242, 248))
	if a.detail {
		a.treeCluster(-0.34, -0.22, 11)
		a.treeCluster(0.34, -0.22, 11)
	}
	a.sign(0, 0.40, "海角海景灯塔观景台", "CAPE LIGHTHOUSE LOOKOUT")
}
