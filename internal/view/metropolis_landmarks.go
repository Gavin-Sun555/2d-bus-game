package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
)

func (a metroCanvas) drawLotTitle(x, y float64, defaultZh, defaultEn string) {
	if !a.lot.landmarkLabel || !a.detail || a.cam.Zoom < .44 {
		return
	}
	zh, en := a.lot.titleZh, a.lot.titleEn
	if zh == "" {
		zh, en = defaultZh, defaultEn
	}
	a.renderBadge(x, y, zh, en)
}

func (a metroCanvas) sign(x, y float64, zh, en string) {
	a.drawLotTitle(x, y, zh, en)
}

func (a metroCanvas) renderBadge(x, y float64, zh, en string) {
	center := a.cam.WorldToScreen(a.point(x, y))
	label := locale.T(zh, en)
	size := sim.Clamp(a.cam.Zoom*16.5, 10, 16.5)
	w, h := textui.MeasureText(label, size)
	filledRect(a.screen, float32(center.X-w/2-7), float32(center.Y-h/2-4), float32(w+14), float32(h+8), sim.RGBA(14, 26, 38, 232), true)
	strokeRect(a.screen, float32(center.X-w/2-7), float32(center.Y-h/2-4), float32(w+14), float32(h+8), 1.2, sim.RGBA(96, 165, 198, 210), true)
	textui.DrawText(a.screen, label, center.X-w/2, center.Y-h/2, size, rgb(244, 239, 220))
}

func (a metroCanvas) landmark() {
	switch a.lot.style {
	case metroRail:
		a.railway()
	case metroConventionalRail:
		a.conventionalRailway()
	case metroTerminal:
		a.terminal()
	case metroStadium:
		a.stadium()
	case metroHospital:
		a.hospital()
	case metroCampus:
		a.campus()
		if a.lot.variant == 7 {
			a.drawLotTitle(0, .36, "大学城园区", "UNIVERSITY PARK")
		} else {
			a.drawLotTitle(0, .36, "科技创新园区", "TECH CAMPUS")
		}
	case metroExhibit:
		a.exhibition()
	case metroCanal:
		a.canalPort()
	case metroPlaza:
		a.plaza()

	// Coastal Bay landmarks
	case coastalPort:
		a.coastalPort()
	case coastalCivic:
		a.coastalCivic()
	case coastalBeach:
		a.coastalBeach()
	case coastalResort:
		a.coastalResort()
	case coastalBridgeWest:
		a.coastalBridgeWest()
	case coastalDeepwaterPort:
		a.coastalDeepwaterPort()
	case coastalBondedPort:
		a.coastalBondedPort()
	case coastalYachtClub:
		a.coastalYachtClub()
	case coastalFishermanWharf:
		a.coastalFishermanWharf()
	case coastalSailingMarina:
		a.coastalSailingMarina()
	case coastalContainerHub:
		a.coastalContainerHub()
	case coastalLighthouse:
		a.coastalLighthouse()

	// River Valley landmarks
	case valleyRailHub:
		a.valleyRailHub()
	case valleyHSRHQ:
		a.valleyHSRHQ()
	case valleyFinancialTowers:
		a.valleyFinancialTowers()
	case valleyCultureVista:
		a.valleyCultureVista()
	case valleyNorthForest:
		a.valleyNorthForest()
	case valleyLookout:
		a.valleyLookout()
	case valleyEcoNorth:
		a.valleyEcoNorth()
	case valleyBridgeheadNorth:
		a.valleyBridgeheadNorth()
	case valleyBridgeheadSouth:
		a.valleyBridgeheadSouth()
	case valleySmartCampus:
		a.valleySmartCampus()
	case valleyCivicCenter:
		a.valleyCivicCenter()
	case valleyMountainLookout:
		a.valleyMountainLookout()

	// Twin Cities landmarks
	case twinAirportTerminal:
		a.twinAirportTerminal()
	case twinAirportLogistics:
		a.twinAirportLogistics()
	case twinRoboticsCampus:
		a.twinRoboticsCampus()
	case twinGigaFactory:
		a.twinGigaFactory()
	case twinRingCampus:
		a.twinRingCampus()
	case twinUniversity:
		a.twinUniversity()
	case twinAIScienceCity:
		a.twinAIScienceCity()
	case twinUnicornHQ:
		a.twinUnicornHQ()
	case twinIndustrialWest:
		a.twinIndustrialWest()
	case twinInterchangeWest:
		a.twinInterchangeWest()
	case twinInterchangeEast:
		a.twinInterchangeEast()
	case twinSmartCityCenter:
		a.twinSmartCityCenter()
	}
}

func (a metroCanvas) plaza() {
	// Radial stone civic square with colonnade pergolas, concentric water fountain & spire.
	a.oval(0, -0.02, 0.90, 0.84, rgb(174, 168, 152))
	a.oval(0, -0.02, 0.78, 0.72, rgb(146, 154, 158))
	if a.detail {
		for i := 0; i < 8; i++ {
			ang := float64(i) * math.Pi / 4
			a.line(0, -0.02, 0.42*math.Cos(ang), -0.02+0.38*math.Sin(ang), 3.0, rgb(186, 180, 164))
		}
	}
	// Tiered water fountain basin & central monument obelisk.
	a.oval(0, -0.02, 0.50, 0.52, rgb(188, 184, 170))
	a.oval(0, -0.02, 0.44, 0.46, rgb(34, 114, 136))
	a.oval(0, -0.02, 0.30, 0.32, rgb(68, 162, 178))
	a.oval(0, -0.02, 0.16, 0.18, rgb(144, 214, 222))
	a.circlePx(0, -0.02, 9.0, rgb(226, 212, 176))
	a.circlePx(0, -0.02, 4.5, rgb(238, 182, 76))
	// Flanking architectural pergolas & corner tree beds.
	for _, x := range []float64{-0.36, 0.36} {
		a.archBlock(x, -0.02, 0.11, 0.36, 4, rgb(118, 96, 82), rgb(172, 148, 128), 8, false)
		a.treeCluster(x, -0.28, 12)
		a.treeCluster(x, 0.24, 12)
	}
	a.sign(0, .33, "市民环岛广场", "CIVIC PLAZA")
}

func (a metroCanvas) railPlatform(y float64, roof color.RGBA) {
	a.shell(0, y, .90, .14, 5, rgb(169, 178, 183), 5)
	a.rect(0, y, .78, .065, roof)
	for _, edge := range [...]float64{-.056, .056} {
		a.line(-.43, y+edge, .43, y+edge, 1.6, rgb(236, 186, 72))
	}
}

func (a metroCanvas) conventionalRailway() {
	a.railPlatform(-.25, rgb(83, 119, 101))
	a.softPlaza(0, .28, .86, .41, .05, .05, rgb(157, 148, 132))
	a.archBlock(0, .17, .68, .30, 9, rgb(144, 98, 74), rgb(190, 122, 92), 7, true)
	a.rect(0, .17, .61, .05, rgb(214, 157, 119))
	a.shell(0, .34, .38, .08, 4, rgb(53, 97, 85), 4)
	a.sign(0, .23, "都会火车站", "RAILWAY STATION")
}

func (a metroCanvas) railway() {
	a.railPlatform(-.255, rgb(104, 136, 152))
	// Stone forecourt plaza connecting the station concourse to the street sidewalk.
	a.softPlaza(0, 0.24, 0.90, 0.44, 0.08, 0.08, rgb(124, 134, 140))

	// Monumental wave-roofed High-Speed Station Concourse Building (-0.01 .. +0.34).
	a.archBlock(0, 0.16, 0.90, 0.36, 18, rgb(146, 162, 172), rgb(206, 216, 222), 18, true)
	a.archBlock(0, 0.15, 0.68, 0.26, 12, rgb(74, 118, 142), rgb(176, 194, 204), 8, false)
	// Central diamond/grid vaulted glass atrium skylight + HVAC chiller banks.
	a.skylight(0, 0.15, 0.46, 0.16, 7, 3)
	a.chillerBank(-0.32, 0.15, 3)
	a.chillerBank(0.32, 0.15, 3)
	if a.detail {
		for i := -4; i <= 4; i++ {
			rx := float64(i) * 0.095
			a.line(rx, -0.01, rx, 0.32, 1.8, rgb(162, 178, 186))
		}
	}
	// Cantilevered front departure hall glass canopy & red station emblem.
	a.shell(0, 0.345, 0.42, 0.085, 8, rgb(38, 88, 114), 8)
	a.rect(0, 0.345, 0.36, 0.045, rgb(118, 182, 204))
	a.rect(0, 0.315, 0.12, 0.028, rgb(214, 58, 52))
	// Forecourt drop-off lane, waiting taxis & flanking station greenery.
	a.miniCar(-0.26, 0.41, 0, rgb(235, 186, 46), true)
	a.miniCar(0.26, 0.41, 0, rgb(235, 186, 46), true)
	a.treeCluster(-0.44, 0.38, 11.5)
	a.treeCluster(0.44, 0.38, 11.5)
	a.sign(0, .15, "高铁站", "HIGH-SPEED RAILWAY STATION")
}

func (a metroCanvas) terminal() {
	// Central Bus Terminal (中央枢纽):
	// 1. Rear concourse terminal building (-0.42 .. -0.06) with vaulted skylight & dispatch tower.
	a.archBlock(0, -0.22, 0.88, 0.34, 16, rgb(58, 98, 114), rgb(156, 186, 192), 16, true)
	a.skylight(0, -0.22, 0.52, 0.16, 6, 2)
	a.chillerBank(-0.34, -0.22, 2)
	a.chillerBank(0.34, -0.22, 2)
	// Cantilevered boarding concourse canopy.
	a.rect(0, -0.02, 0.84, 0.075, rgb(84, 146, 162))
	a.line(-0.42, 0.015, 0.42, 0.015, 2.0, rgb(236, 186, 72))

	// 2. Front asphalt bus apron & 5 parallel boarding islands with docked transit buses.
	a.softPlaza(0, 0.18, 0.88, 0.38, 0.08, 0.08, rgb(42, 52, 62))
	if a.detail {
		// Pedestrian safety crosswalk across the boarding bays.
		for i := -8; i <= 8; i++ {
			a.rect(float64(i)*0.048, 0.045, 0.024, 0.032, rgb(226, 232, 236))
		}
	}
	busColors := [...]color.RGBA{
		rgb(14, 165, 233),
		rgb(16, 185, 129),
		rgb(245, 158, 11),
		rgb(59, 130, 246),
	}
	for i := 0; i < 5; i++ {
		ix := -0.36 + float64(i)*0.18
		// Concrete boarding island + yellow safety curb + glass shelter roof.
		a.shell(ix, 0.19, 0.068, 0.28, 4, rgb(138, 150, 158), 5)
		a.line(ix+0.034, 0.06, ix+0.034, 0.32, 1.8, rgb(236, 184, 68))
		a.rect(ix, 0.18, 0.044, 0.18, rgb(48, 108, 128))
		if i < 4 {
			bx := ix + 0.09
			a.miniBus(bx, 0.19, math.Pi/2, busColors[i], i == 1)
		}
	}
	a.sign(0, -.22, "中央枢纽", "CENTRAL TERMINAL")
}

func (a metroCanvas) stadium() {
	// Olympic Stadium (奥体中心):
	// 1. Outer plaza apron & 4 corner floodlight towers.
	a.oval(0, -0.02, 0.95, 0.90, rgb(186, 192, 196))
	for _, sx := range []float64{-0.39, 0.39} {
		for _, sy := range []float64{-0.34, 0.30} {
			a.circlePx(sx, sy, 8.5, rgb(76, 92, 104))
			a.circlePx(sx, sy, 4.5, rgb(236, 242, 246))
		}
	}
	// 2. Elliptical tensioned-membrane stadium canopy with radial steel truss ribs.
	a.oval(0, -0.03, 0.88, 0.82, rgb(92, 114, 128))
	a.oval(0, -0.03, 0.85, 0.79, rgb(222, 228, 232))
	if a.detail {
		for i := 0; i < 24; i++ {
			ang := float64(i) * math.Pi * 2 / 24
			c, s := math.Cos(ang), math.Sin(ang)
			a.line(c*0.28, -0.03+s*0.23, c*0.425, -0.03+s*0.395, 1.8, rgb(154, 168, 178))
		}
	}
	// 3. Inner spectator seating tiers & 8-lane brick-red Tartan athletics track.
	a.oval(0, -0.03, 0.68, 0.60, rgb(44, 62, 78))
	a.oval(0, -0.03, 0.62, 0.54, rgb(68, 92, 112))
	a.oval(0, -0.03, 0.55, 0.46, rgb(186, 74, 60))
	if a.detail {
		a.oval(0, -0.03, 0.51, 0.42, rgb(232, 224, 214))
		a.oval(0, -0.03, 0.495, 0.405, rgb(186, 74, 60))
	}
	// 4. Striped green turf football pitch & complete white pitch markings.
	a.oval(0, -0.03, 0.44, 0.34, rgb(48, 128, 72))
	a.rect(0, -0.03, 0.32, 0.23, rgb(58, 144, 82))
	if a.detail {
		for i := -2; i <= 2; i++ {
			if i%2 == 0 {
				a.rect(float64(i)*0.056, -0.03, 0.056, 0.22, rgb(46, 124, 68))
			}
		}
		// White touchlines, halfway line, center circle & penalty areas.
		a.line(-0.16, -0.14, 0.16, -0.14, 1.5, rgb(236, 242, 236))
		a.line(-0.16, 0.08, 0.16, 0.08, 1.5, rgb(236, 242, 236))
		a.line(-0.16, -0.14, -0.16, 0.08, 1.5, rgb(236, 242, 236))
		a.line(0.16, -0.14, 0.16, 0.08, 1.5, rgb(236, 242, 236))
		a.line(0, -0.14, 0, 0.08, 1.5, rgb(236, 242, 236))
		a.oval(0, -0.03, 0.075, 0.095, rgb(236, 242, 236))
		a.oval(0, -0.03, 0.062, 0.078, rgb(46, 124, 68))
	}
	// Grand southern entrance steps.
	a.rect(0, 0.38, 0.32, 0.065, rgb(148, 158, 166))
	a.sign(0, .37, "奥体中心", "OLYMPIC STADIUM")
}

func (a metroCanvas) hospital() {
	// City Hospital / Medical Center (市立第一医院):
	// H-shaped clinical white & teal-glass medical complex with rooftop emergency helipad,
	// Red Cross emergency canopy, and parked ambulances.
	a.softPlaza(0, 0.28, 0.80, 0.24, 0.08, 0.06, rgb(58, 70, 80))
	// Connecting inpatient corridor wing.
	a.archBlock(0, -0.04, 0.78, 0.26, 8, rgb(166, 188, 194), rgb(212, 224, 228), 12, true)
	// West & East medical ward wings with skylights & HVAC chillers.
	for _, x := range []float64{-0.30, 0.30} {
		a.archBlock(x, -0.02, 0.24, 0.64, 9, rgb(154, 180, 188), rgb(216, 228, 230), 14, true)
		a.skylight(x, 0.08, 0.12, 0.24, 2, 3)
		a.hvac(x, -0.22, 18, 13)
	}
	// Central high-rise inpatient tower + rooftop aviation helipad ('H').
	a.archBlock(0, -0.06, 0.32, 0.68, 12, rgb(142, 172, 182), rgb(226, 236, 238), 18, true)
	a.chillerBank(0, 0.12, 2)
	a.helipad(0, -0.15, 22)

	// Emergency Department (急诊) drop-off canopy with bold Red Cross emblem.
	a.shell(0, 0.27, 0.26, 0.14, 6, rgb(196, 212, 216), 8)
	a.rect(0, 0.27, 0.048, 0.095, rgb(216, 48, 48))
	a.rect(0, 0.27, 0.11, 0.038, rgb(216, 48, 48))
	// Parked emergency ambulances in the front bay.
	a.miniAmbulance(-0.24, 0.34, 0)
	a.miniAmbulance(0.24, 0.34, 0)
	a.treeCluster(-0.42, 0.34, 10)
	a.treeCluster(0.42, 0.34, 10)
	a.sign(0, .36, "市立第一医院", "CITY HOSPITAL")
}

func (a metroCanvas) campus() {
	isSchoolOrUni := a.lot.variant == 7 || (a.lot.titleZh != "" && a.lot.variant >= 12)
	if isSchoolOrUni {
		// University / Secondary School / Library Campus on organic lawn quads (no rectangular box).
		a.softLawn(-0.14, 0.02, 0.62, 0.68)
		a.softLawn(0.16, 0.02, 0.62, 0.68)
		a.rect(0, 0.06, 0.09, 0.66, rgb(168, 164, 148))
		a.rect(0, -0.02, 0.58, 0.065, rgb(168, 164, 148))
		// Central Library Rotunda with copper-teal dome & radial skylight.
		a.circlePx(0, -0.02, 28, rgb(186, 180, 164))
		a.circlePx(0, -0.02, 22, rgb(48, 124, 138))
		a.circlePx(0, -0.02, 10, rgb(118, 186, 198))
		// West Academic Lecture Wing.
		a.archBlock(-0.30, -0.02, 0.28, 0.62, 9, rgb(148, 92, 76), rgb(194, 156, 138), 14, true)
		a.skylight(-0.30, -0.06, 0.14, 0.28, 2, 3)
		a.hvac(-0.30, 0.18, 18, 12)
		// North-East Science Wing.
		a.archBlock(0.28, -0.18, 0.30, 0.28, 8, rgb(76, 106, 124), rgb(148, 172, 184), 12, true)
		a.hvac(0.28, -0.18, 18, 12)
		// South-East Campus Sports Court (Basketball court with red key & white lines).
		a.rect(0.28, 0.14, 0.28, 0.28, rgb(46, 114, 86))
		a.rect(0.28, 0.14, 0.22, 0.21, rgb(184, 84, 66))
		if a.detail {
			a.line(0.17, 0.14, 0.39, 0.14, 1.3, rgb(238, 242, 238))
			a.oval(0.28, 0.14, 0.06, 0.07, rgb(238, 242, 238))
		}
		a.treeCluster(-0.11, 0.22, 10.5)
		a.treeCluster(0.11, 0.22, 10.5)
		a.treeCluster(-0.44, 0.30, 10.5)
		a.treeCluster(0.44, 0.30, 10.5)
		return
	}
	// High-Tech R&D Campus (科技创新园区):
	a.softLawn(0, 0.02, 0.54, 0.62)
	a.oval(0, 0.02, 0.24, 0.42, rgb(36, 112, 132))
	a.oval(0, 0.02, 0.16, 0.30, rgb(76, 162, 178))
	a.archBlock(0, -0.04, 0.34, 0.18, 6, rgb(48, 88, 112), rgb(118, 168, 188), 10, true)
	for _, x := range []float64{-0.29, 0.29} {
		a.archBlock(x, -0.02, 0.30, 0.62, 12, rgb(52, 88, 112), rgb(128, 158, 172), 16, true)
		a.solarBank(x, -0.10, 0.18, 0.28, 3, 2)
		a.chillerBank(x, 0.16, 2)
	}
	a.treeCluster(-0.14, 0.28, 10.5)
	a.treeCluster(0.14, 0.28, 10.5)
}

func (a metroCanvas) exhibition() {
	// International Expo Center / Cultural Hall (国际会展中心):
	// Long-span metallic wave-shell exhibition halls separated by continuous glass skylights
	// and spanned by structural steel arch trusses.
	a.archBlock(0, -0.03, 0.90, 0.64, 16, rgb(78, 106, 122), rgb(148, 168, 178), 18, true)
	for i := 0; i < 5; i++ {
		hx := -0.34 + float64(i)*0.17
		a.shell(hx, -0.04, 0.135, 0.54, 8, rgb(212, 220, 226), 6)
		a.rect(hx, -0.04, 0.085, 0.46, rgb(232, 238, 242))
		if i < 4 {
			a.skylight(hx+0.085, -0.04, 0.032, 0.48, 1, 5)
		}
	}
	if a.detail {
		// Transverse structural steel arch trusses across the exhibition halls.
		for _, ty := range []float64{-0.18, -0.04, 0.10} {
			a.line(-0.42, ty, 0.42, ty, 2.2, rgb(118, 142, 156))
		}
	}
	// Glazed front entrance concourse foyer & ceremonial flagpoles.
	a.shell(0, 0.29, 0.56, 0.095, 8, rgb(32, 78, 104), 8)
	a.rect(0, 0.29, 0.48, 0.048, rgb(118, 184, 206))
	if a.detail {
		for i := -3; i <= 3; i++ {
			if i != 0 {
				a.circlePx(float64(i)*0.09, 0.37, 3.2, metroAwningColors[(i+4)%len(metroAwningColors)])
			}
		}
	}
	a.treeCluster(-0.38, 0.35, 11)
	a.treeCluster(0.38, 0.35, 11)
	a.sign(0, .32, "国际会展中心", "EXPO CENTER")
}

// passengerFerry renders a realistic passenger water-bus / ferry vessel at the Canal Port.
func (a metroCanvas) passengerFerry(x, y float64, trimCol color.RGBA) {
	pos := a.point(x, y)
	heading := a.lot.heading + math.Pi/2
	u := sim.UnitFromAngle(heading)
	length, width := 68.0, 22.0
	// Soft water wake & hull shadow.
	drawRotatedBox(a.screen, a.cam, pos.Add(sim.Vec2{X: 3, Y: 4}), u.X, u.Y, length, width, sim.RGBA(12, 38, 52, 95))
	drawVehicleShell(a.screen, a.cam, pos, heading, length, width, rgb(236, 242, 246), 9.0, 4.5)
	// Cabin superstructure, wrap-around dark glass windows &colored sun deck.
	drawVehicleShell(a.screen, a.cam, pos.Sub(u.Mul(3)), heading, length*0.68, width-5.0, rgb(22, 46, 62), 5.0, 3.0)
	drawVehicleShell(a.screen, a.cam, pos.Sub(u.Mul(4)), heading, length*0.56, width-8.0, rgb(224, 232, 238), 4.0, 2.5)
	drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(4)), u.X, u.Y, length*0.36, width-11.0, trimCol)
	if a.detail {
		ac := pos.Sub(u.Mul(6))
		drawRotatedBox(a.screen, a.cam, ac, u.X, u.Y, 10, 7, rgb(172, 186, 194))
	}
}

func (a metroCanvas) canalPort() {
	// West Canal Passenger Port (西运河客运港):
	// The harbour basin and West Grand Canal water (-0.50 .. +0.005) are rendered
	// continuously by drawMetropolisCanalCorridor so the port opens directly onto
	// the connected river and sea estuary without any clipped water box.
	if !a.lot.landmarkLabel {
		a.rect(0, -0.22, 0.94, 0.48, rgb(26, 86, 110))
		a.rect(0, -0.02, 0.94, 0.09, rgb(46, 122, 142))
	}
	// Waterfront stone forecourt & quay promenade on the landward side (+0.01 .. +0.46).
	a.softPlaza(0, 0.24, 0.88, 0.44, 0.07, 0.07, rgb(142, 144, 136))
	a.rect(0, 0.045, 0.92, 0.085, rgb(164, 158, 142))
	a.line(-0.45, 0.005, 0.45, 0.005, 2.4, rgb(212, 206, 190))
	// Wooden boarding piers & docked passenger water-bus ferries jutting out into the continuous canal.
	for i, px := range []float64{-0.26, 0.22} {
		a.shell(px, -0.22, 0.055, 0.44, 3, rgb(162, 134, 98), 5)
		trim := rgb(38, 136, 158)
		if i == 1 {
			trim = rgb(214, 138, 62)
		}
		a.passengerFerry(px+0.11, -0.25, trim)
	}
	// Modern Maritime Passenger Terminal Building (+0.09 .. +0.36).
	a.archBlock(0, 0.23, 0.74, 0.25, 12, rgb(68, 108, 124), rgb(168, 192, 198), 14, true)
	a.skylight(0, 0.23, 0.36, 0.11, 5, 2)
	a.hvac(-0.26, 0.23, 18, 12)
	a.hvac(0.26, 0.23, 18, 12)
	a.treeCluster(-0.43, 0.26, 11.5)
	a.treeCluster(0.43, 0.26, 11.5)
	a.sign(0, .24, "西运河客运港", "CANAL PORT")
}
