package view

import (
	"image/color"
	"math"
)

// twinAirportTerminal renders Airport Terminal T1/T2 (临空国际机场 T1/T2航站楼).
func (a metroCanvas) twinAirportTerminal() {
	// Airside tarmac apron
	a.rect(0, -0.22, 0.96, 0.46, rgb(112, 120, 126))
	// Aircraft parking stand yellow guidelines & safety red stop bars
	a.line(-0.24, -0.40, -0.24, -0.06, 2.0, rgb(240, 204, 52))
	a.line(0.24, -0.40, 0.24, -0.06, 2.0, rgb(240, 204, 52))
	a.line(-0.30, -0.16, -0.18, -0.16, 2.5, rgb(214, 54, 46))
	a.line(0.18, -0.16, 0.30, -0.16, 2.5, rgb(214, 54, 46))
	// Commercial airliner docked at gate
	if a.detail {
		pos := a.point(-0.24, -0.22)
		u := a.forward
		// Main wings
		drawRotatedBox(a.screen, a.cam, pos, u.X, u.Y, 74, 12, rgb(238, 242, 246))
		// Fuselage
		drawRotatedBox(a.screen, a.cam, pos, -u.Y, u.X, 86, 16, rgb(248, 250, 252))
		// Nose cone & cockpit
		drawRotatedBox(a.screen, a.cam, pos.Add(a.front.Mul(38)), -u.Y, u.X, 14, 12, rgb(48, 54, 62))
		// Tail fin with cyan airline livery
		drawRotatedBox(a.screen, a.cam, pos.Sub(a.front.Mul(32)), -u.Y, u.X, 18, 6, rgb(52, 172, 224))
		// Horizontal stabilizer
		drawRotatedBox(a.screen, a.cam, pos.Sub(a.front.Mul(30)), u.X, u.Y, 32, 7, rgb(234, 238, 242))
		// Twin jet engines under wings
		drawRotatedBox(a.screen, a.cam, pos.Add(u.Mul(20)), -u.Y, u.X, 16, 8, rgb(68, 76, 84))
		drawRotatedBox(a.screen, a.cam, pos.Sub(u.Mul(20)), -u.Y, u.X, 16, 8, rgb(68, 76, 84))
		// Telescopic passenger boarding bridge (jet bridge)
		a.rect(-0.10, -0.12, 0.16, 0.05, rgb(184, 192, 198))
		a.circlePx(-0.04, -0.12, 4.0, rgb(228, 142, 54))
		// Baggage tug trailer
		a.rect(0.24, -0.26, 0.08, 0.05, rgb(234, 148, 42))
	}
	// Terminal building with sweeping aerodynamic silver roof wings
	a.softPlaza(0, 0.22, 0.94, 0.46, 0.04, 0.04, rgb(216, 220, 224))
	a.archBlock(0, 0.18, 0.90, 0.32, 6, rgb(48, 64, 78), rgb(244, 248, 252), 10, true)
	a.skylight(0, 0.18, 0.76, 0.16, 8, 2)
	// Departure flyover drop-off road & canopy
	a.rect(0, 0.38, 0.62, 0.06, rgb(52, 172, 224))
	if a.detail {
		a.miniBus(-0.16, 0.42, 0, rgb(56, 142, 224), true)
		a.miniCar(0.12, 0.42, 0, rgb(240, 196, 38), true)
		a.miniCar(0.24, 0.42, 0, rgb(245, 245, 245), false)
	}
	a.sign(0, 0.44, "临空国际机场 T1/T2航站楼", "AIRPORT TERMINAL T1/T2")
}

// twinAirportLogistics renders the Air Cargo Logistics Hub (临空国际物流港总站).
func (a metroCanvas) twinAirportLogistics() {
	a.rect(0, 0, 0.94, 0.90, rgb(132, 140, 146))
	// Air freight cargo warehouse
	a.archBlock(-0.10, -0.08, 0.74, 0.48, 5, rgb(44, 58, 72), rgb(224, 232, 240), 9, false)
	a.skylight(-0.10, -0.08, 0.58, 0.26, 5, 2)
	// Loading dock bays with trailers
	a.rect(-0.10, 0.18, 0.68, 0.06, rgb(78, 86, 94))
	if a.detail {
		for _, x := range []float64{-0.32, -0.18, -0.04, 0.10} {
			a.rect(x, 0.26, 0.06, 0.10, rgb(224, 232, 238))
			a.rect(x, 0.30, 0.05, 0.04, rgb(52, 148, 214))
		}
	}
	// Air traffic control radar tower with dish
	a.circlePx(0.36, -0.22, 14.0, rgb(52, 68, 84))
	a.circlePx(0.36, -0.22, 10.0, rgb(240, 244, 248))
	a.circlePx(0.36, -0.22, 4.5, rgb(238, 142, 54))
	// Air cargo ULD container pallet stacks
	cCols := []color.RGBA{rgb(188, 194, 200), rgb(48, 142, 214), rgb(224, 142, 52)}
	for i, x := range []float64{0.28, 0.36, 0.42} {
		for j, y := range []float64{0.14, 0.24} {
			a.rect(x, y, 0.06, 0.06, cCols[(i+j)%len(cCols)])
		}
	}
	a.sign(0, 0.44, "临空国际物流港总站", "AIRPORT LOGISTICS HUB")
}

// twinRoboticsCampus renders Advanced Robotics & Semiconductor Fab (智能制造产业园站).
func (a metroCanvas) twinRoboticsCampus() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(214, 218, 222))
	// Cleanroom manufacturing envelope with rooftop air filtration & cooling towers
	a.archBlock(-0.14, -0.10, 0.62, 0.42, 5, rgb(48, 64, 78), rgb(236, 242, 248), 10, false)
	a.chillerBank(-0.14, -0.10, 5)
	// Nitrogen / cryogenic gas storage tanks
	for _, x := range []float64{0.26, 0.34} {
		a.circlePx(x, -0.20, 8.0, rgb(240, 244, 248))
		a.circlePx(x, -0.20, 3.5, rgb(56, 172, 224))
	}
	// Corporate administration & tech sculpture plaza
	a.archBlock(0.12, 0.22, 0.58, 0.24, 4, rgb(48, 64, 78), rgb(236, 242, 248), 8, true)
	a.oval(-0.28, 0.22, 0.22, 0.22, rgb(56, 172, 224))
	a.circlePx(-0.28, 0.22, 4.5, rgb(248, 248, 252))
	if a.detail {
		a.treeCluster(-0.38, -0.26, 12)
		a.treeCluster(0.38, 0.26, 12)
	}
	a.sign(0, 0.42, "智能制造产业园站", "SMART FACTORY CAMPUS")
}

// twinGigaFactory renders the New Energy Gigafactory (新能源智造超级工厂).
func (a metroCanvas) twinGigaFactory() {
	a.rect(0, 0, 0.96, 0.92, rgb(138, 144, 150))
	// Massive clean mega-factory with red accent stripe
	a.archBlock(0, -0.06, 0.90, 0.54, 6, rgb(38, 52, 66), rgb(244, 248, 252), 11, false)
	// Full-coverage photovoltaic solar roof grid
	a.solarBank(0, -0.06, 0.82, 0.42, 6, 8)
	// Signature red accent band
	a.rect(0, 0.22, 0.90, 0.03, rgb(224, 48, 48))
	// Battery energy storage system (BESS) container packs
	for _, x := range []float64{-0.34, -0.20, -0.06} {
		a.rect(x, 0.32, 0.10, 0.08, rgb(54, 156, 108))
	}
	// Automated logistics loading bays & EV testing
	a.rect(0.24, 0.32, 0.36, 0.08, rgb(56, 172, 224))
	if a.detail {
		a.miniCar(0.18, 0.32, 0, rgb(240, 240, 240), false)
		a.miniCar(0.30, 0.32, 0, rgb(214, 52, 48), false)
	}
	a.sign(0, 0.44, "新能源智造超级工厂", "GIGAFACTORY CAMPUS")
}

// twinRingCampus renders the Software Innovation Ring HQ (软件研发创新中心).
func (a metroCanvas) twinRingCampus() {
	// Park plinth
	a.softLawn(0, 0, 0.94, 0.90)
	// Iconic circular ring campus headquarters (inspired by Apple Park)
	a.oval(0, 0, 0.82, 0.76, rgb(52, 68, 82))
	a.oval(0, 0, 0.74, 0.68, rgb(240, 244, 250))
	// Solar roof ring
	a.oval(0, 0, 0.66, 0.60, rgb(44, 88, 128))
	// Central interior park
	a.oval(0, 0, 0.52, 0.46, rgb(68, 126, 82))
	// Central reflecting pond
	a.oval(0, 0, 0.30, 0.24, rgb(42, 142, 178))
	a.circlePx(0, 0, 5.0, rgb(248, 248, 252))
	if a.detail {
		for i := 0; i < 8; i++ {
			ang := float64(i) * math.Pi / 4
			px, py := 0.20*math.Cos(ang), 0.17*math.Sin(ang)
			a.circlePx(px, py, 4.0, rgb(38, 112, 68))
		}
		a.treeCluster(-0.38, -0.32, 14)
		a.treeCluster(0.38, -0.32, 14)
		a.treeCluster(-0.38, 0.32, 14)
		a.treeCluster(0.38, 0.32, 14)
	}
	a.sign(0, 0.42, "软件研发创新中心", "SOFTWARE INNOVATION HUB")
}

// twinUniversity renders Silicon Valley University (硅谷大学科技园站).
func (a metroCanvas) twinUniversity() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(212, 208, 196))
	// University academic quadrangle & library with dome
	a.archBlock(-0.20, -0.12, 0.48, 0.42, 5, rgb(138, 68, 54), rgb(242, 238, 228), 9, true)
	a.circlePx(-0.20, -0.12, 12.0, rgb(52, 148, 186))
	a.circlePx(-0.20, -0.12, 4.5, rgb(238, 196, 68))
	// Athletic running track (400m oval) with green soccer pitch
	a.oval(0.25, -0.12, 0.38, 0.42, rgb(204, 88, 64))
	a.oval(0.25, -0.12, 0.28, 0.32, rgb(58, 142, 78))
	a.rect(0.25, -0.12, 0.02, 0.26, rgb(245, 245, 245))
	// Campus quadrangle park
	a.softLawn(0, 0.22, 0.86, 0.22)
	if a.detail {
		a.treeCluster(-0.32, 0.22, 13)
		a.treeCluster(0.32, 0.22, 13)
		a.treeCluster(0, 0.22, 13)
	}
	a.sign(0, 0.42, "硅谷大学科技园站", "SILICON VALLEY UNIVERSITY")
}

// twinAIScienceCity renders the AI Research Center & Supercomputer Hub (人工智能未来科学城).
func (a metroCanvas) twinAIScienceCity() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(208, 214, 220))
	// Futuristic AI research towers with cyan illumination fins
	a.archBlock(-0.22, -0.06, 0.42, 0.46, 6, rgb(38, 54, 70), rgb(232, 238, 246), 11, true)
	a.line(-0.38, -0.26, -0.06, -0.26, 3.0, rgb(56, 189, 248))
	a.line(-0.38, 0.14, -0.06, 0.14, 3.0, rgb(56, 189, 248))
	a.skylight(-0.22, -0.06, 0.28, 0.24, 3, 3)
	// Cubic supercomputer data center with cooling fins
	a.archBlock(0.24, -0.12, 0.36, 0.32, 4, rgb(48, 62, 76), rgb(72, 92, 112), 8, false)
	a.chillerBank(0.24, -0.12, 4)
	// Sunken reflecting pool court with tech monument
	a.rect(0.24, 0.22, 0.36, 0.24, rgb(38, 122, 156))
	a.rect(0.24, 0.22, 0.28, 0.18, rgb(58, 172, 214))
	a.circlePx(0.24, 0.22, 5.0, rgb(248, 248, 252))
	if a.detail {
		a.treeCluster(-0.38, 0.26, 13)
	}
	a.sign(0, 0.42, "人工智能未来科学城", "AI SCIENCE CITY")
}

// twinUnicornHQ renders the Tech Unicorn Headquarters (独角兽企业总部基地).
func (a metroCanvas) twinUnicornHQ() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(212, 216, 222))
	// Stepped modern corporate towers with glass curtain walls
	a.archBlock(-0.20, -0.10, 0.46, 0.44, 6, rgb(42, 58, 74), rgb(234, 240, 248), 11, true)
	a.archBlock(0.22, 0.08, 0.38, 0.46, 6, rgb(42, 58, 74), rgb(234, 240, 248), 11, true)
	a.skylight(-0.20, -0.10, 0.32, 0.24, 4, 3)
	a.skylight(0.22, 0.08, 0.24, 0.28, 3, 3)
	// Skybridge connector
	a.rect(0.01, -0.02, 0.12, 0.10, rgb(68, 92, 116))
	a.line(-0.04, -0.02, 0.06, -0.02, 2.0, rgb(56, 189, 248))
	// Sunken amphitheater & plaza
	a.oval(-0.20, 0.24, 0.32, 0.20, rgb(186, 194, 202))
	a.oval(-0.20, 0.24, 0.22, 0.14, rgb(214, 220, 226))
	a.circlePx(-0.20, 0.24, 4.0, rgb(238, 142, 54))
	if a.detail {
		a.treeCluster(0.36, -0.24, 13)
		a.treeCluster(-0.38, -0.26, 12)
	}
	a.sign(0, 0.42, "独角兽企业总部基地", "UNICORN HEADQUARTERS")
}

// twinIndustrialWest renders West Industrial Center (工业中心西站).
func (a metroCanvas) twinIndustrialWest() {
	a.rect(0, 0, 0.94, 0.90, rgb(136, 144, 150))
	a.archBlock(0, -0.10, 0.82, 0.46, 5, rgb(46, 60, 74), rgb(222, 230, 238), 9, false)
	a.skylight(0, -0.10, 0.68, 0.28, 6, 2)
	a.rect(0, 0.24, 0.74, 0.22, rgb(188, 194, 200))
	if a.detail {
		a.miniCar(-0.22, 0.24, 0, rgb(48, 142, 214), false)
		a.miniCar(-0.10, 0.24, 0, rgb(240, 240, 240), false)
		a.miniCar(0.02, 0.24, 0, rgb(214, 68, 54), false)
	}
	a.sign(0, 0.42, "工业中心西站", "WEST INDUSTRIAL CENTER")
}

// twinInterchangeWest renders Interchange West Gate (枢纽立交西互通站).
func (a metroCanvas) twinInterchangeWest() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(210, 214, 218))
	a.archBlock(-0.14, 0.04, 0.58, 0.42, 5, rgb(52, 68, 84), rgb(234, 238, 244), 9, true)
	a.rect(0.24, 0.04, 0.32, 0.46, rgb(184, 192, 198))
	a.line(0.08, 0.04, 0.24, 0.04, 3.0, rgb(56, 172, 224))
	if a.detail {
		a.treeCluster(-0.36, 0.26, 13)
		a.treeCluster(0.36, -0.24, 13)
	}
	a.sign(0, 0.42, "枢纽立交西互通站", "INTERCHANGE WEST GATE")
}

// twinInterchangeEast renders Interchange East Gate (枢纽立交东互通站).
func (a metroCanvas) twinInterchangeEast() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.05, 0.05, rgb(210, 214, 218))
	a.archBlock(0.14, 0.04, 0.58, 0.42, 5, rgb(52, 68, 84), rgb(234, 238, 244), 9, true)
	a.rect(-0.24, 0.04, 0.32, 0.46, rgb(184, 192, 198))
	a.oval(-0.24, 0.04, 0.22, 0.22, rgb(48, 138, 172))
	if a.detail {
		a.treeCluster(0.36, 0.26, 13)
		a.treeCluster(-0.36, -0.24, 13)
	}
	a.sign(0, 0.42, "枢纽立交东互通站", "INTERCHANGE EAST GATE")
}

// twinSmartCityCenter renders the Smart Ecological City Center (智慧生态新城站).
func (a metroCanvas) twinSmartCityCenter() {
	a.softPlaza(0, 0, 0.94, 0.90, 0.06, 0.06, rgb(212, 216, 218))
	// Biophilic ecological tower with tiered green roof terraces
	a.archBlock(-0.16, -0.06, 0.54, 0.48, 6, rgb(48, 84, 76), rgb(236, 244, 240), 11, true)
	a.rect(-0.16, -0.06, 0.42, 0.36, rgb(74, 142, 108))
	a.oval(-0.16, -0.06, 0.26, 0.22, rgb(54, 118, 92))
	// Central eco-lake & fountain
	a.oval(0.26, 0.04, 0.34, 0.44, rgb(42, 148, 184))
	a.circlePx(0.26, 0.04, 5.5, rgb(245, 245, 250))
	if a.detail {
		a.treeCluster(0.32, -0.26, 14)
		a.treeCluster(0.32, 0.28, 14)
		a.treeCluster(-0.38, 0.26, 13)
	}
	a.sign(0, 0.42, "智慧生态新城站", "SMART CITY CENTER")
}
