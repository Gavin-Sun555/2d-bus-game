package view

import (
	"math"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func splitStopBilingualTitle(name string) (string, string) {
	if strings.Contains(name, "街区") && strings.Contains(name, "行站") {
		return "", ""
	}
	base := sim.StripStopDirection(name)
	if idx := strings.Index(base, " ("); idx > 0 && strings.HasSuffix(base, ")") {
		return base[:idx], base[idx+2 : len(base)-1]
	}
	return base, base
}

func (s *streetScenery) addMetropolisLot(w *sim.World, pos sim.Vec2, heading, width, depth float64, style metroStyle, variant int, label bool, titleZh, titleEn string) bool {
	b := sim.Building{Pos: pos, W: width, H: depth}
	if math.Abs(math.Sin(heading)) > .5 {
		b.W, b.H = b.H, b.W
	}
	isWaterfront := (style == coastalLighthouse || style == coastalDeepwaterPort || style == coastalBondedPort || style == coastalPort || style == coastalYachtClub || style == coastalFishermanWharf || style == coastalSailingMarina || style == coastalBeach || style == coastalResort)
	if !s.corridorsClear(b) || !s.placement.clear(w, b, isWaterfront) {
		return false
	}
	s.placement.add(b.Pos, b.W/2+16, b.H/2+16)
	s.metroLots = append(s.metroLots, metroLot{
		footprint:     b,
		heading:       heading,
		style:         style,
		variant:       variant,
		landmarkLabel: label,
		titleZh:       titleZh,
		titleEn:       titleEn,
	})
	return true
}

func classifyMetropolisStopStyle(stop *sim.BusStop) (metroStyle, bool) {
	name := stop.Name
	switch {
	case strings.Contains(name, "卫生中心"), strings.Contains(name, "医院"):
		return metroHospital, true
	case strings.Contains(name, "体育馆"), strings.Contains(name, "奥体"):
		return metroStadium, true
	case strings.Contains(name, "图书馆"), strings.Contains(name, "中学"), strings.Contains(name, "少年宫"), strings.Contains(name, "大学"):
		return metroCampus, true
	case strings.Contains(name, "公园"), strings.Contains(name, "湿地"):
		return metroGarden, true
	case strings.Contains(name, "文化广场"), strings.Contains(name, "广场"):
		return metroPlaza, true
	case strings.Contains(name, "美术馆"), strings.Contains(name, "剧院"), strings.Contains(name, "会展"):
		return metroExhibit, true
	case strings.Contains(name, "邮局"), strings.Contains(name, "书城"), strings.Contains(name, "社区服务中心"), strings.Contains(name, "文创"):
		return metroCourt, true
	case strings.Contains(name, "市场"), strings.Contains(name, "商业街"):
		return metroShops, true
	case strings.Contains(name, "居民区"):
		return metroHomes, true
	case strings.Contains(name, "都会新区"), strings.Contains(name, "新区"), strings.Contains(name, "海岸新区"), strings.Contains(name, "港区"), strings.Contains(name, "科创新城"), strings.Contains(name, "临空新区"):
		styles := [...]metroStyle{metroTower, metroShops, metroCampus, metroCourt}
		return styles[stop.ID%len(styles)], true
	default:
		return metroShops, false
	}
}

func (s *streetScenery) addMetropolisLandmarks(w *sim.World, lw float64) {
	type lmDef struct {
		id           int
		style        metroStyle
		width, depth float64
	}
	var landmarks []lmDef
	switch w.Theme {
	case sim.MapThemeMetropolis:
		landmarks = []lmDef{
			{5, metroRail, 640, 310}, {4, metroStadium, 530, 420}, {8, metroHospital, 380, 270},
			{0, metroTerminal, 490, 286}, {1, metroTower, 345, 255}, {3, metroCampus, 410, 285},
			{6, metroCanal, 355, 245}, {7, metroCampus, 435, 300}, {9, metroGarden, 430, 310},
			{10, metroCourt, 365, 245}, {11, metroExhibit, 480, 285}, {2, metroPlaza, 300, 225},
		}
	case sim.MapThemeCoastalBay:
		landmarks = []lmDef{
			{0, coastalPort, 480, 280}, {1, coastalCivic, 440, 320}, {2, coastalBeach, 450, 280},
			{3, coastalResort, 480, 310}, {4, coastalBridgeWest, 400, 260}, {5, coastalDeepwaterPort, 520, 340},
			{6, coastalBondedPort, 480, 280}, {7, coastalYachtClub, 440, 280}, {8, coastalFishermanWharf, 450, 270},
			{9, coastalSailingMarina, 430, 270}, {10, coastalContainerHub, 460, 280}, {11, coastalLighthouse, 360, 340},
		}
	case sim.MapThemeRiverValley:
		landmarks = []lmDef{
			{0, valleyNorthForest, 460, 320}, {1, valleyLookout, 420, 280}, {2, valleyEcoNorth, 430, 280},
			{3, valleyBridgeheadNorth, 400, 260}, {4, valleyRailHub, 640, 320}, {5, valleyFinancialTowers, 440, 300},
			{6, valleyBridgeheadSouth, 400, 260}, {7, valleySmartCampus, 440, 280}, {8, valleyCivicCenter, 420, 270},
			{9, valleyMountainLookout, 420, 280}, {10, valleyCultureVista, 460, 290}, {11, valleyHSRHQ, 450, 280},
		}
	case sim.MapThemeTwinCities:
		landmarks = []lmDef{
			{0, twinAirportLogistics, 480, 290}, {1, twinRoboticsCampus, 460, 280}, {2, twinIndustrialWest, 440, 280},
			{3, twinInterchangeWest, 420, 270}, {4, twinRingCampus, 520, 360}, {5, twinUniversity, 500, 320},
			{6, twinSmartCityCenter, 460, 300}, {7, twinInterchangeEast, 420, 270}, {8, twinAirportTerminal, 580, 340},
			{9, twinGigaFactory, 540, 320}, {10, twinAIScienceCity, 460, 290}, {11, twinUnicornHQ, 460, 300},
		}
	default:
		landmarks = []lmDef{
			{0, metroTerminal, 480, 280}, {1, metroTower, 360, 260}, {2, metroPlaza, 320, 240},
			{3, metroCampus, 420, 280}, {4, metroStadium, 500, 380}, {5, metroExhibit, 620, 300},
			{6, metroCanal, 360, 250}, {7, metroCampus, 420, 280}, {8, metroHospital, 380, 260},
			{9, metroGarden, 420, 300}, {10, metroCourt, 360, 250}, {11, metroExhibit, 460, 280},
		}
	}
	for _, rail := range w.Railways() {
		if !rail.Conventional {
			continue
		}
		footprint := sim.Building{Pos: rail.StationCenter, W: 560, H: rail.StationDepth}
		s.placement.add(footprint.Pos, footprint.W/2+16, footprint.H/2+16)
		s.metroLots = append(s.metroLots, metroLot{
			footprint: footprint, heading: rail.StationHeading, style: metroConventionalRail, landmarkLabel: true,
		})
	}
	placedTitles := map[string]bool{}
	for _, landmark := range landmarks {
		if landmark.id < len(w.Stops) {
			if rail := w.Railway(); rail != nil && rail.StationStopID == landmark.id {
				footprint := sim.Building{Pos: rail.StationCenter, W: landmark.width, H: rail.StationDepth}
				s.placement.add(footprint.Pos, footprint.W/2+16, footprint.H/2+16)
				s.metroLots = append(s.metroLots, metroLot{
					footprint: footprint, heading: rail.StationHeading,
					style: landmark.style, variant: landmark.id, landmarkLabel: true,
				})
				continue
			}
			s.placeMetropolisLandmark(w, w.Stops[landmark.id], landmark.style, landmark.width, landmark.depth, lw, true)
			zh, _ := splitStopBilingualTitle(w.Stops[landmark.id].Name)
			if zh != "" {
				placedTitles[zh] = true
			}
		}
	}
	s.setupMetropolisCorridors(w)
	// Match neighborhood and expansion stations to their station names so a school,
	// clinic, post office, sports hall, market, or park sits directly behind the stop.
	for _, stop := range w.Stops {
		if stop.ID < 12 {
			continue
		}
		zh, _ := splitStopBilingualTitle(stop.Name)
		if zh != "" && !strings.Contains(zh, "都会新区") {
			if placedTitles[zh] {
				continue
			}
			placedTitles[zh] = true
		}
		style, matched := classifyMetropolisStopStyle(stop)
		if !matched {
			continue
		}
		s.placeMetropolisLandmark(w, stop, style, 276, 178, lw, false)
	}
}

func (s *streetScenery) placeMetropolisLandmark(w *sim.World, stop *sim.BusStop, style metroStyle, width, depth, lw float64, major bool) {
	forward := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	roadHalf := float64(stop.RoadLanes)*lw/2 + 10
	shifts := []float64{0, -140, 140, -300, 300}
	scales := []float64{1, .76}
	sides := []float64{1, -1}
	if !major {
		// Prefer the same side of the road as the stop, falling back across the street if needed.
		sides = []float64{1, -1}
		shifts = []float64{0, -110, 110, -220, 220}
	}
	if style == metroPlaza && major {
		// The stop is at the canal crossing. Look farther along the bank.
		shifts = []float64{-220, 220, -260, 260, -300, 300, -360, 360, -450, 450, -560, 560}
		scales = append(scales, .58)
	}
	zh, en := "", ""
	if !major {
		zh, en = splitStopBilingualTitle(stop.Name)
	}
	for _, scale := range scales {
		for _, shift := range shifts {
			for _, side := range sides {
				heading := stop.Heading
				if side > 0 {
					heading += math.Pi
				}
				edge := roadHalf + sim.ShoulderWidth + 32
				// Keep the forecourt behind the existing passenger shelter.
				if side > 0 && !stop.CenterBRT {
					edge = math.Max(edge, stop.ShelterPos.Sub(stop.RoadCenter).Dot(right)+42)
				}
				pos := stop.RoadCenter.Add(forward.Mul(shift)).Add(right.Mul(side * (edge + depth*scale/2)))
				if s.addMetropolisLot(w, pos, heading, width*scale, depth*scale, style, stop.ID, major, zh, en) {
					return
				}
			}
		}
	}
}

func (s *streetScenery) addMetropolisFrontage(w *sim.World, road sim.RoadSegment, forward, right sim.Vec2, d, side, lw float64, slot int) {
	street := road.P1.Add(forward.Mul(d))
	variant := metropolisVariant(street.Add(right.Mul(side * 75)))
	phase := metropolisVariant(road.P1.Add(right.Mul(side * 75)))
	// Alternate silhouettes along a frontage, with different phases across blocks.
	style := metroStyle((phase + slot) % 6)
	widths := [6]float64{300, 248, 268, 252, 272, 220}
	depths := [6]float64{166, 204, 186, 184, 162, 168}
	width := widths[style] + float64(variant%4)*8
	depth := depths[style] + float64(variant%5)*4
	inset := float64(variant%3) * 4
	pos := street.Add(right.Mul(side * (road.GetWidth(lw)/2 + sim.ShoulderWidth + 10 + depth/2 + inset)))
	heading := math.Atan2(forward.Y, forward.X)
	if side > 0 {
		heading += math.Pi
	}
	s.addMetropolisLot(w, pos, heading, width, depth, style, variant, false, "", "")

	// Populate the inner city block behind the street frontage so blocks read as
	// complete urban districts rather than a single thin row in front of empty ground.
	backVariant := metropolisVariant(street.Add(right.Mul(side * 290)))
	backStyles := [...]metroStyle{metroHomes, metroTower, metroCourt, metroRowhouses, metroShops, metroGarden}
	backStyle := backStyles[(phase+slot*3+backVariant)%len(backStyles)]
	if backStyle == style {
		backStyle = backStyles[(int(backStyle)+1)%len(backStyles)]
	}
	backW := 268.0 + float64(backVariant%4)*12
	backD := 206.0 + float64(backVariant%4)*10
	backOffset := road.GetWidth(lw)/2 + sim.ShoulderWidth + 10 + depth + 28 + backD/2
	backPos := street.Add(right.Mul(side * backOffset))
	s.addMetropolisLot(w, backPos, heading, backW, backD, backStyle, backVariant, false, "", "")
}
