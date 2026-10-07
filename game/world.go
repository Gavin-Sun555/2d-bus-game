package game

import (
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

const (
	ShoulderWidth = 40.0 // Width of shoulder outside road curb where bus can drive but slows down
)

type RoadType int

const (
	RoadAvenue4Lane RoadType = iota // 4-lane major avenue / ring highway
	RoadStreet2Lane                 // 2-lane city street
)

type RoadZone int

const (
	ZoneRoad        RoadZone = iota // Asphalt road surface
	ZoneShoulder                    // Roadside shoulder / sidewalk (slow down)
	ZoneHardBarrier                 // Off-limits / hard barrier (collision & damage & fine)
)

type BoundaryCheckResult struct {
	Zone         RoadZone
	Pushback     Vec2
	Penetration  float64
	ObstacleName string
}

// RoadSegment represents an interconnected road segment
type RoadSegment struct {
	P1       Vec2
	P2       Vec2
	Lanes    int
	Type     RoadType
	HasLines bool
	Name     string
}

func (r RoadSegment) GetWidth(laneWidth float64) float64 {
	if r.Lanes == 4 {
		return float64(r.Lanes)*laneWidth + 24.0 // 4 lanes + central median
	}
	return float64(r.Lanes)*laneWidth + 12.0 // 2 lanes + median divider
}

// Building represents decorative scenery blocks
type Building struct {
	Pos   Vec2
	W     float64
	H     float64
	Color color.RGBA
	Roof  color.RGBA
	Name  string
}

// Tree represents roadside greenery
type Tree struct {
	Pos    Vec2
	Radius float64
	Color  color.RGBA
}

// River represents a scenic waterway
type River struct {
	Y      float64
	Height float64
}

// World holds the city roads, buildings, trees, water, and stops
type World struct {
	Roads     []RoadSegment
	Buildings []Building
	Trees     []Tree
	Stops     []*BusStop
	Rivers    []River
}

func NewWorld(initialLaneWidth float64) *World {
	w := &World{
		Roads:     make([]RoadSegment, 0),
		Buildings: make([]Building, 0),
		Trees:     make([]Tree, 0),
		Stops:     make([]*BusStop, 0),
		Rivers:    make([]River, 0),
	}

	w.buildRoadNetwork()
	w.buildStops(initialLaneWidth)
	w.buildBuildings()
	w.buildGreenery()
	w.buildRivers()

	return w
}

func (w *World) buildRoadNetwork() {
	// Map spans X in [-2800, 2800], Y in [-1500, 1500] (over 5600 x 3000 world canvas)

	// --- 1. East-West Avenues ---
	// North Ring Expressway (4-Lane Avenue)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -2800, Y: -1200}, P2: Vec2{X: 2800, Y: -1200},
		Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "北环快速路",
	})
	// Inner North Street (2-Lane Street)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -2800, Y: -600}, P2: Vec2{X: 2800, Y: -600},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "北二路",
	})
	// Central Boulevard (4-Lane Grand Avenue)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -2800, Y: 0}, P2: Vec2{X: 2800, Y: 0},
		Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "中央主干大道",
	})
	// Inner South Street (2-Lane Street)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -2800, Y: 600}, P2: Vec2{X: 2800, Y: 600},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "南二路",
	})
	// South Ring Expressway (4-Lane Avenue)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -2800, Y: 1200}, P2: Vec2{X: 2800, Y: 1200},
		Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "南环快速路",
	})

	// --- 2. North-South Boulevards ---
	// Far-West Boulevard (Harbor Port)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -1800, Y: -1500}, P2: Vec2{X: -1800, Y: 1500},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西海滨大道",
	})
	// Mid-West Boulevard (West Park / Hospital)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: -900, Y: -1500}, P2: Vec2{X: -900, Y: 1500},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "西公园大道",
	})
	// Central Avenue (Axis of the City)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: 0, Y: -1500}, P2: Vec2{X: 0, Y: 1500},
		Lanes: 4, Type: RoadAvenue4Lane, HasLines: true, Name: "城市中轴大道",
	})
	// Mid-East Boulevard (Civic Center / Commercial)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: 900, Y: -1500}, P2: Vec2{X: 900, Y: 1500},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "东市民大道",
	})
	// Far-East Boulevard (High-Tech Campus / Expo)
	w.Roads = append(w.Roads, RoadSegment{
		P1: Vec2{X: 1800, Y: -1500}, P2: Vec2{X: 1800, Y: 1500},
		Lanes: 2, Type: RoadStreet2Lane, HasLines: true, Name: "东高新大道",
	})
}

func (w *World) buildStops(laneWidth float64) {
	// 12 Distinct Bus Stations across the expanded metropolitan area
	w.Stops = append(w.Stops,
		// 1. Central Terminal (Central Blvd Eastbound, 4-lane, heading 0, busy terminal: 12 pax)
		NewBusStop(0, "中央枢纽 (Central Terminal)", Vec2{X: -350, Y: 0}, 0, 4,
			RGBA(59, 130, 246, 255), laneWidth, 12),

		// 2. IFC Tower (Central Blvd Westbound, 4-lane, heading Pi: 8 pax)
		NewBusStop(1, "国际金融中心 (IFC Tower)", Vec2{X: 350, Y: 0}, math.Pi, 4,
			RGBA(16, 185, 129, 255), laneWidth, 8),

		// 3. Civic Center Square (Civic Blvd Southbound, 2-lane, heading Pi/2: 0 pax empty station, can skip!)
		NewBusStop(2, "市民广场 (Civic Center)", Vec2{X: 900, Y: -300}, math.Pi/2, 2,
			RGBA(168, 85, 247, 255), laneWidth, 0),

		// 4. Tech Innovation Campus (Tech Blvd Northbound, 2-lane, heading -Pi/2: 0 pax)
		NewBusStop(3, "科技创新园区 (Tech Campus)", Vec2{X: 1800, Y: -900}, -math.Pi/2, 2,
			RGBA(14, 165, 233, 255), laneWidth, 0),

		// 5. Olympic Sports Stadium (North Ring Westbound, 4-lane, heading Pi: 8 pax)
		NewBusStop(4, "奥体中心 (Olympic Stadium)", Vec2{X: 1350, Y: -1200}, math.Pi, 4,
			RGBA(236, 72, 153, 255), laneWidth, 8),

		// 6. North Railway Station (North Ring Eastbound, 4-lane, heading 0: 16 pax)
		NewBusStop(5, "高铁北站 (North Railway Hub)", Vec2{X: -450, Y: -1200}, 0, 4,
			RGBA(245, 158, 11, 255), laneWidth, 16),

		// 7. Ocean Ferry Wharf (Harbor Blvd Southbound, 2-lane, heading Pi/2: 6 pax)
		NewBusStop(6, "海滨客运港 (Ocean Ferry Port)", Vec2{X: -1800, Y: -900}, math.Pi/2, 2,
			RGBA(6, 182, 212, 255), laneWidth, 6),

		// 8. University Park (Harbor Blvd Northbound, 2-lane, heading -Pi/2: 10 pax)
		NewBusStop(7, "大学城园区 (University Park)", Vec2{X: -1800, Y: 900}, -math.Pi/2, 2,
			RGBA(139, 92, 246, 255), laneWidth, 10),

		// 9. City General Hospital (West Park Blvd Northbound, 2-lane, heading -Pi/2: 0 pax empty station, can skip!)
		NewBusStop(8, "市立第一医院 (City Hospital)", Vec2{X: -900, Y: 300}, -math.Pi/2, 2,
			RGBA(239, 68, 68, 255), laneWidth, 0),

		// 10. Eco Nature Wetlands (South Ring Eastbound, 4-lane, heading 0: 0 pax)
		NewBusStop(9, "湿地公园 (Eco Wetlands)", Vec2{X: -1350, Y: 1200}, 0, 4,
			RGBA(34, 197, 94, 255), laneWidth, 0),

		// 11. Cultural Art District (South Ring Westbound, 4-lane, heading Pi: 0 pax empty station, can skip!)
		NewBusStop(10, "文创艺术街区 (Creative District)", Vec2{X: 450, Y: 1200}, math.Pi, 4,
			RGBA(249, 115, 22, 255), laneWidth, 0),

		// 12. Expo Convention Center (Tech Blvd Southbound, 2-lane, heading Pi/2: 4 pax)
		NewBusStop(11, "国际会展中心 (Expo Center)", Vec2{X: 1800, Y: 900}, math.Pi/2, 2,
			RGBA(217, 70, 239, 255), laneWidth, 4),
	)
}

func (w *World) buildRivers() {
	w.Rivers = append(w.Rivers, River{
		Y:      -300,
		Height: 50,
	})
}

func (w *World) buildBuildings() {
	blocks := []struct {
		cx, cy, w, h float64
		col, roof    color.RGBA
	}{
		// Downtown Skyscraper cluster (Mid-East & Mid-West Downtown plazas)
		{cx: 450, cy: 300, w: 220, h: 160, col: RGBA(15, 23, 42, 255), roof: RGBA(30, 41, 59, 255)},
		{cx: -450, cy: 300, w: 200, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},

		// Tech Innovation Campus (East Tech Park, block center)
		{cx: 1350, cy: -900, w: 240, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(59, 130, 246, 255)},

		// Commercial Center & Sports Mall (Pink roof - placed safely inside block X=450, Y=-900)
		{cx: 450, cy: -900, w: 240, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(236, 72, 153, 255)},

		// North Railway Hub Terminals (Safely inside blocks south of North Ring, Y=-900)
		{cx: -450, cy: -900, w: 240, h: 160, col: RGBA(15, 23, 42, 255), roof: RGBA(245, 158, 11, 255)},
		{cx: -1350, cy: -900, w: 240, h: 160, col: RGBA(30, 41, 59, 255), roof: RGBA(71, 85, 105, 255)},

		// University Campus Blocks (West side, Y=850 & Y=300)
		{cx: -1350, cy: 850, w: 260, h: 160, col: RGBA(39, 39, 42, 255), roof: RGBA(63, 63, 70, 255)},
		{cx: -1350, cy: 300, w: 220, h: 150, col: RGBA(30, 41, 59, 255), roof: RGBA(51, 65, 85, 255)},

		// City General Hospital (Red roof - safely centered in block X=-450, Y=850)
		{cx: -450, cy: 850, w: 220, h: 160, col: RGBA(248, 250, 252, 255), roof: RGBA(239, 68, 68, 255)},

		// Expo & Sports Stadium complex (East side, Y=850)
		{cx: 1350, cy: 850, w: 260, h: 160, col: RGBA(24, 24, 27, 255), roof: RGBA(51, 65, 85, 255)},

		// East Civic Cultural Center (East side, Y=300)
		{cx: 1350, cy: 300, w: 240, h: 150, col: RGBA(15, 23, 42, 255), roof: RGBA(217, 70, 239, 255)},
	}

	for _, b := range blocks {
		w.Buildings = append(w.Buildings, Building{
			Pos:   Vec2{X: b.cx, Y: b.cy},
			W:     b.w,
			H:     b.h,
			Color: b.col,
			Roof:  b.roof,
		})
	}
}

func (w *World) buildGreenery() {
	// Park greenery clusters in block interiors (away from road shoulders)
	for x := -800.0; x <= -100.0; x += 70.0 {
		for y := 720.0; y <= 1080.0; y += 70.0 {
			w.Trees = append(w.Trees, Tree{
				Pos:    Vec2{X: x, Y: y},
				Radius: 16,
				Color:  RGBA(22, 101, 52, 255),
			})
		}
	}

	// Avenue trees placed safely in block medians / plaza perimeters away from roads
	for _, x := range []float64{-1350, -450, 450, 1350} {
		for _, y := range []float64{-900, 300, 850} {
			offsets := []Vec2{
				{X: -160, Y: -100}, {X: 160, Y: -100},
				{X: -160, Y: 100}, {X: 160, Y: 100},
			}
			for _, off := range offsets {
				w.Trees = append(w.Trees, Tree{
					Pos:    Vec2{X: x + off.X, Y: y + off.Y},
					Radius: 14,
					Color:  RGBA(34, 197, 94, 255),
				})
			}
		}
	}
}

// CheckBoundaries checks if a position is on the road, on the shoulder, or hitting a hard barrier
func (w *World) CheckBoundaries(pos Vec2, busRadius float64, laneWidth float64) BoundaryCheckResult {
	// 1. Check decorative Building collisions (Hard Barrier)
	for _, b := range w.Buildings {
		hw := b.W/2 + busRadius
		hh := b.H/2 + busRadius
		if math.Abs(pos.X-b.Pos.X) < hw && math.Abs(pos.Y-b.Pos.Y) < hh {
			// Collision with building
			dx := pos.X - b.Pos.X
			dy := pos.Y - b.Pos.Y
			var push Vec2
			if math.Abs(dx)/hw > math.Abs(dy)/hh {
				if dx > 0 {
					push = Vec2{X: 1, Y: 0}
				} else {
					push = Vec2{X: -1, Y: 0}
				}
			} else {
				if dy > 0 {
					push = Vec2{X: 0, Y: 1}
				} else {
					push = Vec2{X: 0, Y: -1}
				}
			}
			return BoundaryCheckResult{
				Zone:         ZoneHardBarrier,
				Pushback:     push,
				Penetration:  10.0,
				ObstacleName: "建筑墙体硬阻隔",
			}
		}
	}

	// 2. Check Outer Metropolitan Perimeter Wall (Hard Barrier)
	if pos.X > 2800 {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: -1, Y: 0}, Penetration: pos.X - 2800, ObstacleName: "东侧边界隔离墙"}
	}
	if pos.X < -2800 {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 1, Y: 0}, Penetration: -2800 - pos.X, ObstacleName: "西侧边界隔离墙"}
	}
	if pos.Y > 1480 {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: -1}, Penetration: pos.Y - 1480, ObstacleName: "南侧边界隔离墙"}
	}
	if pos.Y < -1480 {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: 1}, Penetration: -1480 - pos.Y, ObstacleName: "北侧边界隔离墙"}
	}

	// 2.5 Check Intersection Turning Clearances:
	// Provide generous turning clearance at all 25 intersections so long buses can comfortably turn
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}
	interTurnClearance := math.Max(170.0, laneWidth*2.8 + 35.0)
	for _, ix := range xs {
		for _, iy := range ys {
			dx := pos.X - ix
			dy := pos.Y - iy
			if dx*dx+dy*dy < interTurnClearance*interTurnClearance {
				return BoundaryCheckResult{Zone: ZoneRoad}
			}
		}
	}

	// 3. Check Road and Shoulder distances
	minDistBeyondCurb := math.MaxFloat64
	var closestPush Vec2

	for _, r := range w.Roads {
		roadW := r.GetWidth(laneWidth)
		roadHW := roadW / 2

		isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0

		if isEW {
			// Horizontal road: Y = r.P1.Y, X in [r.P1.X, r.P2.X]
			if pos.X >= r.P1.X-40 && pos.X <= r.P2.X+40 {
				dCenter := math.Abs(pos.Y - r.P1.Y)
				dBeyond := dCenter - roadHW
				if dBeyond < minDistBeyondCurb {
					minDistBeyondCurb = dBeyond
					if pos.Y > r.P1.Y {
						closestPush = Vec2{X: 0, Y: -1} // Push North towards road
					} else {
						closestPush = Vec2{X: 0, Y: 1} // Push South towards road
					}
				}
			}
		} else {
			// Vertical road: X = r.P1.X, Y in [r.P1.Y, r.P2.Y]
			if pos.Y >= r.P1.Y-40 && pos.Y <= r.P2.Y+40 {
				dCenter := math.Abs(pos.X - r.P1.X)
				dBeyond := dCenter - roadHW
				if dBeyond < minDistBeyondCurb {
					minDistBeyondCurb = dBeyond
					if pos.X > r.P1.X {
						closestPush = Vec2{X: -1, Y: 0} // Push West towards road
					} else {
						closestPush = Vec2{X: 1, Y: 0} // Push East towards road
					}
				}
			}
		}
	}

	// Decision based on minimum distance beyond curb
	if minDistBeyondCurb <= 0 {
		// Fully on asphalt road
		return BoundaryCheckResult{Zone: ZoneRoad}
	} else if minDistBeyondCurb <= ShoulderWidth {
		// On roadside shoulder/sidewalk (can drive, but slows down)
		return BoundaryCheckResult{
			Zone:        ZoneShoulder,
			Penetration: minDistBeyondCurb,
		}
	}

	// Beyond shoulder -> Hard barrier impact!
	return BoundaryCheckResult{
		Zone:         ZoneHardBarrier,
		Pushback:     closestPush,
		Penetration:  minDistBeyondCurb - ShoulderWidth,
		ObstacleName: "路侧硬阻隔防撞护栏",
	}
}

// UpdateLaneWidth updates all bus stop positions dynamically when lane width changes
func (w *World) UpdateLaneWidth(laneWidth float64) {
	for _, s := range w.Stops {
		s.UpdateGeometry(laneWidth)
	}
}

func (w *World) Update(dt float64) {
	for _, s := range w.Stops {
		s.Update(dt)
	}
}

func (w *World) Draw(screen *ebiten.Image, cam *Camera, targetStopID int, laneWidth float64) {
	// 1. Draw base ground
	screen.Fill(RGBA(15, 23, 42, 255)) // Dark metropolitan slate

	// 2. Draw Scenic Rivers & Canals
	for _, riv := range w.Rivers {
		drawRiver(screen, cam, riv)
	}

	// 3. Draw Road Network, Soft Shoulders & Hard Crash Barriers
	drawIntersectionPads(screen, cam, laneWidth)
	for _, r := range w.Roads {
		drawRoad(screen, cam, r, laneWidth)
	}
	drawIntersectionAsphalt(screen, cam, laneWidth)

	// 4. Draw Intersections & Crosswalks
	drawCrosswalks(screen, cam, laneWidth)

	// 5. Draw Bus Stops
	for _, s := range w.Stops {
		s.Draw(screen, cam, s.ID == targetStopID, false)
	}

	// 6. Draw Buildings & Shadows
	for _, b := range w.Buildings {
		drawBuilding(screen, cam, b)
	}

	// 7. Draw Trees
	for _, t := range w.Trees {
		drawTree(screen, cam, t)
	}
}

func drawIntersectionPads(screen *ebiten.Image, cam *Camera, laneWidth float64) {
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}
	// Expanded turning pad: generous intersection plaza clearance at every intersection
	padHalf := math.Max(130.0, laneWidth*2.6 + 24.0)
	shoulderHalf := padHalf + ShoulderWidth

	for _, x := range xs {
		for _, y := range ys {
			c := Vec2{X: x, Y: y}
			// Draw shoulder backing apron
			drawRotatedBox(screen, cam, c, 1, 0, shoulderHalf*2, shoulderHalf*2, RGBA(51, 65, 85, 255))
			// Draw smooth asphalt intersection apron
			drawRotatedBox(screen, cam, c, 1, 0, padHalf*2, padHalf*2, RGBA(30, 41, 59, 255))
		}
	}
}

func drawIntersectionAsphalt(screen *ebiten.Image, cam *Camera, laneWidth float64) {
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}
	padHalf := math.Max(130.0, laneWidth*2.6 + 24.0)

	for _, x := range xs {
		for _, y := range ys {
			c := Vec2{X: x, Y: y}
			drawRotatedBox(screen, cam, c, 1, 0, padHalf*2, padHalf*2, RGBA(30, 41, 59, 255))
		}
	}
}

func drawRiver(screen *ebiten.Image, cam *Camera, riv River) {
	drawRotatedBox(screen, cam, Vec2{X: 0, Y: riv.Y}, 1, 0, 8000, riv.Height, RGBA(14, 116, 144, 210)) // Cyan-blue canal water
}

func drawRoad(screen *ebiten.Image, cam *Camera, r RoadSegment, laneWidth float64) {
	dir := r.P2.Sub(r.P1)
	len := dir.Length()
	if len == 0 {
		return
	}
	heading := math.Atan2(dir.Y, dir.X)
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	center := r.P1.Add(r.P2).Mul(0.5)

	roadW := r.GetWidth(laneWidth)

	// 1. Soft Shoulder (drivable buffer with deceleration, ShoulderWidth = 36px)
	shoulderTotalW := roadW + ShoulderWidth*2
	drawRotatedBox(screen, cam, center, cosH, sinH, len, shoulderTotalW, RGBA(51, 65, 85, 255)) // Slate sidewalk / shoulder

	// 2. Asphalt Road Surface (#1e293b)
	drawRotatedBox(screen, cam, center, cosH, sinH, len, roadW, RGBA(30, 41, 59, 255))

	// 3. Road Curb dividing lines (marking road edge vs shoulder)
	curbOffset := roadW / 2
	drawParallelLine(screen, cam, center, cosH, sinH, len, curbOffset, RGBA(148, 163, 184, 180), 2)
	drawParallelLine(screen, cam, center, cosH, sinH, len, -curbOffset, RGBA(148, 163, 184, 180), 2)

	// 4. Hard Crash Barrier (硬阻隔防撞护栏) along outer edge of shoulder
	barrierOffset := roadW/2 + ShoulderWidth
	drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, barrierOffset, laneWidth)
	drawHazardBarrier(screen, cam, r, center, cosH, sinH, len, -barrierOffset, laneWidth)

	// 5. Center Dividers & Lane Markings
	if r.HasLines {
		if r.Lanes == 4 {
			drawCenterDividers(screen, cam, center, cosH, sinH, len, 6.0, RGBA(250, 204, 21, 240))
			drawCenterDividers(screen, cam, center, cosH, sinH, len, -6.0, RGBA(250, 204, 21, 240))

			laneOffset1 := laneWidth
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, laneOffset1)
			drawDashedWhiteLine(screen, cam, center, cosH, sinH, len, -laneOffset1)
		} else {
			drawDashedYellowLine(screen, cam, center, cosH, sinH, len)
		}
	}
}

func drawParallelLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, offset float64, c color.RGBA, stroke float32) {
	p1 := center.Add(Vec2{X: -cosH*len/2 - sinH*offset, Y: -sinH*len/2 + cosH*offset})
	p2 := center.Add(Vec2{X: cosH*len/2 - sinH*offset, Y: sinH*len/2 + cosH*offset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), stroke*float32(cam.Zoom), c, true)
}

func drawHazardBarrier(screen *ebiten.Image, cam *Camera, r RoadSegment, center Vec2, cosH, sinH, len, offset, laneWidth float64) {
	// Draw hard barrier with hazard stripes, skipping intersection openings
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}
	isEW := math.Abs(r.P2.Y-r.P1.Y) < 1.0

	gapRadius := math.Max(175.0, laneWidth*2.8 + 40.0) // generous opening at intersections for comfortable turning

	if isEW {
		// Draw segments between vertical intersections
		prevX := r.P1.X
		for _, ix := range xs {
			segStart := prevX + gapRadius
			segEnd := ix - gapRadius
			if segEnd > segStart {
				p1 := Vec2{X: segStart, Y: r.P1.Y + offset}
				p2 := Vec2{X: segEnd, Y: r.P1.Y + offset}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				// Thick dark concrete barrier line
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
				// Yellow reflective hazard trim
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
			}
			prevX = ix
		}
		// Final segment to road end
		segStart := prevX + gapRadius
		segEnd := r.P2.X
		if segEnd > segStart {
			p1 := Vec2{X: segStart, Y: r.P1.Y + offset}
			p2 := Vec2{X: segEnd, Y: r.P1.Y + offset}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
		}
	} else {
		// Vertical road: draw segments between horizontal intersections
		prevY := r.P1.Y
		for _, iy := range ys {
			segStart := prevY + gapRadius
			segEnd := iy - gapRadius
			if segEnd > segStart {
				p1 := Vec2{X: r.P1.X - offset, Y: segStart}
				p2 := Vec2{X: r.P1.X - offset, Y: segEnd}
				s1 := cam.WorldToScreen(p1)
				s2 := cam.WorldToScreen(p2)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
				vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
			}
			prevY = iy
		}
		segStart := prevY + gapRadius
		segEnd := r.P2.Y
		if segEnd > segStart {
			p1 := Vec2{X: r.P1.X - offset, Y: segStart}
			p2 := Vec2{X: r.P1.X - offset, Y: segEnd}
			s1 := cam.WorldToScreen(p1)
			s2 := cam.WorldToScreen(p2)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(5*cam.Zoom), RGBA(15, 23, 42, 255), true)
			vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), float32(2*cam.Zoom), RGBA(234, 179, 8, 255), true)
		}
	}
}

func drawCenterDividers(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, perpOffset float64, c color.RGBA) {
	p1 := center.Add(Vec2{X: -cosH*len/2 - sinH*perpOffset, Y: -sinH*len/2 + cosH*perpOffset})
	p2 := center.Add(Vec2{X: cosH*len/2 - sinH*perpOffset, Y: sinH*len/2 + cosH*perpOffset})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), c, true)
}

func drawDashedYellowLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len float64) {
	dashLen := 26.0
	gapLen := 20.0
	step := dashLen + gapLen
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(Vec2{X: cosH * dStart, Y: sinH * dStart})
		db := center.Add(Vec2{X: cosH * dEnd, Y: sinH * dEnd})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(3*cam.Zoom), RGBA(250, 204, 21, 230), true)
	}
}

func drawDashedWhiteLine(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, perpOffset float64) {
	dashLen := 24.0
	gapLen := 22.0
	step := dashLen + gapLen
	numDashes := int(len / step)
	for i := 0; i < numDashes; i++ {
		dStart := -len/2 + float64(i)*step
		dEnd := dStart + dashLen
		da := center.Add(Vec2{X: cosH*dStart - sinH*perpOffset, Y: sinH*dStart + cosH*perpOffset})
		db := center.Add(Vec2{X: cosH*dEnd - sinH*perpOffset, Y: sinH*dEnd + cosH*perpOffset})
		sa := cam.WorldToScreen(da)
		sb := cam.WorldToScreen(db)
		vector.StrokeLine(screen, float32(sa.X), float32(sa.Y), float32(sb.X), float32(sb.Y),
			float32(2*cam.Zoom), RGBA(241, 245, 249, 180), true)
	}
}

func drawCrosswalks(screen *ebiten.Image, cam *Camera, laneWidth float64) {
	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}

	// Geometry: Stop line is at stopLineDist.
	// Zebra crossing is located just outside the intersection conflict box,
	// BEFORE the stop line (closer to intersection center than stop line).
	// Incoming cars stop behind the stop line, leaving a clean buffer before the zebra crossing!
	stopLineDist := laneWidth*1.6 + 18.0
	crosswalkDist := stopLineDist - 25.0

	for _, x := range xs {
		for _, y := range ys {
			pt := Vec2{X: x, Y: y}
			isEW4Lane := (y == -1200 || y == 0 || y == 1200)
			isNS4Lane := (x == 0)

			// 1. Draw 4 Zebra Crossings around intersection
			// North branch (pedestrians cross EW):
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: 0, Y: -crosswalkDist}), true, isNS4Lane, laneWidth)
			// South branch (pedestrians cross EW):
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: 0, Y: crosswalkDist}), true, isNS4Lane, laneWidth)
			// West branch (pedestrians cross NS):
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: -crosswalkDist, Y: 0}), false, isEW4Lane, laneWidth)
			// East branch (pedestrians cross NS):
			drawZebraBranch(screen, cam, pt.Add(Vec2{X: crosswalkDist, Y: 0}), false, isEW4Lane, laneWidth)

			// 2. Draw dedicated turning lane markings & pavement arrows for 4-lane avenues
			if isEW4Lane {
				// West approach (Eastbound, incoming lane on +Y)
				wCenter := pt.X - stopLineDist
				wInnerY := pt.Y + 4 + laneWidth*0.5
				wOuterY := pt.Y + 4 + laneWidth*1.5
				// Left-turn arrow (↰) on inner lane
				drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wInnerY}, 0, true)
				drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wInnerY}, 0, true)
				// Straight arrow (↑) on outer lane
				drawGroundArrow(screen, cam, Vec2{X: wCenter - 45, Y: wOuterY}, 0, false)
				drawGroundArrow(screen, cam, Vec2{X: wCenter - 110, Y: wOuterY}, 0, false)
				// Solid white dividing line separating left-turn lane and through lane
				wLineY := pt.Y + 4 + laneWidth
				drawSolidWhiteDivider(screen, cam, Vec2{X: wCenter - 60, Y: wLineY}, 1, 0, 120)

				// East approach (Westbound, incoming lane on -Y)
				eCenter := pt.X + stopLineDist
				eInnerY := pt.Y - 4 - laneWidth*0.5
				eOuterY := pt.Y - 4 - laneWidth*1.5
				drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eInnerY}, math.Pi, true)
				drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eInnerY}, math.Pi, true)
				drawGroundArrow(screen, cam, Vec2{X: eCenter + 45, Y: eOuterY}, math.Pi, false)
				drawGroundArrow(screen, cam, Vec2{X: eCenter + 110, Y: eOuterY}, math.Pi, false)
				eLineY := pt.Y - 4 - laneWidth
				drawSolidWhiteDivider(screen, cam, Vec2{X: eCenter + 60, Y: eLineY}, -1, 0, 120)
			}

			if isNS4Lane {
				// North approach (Southbound, incoming lane on -X)
				nCenter := pt.Y - stopLineDist
				nInnerX := pt.X - 4 - laneWidth*0.5
				nOuterX := pt.X - 4 - laneWidth*1.5
				drawGroundArrow(screen, cam, Vec2{X: nInnerX, Y: nCenter - 45}, math.Pi/2, true)
				drawGroundArrow(screen, cam, Vec2{X: nInnerX, Y: nCenter - 110}, math.Pi/2, true)
				drawGroundArrow(screen, cam, Vec2{X: nOuterX, Y: nCenter - 45}, math.Pi/2, false)
				drawGroundArrow(screen, cam, Vec2{X: nOuterX, Y: nCenter - 110}, math.Pi/2, false)
				nLineX := pt.X - 4 - laneWidth
				drawSolidWhiteDivider(screen, cam, Vec2{X: nLineX, Y: nCenter - 60}, 0, 1, 120)

				// South approach (Northbound, incoming lane on +X)
				sCenter := pt.Y + stopLineDist
				sInnerX := pt.X + 4 + laneWidth*0.5
				sOuterX := pt.X + 4 + laneWidth*1.5
				drawGroundArrow(screen, cam, Vec2{X: sInnerX, Y: sCenter + 45}, -math.Pi/2, true)
				drawGroundArrow(screen, cam, Vec2{X: sInnerX, Y: sCenter + 110}, -math.Pi/2, true)
				drawGroundArrow(screen, cam, Vec2{X: sOuterX, Y: sCenter + 45}, -math.Pi/2, false)
				drawGroundArrow(screen, cam, Vec2{X: sOuterX, Y: sCenter + 110}, -math.Pi/2, false)
				sLineX := pt.X + 4 + laneWidth
				drawSolidWhiteDivider(screen, cam, Vec2{X: sLineX, Y: sCenter + 60}, 0, -1, 120)
			}
		}
	}
}

func drawZebraBranch(screen *ebiten.Image, cam *Camera, center Vec2, horizontal bool, is4Lane bool, laneWidth float64) {
	numStripes := 6
	step := 15.0
	if is4Lane {
		numStripes = 12
	}
	for i := -numStripes / 2; i <= numStripes/2; i++ {
		var p1, p2 Vec2
		if horizontal {
			offset := float64(i) * step
			p1 = center.Add(Vec2{X: offset, Y: -10})
			p2 = center.Add(Vec2{X: offset, Y: 10})
		} else {
			offset := float64(i) * step
			p1 = center.Add(Vec2{X: -10, Y: offset})
			p2 = center.Add(Vec2{X: 10, Y: offset})
		}
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(7.5*cam.Zoom), RGBA(241, 245, 249, 215), true)
	}
}

func drawSolidWhiteDivider(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, length float64) {
	halfLen := length * 0.5
	p1 := center.Add(Vec2{X: -cosH * halfLen, Y: -sinH * halfLen})
	p2 := center.Add(Vec2{X: cosH * halfLen, Y: sinH * halfLen})
	s1 := cam.WorldToScreen(p1)
	s2 := cam.WorldToScreen(p2)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
		float32(2.5*cam.Zoom), RGBA(248, 250, 252, 220), true)
}

func drawGroundArrow(screen *ebiten.Image, cam *Camera, pos Vec2, heading float64, isLeftTurn bool) {
	scale := float32(cam.Zoom)
	if scale < 0.28 {
		return
	}
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	fwd := Vec2{X: cosH, Y: sinH}
	left := Vec2{X: sinH, Y: -cosH} // In screen coords (+Y is down), driver's left is (sinH, -cosH)

	col := RGBA(248, 250, 252, 225)
	lw := float32(2.8 * scale)

	if !isLeftTurn {
		// Straight Arrow (↑)
		p1 := pos.Sub(fwd.Mul(14))
		p2 := pos.Add(fwd.Mul(14))
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lw, col, true)

		pl := pos.Add(fwd.Mul(6)).Add(left.Mul(6))
		pr := pos.Add(fwd.Mul(6)).Sub(left.Mul(6))
		sl := cam.WorldToScreen(pl)
		sr := cam.WorldToScreen(pr)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sl.X), float32(sl.Y), lw, col, true)
		vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(sr.X), float32(sr.Y), lw, col, true)
	} else {
		// Left-Turn Arrow (↰)
		p1 := pos.Sub(fwd.Mul(14))
		pCurve := pos.Add(fwd.Mul(4))
		s1 := cam.WorldToScreen(p1)
		sc := cam.WorldToScreen(pCurve)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(sc.X), float32(sc.Y), lw, col, true)

		pTip := pCurve.Add(left.Mul(15))
		stip := cam.WorldToScreen(pTip)
		vector.StrokeLine(screen, float32(sc.X), float32(sc.Y), float32(stip.X), float32(stip.Y), lw, col, true)

		pBarbT := pTip.Sub(left.Mul(5)).Add(fwd.Mul(4))
		pBarbB := pTip.Sub(left.Mul(5)).Sub(fwd.Mul(4))
		sbt := cam.WorldToScreen(pBarbT)
		sbb := cam.WorldToScreen(pBarbB)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbt.X), float32(sbt.Y), lw, col, true)
		vector.StrokeLine(screen, float32(stip.X), float32(stip.Y), float32(sbb.X), float32(sbb.Y), lw, col, true)
	}
}

func drawBuilding(screen *ebiten.Image, cam *Camera, b Building) {
	shadowOffset := Vec2{X: 12, Y: 16}
	drawRotatedBox(screen, cam, b.Pos.Add(shadowOffset), 1, 0, b.W, b.H, RGBA(2, 6, 23, 110))
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W, b.H, b.Color)

	roofPadding := 10.0
	drawRotatedBox(screen, cam, b.Pos, 1, 0, b.W-roofPadding, b.H-roofPadding, b.Roof)

	for wx := -b.W/2 + 25; wx < b.W/2-25; wx += 32 {
		for wy := -b.H/2 + 25; wy < b.H/2-25; wy += 32 {
			wp := b.Pos.Add(Vec2{X: wx, Y: wy})
			ws := cam.WorldToScreen(wp)
			vector.DrawFilledCircle(screen, float32(ws.X), float32(ws.Y), float32(3*cam.Zoom),
				RGBA(254, 240, 138, 160), true)
		}
	}
}

func drawTree(screen *ebiten.Image, cam *Camera, t Tree) {
	ts := cam.WorldToScreen(t.Pos)
	r := float32(t.Radius * cam.Zoom)
	if r < 2 {
		r = 2
	}
	vector.DrawFilledCircle(screen, float32(ts.X+float64(r*0.4)), float32(ts.Y+float64(r*0.4)), r, RGBA(2, 6, 23, 70), true)
	vector.DrawFilledCircle(screen, float32(ts.X), float32(ts.Y), r, t.Color, true)
	vector.DrawFilledCircle(screen, float32(ts.X-float64(r*0.2)), float32(ts.Y-float64(r*0.2)), r*0.5,
		RGBA(74, 222, 128, 140), true)
}
