package game

import (
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type CarType int

const (
	CarSedan CarType = iota
	CarTaxi
	CarSUV
)

// TrafficCar represents an autonomous AI civilian vehicle
type TrafficCar struct {
	ID          int
	Pos         Vec2
	Heading     float64
	Speed       float64
	TargetSpeed float64
	Type        CarType
	Color       color.RGBA

	Length float64
	Width  float64

	IsBraking  bool
	RoadCenter float64 // The Y or X axis of the road being followed
	IsEW       bool    // true if on East-West road, false if on North-South
	LaneIndex  int     // 0 = inner / left-turn lane, 1 = outer / through lane

	// Roundabout navigation
	InRoundabout          bool
	RoundaboutCenter      Vec2
	RoundaboutRadius      float64
	RoundaboutAngle       float64
	RoundaboutExitAngle   float64
	RoundaboutExitHeading float64
	RoundaboutExitEW      bool
	RoundaboutExitRC      float64
	RoundaboutCooldown    float64
	RoundaboutTraveled    float64
	RoundaboutTargetR     float64
	RoundaboutOuterRadius float64
}

type TrafficManager struct {
	Cars     []*TrafficCar
	AIBusMgr *AIBusManager
}

func NewTrafficManager(initialLaneWidth float64) *TrafficManager {
	tm := &TrafficManager{
		Cars: make([]*TrafficCar, 0),
	}
	tm.initializeCars(36, initialLaneWidth)
	return tm
}

// GetActiveCars returns the slice of cars active under the given density level
func (tm *TrafficManager) GetActiveCars(densityLevel int) []*TrafficCar {
	if tm == nil {
		return nil
	}
	activeCount := 24
	if densityLevel == 1 {
		activeCount = 12
	} else if densityLevel == 3 {
		activeCount = 36
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}
	return tm.Cars[:activeCount]
}

var carPalette = []color.RGBA{
	RGBA(239, 68, 68, 255),   // Crimson
	RGBA(59, 130, 246, 255),  // Cobalt Blue
	RGBA(16, 185, 129, 255),  // Emerald
	RGBA(248, 250, 252, 255), // Pearl White
	RGBA(71, 85, 105, 255),   // Slate Gray
	RGBA(168, 85, 247, 255),  // Violet
	RGBA(234, 179, 8, 255),   // Taxi Yellow
}

func (tm *TrafficManager) initializeCars(count int, laneWidth float64) {
	// 36 spawn points spread across safe mid-blocks of the metropolitan grid
	// Every spawn point is at least 300~500px away from any intersection center
	spawnDefs := []struct {
		pos Vec2
		h   float64
		rc  float64
		ew  bool
	}{
		// North Ring (Y = -1200, 4-lane) Eastbound & Westbound
		{pos: Vec2{X: -2400, Y: -1200 + laneWidth*1.5}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: -1950, Y: -1200 + laneWidth*1.5}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: -1000, Y: -1200 + laneWidth*1.5}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 500, Y: -1200 + laneWidth*1.5}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 1600, Y: -1200 + laneWidth*1.5}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 2400, Y: -1200 - laneWidth*1.5}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: 2300, Y: -1200 - laneWidth*1.5}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: 800, Y: -1200 - laneWidth*1.5}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: -350, Y: -1200 - laneWidth*1.5}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: -1950, Y: -1200 - laneWidth*1.5}, h: math.Pi, rc: -1200, ew: true},

		// Central Boulevard (Y = 0, 4-lane) Eastbound & Westbound
		{pos: Vec2{X: -2400, Y: 0 + laneWidth*1.5}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: -1750, Y: 0 + laneWidth*1.5}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: -1000, Y: 0 + laneWidth*1.5}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: 350, Y: 0 + laneWidth*1.5}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: 1750, Y: 0 + laneWidth*1.5}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: 2400, Y: 0 - laneWidth*1.5}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: 2150, Y: 0 - laneWidth*1.5}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: 1000, Y: 0 - laneWidth*1.5}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: -350, Y: 0 - laneWidth*1.5}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: -2150, Y: 0 - laneWidth*1.5}, h: math.Pi, rc: 0, ew: true},

		// South Ring (Y = 1200, 4-lane) Eastbound & Westbound
		{pos: Vec2{X: -2400, Y: 1200 + laneWidth*1.5}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: -1500, Y: 1200 + laneWidth*1.5}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: -500, Y: 1200 + laneWidth*1.5}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 350, Y: 1200 + laneWidth*1.5}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 1750, Y: 1200 + laneWidth*1.5}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 2400, Y: 1200 - laneWidth*1.5}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: 2150, Y: 1200 - laneWidth*1.5}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: 1000, Y: 1200 - laneWidth*1.5}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: -800, Y: 1200 - laneWidth*1.5}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: -1500, Y: 1200 - laneWidth*1.5}, h: math.Pi, rc: 1200, ew: true},

		// North-South: Far-West Blvd (X = -2600, 2-lane)
		{pos: Vec2{X: -2600 - laneWidth*0.5, Y: -900}, h: math.Pi / 2, rc: -2600, ew: false},
		{pos: Vec2{X: -2600 - laneWidth*0.5, Y: 300}, h: math.Pi / 2, rc: -2600, ew: false},
		{pos: Vec2{X: -2600 + laneWidth*0.5, Y: 900}, h: -math.Pi / 2, rc: -2600, ew: false},
		{pos: Vec2{X: -2600 + laneWidth*0.5, Y: -300}, h: -math.Pi / 2, rc: -2600, ew: false},

		// North-South: Central Axis (X = 0, 4-lane)
		{pos: Vec2{X: 0 - laneWidth*1.5, Y: -900}, h: math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 - laneWidth*1.5, Y: 450}, h: math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 + laneWidth*1.5, Y: 900}, h: -math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 + laneWidth*1.5, Y: -450}, h: -math.Pi / 2, rc: 0, ew: false},

		// North-South: Far-East Blvd (X = 2600, 2-lane)
		{pos: Vec2{X: 2600 - laneWidth*0.5, Y: -900}, h: math.Pi / 2, rc: 2600, ew: false},
		{pos: Vec2{X: 2600 - laneWidth*0.5, Y: 300}, h: math.Pi / 2, rc: 2600, ew: false},
		{pos: Vec2{X: 2600 + laneWidth*0.5, Y: 900}, h: -math.Pi / 2, rc: 2600, ew: false},
		{pos: Vec2{X: 2600 + laneWidth*0.5, Y: -300}, h: -math.Pi / 2, rc: 2600, ew: false},
	}

	// 25 City Intersections coordinates for programmatic validation
	interCenters := []Vec2{
		{X: -2600, Y: -1200}, {X: -1300, Y: -1200}, {X: 0, Y: -1200}, {X: 1300, Y: -1200}, {X: 2600, Y: -1200},
		{X: -2600, Y: -600}, {X: -1300, Y: -600}, {X: 0, Y: -600}, {X: 1300, Y: -600}, {X: 2600, Y: -600},
		{X: -2600, Y: 0}, {X: -1300, Y: 0}, {X: 0, Y: 0}, {X: 1300, Y: 0}, {X: 2600, Y: 0},
		{X: -2600, Y: 600}, {X: -1300, Y: 600}, {X: 0, Y: 600}, {X: 1300, Y: 600}, {X: 2600, Y: 600},
		{X: -2600, Y: 1200}, {X: -1300, Y: 1200}, {X: 0, Y: 1200}, {X: 1300, Y: 1200}, {X: 2600, Y: 1200},
	}

	for i := 0; i < count && i < len(spawnDefs); i++ {
		sd := spawnDefs[i]
		spawnPos := sd.pos

		// Programmatic guarantee: never spawn inside or within 220px of any intersection
		for _, ic := range interCenters {
			if spawnPos.Distance(ic) < 240.0 {
				if sd.ew {
					if math.Cos(sd.h) >= 0 {
						spawnPos.X = ic.X + 280
					} else {
						spawnPos.X = ic.X - 280
					}
				} else {
					if math.Sin(sd.h) >= 0 {
						spawnPos.Y = ic.Y + 280
					} else {
						spawnPos.Y = ic.Y - 280
					}
				}
			}
		}

		cType := CarSedan
		cColor := carPalette[i%len(carPalette)]
		if i%4 == 0 {
			cType = CarTaxi
			cColor = RGBA(234, 179, 8, 255)
		} else if i%3 == 0 {
			cType = CarSUV
		}

		car := &TrafficCar{
			ID:          i,
			Pos:         spawnPos,
			Heading:     sd.h,
			Speed:       130 + simRand.Float64()*30,
			TargetSpeed: 150 + simRand.Float64()*40,
			Type:        cType,
			Color:       cColor,
			Length:      46,
			Width:       22,
			RoadCenter:  sd.rc,
			IsEW:        sd.ew,
			LaneIndex:   i % 2,
		}
		if cType == CarSUV {
			car.Length = 50
			car.Width = 24
		}
		tm.Cars = append(tm.Cars, car)
	}
}

// BuildForWorld adapts civilian AI traffic to the active city map's road network
func (tm *TrafficManager) BuildForWorld(w *World, count int, laneWidth float64) {
	if w == nil {
		return
	}
	tm.Cars = make([]*TrafficCar, 0)
	if w.Theme == MapThemeMetropolis {
		tm.initializeCars(count, laneWidth)
		return
	}

	type spawnDef struct {
		pos Vec2
		h   float64
		rc  float64
		ew  bool
	}
	var defs []spawnDef

	for _, r := range w.Roads {
		dir := r.P2.Sub(r.P1)
		lenR := dir.Length()
		if lenR < 500 {
			continue
		}
		isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		var roadCoord float64
		if isEW {
			roadCoord = r.P1.Y
		} else {
			roadCoord = r.P1.X
		}

		fractions := []float64{0.25, 0.5, 0.75}
		for _, frac := range fractions {
			pt := r.P1.Add(dir.Mul(frac))

			nearInter := false
			for _, ipt := range w.Intersections {
				if pt.Distance(ipt) < 220 {
					nearInter = true
					break
				}
			}
			if nearInter {
				continue
			}

			nearRb := false
			for _, rb := range w.Roundabouts {
				if pt.Distance(rb.Center) < rb.Radius+50 {
					nearRb = true
					break
				}
			}
			if nearRb {
				continue
			}

			nearStop := false
			for _, st := range w.Stops {
				if pt.Distance(st.Pos) < 160 {
					nearStop = true
					break
				}
			}
			if nearStop {
				continue
			}

			is4Lane := w.IsRoad4Lane(roadCoord, isEW, pt)
			offset := laneWidth * 0.5
			if is4Lane {
				offset = laneWidth * 1.5 // outer travel lane
			}

			if isEW {
				defs = append(defs,
					spawnDef{pos: Vec2{X: pt.X, Y: roadCoord + offset}, h: 0, rc: roadCoord, ew: true},
					spawnDef{pos: Vec2{X: pt.X, Y: roadCoord - offset}, h: math.Pi, rc: roadCoord, ew: true},
				)
			} else {
				defs = append(defs,
					spawnDef{pos: Vec2{X: roadCoord - offset, Y: pt.Y}, h: math.Pi / 2, rc: roadCoord, ew: false},
					spawnDef{pos: Vec2{X: roadCoord + offset, Y: pt.Y}, h: -math.Pi / 2, rc: roadCoord, ew: false},
				)
			}
		}
	}

	if count > len(defs) {
		count = len(defs)
	}

	for i := 0; i < count; i++ {
		sd := defs[i]
		cType := CarSedan
		cColor := carPalette[i%len(carPalette)]
		if i%4 == 0 {
			cType = CarTaxi
			cColor = RGBA(234, 179, 8, 255)
		} else if i%3 == 0 {
			cType = CarSUV
		}

		car := &TrafficCar{
			ID:          i,
			Pos:         sd.pos,
			Heading:     sd.h,
			Speed:       130 + simRand.Float64()*30,
			TargetSpeed: 150 + simRand.Float64()*40,
			Type:        cType,
			Color:       cColor,
			Length:      46,
			Width:       22,
			RoadCenter:  sd.rc,
			IsEW:        sd.ew,
			LaneIndex:   i % 2,
		}
		if cType == CarSUV {
			car.Length = 50
			car.Width = 24
		}
		tm.Cars = append(tm.Cars, car)
	}
}

// Update updates all AI cars according to traffic rules, bus proximity, traffic density and customizable lane width
func (tm *TrafficManager) Update(dt float64, bus *Bus, tls *TrafficLightSystem, densityLevel int, laneWidth float64, worldOptional ...*World) {
	activeCount := 24
	if densityLevel == 1 {
		activeCount = 12
	} else if densityLevel == 3 {
		activeCount = 36
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}

	var w *World
	if len(worldOptional) > 0 && worldOptional[0] != nil {
		w = worldOptional[0]
	}

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]
		tm.updateSingleCar(dt, car, i, bus, tls, activeCount, laneWidth, w)
	}
}

func (tm *TrafficManager) updateSingleCar(dt float64, car *TrafficCar, index int, bus *Bus, tls *TrafficLightSystem, activeCount int, laneWidth float64, w *World) {
	// 1. Perception & Sensors
	targetSpd := car.TargetSpeed
	car.IsBraking = false

	if car.RoundaboutCooldown > 0 {
		car.RoundaboutCooldown -= dt
	}

	// A. Check Traffic Light ahead & Stop Line
	lookAheadLight := laneWidth*1.5 + 85.0
	halfLen := car.Length * 0.5
	cosH := math.Cos(car.Heading)
	sinH := math.Sin(car.Heading)
	bumperPos := Vec2{
		X: car.Pos.X + cosH*halfLen,
		Y: car.Pos.Y + sinH*halfLen,
	}

	wantLeft := false
	if car.IsEW {
		if math.Abs(car.Pos.Y-car.RoadCenter) < laneWidth*0.95 {
			wantLeft = true
		}
	} else {
		if math.Abs(car.Pos.X-car.RoadCenter) < laneWidth*0.95 {
			wantLeft = true
		}
	}

	if tls != nil && !car.InRoundabout {
		if shouldStop, distToLine := tls.CheckStopForVehicle(bumperPos, car.Heading, lookAheadLight, laneWidth, wantLeft); shouldStop {
			car.IsBraking = true
			if distToLine <= 14.0 {
				// Firm stop: front bumper stays 10~14px behind the stop line (and >25px clear of zebra crossing)
				targetSpd = 0
			} else {
				// Smooth deceleration towards the stop line
				targetSpd = math.Min(targetSpd, math.Max(0, (distToLine-10.0)*1.8))
			}
		}
	}

	// B. Check Player Bus ahead
	if !car.InRoundabout {
		distToBus := car.Pos.Distance(bus.Pos)
		if distToBus < 260.0 {
			toBus := bus.Pos.Sub(car.Pos)
			cosH := math.Cos(car.Heading)
			sinH := math.Sin(car.Heading)
			forwardProj := toBus.X*cosH + toBus.Y*sinH
			lateralProj := math.Abs(-toBus.X*sinH + toBus.Y*cosH)

			// Oriented extent of bus projected onto car coordinate axes
			angleDiff := bus.Heading - car.Heading
			busHalfLen := bus.Length / 2.0 // 48.0
			busHalfWid := bus.Width / 2.0  // 18.0
			hLat := busHalfLen*math.Abs(math.Sin(angleDiff)) + busHalfWid*math.Abs(math.Cos(angleDiff))
			hFwd := busHalfLen*math.Abs(math.Cos(angleDiff)) + busHalfWid*math.Abs(math.Sin(angleDiff))

			carHalfLen := car.Length / 2.0 // 23~25
			carHalfWid := car.Width / 2.0  // 11.0

			// Bumper-to-bumper distance along car forward axis
			bumperDist := forwardProj - carHalfLen - hFwd

			isBusStopped := math.Abs(bus.Speed) < 8.0
			isWrongWay := IsBusFacingWrongWay(bus)

			// Required lateral clearance to pass safely without clipping
			lateralMargin := 6.0
			if isBusStopped {
				lateralMargin = 8.0 // Safety margin around stopped bus
			}
			lateralClearanceNeeded := carHalfWid + hLat + lateralMargin

			// If bus is in front of the car and in car's travel corridor
			if forwardProj > 0 && forwardProj < 240.0 && lateralProj < lateralClearanceNeeded {
				if isBusStopped && !isWrongWay {
					// Lawfully stopped bus (at bus stop bay, red light, or traffic queue):
					// SOCIAL VEHICLES MUST NOT ACTIVELY HIT IT!
					car.IsBraking = true

					if bumperDist <= 35.0 {
						// Firm stop behind the bus: front bumper stays at least 25px clear of bus
						targetSpd = 0
						if bumperDist <= 15.0 && car.Speed > 0 {
							car.Speed = 0
						}
					} else {
						// Smooth, early deceleration from up to 200px away
						decelSpd := math.Max(0, (bumperDist-32.0)*1.8)
						targetSpd = math.Min(targetSpd, decelSpd)
					}
				} else if !isWrongWay {
					// Moving bus in legal flow: adaptive cruise following
					car.IsBraking = true
					if bumperDist <= 24.0 {
						targetSpd = 0
						if bumperDist <= 10.0 && car.Speed > 0 {
							car.Speed = 0
						}
					} else {
						decelSpd := math.Max(0, (bumperDist-20.0)*1.6)
						targetSpd = math.Min(targetSpd, decelSpd)
					}
				} else {
					// Bus is driving wrong-way (逆行) into oncoming traffic:
					// AI car attempts emergency braking if possible
					if bumperDist <= 15.0 {
						targetSpd = 0
						car.IsBraking = true
					} else if bumperDist < 90.0 {
						targetSpd = math.Min(targetSpd, math.Max(0, (bumperDist-15.0)*1.5))
						car.IsBraking = true
					}
				}
			}
		}
	}

	// C. Check Other AI Cars ahead in same lane
	if !car.InRoundabout {
		for j := 0; j < activeCount; j++ {
			if j == index {
				continue
			}
			other := tm.Cars[j]
			if other.InRoundabout {
				continue // Other is in roundabout; entry yield check in Section E handles right of way
			}
			dist := car.Pos.Distance(other.Pos)
			if dist < 120 {
				toOther := other.Pos.Sub(car.Pos)
				cosH := math.Cos(car.Heading)
				sinH := math.Sin(car.Heading)
				fProj := toOther.X*cosH + toOther.Y*sinH
				lProj := math.Abs(-toOther.X*sinH + toOther.Y*cosH)

				if fProj > 0 && fProj < 110 && lProj < laneWidth*0.65 {
					car.IsBraking = true
					if fProj < 60 {
						targetSpd = 0
					} else {
						targetSpd = math.Min(targetSpd, math.Max(0, (fProj-52)*1.8))
					}
				}
			}
		}
	}

	// D. Check Left-Turn Intersection Yielding to Stopped Bus
	if car.LaneIndex == 0 && tls != nil && !car.InRoundabout {
		for _, inter := range tls.Intersections {
			if inter.ApproachHasDedicatedLanes(car.IsEW) && car.Pos.Distance(inter.Center) < 120.0 {
				// If a lawfully stopped bus is inside this intersection:
				if inter.Center.Distance(bus.Pos) < 110.0 && math.Abs(bus.Speed) < 8.0 && !IsBusFacingWrongWay(bus) {
					distToInter := car.Pos.Distance(inter.Center)
					car.IsBraking = true
					if distToInter <= 32.0 {
						targetSpd = 0
						if distToInter <= 15.0 && car.Speed > 0 {
							car.Speed = 0
						}
					} else {
						targetSpd = math.Min(targetSpd, math.Max(0, (distToInter-30.0)*1.5))
					}
					break
				}
			}
		}
	}

	// E. Check Roundabout Navigation & Yielding
	if w != nil && len(w.Roundabouts) > 0 {
		if car.InRoundabout {
			// Speed limit inside roundabout for safe, smooth rotary circulation
			targetSpd = math.Min(targetSpd, 115.0)

			// Check if bus is ahead inside the roundabout or in direct proximity
			if w.IsAtRoundabout(bus.Pos) || bus.Pos.Distance(car.RoundaboutCenter) <= car.RoundaboutOuterRadius+25.0 {
				directDist := car.Pos.Distance(bus.Pos)
				if directDist < 85.0 {
					car.IsBraking = true
					targetSpd = 0
				} else {
					busDist := bus.Pos.Distance(car.RoundaboutCenter)
					if math.Abs(car.RoundaboutTargetR-busDist) <= 45.0 {
						busAngle := math.Atan2(bus.Pos.Y-car.RoundaboutCenter.Y, bus.Pos.X-car.RoundaboutCenter.X)
						angDiff := NormalizeAngle(car.RoundaboutAngle - busAngle)
						if angDiff > 0.0 && angDiff < 0.70 {
							car.IsBraking = true
							targetSpd = 0
						} else if angDiff >= 0.70 && angDiff < 1.20 {
							car.IsBraking = true
							targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.65)*160.0))
						}
					}
				}
			}

			// Check if AI route bus is ahead inside roundabout
			if tm.AIBusMgr != nil {
				for _, ab := range tm.AIBusMgr.Buses {
					if w.IsAtRoundabout(ab.Bus.Pos) || ab.Bus.Pos.Distance(car.RoundaboutCenter) <= car.RoundaboutOuterRadius+25.0 {
						directDist := car.Pos.Distance(ab.Bus.Pos)
						if directDist < 85.0 {
							car.IsBraking = true
							targetSpd = 0
							break
						} else {
							abDist := ab.Bus.Pos.Distance(car.RoundaboutCenter)
							if math.Abs(car.RoundaboutTargetR-abDist) <= 45.0 {
								abAngle := math.Atan2(ab.Bus.Pos.Y-car.RoundaboutCenter.Y, ab.Bus.Pos.X-car.RoundaboutCenter.X)
								angDiff := NormalizeAngle(car.RoundaboutAngle - abAngle)
								if angDiff > 0.0 && angDiff < 0.70 {
									car.IsBraking = true
									targetSpd = 0
									break
								} else if angDiff >= 0.70 && angDiff < 1.20 {
									car.IsBraking = true
									targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.65)*160.0))
									break
								}
							}
						}
					}
				}
			}

			// Check if another AI car is ahead inside the roundabout in our lane
			for j := 0; j < activeCount; j++ {
				if j == index {
					continue
				}
				other := tm.Cars[j]
				if other.InRoundabout && other.RoundaboutCenter == car.RoundaboutCenter {
					directDist := car.Pos.Distance(other.Pos)
					if directDist < 60.0 {
						car.IsBraking = true
						targetSpd = 0
						break
					}
					if math.Abs(car.RoundaboutTargetR-other.RoundaboutTargetR) > 35.0 {
						continue // In a different concentric circular lane
					}
					angDiff := NormalizeAngle(car.RoundaboutAngle - other.RoundaboutAngle)
					if angDiff > 0.0 && angDiff < 0.52 {
						car.IsBraking = true
						targetSpd = 0
						break
					} else if angDiff >= 0.52 && angDiff < 0.95 {
						car.IsBraking = true
						targetSpd = math.Min(targetSpd, math.Max(0, (angDiff-0.5)*180.0))
						break
					}
				}
			}
		} else {
			// Approaching roundabout check
			for _, rb := range w.Roundabouts {
				distToCenter := car.Pos.Distance(rb.Center)
				if distToCenter <= rb.Radius+95.0 && car.RoundaboutCooldown <= 0 {
					toCenter := rb.Center.Sub(car.Pos)
					fwdProj := toCenter.X*math.Cos(car.Heading) + toCenter.Y*math.Sin(car.Heading)
					if fwdProj > 0.0 {
						// 1. Yield check at entry line:
						// Circulating vehicles coming from the left or traversing across entry mouth have right of way
						myAngle := math.Atan2(car.Pos.Y-rb.Center.Y, car.Pos.X-rb.Center.X)
						needYield := false

						// Check if player bus is circulating approaching this entry or crossing the entry mouth
						if w.IsAtRoundabout(bus.Pos) || bus.Pos.Distance(rb.Center) <= rb.Radius+25.0 {
							busAng := math.Atan2(bus.Pos.Y-rb.Center.Y, bus.Pos.X-rb.Center.X)
							diff := NormalizeAngle(busAng - myAngle)
							if (diff > -0.40 && diff < 1.15) || car.Pos.Distance(bus.Pos) < 140.0 {
								needYield = true
							}
						}

						// Check if AI route bus is circulating approaching this entry or crossing the entry mouth
						if !needYield && tm.AIBusMgr != nil {
							for _, ab := range tm.AIBusMgr.Buses {
								if w.IsAtRoundabout(ab.Bus.Pos) || ab.Bus.Pos.Distance(rb.Center) <= rb.Radius+25.0 {
									abAng := math.Atan2(ab.Bus.Pos.Y-rb.Center.Y, ab.Bus.Pos.X-rb.Center.X)
									diff := NormalizeAngle(abAng - myAngle)
									if (diff > -0.40 && diff < 1.15) || car.Pos.Distance(ab.Bus.Pos) < 140.0 {
										needYield = true
										break
									}
								}
							}
						}

						// Check if another AI car is circulating approaching this entry
						if !needYield {
							for j := 0; j < activeCount; j++ {
								if j == index {
									continue
								}
								other := tm.Cars[j]
								if other.InRoundabout && other.RoundaboutCenter == rb.Center {
									diff := NormalizeAngle(other.RoundaboutAngle - myAngle)
									if (diff > -0.30 && diff < 1.05) || car.Pos.Distance(other.Pos) < 110.0 {
										needYield = true
										break
									}
								}
							}
						}

						stopLineDist := rb.Radius + car.Length*0.5 + 16.0
						if needYield {
							if distToCenter > stopLineDist+4.0 {
								targetSpd = math.Min(targetSpd, math.Max(0.0, (distToCenter-stopLineDist)*2.0))
							} else {
								car.IsBraking = true
								targetSpd = 0.0
								if distToCenter <= stopLineDist+1.0 {
									car.Speed = 0.0
								}
							}
						}

						// 2. Enter roundabout when at/inside the circular roadway and NOT yielding
						canEnter := !needYield && distToCenter <= rb.Radius+12.0
						if canEnter {
							// Strict safety clearance: cannot enter if any bus/car is within 85px of entrance mouth
							if car.Pos.Distance(bus.Pos) < 85.0 {
								canEnter = false
							} else if tm.AIBusMgr != nil {
								for _, ab := range tm.AIBusMgr.Buses {
									if car.Pos.Distance(ab.Bus.Pos) < 85.0 {
										canEnter = false
										break
									}
								}
							}
						}

						if canEnter {
							incomingHeading := car.Heading
							car.InRoundabout = true
							car.RoundaboutCenter = rb.Center
							car.RoundaboutTraveled = 0

							rApron := rb.InnerRadius + 16.0
							rOuter := rb.Radius
							rMid := (rApron + rOuter) * 0.5
							circulatingR := (rMid + rOuter) * 0.5 // Outer lane center
							if car.LaneIndex == 0 {
								circulatingR = (rApron + rMid) * 0.5 // Inner lane center
							}
							car.RoundaboutTargetR = circulatingR
							car.RoundaboutOuterRadius = rb.Radius

							// Smooth continuous entry: initialize radius at current distance to center to eliminate any position snapping
							car.RoundaboutRadius = distToCenter
							car.RoundaboutAngle = math.Atan2(car.Pos.Y-rb.Center.Y, car.Pos.X-rb.Center.X)

							isOutgoing4Lane := false
							if math.Abs(math.Cos(incomingHeading)) > math.Abs(math.Sin(incomingHeading)) {
								// East-West road
								if rb.Center.Y == -1200 || rb.Center.Y == 0 || rb.Center.Y == 1200 {
									isOutgoing4Lane = true
								}
							} else {
								// North-South road
								if rb.Center.X == 0 {
									isOutgoing4Lane = true
								}
							}
							laneOff := laneWidth * 0.75
							if isOutgoing4Lane {
								if car.LaneIndex == 0 {
									laneOff = laneWidth * 0.52
								} else {
									laneOff = laneWidth * 1.48
								}
							}
							offsetAngle := math.Asin(math.Min(0.7, laneOff/circulatingR))

							// Determine destination exit based on incoming road approach
							if math.Abs(math.Cos(incomingHeading)) > math.Abs(math.Sin(incomingHeading)) {
								// East-West road
								if math.Cos(incomingHeading) > 0 { // Eastbound
									car.RoundaboutExitHeading = 0
									car.RoundaboutExitEW = true
									car.RoundaboutExitRC = rb.Center.Y
									car.RoundaboutExitAngle = NormalizeAngle(0 + offsetAngle)
								} else { // Westbound
									car.RoundaboutExitHeading = math.Pi
									car.RoundaboutExitEW = true
									car.RoundaboutExitRC = rb.Center.Y
									car.RoundaboutExitAngle = NormalizeAngle(-math.Pi + offsetAngle)
								}
							} else {
								// North-South road
								if math.Sin(incomingHeading) > 0 { // Southbound
									car.RoundaboutExitHeading = math.Pi / 2
									car.RoundaboutExitEW = false
									car.RoundaboutExitRC = rb.Center.X
									car.RoundaboutExitAngle = NormalizeAngle(math.Pi/2 + offsetAngle)
								} else { // Northbound
									car.RoundaboutExitHeading = -math.Pi / 2
									car.RoundaboutExitEW = false
									car.RoundaboutExitRC = rb.Center.X
									car.RoundaboutExitAngle = NormalizeAngle(-math.Pi/2 + offsetAngle)
								}
							}
							break
						}
					}
				}
			}
		}
	}

	// 2. Acceleration / Braking Physics
	if car.Speed < targetSpd {
		car.Speed = math.Min(targetSpd, car.Speed+120*dt)
	} else if car.Speed > targetSpd {
		car.Speed = math.Max(targetSpd, car.Speed-280*dt)
	}

	// 3. Move along Heading or Circulate in Roundabout
	if car.InRoundabout {
		omega := car.Speed / car.RoundaboutRadius
		dTheta := omega * dt
		car.RoundaboutAngle -= dTheta
		car.RoundaboutTraveled += dTheta
		car.RoundaboutAngle = NormalizeAngle(car.RoundaboutAngle)

		// Angle remaining to destination exit (decreasing counter-clockwise towards exit)
		diff := NormalizeAngle(car.RoundaboutAngle - car.RoundaboutExitAngle)
		for diff < 0 {
			diff += 2 * math.Pi
		}

		// Keep car strictly tracking circular lane arc at circulating radius
		targetR := car.RoundaboutTargetR
		tangentH := NormalizeAngle(car.RoundaboutAngle - math.Pi/2)

		maxTurnRate := 5.0 // rad/s (generates natural steering rate without snaps)
		maxTurnStep := maxTurnRate * dt

		// Smoothly ease heading into outgoing road heading as the car approaches the exit junction mouth,
		// without expanding radius into corners (targetR remains strictly on circular arc!).
		if car.RoundaboutTraveled >= 1.0 && diff <= 0.45 {
			tExit := 1.0 - math.Max(0, math.Min(1, diff/0.45))
			targetH := LerpAngle(tangentH, car.RoundaboutExitHeading, tExit)
			car.Heading = RotateTowards(car.Heading, targetH, maxTurnStep)
		} else {
			car.Heading = RotateTowards(car.Heading, tangentH, maxTurnStep)
		}

		// Smooth radial approach to circular lane radius capped to smooth physical lateral rate
		radialDiff := targetR - car.RoundaboutRadius
		maxRadialStep := 150.0 * dt
		if radialDiff > maxRadialStep {
			car.RoundaboutRadius += maxRadialStep
		} else if radialDiff < -maxRadialStep {
			car.RoundaboutRadius -= maxRadialStep
		} else {
			car.RoundaboutRadius = targetR
		}

		// Continuous position along circular trajectory
		car.Pos = car.RoundaboutCenter.Add(Vec2{
			X: math.Cos(car.RoundaboutAngle) * car.RoundaboutRadius,
			Y: math.Sin(car.RoundaboutAngle) * car.RoundaboutRadius,
		})

		// Seamless exit onto outgoing lane when aligned with exit junction
		if car.RoundaboutTraveled >= 1.0 && (diff <= 0.06 || diff >= 2*math.Pi-0.06) {
			car.InRoundabout = false
			car.Heading = car.RoundaboutExitHeading
			car.IsEW = car.RoundaboutExitEW
			car.RoadCenter = car.RoundaboutExitRC
			car.RoundaboutCooldown = 6.0
			// Do NOT snap car.Pos! car.Pos is already at the exit perimeter on the lane!
		}
	} else if car.Speed > 0 {
		car.Pos.X += car.Speed * math.Cos(car.Heading) * dt
		car.Pos.Y += car.Speed * math.Sin(car.Heading) * dt
	}

	// 3.5 AI Intersection Turning:
	// If the road does not continue straight (T-junction, corner, or boundary), the vehicle MUST turn!
	// At standard 4-way intersections, cars in Lane 0 (left-turn lane) turn left.
	if tls != nil && !car.InRoundabout {
		for _, inter := range tls.Intersections {
			if w != nil && w.IsAtRoundabout(inter.Center) {
				continue
			}
			if car.Pos.Distance(inter.Center) < 32.0 {
				hasStraight := hasRoadInDirection(w, inter.Center, car.Heading)
				leftHeading := NormalizeAngle(car.Heading - math.Pi/2)
				hasLeft := hasRoadInDirection(w, inter.Center, leftHeading)
				rightHeading := NormalizeAngle(car.Heading + math.Pi/2)
				hasRight := hasRoadInDirection(w, inter.Center, rightHeading)

				shouldTurn := false
				var targetHeading float64
				turnIsLeft := false

				if !hasStraight {
					// Forced turn: dead end / T-junction / map corner
					if hasLeft && hasRight {
						if car.LaneIndex == 0 {
							targetHeading = leftHeading
							turnIsLeft = true
							shouldTurn = true
						} else {
							targetHeading = rightHeading
							turnIsLeft = false
							shouldTurn = true
						}
					} else if hasLeft {
						targetHeading = leftHeading
						turnIsLeft = true
						shouldTurn = true
					} else if hasRight {
						targetHeading = rightHeading
						turnIsLeft = false
						shouldTurn = true
					}
				} else if car.LaneIndex == 0 && hasLeft && inter.ApproachHasDedicatedLanes(car.IsEW) {
					// Left turn from dedicated turning bay
					targetHeading = leftHeading
					turnIsLeft = true
					shouldTurn = true
				}

				if shouldTurn {
					// Yield to stopped player bus inside the intersection
					if inter.Center.Distance(bus.Pos) < 110.0 && math.Abs(bus.Speed) < 8.0 && !IsBusFacingWrongWay(bus) {
						car.Speed = 0
						car.IsBraking = true
						break
					}

					newIsEW := math.Abs(math.Cos(targetHeading)) > math.Abs(math.Sin(targetHeading))
					newRC := inter.Center.Y
					if !newIsEW {
						newRC = inter.Center.X
					}
					isNew4Lane := false
					if w != nil {
						isNew4Lane = w.IsRoad4Lane(newRC, newIsEW, inter.Center)
					}
					newOffset := laneWidth * 0.5
					if isNew4Lane && !turnIsLeft {
						newOffset = laneWidth * 1.5
					}

					car.Heading = targetHeading
					car.IsEW = newIsEW
					car.RoadCenter = newRC

					if newIsEW {
						if math.Cos(targetHeading) > 0 {
							car.Pos.Y = newRC + newOffset
						} else {
							car.Pos.Y = newRC - newOffset
						}
					} else {
						if math.Sin(targetHeading) > 0 {
							car.Pos.X = newRC - newOffset
						} else {
							car.Pos.X = newRC + newOffset
						}
					}
					car.LaneIndex = 1
					break
				}
			}
		}
	}

	// 4. Map Boundary Wrapping & Lane Tracking with customizable lane width
	tm.handleCarNavigation(car, laneWidth, w)
}

func hasRoadInDirection(w *World, center Vec2, heading float64) bool {
	if w == nil {
		return true
	}
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	isEW := math.Abs(cosH) > math.Abs(sinH)

	for _, r := range w.Roads {
		roadIsEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		if roadIsEW != isEW {
			continue
		}
		if isEW {
			if math.Abs(r.P1.Y-center.Y) > 25.0 {
				continue
			}
			minX := math.Min(r.P1.X, r.P2.X)
			maxX := math.Max(r.P1.X, r.P2.X)
			if cosH > 0 { // East
				if maxX >= center.X+40.0 && minX <= center.X+15.0 {
					return true
				}
			} else { // West
				if minX <= center.X-40.0 && maxX >= center.X-15.0 {
					return true
				}
			}
		} else {
			if math.Abs(r.P1.X-center.X) > 25.0 {
				continue
			}
			minY := math.Min(r.P1.Y, r.P2.Y)
			maxY := math.Max(r.P1.Y, r.P2.Y)
			if sinH > 0 { // South
				if maxY >= center.Y+40.0 && minY <= center.Y+15.0 {
					return true
				}
			} else { // North
				if minY <= center.Y-40.0 && maxY >= center.Y-15.0 {
					return true
				}
			}
		}
	}
	return false
}

func (tm *TrafficManager) handleCarNavigation(car *TrafficCar, laneWidth float64, w *World) {
	if car.InRoundabout {
		return // Do not apply straight lane Lerp while circulating in roundabout!
	}

	minX, maxX := -2600.0, 2600.0
	minY, maxY := -1200.0, 1200.0
	if w != nil && len(w.Roads) > 0 {
		minX, maxX = 1e9, -1e9
		minY, maxY = 1e9, -1e9
		for _, r := range w.Roads {
			if r.P1.X < minX {
				minX = r.P1.X
			}
			if r.P2.X < minX {
				minX = r.P2.X
			}
			if r.P1.X > maxX {
				maxX = r.P1.X
			}
			if r.P2.X > maxX {
				maxX = r.P2.X
			}
			if r.P1.Y < minY {
				minY = r.P1.Y
			}
			if r.P2.Y < minY {
				minY = r.P2.Y
			}
			if r.P1.Y > maxY {
				maxY = r.P1.Y
			}
			if r.P2.Y > maxY {
				maxY = r.P2.Y
			}
		}
	}

	// Boundary loop wrapping strictly along road limits
	if car.IsEW {
		if car.Pos.X > maxX+30.0 {
			car.Pos.X = minX - 10.0
		} else if car.Pos.X < minX-30.0 {
			car.Pos.X = maxX + 10.0
		}
	} else {
		if car.Pos.Y > maxY+30.0 {
			car.Pos.Y = minY - 10.0
		} else if car.Pos.Y < minY-30.0 {
			car.Pos.Y = maxY + 10.0
		}
	}

	is4Lane := false
	if w != nil {
		is4Lane = w.IsRoad4Lane(car.RoadCenter, car.IsEW, car.Pos)
	} else {
		if car.IsEW {
			if car.RoadCenter == -1200 || car.RoadCenter == 0 || car.RoadCenter == 1200 {
				is4Lane = true
			}
		} else {
			if car.RoadCenter == 0 {
				is4Lane = true
			}
		}
	}

	laneOffset := laneWidth * 0.5 // Standard 2-lane road: center of travel lane
	if is4Lane {
		if car.LaneIndex == 0 {
			laneOffset = laneWidth * 0.5 // Inner lane (dedicated left turn)
		} else {
			laneOffset = laneWidth * 1.5 // Outer lane (straight through)
		}
	} else {
		car.LaneIndex = 0 // 2-lane road has single lane in each direction
	}

	if car.IsEW {
		targetY := car.RoadCenter
		if math.Cos(car.Heading) > 0 {
			targetY += laneOffset // Eastbound (+Y)
		} else {
			targetY -= laneOffset // Westbound (-Y)
		}
		car.Pos.Y = Lerp(car.Pos.Y, targetY, 0.08)
	} else {
		targetX := car.RoadCenter
		if math.Sin(car.Heading) > 0 {
			targetX -= laneOffset // Southbound (-X)
		} else {
			targetX += laneOffset // Northbound (+X)
		}
		car.Pos.X = Lerp(car.Pos.X, targetX, 0.08)
	}
}

// Draw renders all active AI cars
func (tm *TrafficManager) Draw(screen *ebiten.Image, cam *Camera, densityLevel int) {
	activeCount := 24
	if densityLevel == 1 {
		activeCount = 12
	} else if densityLevel == 3 {
		activeCount = 36
	}
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]
		drawSingleCar(screen, cam, car)
	}
}

func drawSingleCar(screen *ebiten.Image, cam *Camera, c *TrafficCar) {
	cosH := math.Cos(c.Heading)
	sinH := math.Sin(c.Heading)

	// Car shadow
	shadowOffset := Vec2{X: 4, Y: 5}
	drawRotatedBox(screen, cam, c.Pos.Add(shadowOffset), cosH, sinH, c.Length, c.Width, RGBA(2, 6, 23, 90))

	// Main Car Body
	drawRotatedBox(screen, cam, c.Pos, cosH, sinH, c.Length, c.Width, c.Color)

	// Front & Rear Windows
	wsPos := c.Pos.Add(Vec2{X: cosH * (c.Length * 0.18), Y: sinH * (c.Length * 0.18)})
	drawRotatedBox(screen, cam, wsPos, cosH, sinH, 9, c.Width-6, RGBA(30, 41, 59, 230))

	rwPos := c.Pos.Add(Vec2{X: -cosH * (c.Length * 0.22), Y: -sinH * (c.Length * 0.22)})
	drawRotatedBox(screen, cam, rwPos, cosH, sinH, 7, c.Width-6, RGBA(30, 41, 59, 230))

	roofPos := c.Pos.Add(Vec2{X: -cosH * 2, Y: -sinH * 2})
	drawRotatedBox(screen, cam, roofPos, cosH, sinH, c.Length*0.35, c.Width-6, c.Color)

	if c.Type == CarTaxi {
		drawRotatedBox(screen, cam, roofPos, cosH, sinH, 12, 6, RGBA(254, 240, 138, 255))
	}

	// Headlights (Front)
	perpX := -sinH
	perpY := cosH
	hlDist := c.Length / 2
	hlOffset := c.Width/2 - 3
	hl1 := c.Pos.Add(Vec2{X: cosH*hlDist - perpX*hlOffset, Y: sinH*hlDist - perpY*hlOffset})
	hl2 := c.Pos.Add(Vec2{X: cosH*hlDist + perpX*hlOffset, Y: sinH*hlDist + perpY*hlOffset})
	drawRotatedBox(screen, cam, hl1, cosH, sinH, 3, 4, RGBA(254, 240, 138, 255))
	drawRotatedBox(screen, cam, hl2, cosH, sinH, 3, 4, RGBA(254, 240, 138, 255))

	// Soft Headlight beam forward
	b1 := cam.WorldToScreen(hl1)
	b2 := cam.WorldToScreen(hl1.Add(Vec2{X: cosH * 40, Y: sinH * 40}))
	vector.StrokeLine(screen, float32(b1.X), float32(b1.Y), float32(b2.X), float32(b2.Y),
		float32(2*cam.Zoom), RGBA(254, 240, 138, 50), true)

	// Taillights / Brake lights (Rear)
	tl1 := c.Pos.Add(Vec2{X: -cosH*hlDist - perpX*hlOffset, Y: -sinH*hlDist - perpY*hlOffset})
	tl2 := c.Pos.Add(Vec2{X: -cosH*hlDist + perpX*hlOffset, Y: -sinH*hlDist + perpY*hlOffset})
	tlColor := RGBA(185, 28, 28, 255)
	if c.IsBraking {
		tlColor = RGBA(239, 68, 68, 255)
	}
	drawRotatedBox(screen, cam, tl1, cosH, sinH, 3, 4, tlColor)
	drawRotatedBox(screen, cam, tl2, cosH, sinH, 3, 4, tlColor)
}

// IsBusFacingWrongWay checks whether the player bus is driving or facing the wrong way (逆行) on a roadway.
// A bus in normal traffic lanes (顺行) or within an intersection is NOT wrong-way.
func IsBusFacingWrongWay(bus *Bus) bool {
	if bus == nil {
		return false
	}

	ys := []float64{-1200, -600, 0, 600, 1200}
	xs := []float64{-2600, -1300, 0, 1300, 2600}

	// 1. If bus is inside or close to an intersection, it is turning or navigating, not wrong-way
	for _, x := range xs {
		for _, y := range ys {
			if math.Hypot(bus.Pos.X-x, bus.Pos.Y-y) < 120.0 {
				return false
			}
		}
	}

	effHeading := bus.Heading
	if bus.Speed < -1.0 {
		effHeading = NormalizeAngle(bus.Heading + math.Pi)
	}

	// Find closest East-West road
	minDistEW := math.MaxFloat64
	closestY := 0.0
	for _, y := range ys {
		d := math.Abs(bus.Pos.Y - y)
		if d < minDistEW {
			minDistEW = d
			closestY = y
		}
	}

	// Find closest North-South road
	minDistNS := math.MaxFloat64
	closestX := 0.0
	for _, x := range xs {
		d := math.Abs(bus.Pos.X - x)
		if d < minDistNS {
			minDistNS = d
			closestX = x
		}
	}

	if minDistEW <= minDistNS && minDistEW < 160.0 {
		// East-West road:
		// South side (Y > closestY + 16): legal flow is East (heading ~ 0, cos > 0)
		// North side (Y < closestY - 16): legal flow is West (heading ~ pi, cos < 0)
		if bus.Pos.Y > closestY+16.0 {
			if math.Cos(effHeading) < -0.35 {
				return true
			}
		} else if bus.Pos.Y < closestY-16.0 {
			if math.Cos(effHeading) > 0.35 {
				return true
			}
		}
	} else if minDistNS < 160.0 {
		// North-South road:
		// West side (X < closestX - 16): legal flow is South (heading ~ pi/2, sin > 0)
		// East side (X > closestX + 16): legal flow is North (heading ~ -pi/2, sin < 0)
		if bus.Pos.X < closestX-16.0 {
			if math.Sin(effHeading) < -0.35 {
				return true
			}
		} else if bus.Pos.X > closestX+16.0 {
			if math.Sin(effHeading) > 0.35 {
				return true
			}
		}
	}

	return false
}
