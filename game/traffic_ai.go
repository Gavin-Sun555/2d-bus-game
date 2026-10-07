package game

import (
	"image/color"
	"math"
	"math/rand"

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
}

type TrafficManager struct {
	Cars []*TrafficCar
}

func NewTrafficManager(initialLaneWidth float64) *TrafficManager {
	tm := &TrafficManager{
		Cars: make([]*TrafficCar, 0),
	}
	tm.initializeCars(36, initialLaneWidth)
	return tm
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
		// North Ring (Y = -1200) Eastbound & Westbound (Intersections at X = -1800, -900, 0, 900, 1800)
		{pos: Vec2{X: -2300, Y: -1200 + laneWidth*0.8}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: -1350, Y: -1200 + laneWidth*0.8}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: -450, Y: -1200 + laneWidth*0.8}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 450, Y: -1200 + laneWidth*0.8}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 1350, Y: -1200 + laneWidth*0.8}, h: 0, rc: -1200, ew: true},
		{pos: Vec2{X: 2300, Y: -1200 - laneWidth*0.8}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: 1350, Y: -1200 - laneWidth*0.8}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: 450, Y: -1200 - laneWidth*0.8}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: -450, Y: -1200 - laneWidth*0.8}, h: math.Pi, rc: -1200, ew: true},
		{pos: Vec2{X: -1350, Y: -1200 - laneWidth*0.8}, h: math.Pi, rc: -1200, ew: true},

		// Central Boulevard (Y = 0) Eastbound & Westbound
		// Central Terminal is at X = -350 Eastbound. Cars spawn well away from it.
		{pos: Vec2{X: -2300, Y: 0 + laneWidth*0.8}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: -1350, Y: 0 + laneWidth*0.8}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: -700, Y: 0 + laneWidth*0.8}, h: 0, rc: 0, ew: true}, // 350px before Central Terminal
		{pos: Vec2{X: 450, Y: 0 + laneWidth*0.8}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: 1350, Y: 0 + laneWidth*0.8}, h: 0, rc: 0, ew: true},
		{pos: Vec2{X: 2300, Y: 0 - laneWidth*0.8}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: 1350, Y: 0 - laneWidth*0.8}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: 450, Y: 0 - laneWidth*0.8}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: -550, Y: 0 - laneWidth*0.8}, h: math.Pi, rc: 0, ew: true},
		{pos: Vec2{X: -1350, Y: 0 - laneWidth*0.8}, h: math.Pi, rc: 0, ew: true},

		// South Ring (Y = 1200) Eastbound & Westbound
		{pos: Vec2{X: -2300, Y: 1200 + laneWidth*0.8}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: -1350, Y: 1200 + laneWidth*0.8}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: -450, Y: 1200 + laneWidth*0.8}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 450, Y: 1200 + laneWidth*0.8}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 1350, Y: 1200 + laneWidth*0.8}, h: 0, rc: 1200, ew: true},
		{pos: Vec2{X: 2300, Y: 1200 - laneWidth*0.8}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: 1350, Y: 1200 - laneWidth*0.8}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: 450, Y: 1200 - laneWidth*0.8}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: -450, Y: 1200 - laneWidth*0.8}, h: math.Pi, rc: 1200, ew: true},
		{pos: Vec2{X: -1350, Y: 1200 - laneWidth*0.8}, h: math.Pi, rc: 1200, ew: true},

		// North-South: Far-West Blvd (X = -1800) (Intersections at Y = -1200, -600, 0, 600, 1200)
		{pos: Vec2{X: -1800 - laneWidth*0.6, Y: -900}, h: math.Pi / 2, rc: -1800, ew: false},
		{pos: Vec2{X: -1800 - laneWidth*0.6, Y: 300}, h: math.Pi / 2, rc: -1800, ew: false},
		{pos: Vec2{X: -1800 + laneWidth*0.6, Y: 900}, h: -math.Pi / 2, rc: -1800, ew: false},
		{pos: Vec2{X: -1800 + laneWidth*0.6, Y: -300}, h: -math.Pi / 2, rc: -1800, ew: false},

		// North-South: Central Axis (X = 0)
		{pos: Vec2{X: 0 - laneWidth*0.8, Y: -900}, h: math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 - laneWidth*0.8, Y: 300}, h: math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 + laneWidth*0.8, Y: 900}, h: -math.Pi / 2, rc: 0, ew: false},
		{pos: Vec2{X: 0 + laneWidth*0.8, Y: -300}, h: -math.Pi / 2, rc: 0, ew: false},

		// North-South: Far-East Blvd (X = 1800)
		{pos: Vec2{X: 1800 - laneWidth*0.6, Y: -900}, h: math.Pi / 2, rc: 1800, ew: false},
		{pos: Vec2{X: 1800 - laneWidth*0.6, Y: 300}, h: math.Pi / 2, rc: 1800, ew: false},
		{pos: Vec2{X: 1800 + laneWidth*0.6, Y: 900}, h: -math.Pi / 2, rc: 1800, ew: false},
		{pos: Vec2{X: 1800 + laneWidth*0.6, Y: -300}, h: -math.Pi / 2, rc: 1800, ew: false},
	}

	// 25 City Intersections coordinates for programmatic validation
	interCenters := []Vec2{
		{X: -1800, Y: -1200}, {X: -900, Y: -1200}, {X: 0, Y: -1200}, {X: 900, Y: -1200}, {X: 1800, Y: -1200},
		{X: -1800, Y: -600}, {X: -900, Y: -600}, {X: 0, Y: -600}, {X: 900, Y: -600}, {X: 1800, Y: -600},
		{X: -1800, Y: 0}, {X: -900, Y: 0}, {X: 0, Y: 0}, {X: 900, Y: 0}, {X: 1800, Y: 0},
		{X: -1800, Y: 600}, {X: -900, Y: 600}, {X: 0, Y: 600}, {X: 900, Y: 600}, {X: 1800, Y: 600},
		{X: -1800, Y: 1200}, {X: -900, Y: 1200}, {X: 0, Y: 1200}, {X: 900, Y: 1200}, {X: 1800, Y: 1200},
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
			Speed:       130 + rand.Float64()*30,
			TargetSpeed: 150 + rand.Float64()*40,
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
func (tm *TrafficManager) Update(dt float64, bus *Bus, tls *TrafficLightSystem, densityLevel int, laneWidth float64) {
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
		tm.updateSingleCar(dt, car, i, bus, tls, activeCount, laneWidth)
	}
}

func (tm *TrafficManager) updateSingleCar(dt float64, car *TrafficCar, index int, bus *Bus, tls *TrafficLightSystem, activeCount int, laneWidth float64) {
	// 1. Perception & Sensors
	targetSpd := car.TargetSpeed
	car.IsBraking = false

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

	// B. Check Player Bus ahead
	distToBus := car.Pos.Distance(bus.Pos)
	if distToBus < 140 {
		toBus := bus.Pos.Sub(car.Pos)
		cosH := math.Cos(car.Heading)
		sinH := math.Sin(car.Heading)
		forwardProj := toBus.X*cosH + toBus.Y*sinH
		lateralProj := math.Abs(-toBus.X*sinH + toBus.Y*cosH)

		// If bus is in front of this car and in roughly same lane
		if forwardProj > 0 && forwardProj < 130 && lateralProj < laneWidth*0.75 {
			car.IsBraking = true
			if forwardProj < 70 {
				targetSpd = 0
			} else {
				targetSpd = math.Min(targetSpd, math.Max(0, (forwardProj-60)*1.8))
			}
		}
	}

	// C. Check Other AI Cars ahead in same lane
	for j := 0; j < activeCount; j++ {
		if j == index {
			continue
		}
		other := tm.Cars[j]
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

	// 2. Acceleration / Braking Physics
	if car.Speed < targetSpd {
		car.Speed = math.Min(targetSpd, car.Speed+120*dt)
	} else if car.Speed > targetSpd {
		car.Speed = math.Max(targetSpd, car.Speed-280*dt)
	}

	// 3. Move along Heading
	if car.Speed > 0 {
		car.Pos.X += car.Speed * math.Cos(car.Heading) * dt
		car.Pos.Y += car.Speed * math.Sin(car.Heading) * dt
	}

	// 3.5 AI Intersection Turning: Cars in the dedicated left-turn lane (Lane 0) execute their left turn!
	if car.LaneIndex == 0 && tls != nil {
		for _, inter := range tls.Intersections {
			if inter.IsMajor && car.Pos.Distance(inter.Center) < 28.0 {
				if car.IsEW {
					car.IsEW = false
					car.RoadCenter = inter.Center.X
					if math.Cos(car.Heading) > 0 {
						// Eastbound -> turn North (-Pi/2)
						car.Heading = -math.Pi / 2
						car.Pos.X = inter.Center.X + laneWidth*1.48
					} else {
						// Westbound -> turn South (Pi/2)
						car.Heading = math.Pi / 2
						car.Pos.X = inter.Center.X - laneWidth*1.48
					}
				} else {
					car.IsEW = true
					car.RoadCenter = inter.Center.Y
					if math.Sin(car.Heading) > 0 {
						// Southbound -> turn East (0)
						car.Heading = 0
						car.Pos.Y = inter.Center.Y + laneWidth*1.48
					} else {
						// Northbound -> turn West (Pi)
						car.Heading = math.Pi
						car.Pos.Y = inter.Center.Y - laneWidth*1.48
					}
				}
				car.LaneIndex = 1 // joins through lane on the new avenue
				break
			}
		}
	}

	// 4. Map Boundary Wrapping & Lane Tracking with customizable lane width
	tm.handleCarNavigation(car, laneWidth)
}

func (tm *TrafficManager) handleCarNavigation(car *TrafficCar, laneWidth float64) {
	// Expanded Boundary loop wrapping
	if car.Pos.X > 2850 {
		car.Pos.X = -2850
	} else if car.Pos.X < -2850 {
		car.Pos.X = 2850
	}

	if car.Pos.Y > 1550 {
		car.Pos.Y = -1550
	} else if car.Pos.Y < -1550 {
		car.Pos.Y = 1550
	}

	is4Lane := false
	if car.IsEW {
		if car.RoadCenter == -1200 || car.RoadCenter == 0 || car.RoadCenter == 1200 {
			is4Lane = true
		}
	} else {
		if car.RoadCenter == 0 {
			is4Lane = true
		}
	}

	// Keep cars dynamically centered on their lane based on customizable lane width
	if car.IsEW {
		targetY := car.RoadCenter
		laneOffset := laneWidth * 0.75
		if is4Lane {
			if car.LaneIndex == 0 {
				laneOffset = laneWidth * 0.52 // Inner lane (dedicated left turn)
			} else {
				laneOffset = laneWidth * 1.48 // Outer lane (straight through)
			}
		}
		if math.Cos(car.Heading) > 0 {
			targetY += laneOffset // Eastbound (+Y)
		} else {
			targetY -= laneOffset // Westbound (-Y)
		}
		car.Pos.Y = Lerp(car.Pos.Y, targetY, 0.06)
	} else {
		targetX := car.RoadCenter
		laneOffset := laneWidth * 0.75
		if is4Lane {
			if car.LaneIndex == 0 {
				laneOffset = laneWidth * 0.52 // Inner lane
			} else {
				laneOffset = laneWidth * 1.48 // Outer lane
			}
		}
		if math.Sin(car.Heading) > 0 {
			targetX -= laneOffset // Southbound (-X)
		} else {
			targetX += laneOffset // Northbound (+X)
		}
		car.Pos.X = Lerp(car.Pos.X, targetX, 0.06)
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
