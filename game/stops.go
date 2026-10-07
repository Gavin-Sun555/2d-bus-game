package game

import (
	"fmt"
	"image/color"
	"math"
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// BusStop represents a bus stop station in the city
type BusStop struct {
	ID         int
	Name       string
	RoadCenter Vec2    // Center of the adjacent road
	Heading    float64 // Direction of the road beside it
	RoadLanes  int     // Number of lanes on the adjacent road (2 or 4)
	Pos        Vec2    // Center of stop bay (computed from RoadCenter and laneWidth)
	Length     float64 // Length of stop bay along heading
	Width      float64 // Width of stop bay

	WaitingPassengers int
	MaxWaiting        int
	SpawnTimer        float64
	SpawnInterval     float64

	// Shelter pos
	ShelterPos Vec2

	// Station Color accent
	AccentColor color.RGBA
}

func NewBusStop(id int, name string, roadCenter Vec2, heading float64, roadLanes int, accent color.RGBA, laneWidth float64, initialWaiting ...int) *BusStop {
	waitPax := 6 + rand.Intn(10)
	if len(initialWaiting) > 0 {
		waitPax = initialWaiting[0]
	}
	s := &BusStop{
		ID:                id,
		Name:              name,
		RoadCenter:        roadCenter,
		Heading:           heading,
		RoadLanes:         roadLanes,
		Length:            140,
		Width:             44,
		WaitingPassengers: waitPax,
		MaxWaiting:        35,
		SpawnInterval:     20.0 + rand.Float64()*15.0, // 20-35s passenger arrival rate
		AccentColor:       accent,
	}
	s.UpdateGeometry(laneWidth)
	return s
}

// UpdateGeometry repositions the bus bay and shelter to match customizable road/lane width
func (s *BusStop) UpdateGeometry(laneWidth float64) {
	sinH := math.Sin(s.Heading)
	cosH := math.Cos(s.Heading)
	perpX := -sinH
	perpY := cosH

	// Offset from road center to outer curb: (RoadLanes/2)*laneWidth + half bay width
	curbDist := (float64(s.RoadLanes)/2.0)*laneWidth + s.Width*0.48
	s.Pos = s.RoadCenter.Add(Vec2{X: perpX * curbDist, Y: perpY * curbDist})
	s.ShelterPos = s.Pos.Add(Vec2{X: perpX * 36, Y: perpY * 36})
}

func (s *BusStop) Update(dt float64) {
	s.SpawnTimer += dt
	if s.SpawnTimer >= s.SpawnInterval {
		s.SpawnTimer = 0
		if s.WaitingPassengers < s.MaxWaiting {
			s.WaitingPassengers += 1 + rand.Intn(3)
			if s.WaitingPassengers > s.MaxWaiting {
				s.WaitingPassengers = s.MaxWaiting
			}
		}
	}
}

// ContainsBus checks if a bus is parked accurately inside the designated bus stop bay box,
// strictly requiring the bus to have pulled off the main travel road and into the yellow box.
func (s *BusStop) ContainsBus(bus *Bus) bool {
	cosH := math.Cos(s.Heading)
	sinH := math.Sin(s.Heading)

	// Vector from bay box center to bus center
	dx := bus.Pos.X - s.Pos.X
	dy := bus.Pos.Y - s.Pos.Y

	// Longitudinal offset along road/bay heading (length of bay is 140, bus is 96)
	longitudinal := dx*cosH + dy*sinH
	if math.Abs(longitudinal) > (s.Length/2 - 10.0) { // Within ±60 px along bay length
		return false
	}

	// Lateral offset perpendicular to road/bay heading (width of bay is 44, bus width is 36)
	// Positive lateral is towards the curb/shelter, negative lateral is towards the road.
	// Bus must be squarely inside the yellow bay box, NOT out on the road travel lanes!
	lateral := -dx*sinH + dy*cosH
	if math.Abs(lateral) > (s.Width*0.5 - 4.0) { // Within ±18 px (bay box half-width is 22)
		return false
	}

	// Check heading alignment: must be parked parallel to the curb (within 30 degrees)
	angleDiff := math.Abs(NormalizeAngle(bus.Heading - s.Heading))
	if angleDiff > math.Pi/6 {
		return false
	}

	return true
}

// Draw renders the bus stop, waiting shelter, and passenger dots
func (s *BusStop) Draw(screen *ebiten.Image, cam *Camera, isTarget bool, isHovered bool) {
	cosH := math.Cos(s.Heading)
	sinH := math.Sin(s.Heading)

	// 1. Bus Stop Bay (Yellow bordered bay marked on the road)
	bayBorderColor := RGBA(234, 179, 8, 220) // Vibrant yellow
	if isTarget {
		bayBorderColor = RGBA(59, 130, 246, 255) // Neon blue for next stop target
	}
	drawRotatedBox(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, RGBA(24, 24, 27, 240))
	drawRotatedBoxOutline(screen, cam, s.Pos, cosH, sinH, s.Length, s.Width, 3, bayBorderColor)

	// Diagonal yellow hazard stripes inside the bay
	numStripes := 6
	step := s.Length / float64(numStripes)
	for i := -numStripes / 2; i <= numStripes / 2; i++ {
		offset := float64(i) * step
		p1 := s.Pos.Add(Vec2{X: cosH*offset - sinH*(s.Width/2 - 4), Y: sinH*offset + cosH*(s.Width/2 - 4)})
		p2 := s.Pos.Add(Vec2{X: cosH*(offset+20) + sinH*(s.Width/2 - 4), Y: sinH*(offset+20) - cosH*(s.Width/2 - 4)})
		s1 := cam.WorldToScreen(p1)
		s2 := cam.WorldToScreen(p2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(2.5*cam.Zoom), RGBA(234, 179, 8, 140), true)
	}

	// 2. Glass Bus Shelter on Sidewalk
	shelterLen := 68.0
	shelterWid := 18.0
	drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen, shelterWid, RGBA(30, 41, 59, 240))
	// Shelter roof accent bar
	drawRotatedBox(screen, cam, s.ShelterPos, cosH, sinH, shelterLen-4, 4, s.AccentColor)

	// 3. Waiting Passengers visualization (dots lined up near shelter)
	shelterScreen := cam.WorldToScreen(s.ShelterPos)
	perpX := -sinH
	perpY := cosH
	for i := 0; i < s.WaitingPassengers && i < 24; i++ {
		row := float64(i / 12)
		col := float64(i % 12)
		pWorld := s.ShelterPos.Add(Vec2{
			X: cosH*(col*6 - 32) + perpX*(row*6 + 12),
			Y: sinH*(col*6 - 32) + perpY*(row*6 + 12),
		})
		ps := cam.WorldToScreen(pWorld)
		vector.DrawFilledCircle(screen, float32(ps.X), float32(ps.Y), float32(2.8*cam.Zoom),
			RGBA(251, 191, 36, 230), true)
	}

	// 4. Station Marker Pole / Flag
	sc := cam.WorldToScreen(s.Pos)
	badgeRadius := float32(14 * cam.Zoom)
	if badgeRadius < 8 {
		badgeRadius = 8
	}

	// Glowing circle badge
	badgeColor := s.AccentColor
	if isTarget {
		badgeColor = RGBA(59, 130, 246, 255)
	}
	vector.DrawFilledCircle(screen, float32(shelterScreen.X), float32(shelterScreen.Y-float64(badgeRadius*2.5)),
		badgeRadius, badgeColor, true)
	vector.StrokeCircle(screen, float32(shelterScreen.X), float32(shelterScreen.Y-float64(badgeRadius*2.5)),
		badgeRadius, float32(2*cam.Zoom), RGBA(255, 255, 255, 255), true)

	// Station info label
	infoText := fmt.Sprintf("#%d %s (%d人)", s.ID+1, s.Name, s.WaitingPassengers)
	if isTarget {
		infoText = "★ [目标站] " + infoText
		if s.WaitingPassengers == 0 {
			infoText += " [空站无候车]"
		}
	}
	DrawText(screen, infoText, shelterScreen.X-60, shelterScreen.Y-float64(badgeRadius*4), 12, color.White)

	_ = sc
}

func drawRotatedBoxOutline(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, wid, stroke float64, c color.RGBA) {
	perpX := -sinH
	perpY := cosH
	hl := len / 2
	hw := wid / 2

	c1 := center.Add(Vec2{X: cosH*hl - perpX*hw, Y: sinH*hl - perpY*hw})
	c2 := center.Add(Vec2{X: cosH*hl + perpX*hw, Y: sinH*hl + perpY*hw})
	c3 := center.Add(Vec2{X: -cosH*hl + perpX*hw, Y: -sinH*hl + perpY*hw})
	c4 := center.Add(Vec2{X: -cosH*hl - perpX*hw, Y: -sinH*hl - perpY*hw})

	s1 := cam.WorldToScreen(c1)
	s2 := cam.WorldToScreen(c2)
	s3 := cam.WorldToScreen(c3)
	s4 := cam.WorldToScreen(c4)

	sw := float32(stroke * cam.Zoom)
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), sw, c, true)
	vector.StrokeLine(screen, float32(s2.X), float32(s2.Y), float32(s3.X), float32(s3.Y), sw, c, true)
	vector.StrokeLine(screen, float32(s3.X), float32(s3.Y), float32(s4.X), float32(s4.Y), sw, c, true)
	vector.StrokeLine(screen, float32(s4.X), float32(s4.Y), float32(s1.X), float32(s1.Y), sw, c, true)
}
