package sim

import (
	"image/color"
	"math"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
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

// LocalizedName returns the station name localized according to CurrentLanguage
func (s *BusStop) LocalizedName() string {
	if locale.CurrentLanguage == locale.LanguageEN {
		if idx := strings.Index(s.Name, "("); idx != -1 {
			if end := strings.Index(s.Name, ")"); end > idx {
				return strings.TrimSpace(s.Name[idx+1 : end])
			}
		}
	}
	return s.Name
}

// CleanName returns the clean, concise station name without redundant bilingual parenthetical text
// (Chinese only in Chinese mode, English only in English mode)
func (s *BusStop) CleanName() string {
	if idx := strings.Index(s.Name, "("); idx != -1 {
		if locale.CurrentLanguage == locale.LanguageEN {
			if end := strings.Index(s.Name, ")"); end > idx {
				return strings.TrimSpace(s.Name[idx+1 : end])
			}
		} else {
			return strings.TrimSpace(s.Name[:idx])
		}
	}
	return s.Name
}

func NewBusStop(id int, name string, roadCenter Vec2, heading float64, roadLanes int, accent color.RGBA, laneWidth float64, initialWaiting ...int) *BusStop {
	waitPax := 6 + RandomIntn(10)
	if len(initialWaiting) > 0 {
		waitPax = initialWaiting[0]
	}
	s := &BusStop{
		ID:                id,
		Name:              name,
		RoadCenter:        roadCenter,
		Heading:           heading,
		RoadLanes:         roadLanes,
		Length:            StandardStopLength,
		Width:             44,
		WaitingPassengers: waitPax,
		MaxWaiting:        45,
		SpawnInterval:     20.0 + RandomFloat64()*15.0, // 20-35s passenger arrival rate
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
			s.WaitingPassengers += 1 + RandomIntn(3)
			if s.WaitingPassengers > s.MaxWaiting {
				s.WaitingPassengers = s.MaxWaiting
			}
		}
	}
}

// ContainsBus checks if a bus is parked accurately inside the designated bus stop bay box,
// strictly requiring the bus to have pulled off the main travel road and into the yellow box.
func (s *BusStop) ContainsBus(bus *Bus) bool {
	sinH, cosH := math.Sincos(s.Heading)

	// Vector from bay box center to bus center
	dx := bus.Pos.X - s.Pos.X
	dy := bus.Pos.Y - s.Pos.Y

	// Both sections must fit longitudinally in the boarding platform.
	forward := Vec2{X: cosH, Y: sinH}
	if !s.containsBusLength(bus, forward) {
		return false
	}

	// Lateral offset perpendicular to road/bay heading (width of bay is 44, bus width is 36)
	// Positive lateral is towards the curb/shelter, negative lateral is towards the road.
	// Bus must be squarely inside or along the yellow bay box, NOT out on the road travel lanes!
	lateral := -dx*sinH + dy*cosH
	if math.Abs(lateral) > (s.Width*0.5 + 4.0) { // Within ±26 px (bay box half-width is 22)
		return false
	}

	// Check heading alignment: must be parked parallel to the curb (within 36 degrees)
	angleDiff := math.Abs(NormalizeAngle(bus.Heading - s.Heading))
	if angleDiff > math.Pi*0.20 {
		return false
	}

	return true
}

// ContainsBusInOuterLane checks if a bus is positioned in an outer roadway travel lane directly alongside the stop bay.
// Exclusively used for player bus in Berserk mode, allowing rapid passenger boarding/alighting from outer travel lanes.
// On two-way two-lane roads (双向双车道), temporary parking in the opposite-direction travel lane is also permitted.
func (s *BusStop) ContainsBusInOuterLane(bus *Bus, laneWidth float64) bool {
	if bus == nil {
		return false
	}
	sinH, cosH := math.Sincos(s.Heading)

	// Vector from bay box center to bus center
	dx := bus.Pos.X - s.Pos.X
	dy := bus.Pos.Y - s.Pos.Y

	// Roadside boarding follows the same forward platform markers.
	longitudinal := dx*cosH + dy*sinH
	front := s.FrontBerthPosition().Sub(s.Pos).Dot(Vec2{X: cosH, Y: sinH})
	alignedBerth := math.Abs(longitudinal-front) <= 42
	if bus.IsArticulated() {
		// An outer long bus may serve anywhere its complete two-section body
		// fits alongside the platform, without reaching an occupied front bay.
		alignedBerth = s.containsBusLength(bus, Vec2{X: cosH, Y: sinH})
	}
	if s.HasTwoBerths() && !bus.IsArticulated() {
		alignedBerth = alignedBerth || math.Abs(longitudinal-(front-120)) <= 42
	}
	if !alignedBerth {
		return false
	}

	// Heading alignment:
	angleDiff := math.Abs(NormalizeAngle(bus.Heading - s.Heading))
	isAligned := angleDiff <= math.Pi*0.25
	if s.RoadLanes <= 2 {
		// On 2-lane roads (双向双车道), also allow temporary stopping facing opposite direction
		isReverse := math.Abs(angleDiff-math.Pi) <= math.Pi*0.25
		if !isAligned && !isReverse {
			return false
		}
	} else if !isAligned {
		return false
	}

	// Lateral check:
	toCenter := bus.Pos.Sub(s.RoadCenter)
	perpVec := Vec2{X: -sinH, Y: cosH}
	sideDist := toCenter.X*perpVec.X + toCenter.Y*perpVec.Y
	curbDist := s.Pos.Distance(s.RoadCenter)

	minSideDist := -15.0
	if s.RoadLanes <= 2 {
		// On 2-lane roads (双向双车道), allow temporary parking in the opposite-direction travel lane
		minSideDist = -curbDist - 25.0
	}
	maxSideDist := curbDist + s.Width*0.5 + 25.0

	if sideDist < minSideDist || sideDist > maxSideDist {
		return false
	}

	return true
}

func (s *BusStop) containsBusLength(bus *Bus, forward Vec2) bool {
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - s.Heading
		extent := math.Abs(math.Cos(yaw))*part.HalfLen + math.Abs(math.Sin(yaw))*part.HalfWid
		if math.Abs(part.Center.Sub(s.Pos).Dot(forward))+extent > s.Length/2+2 {
			return false
		}
	}
	return true
}
