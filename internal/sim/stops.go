package sim

import (
	"fmt"
	"image/color"
	"math"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
)

// DefaultStopMaxWaiting is a high guardrail, independent of passenger arrival rates.
const DefaultStopMaxWaiting = 2048

// BusStop represents a bus stop station in the city
type BusStop struct {
	ID           int
	Name         string
	CustomNameZH string  `json:"-"`
	CustomNameEN string  `json:"-"`
	RoadCenter   Vec2    // Center of the adjacent road
	Heading      float64 // Direction of the road beside it
	CenterBRT    bool    `json:",omitempty"`
	RoadLanes    int     // Number of lanes on the adjacent road (2 or 4)
	Pos          Vec2    // Center of stop bay (computed from RoadCenter and laneWidth)
	Length       float64 // Length of stop bay along heading
	Width        float64 // Width of stop bay

	WaitingPassengers   int
	WaitingDestinations []int `json:",omitempty"`
	MaxWaiting          int
	SpawnTimer          float64
	SpawnInterval       float64

	// Shelter pos
	ShelterPos Vec2

	// Station Color accent
	AccentColor color.RGBA

	cachedInfoText   string          `json:"-"`
	cachedInfoPax    int             `json:"-"`
	cachedInfoTarget bool            `json:"-"`
	cachedInfoPlan   bool            `json:"-"`
	cachedInfoZoom   bool            `json:"-"`
	cachedInfoLang   locale.Language `json:"-"`
}

// FormatInfoText returns a cached station label string, avoiding re-allocations on every frame.
func (s *BusStop) FormatInfoText(isTarget, isHovered, planMode bool, zoom float64) string {
	zoomSmall := planMode && zoom < 0.2 && !isTarget && !isHovered
	if s.cachedInfoText != "" &&
		s.cachedInfoPax == s.WaitingPassengers &&
		s.cachedInfoTarget == isTarget &&
		s.cachedInfoPlan == planMode &&
		s.cachedInfoZoom == zoomSmall &&
		s.cachedInfoLang == locale.CurrentLanguage {
		return s.cachedInfoText
	}

	infoText := fmt.Sprintf("#%d %s (%d人)", s.ID+1, s.LocalizedName(), s.WaitingPassengers)
	if isTarget {
		infoText = "★ [目标站] " + infoText
		if s.WaitingPassengers == 0 {
			infoText += " [空站无候车]"
		}
	}
	if zoomSmall {
		infoText = fmt.Sprintf("#%d %s", s.ID+1, s.CleanName())
	}

	s.cachedInfoText = infoText
	s.cachedInfoPax = s.WaitingPassengers
	s.cachedInfoTarget = isTarget
	s.cachedInfoPlan = planMode
	s.cachedInfoZoom = zoomSmall
	s.cachedInfoLang = locale.CurrentLanguage
	return infoText
}

// LocalizedName returns the station name localized according to CurrentLanguage
func (s *BusStop) LocalizedName() string {
	if locale.CurrentLanguage == locale.LanguageEN && s.CustomNameEN != "" {
		return s.CustomNameEN
	}
	if locale.CurrentLanguage != locale.LanguageEN && s.CustomNameZH != "" {
		return s.CustomNameZH
	}
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
	if locale.CurrentLanguage == locale.LanguageEN && s.CustomNameEN != "" {
		return s.CustomNameEN
	}
	if locale.CurrentLanguage != locale.LanguageEN && s.CustomNameZH != "" {
		return s.CustomNameZH
	}
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
		MaxWaiting:        DefaultStopMaxWaiting,
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
	if s.CenterBRT {
		s.Pos = s.RoadCenter.Add(Vec2{X: perpX * laneWidth * .5, Y: perpY * laneWidth * .5})
		s.ShelterPos = s.RoadCenter.Add(Vec2{X: cosH * 80, Y: sinH * 80})
	}
}

// SyncWaitingQueue ensures WaitingDestinations has length equal to WaitingPassengers.
// Missing destinations are chosen from fallbackCandidates (or -1 if empty).
func (s *BusStop) SyncWaitingQueue(fallbackCandidates []int) {
	if s.WaitingPassengers <= 0 {
		s.WaitingPassengers = 0
		s.WaitingDestinations = s.WaitingDestinations[:0]
		return
	}
	if len(s.WaitingDestinations) > s.WaitingPassengers {
		s.WaitingDestinations = s.WaitingDestinations[:s.WaitingPassengers]
		return
	}
	for len(s.WaitingDestinations) < s.WaitingPassengers {
		dest := -1
		if len(fallbackCandidates) > 0 {
			dest = fallbackCandidates[RandomIntn(len(fallbackCandidates))]
		}
		s.WaitingDestinations = append(s.WaitingDestinations, dest)
	}
	s.WaitingPassengers = len(s.WaitingDestinations)
}

// CountMatchingWaiting returns how many waiting passengers have their target destination in candidates.
// If candidates is empty (e.g. single-stop fallback or legacy tests), all waiting passengers match.
func (s *BusStop) CountMatchingWaiting(candidates []int) int {
	s.SyncWaitingQueue(candidates)
	if len(candidates) == 0 {
		return len(s.WaitingDestinations)
	}
	candMap := make(map[int]bool, len(candidates))
	for _, c := range candidates {
		candMap[c] = true
	}
	count := 0
	for _, dest := range s.WaitingDestinations {
		if dest == -1 || candMap[dest] {
			count++
		}
	}
	return count
}

// BoardMatchingPassengers extracts up to maxCount passengers whose destination is in candidates.
// Returns a slice of the boarded passengers' target destination IDs.
func (s *BusStop) BoardMatchingPassengers(maxCount int, candidates []int) []int {
	s.SyncWaitingQueue(candidates)
	if maxCount <= 0 || len(s.WaitingDestinations) == 0 {
		return nil
	}
	if len(candidates) == 0 {
		take := min(maxCount, len(s.WaitingDestinations))
		boarded := make([]int, take)
		copy(boarded, s.WaitingDestinations[:take])
		s.WaitingDestinations = s.WaitingDestinations[take:]
		s.WaitingPassengers = len(s.WaitingDestinations)
		return boarded
	}

	candMap := make(map[int]bool, len(candidates))
	for _, c := range candidates {
		candMap[c] = true
	}

	boarded := make([]int, 0, maxCount)
	remaining := make([]int, 0, len(s.WaitingDestinations))
	for _, dest := range s.WaitingDestinations {
		if len(boarded) < maxCount && (dest == -1 || candMap[dest]) {
			targetDest := dest
			if targetDest == -1 && len(candidates) > 0 {
				targetDest = candidates[RandomIntn(len(candidates))]
			}
			boarded = append(boarded, targetDest)
		} else {
			remaining = append(remaining, dest)
		}
	}
	s.WaitingDestinations = remaining
	s.WaitingPassengers = len(s.WaitingDestinations)
	return boarded
}

// Update advances the spawn timer with default 1.0x rate and nil destinations.
func (s *BusStop) Update(dt float64) {
	s.UpdateWithDestinations(dt, 1.0, nil)
}

// UpdateWithDestinations advances passenger arrivals according to rateMultiplier,
// assigning new passengers randomly to candidateDestinations.
func (s *BusStop) UpdateWithDestinations(dt float64, rateMultiplier float64, candidateDestinations []int) {
	if rateMultiplier <= 0 {
		rateMultiplier = 1.0
	}
	s.SyncWaitingQueue(candidateDestinations)

	// Reconcile any existing destinations that are no longer reachable by any passing line
	if len(candidateDestinations) > 0 {
		candSet := make(map[int]bool, len(candidateDestinations))
		for _, c := range candidateDestinations {
			candSet[c] = true
		}
		for i, dest := range s.WaitingDestinations {
			if dest != -1 && !candSet[dest] {
				s.WaitingDestinations[i] = candidateDestinations[RandomIntn(len(candidateDestinations))]
			}
		}
	}

	s.SpawnTimer += dt * rateMultiplier
	if s.SpawnTimer >= s.SpawnInterval {
		s.SpawnTimer = 0
		if len(s.WaitingDestinations) < s.MaxWaiting {
			toSpawn := 1 + RandomIntn(3)
			if len(s.WaitingDestinations)+toSpawn > s.MaxWaiting {
				toSpawn = s.MaxWaiting - len(s.WaitingDestinations)
			}
			for i := 0; i < toSpawn; i++ {
				dest := -1
				if len(candidateDestinations) > 0 {
					dest = candidateDestinations[RandomIntn(len(candidateDestinations))]
				}
				s.WaitingDestinations = append(s.WaitingDestinations, dest)
			}
			s.WaitingPassengers = len(s.WaitingDestinations)
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

	// Rear vehicles may overhang a standard platform; their powered section
	// still has to reach the boarding area.
	forward := Vec2{X: cosH, Y: sinH}
	if !s.containsBoardingLength(bus, forward) {
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
	if s.CenterBRT {
		return s.ContainsBus(bus)
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

func (s *BusStop) containsBoardingLength(bus *Bus, forward Vec2) bool {
	if s.containsBusLength(bus, forward) {
		return true
	}
	if !s.HasTwoBerths() {
		return false
	}
	along := bus.Pos.Sub(s.Pos).Dot(forward)
	if along < -s.Length/2-2 {
		return false
	}
	for _, part := range bus.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - s.Heading
		extent := math.Abs(math.Cos(yaw))*part.HalfLen + math.Abs(math.Sin(yaw))*part.HalfWid
		if part.Center.Sub(s.Pos).Dot(forward)+extent > s.Length/2+2 {
			return false
		}
	}
	return true
}
