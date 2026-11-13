package ui

import (
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

var berserkSpeedPresets = [...]int{80, 120, 160, 180}

// Settings manages user game configuration
type Settings struct {
	ClockRate                    sim.ClockRate
	clockRateBounds              [3]sim.Rect
	CareerMode                   bool `json:"-"`
	OpenCareerRequested          bool `json:"-"`
	IsOpen                       bool
	SaveRequested, LoadRequested bool
	saveBounds, loadBounds       sim.Rect
	Fare                         sim.FarePolicy
	OpenOperationsRequested      bool
	OpenNetworkRequested         bool
	operationsBounds             sim.Rect
	TrafficFinesEnabled          bool
	BusSignalPriority            bool
	busSignalPriorityBounds      sim.Rect
	FineAmount                   int
	TrafficDensity               int                   // 1: Low (144), 2: Medium (288), 3: High (576), 4: Congested (864)
	LaneWidthLevel               int                   // 0: 42px (标准紧凑), 1: 52px (宽敞舒适), 2: 64px (开阔大道), 3: 78px (超宽城际)
	CameraViewMode               sim.CameraMode        // 0: CameraModeNorthUp, 1: CameraModeHeadingUp
	MapTheme                     sim.MapTheme          // Active city map
	LargeRoadNetworks            map[sim.MapTheme]bool `json:",omitempty"` // Per-map road network mode (true = 4x large, false = 1x small)
	LargeRoadNetwork             bool                  `json:",omitempty"` // Active map's 4x mode (legacy & active sync)
	CenterBRTMaps                map[sim.MapTheme]bool `json:",omitempty"`
	CenterBRT                    bool                  `json:",omitempty"` // Active-map value; legacy saves used it only for Metropolis
	LargeRoadNetworkChanged      bool                  `json:"-"`
	ThemeChanged                 bool
	ReturnToMenuRequested        bool

	// AI Autopilot & Route AI Buses Configurations
	AutopilotEnabled         bool
	AutopilotBerserk         bool // Berserk mode: straightaways top speed boost while obeying traffic rules
	AutopilotBerserkMaxSpeed int  // Berserk top speed in km/h.
	AICarMinSpeed            int
	AICarMaxSpeed            int
	AIBusMinSpeed            int
	AIBusMaxSpeed            int
	speedRangeBounds         [2][2]sim.Rect // decrement/increment for car min/max
	AIBusCount               int            // Number of AI buses on the route
	AIBusFleetChanged        bool
	PlayerModel              sim.BusModel `json:",omitempty"` // Configured player bus model (standard, double-decker, articulated)

	// UI Localization Language
	Language                  locale.Language
	Announcements             AnnouncementMode
	AnnouncementVoice         sound.VoiceStyle
	PassengerCapacities       *[sim.BusModelCount]int `json:",omitempty"` // Legacy absolute capacities, migrated on load.
	PassengerOverloadPercents *[sim.BusModelCount]int `json:",omitempty"`
	passengerCapacityBounds   [sim.BusModelCount][2]sim.Rect
	PassengerRates            *[sim.BusModelCount]sim.PassengerRates `json:",omitempty"`
	PassengerTimingOpen       bool                                   `json:"-"`
	passengerTimingBounds     sim.Rect
	passengerTimingClose      sim.Rect
	passengerRateBounds       [sim.BusModelCount][2]sim.Rect
	AudioVolumes              *sound.Volumes `json:",omitempty"`
	volumeBounds              [3][2]sim.Rect
	announcementVoiceBounds   sim.Rect
	announcementsBounds       sim.Rect
	PassengerSpawnRate        sim.PassengerSpawnRate
	passengerSpawnRateBounds  sim.Rect

	// Button bounds for click detection
	fineToggleBounds    sim.Rect
	densityToggleBounds sim.Rect
	cameraToggleBounds  sim.Rect
	autopilotBounds     sim.Rect
	berserkSpeedBounds  [len(berserkSpeedPresets)]sim.Rect
	aiBusToggleBounds   sim.Rect
	langToggleBounds    sim.Rect
	menuButtonBounds    sim.Rect
	closeButtonBounds   sim.Rect
}

func NewSettings() *Settings {
	return &Settings{
		IsOpen:                   false,
		Fare:                     sim.DefaultFarePolicy(),
		TrafficFinesEnabled:      true,
		FineAmount:               50,
		TrafficDensity:           2,                      // Default: Medium
		LaneWidthLevel:           1,                      // Default: 52px (宽敞市区 Comfortable Wide)
		CameraViewMode:           sim.CameraModeNorthUp,  // Default: North-Up standard
		MapTheme:                 sim.MapThemeMetropolis, // Default: Modern Metropolis
		AutopilotEnabled:         false,                  // Manual driving by default
		AutopilotBerserk:         false,                  // Standard mode by default
		AutopilotBerserkMaxSpeed: 80,                     // Default: 80 km/h (445 px/s)
		AICarMinSpeed:            30, AICarMaxSpeed: 60, AIBusMinSpeed: 30, AIBusMaxSpeed: 50,
		AIBusCount:         0, // 0 AI buses by default
		Announcements:      AnnouncementsFollowLanguage,
		Language:           locale.CurrentLanguage,     // Default to CurrentLanguage
		PassengerSpawnRate: sim.PassengerSpawnModerate, // Default to Moderate
	}
}

func (s *Settings) CyclePassengerSpawnRate() sim.PassengerSpawnRate {
	switch s.PassengerSpawnRate {
	case sim.PassengerSpawnModerate:
		s.PassengerSpawnRate = sim.PassengerSpawnFew
	case sim.PassengerSpawnFew:
		s.PassengerSpawnRate = sim.PassengerSpawnMany
	default:
		s.PassengerSpawnRate = sim.PassengerSpawnModerate
	}
	return s.PassengerSpawnRate
}

// BerserkSpeedKPH also resolves retired 60/100 km/h presets from old saves.
func (s *Settings) BerserkSpeedKPH() int {
	if slices.Contains(berserkSpeedPresets[:], s.AutopilotBerserkMaxSpeed) {
		return s.AutopilotBerserkMaxSpeed
	}
	return berserkSpeedPresets[0]
}

func (s *Settings) GetBerserkCruiseSpeed() float64 {
	return sim.SpeedKPH(s.BerserkSpeedKPH())
}

// GetBusMaxSpeed is the player bus top speed shared by manual driving and
// both autopilot modes.
func (s *Settings) GetBusMaxSpeed() float64 {
	return s.GetBerserkCruiseSpeed()
}

func (s *Settings) GetPlayerModel() sim.BusModel {
	if s == nil || !s.PlayerModel.Valid() {
		return sim.BusStandard
	}
	return s.PlayerModel
}

func (s *Settings) SetBerserkMaxSpeed(speed int) {
	if slices.Contains(berserkSpeedPresets[:], speed) {
		s.AutopilotBerserkMaxSpeed = speed
	}
}

func (s *Settings) SetMaxSpeed(speed int) {
	s.SetBerserkMaxSpeed(speed)
}

func (s *Settings) CycleBerserkMaxSpeed() int {
	index := slices.Index(berserkSpeedPresets[:], s.AutopilotBerserkMaxSpeed)
	s.AutopilotBerserkMaxSpeed = berserkSpeedPresets[(index+1)%len(berserkSpeedPresets)]
	return s.AutopilotBerserkMaxSpeed
}

func (s *Settings) CycleMaxSpeed() int {
	return s.CycleBerserkMaxSpeed()
}

func (s *Settings) GetLaneWidth() float64 {
	return 52.0 // 默认固定为宽敞市区 (52px)
}

func (s *Settings) Toggle() {
	s.IsOpen = !s.IsOpen
}

func (s *Settings) HasSaveLoadButtons() bool {
	return s != nil && s.saveBounds.W > 0 && s.loadBounds.W > 0
}

func (s *Settings) Update(cam ...*sim.Camera) {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		if s.IsOpen && s.PassengerTimingOpen {
			s.PassengerTimingOpen = false
			return
		}
		s.Toggle()
		return
	}

	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		s.Language = locale.ToggleLanguage()
	}

	if !s.IsOpen {
		return
	}

	var activeCam *sim.Camera
	if len(cam) > 0 && cam[0] != nil {
		activeCam = cam[0]
		s.CameraViewMode = activeCam.Mode
	}

	// Handle mouse clicks on buttons
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mPos := sim.Vec2{X: float64(mx), Y: float64(my)}

		if s.PassengerTimingOpen {
			if s.passengerTimingClose.Contains(mPos) {
				s.PassengerTimingOpen = false
				return
			}
			for model, row := range s.passengerCapacityBounds {
				for direction, b := range row {
					if b.Contains(mPos) {
						s.AdjustPassengerOverload(sim.BusModel(model), direction*2-1)
						return
					}
				}
			}
			for model, row := range s.passengerRateBounds {
				for door, b := range row {
					if b.Contains(mPos) {
						s.CyclePassengerRate(sim.BusModel(model), door)
						return
					}
				}
			}
			return
		}
		for rate, bounds := range s.clockRateBounds {
			if bounds.Contains(mPos) {
				s.ClockRate = sim.ClockRate(rate)
				return
			}
		}
		if s.passengerTimingBounds.Contains(mPos) {
			s.PassengerTimingOpen = true
			return
		}
		for channel, buttons := range s.volumeBounds {
			for direction, bounds := range buttons {
				if bounds.Contains(mPos) {
					delta := -10
					if direction == 1 {
						delta = 10
					}
					s.AdjustVolume(channel, delta)
					return
				}
			}
		}
		for field, buttons := range s.speedRangeBounds {
			for direction, bounds := range buttons {
				if bounds.Contains(mPos) {
					delta := -10
					if direction == 1 {
						delta = 10
					}
					s.AdjustAISpeed(field, delta)
					return
				}
			}
		}
		if s.announcementVoiceBounds.Contains(mPos) {
			s.AnnouncementVoice = (s.AnnouncementVoice + 1) % 4
			return
		}
		if s.announcementsBounds.Contains(mPos) {
			s.CycleAnnouncements()
			return
		}
		if s.passengerSpawnRateBounds.Contains(mPos) {
			s.CyclePassengerSpawnRate()
			return
		}
		if s.busSignalPriorityBounds.Contains(mPos) {
			s.BusSignalPriority = !s.BusSignalPriority
			return
		}
		if !s.CareerMode && s.saveBounds.Contains(mPos) {
			s.SaveRequested = true
			return
		}
		if !s.CareerMode && s.loadBounds.Contains(mPos) {
			s.LoadRequested = true
			return
		}
		if s.operationsBounds.Contains(mPos) {
			s.OpenOperationsRequested = true
			s.IsOpen = false
			return
		}
		if s.menuButtonBounds.Contains(mPos) {
			s.ReturnToMenuRequested = true
			s.IsOpen = false
		} else if s.fineToggleBounds.Contains(mPos) {
			s.TrafficFinesEnabled = !s.TrafficFinesEnabled
		} else if s.densityToggleBounds.Contains(mPos) {
			s.TrafficDensity = (s.TrafficDensity % sim.TrafficDensityLevels) + 1
		} else if s.cameraToggleBounds.Contains(mPos) {
			switch s.CameraViewMode {
			case sim.CameraModeNorthUp:
				s.CameraViewMode = sim.CameraModeHeadingUp
			case sim.CameraModeHeadingUp:
				s.CameraViewMode = sim.CameraModeSouthUp
			case sim.CameraModeSouthUp:
				s.CameraViewMode = sim.CameraModeNorthUp
			default:
				s.CameraViewMode = sim.CameraModeNorthUp
			}
			if activeCam != nil {
				activeCam.SetMode(s.CameraViewMode)
			}
		} else if s.autopilotBounds.Contains(mPos) {
			if !s.AutopilotEnabled {
				s.AutopilotEnabled = true
				s.AutopilotBerserk = false
			} else if !s.AutopilotBerserk {
				s.AutopilotBerserk = true
			} else {
				s.AutopilotEnabled = false
				s.AutopilotBerserk = false
			}
		} else if index := slices.IndexFunc(s.berserkSpeedBounds[:], func(bounds sim.Rect) bool { return bounds.Contains(mPos) }); index >= 0 {
			s.SetBerserkMaxSpeed(berserkSpeedPresets[index])
		} else if s.aiBusToggleBounds.Contains(mPos) {
			if s.CareerMode {
				s.OpenCareerRequested = true
			} else {
				s.OpenNetworkRequested = true
			}
			s.IsOpen = false
		} else if s.langToggleBounds.Contains(mPos) {
			s.Language = locale.ToggleLanguage()
		} else if s.closeButtonBounds.Contains(mPos) {
			s.IsOpen = false
		}
	}
}

// IsLargeRoadNetwork returns whether the specified map theme is in 4x large road network mode.
func (s *Settings) IsLargeRoadNetwork(theme sim.MapTheme) bool {
	if s == nil || theme == sim.MapThemeCustom {
		return false
	}
	if s.LargeRoadNetworks != nil {
		return s.LargeRoadNetworks[theme]
	}
	if theme == s.MapTheme {
		return s.LargeRoadNetwork
	}
	return false
}

// SetLargeRoadNetwork updates the road network mode for a given map theme.
func (s *Settings) SetLargeRoadNetwork(theme sim.MapTheme, large bool) {
	if s == nil || theme == sim.MapThemeCustom {
		return
	}
	if s.LargeRoadNetworks == nil {
		s.LargeRoadNetworks = make(map[sim.MapTheme]bool)
	}
	s.LargeRoadNetworks[theme] = large
	if theme == s.MapTheme {
		s.LargeRoadNetwork = large
	}
}

// ToggleLargeRoadNetwork flips the road network mode for a given map theme.
func (s *Settings) ToggleLargeRoadNetwork(theme sim.MapTheme) bool {
	current := s.IsLargeRoadNetwork(theme)
	s.SetLargeRoadNetwork(theme, !current)
	return !current
}

// Existing vehicles retain their sampled limit; edits apply to future spawns.
func (s *Settings) AdjustAISpeed(field, delta int) {
	switch field {
	case 0:
		s.AICarMinSpeed = max(30, min(s.AICarMaxSpeed, s.AICarMinSpeed+delta))
	case 1:
		s.AICarMaxSpeed = max(s.AICarMinSpeed, min(sim.TrafficMaxSpeedKPH, s.AICarMaxSpeed+delta))
	case 2:
		s.AIBusMinSpeed = max(30, min(s.AIBusMaxSpeed, s.AIBusMinSpeed+delta))
	case 3:
		s.AIBusMaxSpeed = max(s.AIBusMinSpeed, min(sim.TrafficMaxSpeedKPH, s.AIBusMaxSpeed+delta))
	}
}

// Missing volume fields in older saves retain the original full-volume mix.
func (s *Settings) Volumes() sound.Volumes {
	if s.AudioVolumes == nil {
		return sound.DefaultVolumes()
	}
	return s.AudioVolumes.Normalize()
}
func (s *Settings) AdjustVolume(channel, delta int) {
	v := s.Volumes()
	switch channel {
	case 0:
		v.Engine += delta
	case 1:
		v.Announcements += delta
	case 2:
		v.Horn += delta
	default:
		return
	}
	v = v.Normalize()
	s.AudioVolumes = &v
}

// IsCenterBRT keeps the original first-map-only setting compatible with old saves.
func (s *Settings) IsCenterBRT(theme sim.MapTheme) bool {
	if s == nil {
		return false
	}
	if s.CenterBRTMaps != nil {
		return s.CenterBRTMaps[theme]
	}
	return theme == sim.MapThemeMetropolis && s.CenterBRT
}
func (s *Settings) SetCenterBRT(theme sim.MapTheme, enabled bool) {
	if s == nil || theme < sim.MapThemeMetropolis || theme > sim.MapThemeCustom {
		return
	}
	if s.CenterBRTMaps == nil {
		s.CenterBRTMaps = map[sim.MapTheme]bool{sim.MapThemeMetropolis: s.CenterBRT}
	}
	s.CenterBRTMaps[theme] = enabled
	if theme == s.MapTheme {
		s.CenterBRT = enabled
	}
}
func (s *Settings) ToggleCenterBRT(theme sim.MapTheme) bool {
	enabled := !s.IsCenterBRT(theme)
	s.SetCenterBRT(theme, enabled)
	return enabled
}
