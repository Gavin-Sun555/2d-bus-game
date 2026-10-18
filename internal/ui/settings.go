package ui

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

// Settings manages user game configuration
type Settings struct {
	IsOpen                bool
	TrafficFinesEnabled   bool
	FineAmount            int
	TrafficDensity        int            // 1: Low (12), 2: Med (24), 3: High (36), 4: Very High (72), 5: Extreme Jam (128)
	LaneWidthLevel        int            // 0: 42px (标准紧凑), 1: 52px (宽敞舒适), 2: 64px (开阔大道), 3: 78px (超宽城际)
	CameraViewMode        sim.CameraMode // 0: CameraModeNorthUp, 1: CameraModeHeadingUp
	MapTheme              sim.MapTheme   // Active city map
	ThemeChanged          bool
	ReturnToMenuRequested bool

	// AI Autopilot & Route AI Buses Configurations
	AutopilotEnabled         bool
	AutopilotBerserk         bool // Berserk mode: straightaways top speed boost while obeying traffic rules
	AutopilotBerserkMaxSpeed int  // Berserk top speed in km/h: 60, 80, 100, 120 (default: 80)
	AIBusCount               int  // Number of AI buses on the route (0 ~ 8)
	AIBusFleetChanged        bool

	// UI Localization Language
	Language locale.Language

	// Button bounds for click detection
	fineToggleBounds      sim.Rect
	densityToggleBounds   sim.Rect
	cameraToggleBounds    sim.Rect
	autopilotBounds       sim.Rect
	berserkSpeed60Bounds  sim.Rect
	berserkSpeed80Bounds  sim.Rect
	berserkSpeed100Bounds sim.Rect
	berserkSpeed120Bounds sim.Rect
	aiBusToggleBounds     sim.Rect
	themeToggleBounds     sim.Rect
	langToggleBounds      sim.Rect
	menuButtonBounds      sim.Rect
	closeButtonBounds     sim.Rect
}

func NewSettings() *Settings {
	return &Settings{
		IsOpen:                   false,
		TrafficFinesEnabled:      true,
		FineAmount:               50,
		TrafficDensity:           2,                      // Default: Medium
		LaneWidthLevel:           1,                      // Default: 52px (宽敞市区 Comfortable Wide)
		CameraViewMode:           sim.CameraModeNorthUp,  // Default: North-Up standard
		MapTheme:                 sim.MapThemeMetropolis, // Default: Modern Metropolis
		AutopilotEnabled:         false,                  // Manual driving by default
		AutopilotBerserk:         false,                  // Standard mode by default
		AutopilotBerserkMaxSpeed: 80,                     // Default: 80 km/h (445 px/s)
		AIBusCount:               0,                      // 0 AI buses by default
		Language:                 locale.CurrentLanguage, // Default to CurrentLanguage
	}
}

func (s *Settings) GetBerserkCruiseSpeed() float64 {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		return 333.0
	case 80:
		return 445.0
	case 100:
		return 555.0
	case 120:
		return 667.0
	default:
		return 445.0
	}
}

// GetBusMaxSpeed returns the vehicle physics max forward speed in px/s based on the configured speed limit.
// In manual driving mode, the bus accelerates up to this limit (reaching 60, 80, 100, or 120 km/h).
func (s *Settings) GetBusMaxSpeed() float64 {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		return 334.0 // 334 * 0.18 = 60.12 km/h
	case 80:
		return 445.0 // 445 * 0.18 = 80.1 km/h
	case 100:
		return 556.0 // 556 * 0.18 = 100.08 km/h
	case 120:
		return 667.0 // 667 * 0.18 = 120.06 km/h
	default:
		return 445.0
	}
}

func (s *Settings) SetBerserkMaxSpeed(speed int) {
	if speed == 60 || speed == 80 || speed == 100 || speed == 120 {
		s.AutopilotBerserkMaxSpeed = speed
	}
}

func (s *Settings) SetMaxSpeed(speed int) {
	s.SetBerserkMaxSpeed(speed)
}

func (s *Settings) CycleBerserkMaxSpeed() int {
	switch s.AutopilotBerserkMaxSpeed {
	case 60:
		s.AutopilotBerserkMaxSpeed = 80
	case 80:
		s.AutopilotBerserkMaxSpeed = 100
	case 100:
		s.AutopilotBerserkMaxSpeed = 120
	case 120:
		s.AutopilotBerserkMaxSpeed = 60
	default:
		s.AutopilotBerserkMaxSpeed = 80
	}
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

func (s *Settings) Update(cam ...*sim.Camera) {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
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

		if s.menuButtonBounds.Contains(mPos) {
			s.ReturnToMenuRequested = true
			s.IsOpen = false
		} else if s.fineToggleBounds.Contains(mPos) {
			s.TrafficFinesEnabled = !s.TrafficFinesEnabled
		} else if s.densityToggleBounds.Contains(mPos) {
			s.TrafficDensity = (s.TrafficDensity % 5) + 1
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
		} else if s.berserkSpeed60Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 60
		} else if s.berserkSpeed80Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 80
		} else if s.berserkSpeed100Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 100
		} else if s.berserkSpeed120Bounds.Contains(mPos) {
			s.AutopilotBerserkMaxSpeed = 120
		} else if s.aiBusToggleBounds.Contains(mPos) {
			s.AIBusCount = (s.AIBusCount + 1) % 9
			s.AIBusFleetChanged = true
		} else if s.themeToggleBounds.Contains(mPos) {
			s.ReturnToMenuRequested = true
			s.IsOpen = false
		} else if s.langToggleBounds.Contains(mPos) {
			s.Language = locale.ToggleLanguage()
		} else if s.closeButtonBounds.Contains(mPos) {
			s.IsOpen = false
		}
	}
}
