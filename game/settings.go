package game

import (
	"fmt"
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Settings manages user game configuration
type Settings struct {
	IsOpen              bool
	TrafficFinesEnabled bool
	FineAmount          int
	TrafficDensity      int        // 1: Low (12), 2: Med (24), 3: High (36)
	LaneWidthLevel      int        // 0: 42px (标准紧凑), 1: 52px (宽敞舒适), 2: 64px (开阔大道), 3: 78px (超宽城际)
	CameraViewMode      CameraMode // 0: CameraModeNorthUp, 1: CameraModeHeadingUp

	// Button bounds for click detection
	fineToggleBounds    Rect
	densityToggleBounds Rect
	laneToggleBounds    Rect
	cameraToggleBounds  Rect
	closeButtonBounds   Rect
}

type Rect struct {
	X float64
	Y float64
	W float64
	H float64
}

func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.X && p.X <= r.X+r.W && p.Y >= r.Y && p.Y <= r.Y+r.H
}

func NewSettings() *Settings {
	return &Settings{
		IsOpen:              false,
		TrafficFinesEnabled: true,
		FineAmount:          50,
		TrafficDensity:      2,                 // Default: Medium
		LaneWidthLevel:      1,                 // Default: 52px (Comfortable Wide)
		CameraViewMode:      CameraModeNorthUp, // Default: North-Up standard
	}
}

func (s *Settings) GetLaneWidth() float64 {
	switch s.LaneWidthLevel {
	case 0:
		return 42.0
	case 1:
		return 52.0
	case 2:
		return 64.0
	case 3:
		return 78.0
	default:
		return 52.0
	}
}

func (s *Settings) Toggle() {
	s.IsOpen = !s.IsOpen
}

func (s *Settings) Update(cam ...*Camera) {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		s.Toggle()
		return
	}

	if !s.IsOpen {
		return
	}

	var activeCam *Camera
	if len(cam) > 0 && cam[0] != nil {
		activeCam = cam[0]
		s.CameraViewMode = activeCam.Mode
	}

	// Handle mouse clicks on buttons
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mPos := Vec2{X: float64(mx), Y: float64(my)}

		if s.fineToggleBounds.Contains(mPos) {
			s.TrafficFinesEnabled = !s.TrafficFinesEnabled
		} else if s.densityToggleBounds.Contains(mPos) {
			s.TrafficDensity = (s.TrafficDensity % 3) + 1
		} else if s.laneToggleBounds.Contains(mPos) {
			s.LaneWidthLevel = (s.LaneWidthLevel + 1) % 4
		} else if s.cameraToggleBounds.Contains(mPos) {
			switch s.CameraViewMode {
			case CameraModeNorthUp:
				s.CameraViewMode = CameraModeHeadingUp
			case CameraModeHeadingUp:
				s.CameraViewMode = CameraModeSouthUp
			case CameraModeSouthUp:
				s.CameraViewMode = CameraModeNorthUp
			default:
				s.CameraViewMode = CameraModeNorthUp
			}
			if activeCam != nil {
				activeCam.SetMode(s.CameraViewMode)
			}
		} else if s.closeButtonBounds.Contains(mPos) {
			s.IsOpen = false
		}
	}
}

func (s *Settings) Draw(screen *ebiten.Image, cam ...*Camera) {
	if !s.IsOpen {
		return
	}

	if len(cam) > 0 && cam[0] != nil {
		s.CameraViewMode = cam[0].Mode
	}

	sw := float64(screen.Bounds().Dx())
	sh := float64(screen.Bounds().Dy())

	// Dark semi-transparent backdrop
	vector.DrawFilledRect(screen, 0, 0, float32(sw), float32(sh), RGBA(0, 0, 0, 160), false)

	// Modal card
	modalW := 550.0
	modalH := 485.0
	modalX := (sw - modalW) / 2
	modalY := (sh - modalH) / 2

	// Modal shadow and background
	vector.DrawFilledRect(screen, float32(modalX+6), float32(modalY+6), float32(modalW), float32(modalH),
		RGBA(2, 6, 23, 140), true)
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		RGBA(15, 23, 42, 245), true)
	vector.StrokeRect(screen, float32(modalX), float32(modalY), float32(modalW), float32(modalH),
		2, RGBA(59, 130, 246, 255), true)

	// Modal Header
	vector.DrawFilledRect(screen, float32(modalX), float32(modalY), float32(modalW), 46,
		RGBA(30, 41, 59, 255), true)
	DrawText(screen, "=== 游戏设置与道路自定义 (Settings & Customization) ===", modalX+24, modalY+15, 14, color.White)

	// Option 1: Traffic Fine Toggle
	opt1Y := modalY + 58
	DrawText(screen, "1. 红绿灯违章抓拍与罚款机制:", modalX+30, opt1Y+8, 13, RGBA(226, 232, 240, 255))
	s.fineToggleBounds = Rect{X: modalX + 320, Y: opt1Y, W: 190, H: 32}

	fineBtnColor := RGBA(34, 197, 94, 255) // Green (Enabled)
	fineBtnText := "【已开启 (扣$50)】"
	if !s.TrafficFinesEnabled {
		fineBtnColor = RGBA(239, 68, 68, 255) // Red (Disabled)
		fineBtnText = "【已关闭 (无罚款)】"
	}
	vector.DrawFilledRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.fineToggleBounds.X), float32(s.fineToggleBounds.Y),
		float32(s.fineToggleBounds.W), float32(s.fineToggleBounds.H), 1.5, fineBtnColor, true)
	DrawText(screen, fineBtnText, s.fineToggleBounds.X+24, s.fineToggleBounds.Y+8, 13, fineBtnColor)

	// Option 2: AI Traffic Density
	opt2Y := modalY + 102
	DrawText(screen, "2. AI 私家车交通流密度:", modalX+30, opt2Y+8, 13, RGBA(226, 232, 240, 255))
	s.densityToggleBounds = Rect{X: modalX + 320, Y: opt2Y, W: 190, H: 32}

	densityText := "中等密度 (24辆)"
	if s.TrafficDensity == 1 {
		densityText = "低密度 (12辆)"
	} else if s.TrafficDensity == 3 {
		densityText = "高密度 (36辆)"
	}
	vector.DrawFilledRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.densityToggleBounds.X), float32(s.densityToggleBounds.Y),
		float32(s.densityToggleBounds.W), float32(s.densityToggleBounds.H), 1.5, RGBA(56, 189, 248, 255), true)
	DrawText(screen, densityText, s.densityToggleBounds.X+36, s.densityToggleBounds.Y+8, 13, RGBA(56, 189, 248, 255))

	// Option 3: Customizable Lane Width
	opt3Y := modalY + 146
	DrawText(screen, "3. 自定义全城车道宽度:", modalX+30, opt3Y+8, 13, RGBA(226, 232, 240, 255))
	s.laneToggleBounds = Rect{X: modalX + 320, Y: opt3Y, W: 190, H: 32}

	laneDesc := "宽敞市区 (52px)"
	switch s.LaneWidthLevel {
	case 0:
		laneDesc = "标准紧凑 (42px)"
	case 1:
		laneDesc = "宽敞市区 (52px)★"
	case 2:
		laneDesc = "开阔大道 (64px)"
	case 3:
		laneDesc = "超宽城际 (78px)"
	}
	vector.DrawFilledRect(screen, float32(s.laneToggleBounds.X), float32(s.laneToggleBounds.Y),
		float32(s.laneToggleBounds.W), float32(s.laneToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.laneToggleBounds.X), float32(s.laneToggleBounds.Y),
		float32(s.laneToggleBounds.W), float32(s.laneToggleBounds.H), 1.5, RGBA(250, 204, 21, 255), true)
	DrawText(screen, laneDesc, s.laneToggleBounds.X+28, s.laneToggleBounds.Y+8, 13, RGBA(250, 204, 21, 255))

	// Option 4: Camera View Perspective
	opt4Y := modalY + 190
	DrawText(screen, "4. 驾驶摄像机视角 (Camera View):", modalX+30, opt4Y+8, 13, RGBA(226, 232, 240, 255))
	s.cameraToggleBounds = Rect{X: modalX + 320, Y: opt4Y, W: 190, H: 32}

	camViewText := "【固定正北 (标准)】"
	camViewColor := RGBA(148, 163, 184, 255)
	switch s.CameraViewMode {
	case CameraModeHeadingUp:
		camViewText = "【车头朝上 (跟随)】"
		camViewColor = RGBA(56, 189, 248, 255)
	case CameraModeSouthUp:
		camViewText = "【固定正南 (倒置)】"
		camViewColor = RGBA(251, 146, 60, 255)
	}
	vector.DrawFilledRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), RGBA(30, 41, 59, 255), true)
	vector.StrokeRect(screen, float32(s.cameraToggleBounds.X), float32(s.cameraToggleBounds.Y),
		float32(s.cameraToggleBounds.W), float32(s.cameraToggleBounds.H), 1.5, camViewColor, true)
	DrawText(screen, camViewText, s.cameraToggleBounds.X+22, s.cameraToggleBounds.Y+8, 13, camViewColor)

	// Rules & Customization Explanation Info Box
	infoY := modalY + 238
	vector.DrawFilledRect(screen, float32(modalX+30), float32(infoY), float32(modalW-60), 180,
		RGBA(15, 23, 42, 200), true)
	vector.StrokeRect(screen, float32(modalX+30), float32(infoY), float32(modalW-60), 180,
		1, RGBA(71, 85, 105, 255), true)
	DrawText(screen, "• 道路交通规则与摄像机视角说明:", modalX+40, infoY+12, 12, RGBA(148, 163, 184, 255))
	DrawText(screen, "  - 视角切换: 支持【车头朝上跟随】/【固定正北标准】/【固定正南倒置】，按 [V] 键切换；", modalX+40, infoY+34, 12, RGBA(56, 189, 248, 255))
	DrawText(screen, "  - 车头朝上视角下视野随车旋转并自动前瞻；正南视角将全城地图倒置（南朝上）；", modalX+40, infoY+54, 12, RGBA(203, 213, 225, 255))
	DrawText(screen, "  - 城市支持4车道主干道、2车道市区街道及景观水道；", modalX+40, infoY+74, 12, RGBA(203, 213, 225, 255))
	DrawText(screen, "  - 实时点击第3项可动态切换全城车道标线与路面物理宽度；", modalX+40, infoY+94, 12, RGBA(203, 213, 225, 255))
	DrawText(screen, "  - 全城道路与路口几何尺寸全面加宽，显著降低公交转弯难度；", modalX+40, infoY+114, 12, RGBA(203, 213, 225, 255))

	descFine := fmt.Sprintf("  - 罚款状态: 违章抓拍扣除 $%d (红灯压线/对侧车道判断)。", s.FineAmount)
	descFineColor := RGBA(248, 113, 113, 255)
	if !s.TrafficFinesEnabled {
		descFine = "  - 罚款状态: 违章已豁免扣款 (仅弹出警示提示)。"
		descFineColor = RGBA(74, 222, 128, 255)
	}
	DrawText(screen, descFine, modalX+40, infoY+142, 12, descFineColor)

	// Close / Resume Button
	s.closeButtonBounds = Rect{X: modalX + modalW/2 - 100, Y: modalY + modalH - 52, W: 200, H: 36}
	vector.DrawFilledRect(screen, float32(s.closeButtonBounds.X), float32(s.closeButtonBounds.Y),
		float32(s.closeButtonBounds.W), float32(s.closeButtonBounds.H), RGBA(59, 130, 246, 255), true)
	DrawText(screen, "确认并返回游戏 [ESC]", s.closeButtonBounds.X+30, s.closeButtonBounds.Y+10, 13, color.White)
}
