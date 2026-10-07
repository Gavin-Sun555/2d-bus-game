package game

import (
	"math"
	"math/rand"
)

// CameraMode defines the driving perspective of the camera
type CameraMode int

const (
	CameraModeNorthUp   CameraMode = iota // 固定正北朝上视角 (Current fixed North-Up view)
	CameraModeHeadingUp                  // 跟随车车头一直朝上视角 (Car-centric Chase / Heading-Up view)
	CameraModeSouthUp                    // 固定正南朝上视角 (Fixed South-Up view, South points UP)
)

func (m CameraMode) String() string {
	switch m {
	case CameraModeHeadingUp:
		return "车头朝上 (跟随)"
	case CameraModeSouthUp:
		return "固定正南 (倒置)"
	default:
		return "固定正北 (标准)"
	}
}

// Camera represents the 2D view camera
type Camera struct {
	Pos            Vec2       // Center of the view in world coordinates
	TargetPos      Vec2       // Target position for smooth interpolation
	Zoom           float64    // Scale factor (1.0 = normal, 0.5 = 2x zoomed out)
	TargetZoom     float64
	Rotation       float64    // Camera view rotation in radians (0 = North-Up, pi = South-Up)
	TargetRotation float64    // Target rotation for smooth interpolation
	Mode           CameraMode // Current camera perspective mode
	ShakeIntensity float64    // Screen shake on collision
	ShakeOffset    Vec2

	ScreenWidth  int
	ScreenHeight int
}

func NewCamera(screenWidth, screenHeight int) *Camera {
	return &Camera{
		Pos:            Vec2{X: 0, Y: 0},
		TargetPos:      Vec2{X: 0, Y: 0},
		Zoom:           1.0,
		TargetZoom:     1.0,
		Rotation:       0,
		TargetRotation: 0,
		Mode:           CameraModeNorthUp, // Default to traditional North-Up, switchable via V key / Settings / HUD
		ScreenWidth:    screenWidth,
		ScreenHeight:   screenHeight,
	}
}

func (c *Camera) ToggleMode() CameraMode {
	switch c.Mode {
	case CameraModeNorthUp:
		c.Mode = CameraModeHeadingUp
	case CameraModeHeadingUp:
		c.Mode = CameraModeSouthUp
	case CameraModeSouthUp:
		c.Mode = CameraModeNorthUp
	default:
		c.Mode = CameraModeNorthUp
	}
	return c.Mode
}

func (c *Camera) SetMode(m CameraMode) {
	c.Mode = m
}

func (c *Camera) AddShake(amount float64) {
	c.ShakeIntensity = math.Min(22.0, c.ShakeIntensity+amount)
}

func (c *Camera) Update(dt float64) {
	// Smooth position interpolation
	tPos := 1.0 - math.Pow(0.001, dt) // Responsive exponential lerp
	c.Pos.X = Lerp(c.Pos.X, c.TargetPos.X, tPos)
	c.Pos.Y = Lerp(c.Pos.Y, c.TargetPos.Y, tPos)

	// Smooth zoom interpolation
	tZoom := 1.0 - math.Pow(0.01, dt)
	c.Zoom = Lerp(c.Zoom, c.TargetZoom, tZoom)

	// Smooth rotation interpolation towards TargetRotation
	tRot := 1.0 - math.Pow(0.001, dt)
	if c.Mode == CameraModeHeadingUp {
		tRot = 1.0 - math.Pow(0.0005, dt) // Responsive smooth rotation tracking
	}
	diff := NormalizeAngle(c.TargetRotation - c.Rotation)
	if math.Abs(diff) > 0.0005 {
		c.Rotation += diff * tRot
	} else {
		c.Rotation = c.TargetRotation
	}

	// Screen shake decay
	if c.ShakeIntensity > 0.1 {
		c.ShakeIntensity *= math.Pow(0.05, dt)
		c.ShakeOffset.X = (rand.Float64()*2 - 1) * c.ShakeIntensity
		c.ShakeOffset.Y = (rand.Float64()*2 - 1) * c.ShakeIntensity
	} else {
		c.ShakeIntensity = 0
		c.ShakeOffset = Vec2{}
	}
}

func (c *Camera) FollowBus(bus *Bus) {
	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	switch c.Mode {
	case CameraModeHeadingUp:
		// Heading-Up perspective: Bus heading always points straight UP (-pi/2 on screen)
		c.TargetRotation = bus.Heading + math.Pi/2

		// Offset forward so more of the road ahead is visible
		forwardOffset := 120.0 + 0.35*math.Max(0, bus.Speed)
		c.TargetPos.X = bus.Pos.X + cosH*forwardOffset
		c.TargetPos.Y = bus.Pos.Y + sinH*forwardOffset
	case CameraModeSouthUp:
		// South-Up perspective: Fixed rotation pi (South points UP)
		c.TargetRotation = math.Pi
		forwardOffset := 0.25 * bus.Speed
		c.TargetPos.X = bus.Pos.X + cosH*forwardOffset
		c.TargetPos.Y = bus.Pos.Y + sinH*forwardOffset
	default: // CameraModeNorthUp
		// North-Up perspective: Fixed rotation 0
		c.TargetRotation = 0
		forwardOffset := 0.25 * bus.Speed
		c.TargetPos.X = bus.Pos.X + cosH*forwardOffset
		c.TargetPos.Y = bus.Pos.Y + sinH*forwardOffset
	}
}

func (c *Camera) WorldToScreen(world Vec2) Vec2 {
	halfW := float64(c.ScreenWidth) / 2
	halfH := float64(c.ScreenHeight) / 2

	effX := c.Pos.X + c.ShakeOffset.X
	effY := c.Pos.Y + c.ShakeOffset.Y

	dx := world.X - effX
	dy := world.Y - effY

	if c.Rotation != 0 {
		cosR := math.Cos(c.Rotation)
		sinR := math.Sin(c.Rotation)
		rx := dx*cosR + dy*sinR
		ry := -dx*sinR + dy*cosR
		dx = rx
		dy = ry
	}

	sx := dx*c.Zoom + halfW
	sy := dy*c.Zoom + halfH
	return Vec2{X: sx, Y: sy}
}

func (c *Camera) ScreenToWorld(screen Vec2) Vec2 {
	halfW := float64(c.ScreenWidth) / 2
	halfH := float64(c.ScreenHeight) / 2

	effX := c.Pos.X + c.ShakeOffset.X
	effY := c.Pos.Y + c.ShakeOffset.Y

	dx := (screen.X - halfW) / c.Zoom
	dy := (screen.Y - halfH) / c.Zoom

	if c.Rotation != 0 {
		cosR := math.Cos(c.Rotation)
		sinR := math.Sin(c.Rotation)
		rx := dx*cosR - dy*sinR
		ry := dx*sinR + dy*cosR
		dx = rx
		dy = ry
	}

	wx := dx + effX
	wy := dy + effY
	return Vec2{X: wx, Y: wy}
}
