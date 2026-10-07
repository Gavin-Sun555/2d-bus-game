package game

import (
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type Gear int

const (
	GearReverse Gear = -1 // 倒挡 (R)
	GearNeutral Gear = 0  // 空挡 (N)
	GearDrive   Gear = 1  // 前进挡 (D)

	// 预留手动挡位 (Reserved for manual transmission):
	Gear1       Gear = 2  // 手动 1 挡 (M1)
	Gear2       Gear = 3  // 手动 2 挡 (M2)
	Gear3       Gear = 4  // 手动 3 挡 (M3)
	Gear4       Gear = 5  // 手动 4 挡 (M4)
	Gear5       Gear = 6  // 手动 5 挡 (M5)
)

func (g Gear) String() string {
	switch g {
	case GearReverse:
		return "R"
	case GearNeutral:
		return "N"
	case GearDrive:
		return "D"
	case Gear1:
		return "1"
	case Gear2:
		return "2"
	case Gear3:
		return "3"
	case Gear4:
		return "4"
	case Gear5:
		return "5"
	default:
		return "N"
	}
}

func (g Gear) Name() string {
	switch g {
	case GearReverse:
		return "倒挡 (R)"
	case GearNeutral:
		return "空挡 (N)"
	case GearDrive:
		return "前进挡 (D)"
	case Gear1:
		return "1挡 (M1)"
	case Gear2:
		return "2挡 (M2)"
	case Gear3:
		return "3挡 (M3)"
	case Gear4:
		return "4挡 (M4)"
	case Gear5:
		return "5挡 (M5)"
	default:
		return "空挡 (N)"
	}
}

type TransmissionMode int

const (
	TransmissionAuto TransmissionMode = iota // 自动挡 (D, N, R)
	TransmissionManual                       // 手动挡模式 (预留 1~5, N, R)
)

// Bus represents our driveable city bus
type Bus struct {
	Pos           Vec2    // Center position in world coordinates
	Heading       float64 // Direction in radians (0 = facing +X / East)
	Speed         float64 // Current speed (pixels per second, positive = forward, negative = reverse)
	SteeringAngle float64 // Current front wheel angle in radians

	// Transmission & Gear
	Gear             Gear             // Current gear (R, N, D, or M1-M5)
	TransmissionMode TransmissionMode // Automatic or Manual mode

	// Specs
	Length    float64 // 100 px (~12m)
	Width     float64 // 38 px (~2.5m)
	WheelBase float64 // 62 px (distance between front & rear axles)

	MaxSpeedFwd    float64 // 320 px/s
	MaxSpeedRev    float64 // 90 px/s
	AccelRate      float64 // 180 px/s^2
	BrakeRate      float64 // 360 px/s^2
	HandbrakeRate  float64 // 550 px/s^2
	FrictionRate   float64 // 60 px/s^2 (natural rolling resistance)
	MaxSteerAngle  float64 // ~38 deg
	SteerTurnSpeed float64 // radians/s
	SteerReturnSpd float64 // radians/s

	// State
	IsBraking     bool
	IsHandbraking bool
	IsReversing   bool
	DoorsOpen     bool
	DoorProgress  float64 // 0.0 (closed) to 1.0 (open)

	// Passenger capacity
	PassengerCount int
	Capacity       int

	// Vehicle Health & Collision
	Durability    float64 // 0.0 to 100.0%
	OnShoulder    bool    // true when wheels are on roadside shoulder (slowdown)
	CrashCooldown float64 // Invulnerability timer after impact

	// Skidmarks and trail
	TireTracks []TireMark
}

type TireMark struct {
	Pos1  Vec2
	Pos2  Vec2
	Alpha float32
}

func NewBus(x, y float64, heading float64) *Bus {
	return &Bus{
		Pos:              Vec2{X: x, Y: y},
		Heading:          heading,
		Speed:            0,
		SteeringAngle:    0,
		Gear:             GearDrive,          // Default in Drive
		TransmissionMode: TransmissionAuto,   // Default Automatic
		Length:           96,
		Width:            36,
		WheelBase:        60,
		MaxSpeedFwd:      330,
		MaxSpeedRev:      100,
		AccelRate:        190,
		BrakeRate:        380,
		HandbrakeRate:    600,
		FrictionRate:     70,
		MaxSteerAngle:    0.68, // ~39.0 degrees (agile cornering at intersections)
		SteerTurnSpeed:   2.4,  // steering response speed
		SteerReturnSpd:   3.5,  // auto-center return speed
		Capacity:         45,
		PassengerCount:   0,
		Durability:       100.0,
		OnShoulder:       false,
		CrashCooldown:    0,
		TireTracks:       make([]TireMark, 0, 200),
	}
}

// ShiftTo safely shifts to the specified gear with transmission speed protection
func (b *Bus) ShiftTo(target Gear) bool {
	// Safety interlock: Prevent shifting into reverse while moving forward fast (> 20 px/s)
	if target == GearReverse && b.Speed > 20.0 {
		return false
	}
	// Prevent shifting into forward while reversing fast (< -20 px/s)
	if (target == GearDrive || target >= Gear1) && b.Speed < -20.0 {
		return false
	}
	b.Gear = target
	return true
}

// ShiftUp shifts up one gear level
func (b *Bus) ShiftUp() bool {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearReverse:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearDrive)
		case GearDrive:
			return false
		default:
			return b.ShiftTo(GearDrive)
		}
	} else {
		// Manual mode progression: R -> N -> 1 -> 2 -> 3 -> 4 -> 5
		switch b.Gear {
		case GearReverse:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(Gear1)
		case Gear1:
			return b.ShiftTo(Gear2)
		case Gear2:
			return b.ShiftTo(Gear3)
		case Gear3:
			return b.ShiftTo(Gear4)
		case Gear4:
			return b.ShiftTo(Gear5)
		case Gear5:
			return false
		}
	}
	return false
}

// ShiftDown shifts down one gear level
func (b *Bus) ShiftDown() bool {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearDrive:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearReverse)
		case GearReverse:
			return false
		default:
			return b.ShiftTo(GearNeutral)
		}
	} else {
		// Manual mode progression: 5 -> 4 -> 3 -> 2 -> 1 -> N -> R
		switch b.Gear {
		case Gear5:
			return b.ShiftTo(Gear4)
		case Gear4:
			return b.ShiftTo(Gear3)
		case Gear3:
			return b.ShiftTo(Gear2)
		case Gear2:
			return b.ShiftTo(Gear1)
		case Gear1:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearReverse)
		case GearReverse:
			return false
		}
	}
	return false
}

// GetGearLimits returns top speed and acceleration multiplier for current gear
func (b *Bus) GetGearLimits() (maxSpeed float64, accelFactor float64) {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearDrive:
			return b.MaxSpeedFwd, 1.0
		case GearReverse:
			return b.MaxSpeedRev, 0.75
		default:
			return 0, 0
		}
	}

	// Manual mode specs (reserved functionality)
	switch b.Gear {
	case GearReverse:
		return b.MaxSpeedRev, 0.75
	case GearNeutral:
		return 0, 0
	case Gear1:
		return 75.0, 1.30
	case Gear2:
		return 140.0, 1.10
	case Gear3:
		return 210.0, 0.95
	case Gear4:
		return 275.0, 0.80
	case Gear5:
		return b.MaxSpeedFwd, 0.65
	default:
		return b.MaxSpeedFwd, 1.0
	}
}

// Update handles vehicle kinematics per frame (dt is seconds)
func (b *Bus) Update(dt float64, throttle float64, brake float64, steer float64, handbrake bool, toggleDoors bool) {
	if b.CrashCooldown > 0 {
		b.CrashCooldown -= dt
	}
	// 1. Doors control
	if toggleDoors {
		// Only allow door opening if bus is stopped or nearly stopped (< 5 px/s)
		if math.Abs(b.Speed) < 5 {
			b.DoorsOpen = !b.DoorsOpen
			if b.DoorsOpen {
				b.Speed = 0 // Full stop when doors open
			}
		}
	}

	if b.DoorsOpen {
		b.DoorProgress = math.Min(1.0, b.DoorProgress+dt*3.0)
		// Cannot move while doors are open
		b.Speed = 0
		return
	} else {
		b.DoorProgress = math.Max(0.0, b.DoorProgress-dt*3.0)
	}

	// 2. Steering input with smooth interpolation & auto-centering
	targetSteer := steer * b.MaxSteerAngle
	if steer != 0 {
		if b.SteeringAngle < targetSteer {
			b.SteeringAngle = math.Min(targetSteer, b.SteeringAngle+b.SteerTurnSpeed*dt)
		} else if b.SteeringAngle > targetSteer {
			b.SteeringAngle = math.Max(targetSteer, b.SteeringAngle-b.SteerTurnSpeed*dt)
		}
	} else {
		// Auto return to center
		if b.SteeringAngle > 0 {
			b.SteeringAngle = math.Max(0, b.SteeringAngle-b.SteerReturnSpd*dt)
		} else if b.SteeringAngle < 0 {
			b.SteeringAngle = math.Min(0, b.SteeringAngle+b.SteerReturnSpd*dt)
		}
	}

	// 3. Powertrain: Separated Foot Brake, Handbrake, Accelerator Throttle, and Gears
	b.IsBraking = false
	b.IsHandbraking = handbrake
	b.IsReversing = (b.Gear == GearReverse || b.Speed < -1.0)

	// A. Emergency Handbrake
	if handbrake {
		b.IsBraking = true
		if b.Speed > 0 {
			b.Speed = math.Max(0, b.Speed-b.HandbrakeRate*dt)
		} else if b.Speed < 0 {
			b.Speed = math.Min(0, b.Speed+b.HandbrakeRate*dt)
		}
	} else {
		// B. Foot Brake Pedal (S / Down): strictly decelerates vehicle towards 0 and holds at 0
		// NEVER causes the vehicle to reverse!
		if brake > 0 {
			b.IsBraking = true
			if b.Speed > 0 {
				b.Speed = math.Max(0, b.Speed-b.BrakeRate*brake*dt)
			} else if b.Speed < 0 {
				b.Speed = math.Min(0, b.Speed+b.BrakeRate*brake*dt)
			}
			// When speed reaches 0, foot brake holds the vehicle stationary!
		}

		// C. Accelerator Throttle (W / Up): applies engine torque according to current Gear
		if throttle > 0 && brake == 0 {
			maxSpd, accelRatio := b.GetGearLimits()

			switch b.Gear {
			case GearDrive, Gear1, Gear2, Gear3, Gear4, Gear5:
				// Forward gear: accelerates vehicle forward
				if b.Speed < 0 {
					// Counteract backward roll first
					b.Speed = math.Min(0, b.Speed+b.BrakeRate*dt)
					b.IsBraking = true
				} else {
					b.Speed = math.Min(maxSpd, b.Speed+b.AccelRate*accelRatio*throttle*dt)
				}
			case GearReverse:
				// Reverse gear (R): accelerates vehicle backward
				if b.Speed > 0 {
					// Counteract forward roll first
					b.Speed = math.Max(0, b.Speed-b.BrakeRate*dt)
					b.IsBraking = true
				} else {
					b.Speed = math.Max(-maxSpd, b.Speed-b.AccelRate*accelRatio*throttle*dt)
				}
			case GearNeutral:
				// Neutral gear (N): transmission disengaged, no wheel driving torque
			}
		}

		// D. Natural rolling resistance and coasting deceleration
		if (throttle == 0 || b.Gear == GearNeutral) && brake == 0 {
			if b.Speed > 0 {
				b.Speed = math.Max(0, b.Speed-b.FrictionRate*dt)
			} else if b.Speed < 0 {
				b.Speed = math.Min(0, b.Speed+b.FrictionRate*dt)
			}
		}
	}

	// Extra drag if driving onto roadside shoulder / verge (can drive, but slows down)
	if b.OnShoulder {
		if math.Abs(b.Speed) > 75 {
			b.Speed *= math.Pow(0.91, dt*60)
		}
	}

	// 4. Bicycle Kinematic Model Update
	if math.Abs(b.Speed) > 0.01 {
		// Angular velocity w = (v / L) * tan(delta)
		angularVelocity := (b.Speed / b.WheelBase) * math.Tan(b.SteeringAngle)
		b.Heading = NormalizeAngle(b.Heading + angularVelocity*dt)

		// Move along current heading
		b.Pos.X += b.Speed * math.Cos(b.Heading) * dt
		b.Pos.Y += b.Speed * math.Sin(b.Heading) * dt
	}

	// 5. Update tire tracks
	b.updateTireTracks(dt)
}

func (b *Bus) updateTireTracks(dt float64) {
	// Fade existing tracks
	for i := range b.TireTracks {
		b.TireTracks[i].Alpha -= float32(dt * 0.4)
	}
	// Filter out invisible tracks
	valid := b.TireTracks[:0]
	for _, t := range b.TireTracks {
		if t.Alpha > 0.05 {
			valid = append(valid, t)
		}
	}
	b.TireTracks = valid

	// Add new track if braking hard or drifting
	if (b.IsHandbraking || (b.IsBraking && math.Abs(b.Speed) > 100)) && math.Abs(b.Speed) > 30 {
		rl, rr := b.getRearWheelPositions()
		if len(b.TireTracks) > 1 {
			last := b.TireTracks[len(b.TireTracks)-1]
			if rl.Distance(last.Pos1) > 10 {
				b.TireTracks = append(b.TireTracks,
					TireMark{Pos1: last.Pos1, Pos2: rl, Alpha: 0.6},
					TireMark{Pos1: last.Pos2, Pos2: rr, Alpha: 0.6},
				)
			}
		} else {
			b.TireTracks = append(b.TireTracks,
				TireMark{Pos1: rl, Pos2: rl, Alpha: 0.6},
				TireMark{Pos1: rr, Pos2: rr, Alpha: 0.6},
			)
		}
	}
}

func (b *Bus) getRearWheelPositions() (Vec2, Vec2) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	// Rear axle offset from center (-b.WheelBase/2)
	rearX := b.Pos.X - cosH*(b.WheelBase/2)
	rearY := b.Pos.Y - sinH*(b.WheelBase/2)

	halfTrack := b.Width/2 - 2
	rl := Vec2{X: rearX - perpX*halfTrack, Y: rearY - perpY*halfTrack}
	rr := Vec2{X: rearX + perpX*halfTrack, Y: rearY + perpY*halfTrack}
	return rl, rr
}

// Draw renders the bus and its components in world space
func (b *Bus) Draw(screen *ebiten.Image, cam *Camera) {
	// 1. Draw tire skid marks
	for _, mark := range b.TireTracks {
		s1 := cam.WorldToScreen(mark.Pos1)
		s2 := cam.WorldToScreen(mark.Pos2)
		vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y),
			float32(5*cam.Zoom), RGBA(30, 30, 30, uint8(mark.Alpha*255)), true)
	}

	// 2. Draw vehicle shadow (subtle blur offset)
	shadowOffset := Vec2{X: 6, Y: 8}
	drawBusBody(screen, cam, b.Pos.Add(shadowOffset), b.Heading, b.Length, b.Width,
		RGBA(15, 20, 25, 90), false)

	// 3. Draw Steerable Front Wheels
	b.drawWheels(screen, cam)

	// 4. Draw Main Bus Body
	// Modern Transit Livery: Teal / Turquoise Metallic Body (#0ea5e9 / #0284c7)
	bodyColor := RGBA(14, 165, 233, 255)
	drawBusBody(screen, cam, b.Pos, b.Heading, b.Length, b.Width, bodyColor, true)

	// 5. Draw Roof Details, Windows & Air Conditioner Unit
	b.drawBusRoof(screen, cam)

	// 6. Draw Lights
	b.drawLights(screen, cam)

	// 7. Draw Doors animation on the right side
	b.drawDoors(screen, cam)
}

func (b *Bus) drawWheels(screen *ebiten.Image, cam *Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.WheelBase / 2
	rearDist := -b.WheelBase / 2
	halfWidth := b.Width/2 - 2

	wheelLen := 18.0
	wheelWidth := 7.0

	// Front steerable wheels
	frontAngle := b.Heading + b.SteeringAngle
	cosF := math.Cos(frontAngle)
	sinF := math.Sin(frontAngle)

	flPos := Vec2{
		X: b.Pos.X + cosH*frontDist - perpX*halfWidth,
		Y: b.Pos.Y + sinH*frontDist - perpY*halfWidth,
	}
	frPos := Vec2{
		X: b.Pos.X + cosH*frontDist + perpX*halfWidth,
		Y: b.Pos.Y + sinH*frontDist + perpY*halfWidth,
	}

	drawRotatedBox(screen, cam, flPos, cosF, sinF, wheelLen, wheelWidth, RGBA(35, 38, 45, 255))
	drawRotatedBox(screen, cam, frPos, cosF, sinF, wheelLen, wheelWidth, RGBA(35, 38, 45, 255))

	// Rear dual wheels (fixed heading)
	rlPos := Vec2{
		X: b.Pos.X + cosH*rearDist - perpX*halfWidth,
		Y: b.Pos.Y + sinH*rearDist - perpY*halfWidth,
	}
	rrPos := Vec2{
		X: b.Pos.X + cosH*rearDist + perpX*halfWidth,
		Y: b.Pos.Y + sinH*rearDist + perpY*halfWidth,
	}
	drawRotatedBox(screen, cam, rlPos, cosH, sinH, wheelLen, wheelWidth, RGBA(30, 32, 38, 255))
	drawRotatedBox(screen, cam, rrPos, cosH, sinH, wheelLen, wheelWidth, RGBA(30, 32, 38, 255))
}

func (b *Bus) drawBusRoof(screen *ebiten.Image, cam *Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)

	// Dark glass roof panel
	roofPos := b.Pos.Add(Vec2{X: -cosH * 2, Y: -sinH * 2})
	drawRotatedBox(screen, cam, roofPos, cosH, sinH, b.Length-24, b.Width-10, RGBA(15, 23, 42, 230))

	// Front Windshield (curved gradient glass look)
	windshieldPos := b.Pos.Add(Vec2{X: cosH * (b.Length/2 - 12), Y: sinH * (b.Length/2 - 12)})
	drawRotatedBox(screen, cam, windshieldPos, cosH, sinH, 12, b.Width-8, RGBA(56, 189, 248, 220))

	// AC Unit / Roof Ventilation pods
	acPos := b.Pos.Add(Vec2{X: -cosH * 16, Y: -sinH * 16})
	drawRotatedBox(screen, cam, acPos, cosH, sinH, 24, 18, RGBA(241, 245, 249, 255))
	// AC vents lines
	ventColor := RGBA(148, 163, 184, 255)
	for i := -8; i <= 8; i += 4 {
		p := acPos.Add(Vec2{X: cosH * float64(i), Y: sinH * float64(i)})
		drawRotatedBox(screen, cam, p, cosH, sinH, 2, 14, ventColor)
	}

	// Bus Number/Route display on front forehead
	destPos := b.Pos.Add(Vec2{X: cosH * (b.Length/2 - 5), Y: sinH * (b.Length/2 - 5)})
	drawRotatedBox(screen, cam, destPos, cosH, sinH, 4, 20, RGBA(250, 204, 21, 255)) // Glowing Amber LED banner
}

func (b *Bus) drawLights(screen *ebiten.Image, cam *Camera) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	frontDist := b.Length / 2
	rearDist := -b.Length / 2
	lightOffset := b.Width/2 - 5

	// Headlights (Front Left & Right)
	hlPos := Vec2{X: b.Pos.X + cosH*frontDist - perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist - perpY*lightOffset}
	hrPos := Vec2{X: b.Pos.X + cosH*frontDist + perpX*lightOffset, Y: b.Pos.Y + sinH*frontDist + perpY*lightOffset}

	drawRotatedBox(screen, cam, hlPos, cosH, sinH, 4, 6, RGBA(254, 240, 138, 255))
	drawRotatedBox(screen, cam, hrPos, cosH, sinH, 4, 6, RGBA(254, 240, 138, 255))

	// Headlight Beam cone forward
	b.drawHeadlightBeam(screen, cam, hlPos)
	b.drawHeadlightBeam(screen, cam, hrPos)

	// Taillights / Brake lights (Rear Left & Right)
	tlPos := Vec2{X: b.Pos.X + cosH*rearDist - perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist - perpY*lightOffset}
	trPos := Vec2{X: b.Pos.X + cosH*rearDist + perpX*lightOffset, Y: b.Pos.Y + sinH*rearDist + perpY*lightOffset}

	tailColor := RGBA(153, 27, 27, 255) // Dim red
	if b.IsBraking || b.IsHandbraking {
		tailColor = RGBA(239, 68, 68, 255) // Bright braking red
	} else if b.IsReversing {
		tailColor = RGBA(255, 255, 255, 255) // White reverse lights
	}

	drawRotatedBox(screen, cam, tlPos, cosH, sinH, 4, 6, tailColor)
	drawRotatedBox(screen, cam, trPos, cosH, sinH, 4, 6, tailColor)
}

func (b *Bus) drawHeadlightBeam(screen *ebiten.Image, cam *Camera, startPos Vec2) {
	beamLen := 120.0
	beamSpread := 24.0
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	p1 := cam.WorldToScreen(startPos)
	endCenter := startPos.Add(Vec2{X: cosH * beamLen, Y: sinH * beamLen})
	p2 := cam.WorldToScreen(endCenter.Add(Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))
	p3 := cam.WorldToScreen(endCenter.Sub(Vec2{X: perpX * beamSpread, Y: perpY * beamSpread}))

	// Soft semi-transparent light cone
	beamColor := RGBA(254, 240, 138, 25)
	vector.DrawFilledCircle(screen, float32(p1.X), float32(p1.Y), float32(6*cam.Zoom), RGBA(254, 240, 138, 80), true)

	// Draw lines for beam cone edges
	vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y), float32(1.5*cam.Zoom), beamColor, true)
	vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p3.X), float32(p3.Y), float32(1.5*cam.Zoom), beamColor, true)
}

func (b *Bus) drawDoors(screen *ebiten.Image, cam *Camera) {
	// Doors are on the passenger side (Right side: +perp)
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	doorOffset := b.Width/2 - 1
	// Front door & Middle door
	fDoorPos := Vec2{X: b.Pos.X + cosH*24 + perpX*doorOffset, Y: b.Pos.Y + sinH*24 + perpY*doorOffset}
	mDoorPos := Vec2{X: b.Pos.X - cosH*10 + perpX*doorOffset, Y: b.Pos.Y - sinH*10 + perpY*doorOffset}

	doorColor := RGBA(15, 23, 42, 255)
	if b.DoorProgress > 0.1 {
		// Open door indicator glow
		doorColor = RGBA(34, 197, 94, uint8(b.DoorProgress*255))
	}

	drawRotatedBox(screen, cam, fDoorPos, cosH, sinH, 14, 3, doorColor)
	drawRotatedBox(screen, cam, mDoorPos, cosH, sinH, 14, 3, doorColor)
}

func drawBusBody(screen *ebiten.Image, cam *Camera, center Vec2, heading, length, width float64, c color.RGBA, rounded bool) {
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)
	drawRotatedBox(screen, cam, center, cosH, sinH, length, width, c)

	if rounded {
		// Front aerodynamic bumper
		frontCenter := center.Add(Vec2{X: cosH * (length/2 - 2), Y: sinH * (length/2 - 2)})
		sc := cam.WorldToScreen(frontCenter)
		radius := float32((width / 2) * cam.Zoom)
		vector.DrawFilledCircle(screen, float32(sc.X), float32(sc.Y), radius, c, true)
	}
}

func drawRotatedBox(screen *ebiten.Image, cam *Camera, center Vec2, cosH, sinH, len, wid float64, c color.RGBA) {
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

	// Render quad as 2 triangles using vertices
	vs := []ebiten.Vertex{
		{DstX: float32(s1.X), DstY: float32(s1.Y), ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255},
		{DstX: float32(s2.X), DstY: float32(s2.Y), ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255},
		{DstX: float32(s3.X), DstY: float32(s3.Y), ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255},
		{DstX: float32(s4.X), DstY: float32(s4.Y), ColorR: float32(c.R) / 255, ColorG: float32(c.G) / 255, ColorB: float32(c.B) / 255, ColorA: float32(c.A) / 255},
	}
	indices := []uint16{0, 1, 2, 0, 2, 3}

	whiteImg := getWhiteSubImage()
	screen.DrawTriangles(vs, indices, whiteImg, &ebiten.DrawTrianglesOptions{})
}

var whiteImage *ebiten.Image

func getWhiteSubImage() *ebiten.Image {
	if whiteImage == nil {
		whiteImage = ebiten.NewImage(3, 3)
		whiteImage.Fill(color.White)
	}
	return whiteImage
}
