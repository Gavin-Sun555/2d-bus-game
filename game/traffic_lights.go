package game

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type LightSignal int

const (
	SignalRed LightSignal = iota
	SignalYellow
	SignalGreen
)

type LightPhase int

const (
	PhaseEWGreen LightPhase = iota
	PhaseEWYellow
	PhaseEWLeftGreen
	PhaseEWLeftYellow
	PhaseNSGreen
	PhaseNSYellow
	PhaseNSLeftGreen
	PhaseNSLeftYellow
)

// TrafficIntersection manages signal phases and stop lines for an intersection
type TrafficIntersection struct {
	ID       int
	Center   Vec2
	Phase    LightPhase
	Timer    float64
	Cooldown float64 // Cooldown for player violation detection
	IsMajor  bool    // true for major avenue intersections with dedicated left-turn lanes & signals

	// Durations
	GreenDuration     float64
	YellowDuration    float64
	LeftGreenDuration float64

	// Player intersection traversal tracking
	PlayerInside              bool
	PlayerCrossedBeforeYellow bool
	PlayerCrossedOnRed        bool
	PlayerTurnedRight         bool // true if executed a lawful right turn (exempt from red light fines)
	PlayerTurnedLeft          bool // true if executed a lawful left turn (exempt from through red light fines if left arrow is green)
	PlayerEntryLane           int  // 0 = inner/left-turn lane, 1 = outer/straight-right lane
	PlayerEntryHeading        float64
	PlayerEntryPos            Vec2
	PlayerEntryIsEW           bool // true if vehicle entered along East-West corridor
	PassageCompleted          bool // true once vehicle reached exit/completed traversal, prevents cross-street re-entry
	FinedThisPassage          bool
}

// RoadCamera represents a traffic surveillance camera on straight mid-block road segments
type RoadCamera struct {
	ID         int
	Pos        Vec2
	RoadName   string
	IsEW       bool    // true for East-West road, false for North-South road
	RoadCoord  float64 // Centerline coordinate: Y for EW road, X for NS road
	Cooldown   float64 // Fine cooldown
	FlashTimer float64 // Camera strobe flash visual timer
}

// TrafficLightSystem coordinates all city intersections and surveillance cameras
type TrafficLightSystem struct {
	Intersections []*TrafficIntersection
	RoadCameras   []*RoadCamera
}

func NewTrafficLightSystem() *TrafficLightSystem {
	tls := &TrafficLightSystem{
		Intersections: make([]*TrafficIntersection, 0),
		RoadCameras: []*RoadCamera{
			{ID: 0, Pos: Vec2{X: 450, Y: 0}, RoadName: "中央主干大道 (东段)", IsEW: true, RoadCoord: 0},
			{ID: 1, Pos: Vec2{X: -450, Y: 0}, RoadName: "中央主干大道 (西段)", IsEW: true, RoadCoord: 0},
			{ID: 2, Pos: Vec2{X: 0, Y: -300}, RoadName: "城市中轴大道 (北段)", IsEW: false, RoadCoord: 0},
			{ID: 3, Pos: Vec2{X: 0, Y: 300}, RoadName: "城市中轴大道 (南段)", IsEW: false, RoadCoord: 0},
			{ID: 4, Pos: Vec2{X: 450, Y: -1200}, RoadName: "北环快速路", IsEW: true, RoadCoord: -1200},
			{ID: 5, Pos: Vec2{X: -450, Y: 1200}, RoadName: "南环快速路", IsEW: true, RoadCoord: 1200},
		},
	}

	xs := []float64{-1800, -900, 0, 900, 1800}
	ys := []float64{-1200, -600, 0, 600, 1200}

	id := 0
	for iy, y := range ys {
		for ix, x := range xs {
			// Major intersections are along 4-lane avenues: North Ring, Central Blvd, South Ring, and Central Ave
			isMajor := (y == -1200 || y == 0 || y == 1200 || x == 0)

			inter := &TrafficIntersection{
				ID:                id,
				Center:            Vec2{X: x, Y: y},
				IsMajor:           isMajor,
				GreenDuration:     8.0,
				YellowDuration:    2.5,
				LeftGreenDuration: 5.5,
				Cooldown:          0,
			}

			if isMajor {
				// 8-phase cycle for major intersections: (8.0 + 2.5 + 5.5 + 2.5) * 2 = 37.0s
				offsetTimer := math.Mod(float64(ix*6+iy*7), 37.0)
				t := offsetTimer
				switch {
				case t < 8.0:
					inter.Phase = PhaseEWGreen
					inter.Timer = t
				case t < 10.5:
					inter.Phase = PhaseEWYellow
					inter.Timer = t - 8.0
				case t < 16.0:
					inter.Phase = PhaseEWLeftGreen
					inter.Timer = t - 10.5
				case t < 18.5:
					inter.Phase = PhaseEWLeftYellow
					inter.Timer = t - 16.0
				case t < 26.5:
					inter.Phase = PhaseNSGreen
					inter.Timer = t - 18.5
				case t < 29.0:
					inter.Phase = PhaseNSYellow
					inter.Timer = t - 26.5
				case t < 34.5:
					inter.Phase = PhaseNSLeftGreen
					inter.Timer = t - 29.0
				default:
					inter.Phase = PhaseNSLeftYellow
					inter.Timer = t - 34.5
				}
			} else {
				// Standard 4-phase cycle for minor streets
				offsetTimer := math.Mod(float64(ix*5+iy*7), 26.0)
				t := offsetTimer
				switch {
				case t < 10.0:
					inter.Phase = PhaseEWGreen
					inter.Timer = t
				case t < 13.0:
					inter.Phase = PhaseEWYellow
					inter.Timer = t - 10.0
				case t < 23.0:
					inter.Phase = PhaseNSGreen
					inter.Timer = t - 13.0
				default:
					inter.Phase = PhaseNSYellow
					inter.Timer = t - 23.0
				}
			}

			tls.Intersections = append(tls.Intersections, inter)
			id++
		}
	}

	return tls
}

func (tls *TrafficLightSystem) Update(dt float64) {
	for _, inter := range tls.Intersections {
		inter.Timer += dt
		if inter.Cooldown > 0 {
			inter.Cooldown -= dt
		}

		if inter.IsMajor {
			switch inter.Phase {
			case PhaseEWGreen:
				if inter.Timer >= inter.GreenDuration {
					inter.Phase = PhaseEWYellow
					inter.Timer = 0
				}
			case PhaseEWYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseEWLeftGreen
					inter.Timer = 0
				}
			case PhaseEWLeftGreen:
				if inter.Timer >= inter.LeftGreenDuration {
					inter.Phase = PhaseEWLeftYellow
					inter.Timer = 0
				}
			case PhaseEWLeftYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseNSGreen
					inter.Timer = 0
				}
			case PhaseNSGreen:
				if inter.Timer >= inter.GreenDuration {
					inter.Phase = PhaseNSYellow
					inter.Timer = 0
				}
			case PhaseNSYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseNSLeftGreen
					inter.Timer = 0
				}
			case PhaseNSLeftGreen:
				if inter.Timer >= inter.LeftGreenDuration {
					inter.Phase = PhaseNSLeftYellow
					inter.Timer = 0
				}
			case PhaseNSLeftYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseEWGreen
					inter.Timer = 0
				}
			default:
				inter.Phase = PhaseEWGreen
				inter.Timer = 0
			}
		} else {
			switch inter.Phase {
			case PhaseEWGreen:
				if inter.Timer >= inter.GreenDuration {
					inter.Phase = PhaseEWYellow
					inter.Timer = 0
				}
			case PhaseEWYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseNSGreen
					inter.Timer = 0
				}
			case PhaseNSGreen:
				if inter.Timer >= inter.GreenDuration {
					inter.Phase = PhaseNSYellow
					inter.Timer = 0
				}
			case PhaseNSYellow:
				if inter.Timer >= inter.YellowDuration {
					inter.Phase = PhaseEWGreen
					inter.Timer = 0
				}
			default:
				inter.Phase = PhaseEWGreen
				inter.Timer = 0
			}
		}
	}

	for _, cam := range tls.RoadCameras {
		if cam.Cooldown > 0 {
			cam.Cooldown -= dt
		}
		if cam.FlashTimer > 0 {
			cam.FlashTimer -= dt
		}
	}
}

// GetDetailedSignals returns: (ewThrough, ewLeft, nsThrough, nsLeft)
func (inter *TrafficIntersection) GetDetailedSignals() (ewThrough, ewLeft, nsThrough, nsLeft LightSignal) {
	if !inter.IsMajor {
		// Minor intersection: permissive left turn (shares through light)
		switch inter.Phase {
		case PhaseEWGreen:
			return SignalGreen, SignalGreen, SignalRed, SignalRed
		case PhaseEWYellow:
			return SignalYellow, SignalYellow, SignalRed, SignalRed
		case PhaseNSGreen:
			return SignalRed, SignalRed, SignalGreen, SignalGreen
		case PhaseNSYellow:
			return SignalRed, SignalRed, SignalYellow, SignalYellow
		default:
			return SignalRed, SignalRed, SignalRed, SignalRed
		}
	}

	// Major intersection with dedicated left-turn phases:
	switch inter.Phase {
	case PhaseEWGreen:
		return SignalGreen, SignalRed, SignalRed, SignalRed
	case PhaseEWYellow:
		return SignalYellow, SignalRed, SignalRed, SignalRed
	case PhaseEWLeftGreen:
		return SignalRed, SignalGreen, SignalRed, SignalRed
	case PhaseEWLeftYellow:
		return SignalRed, SignalYellow, SignalRed, SignalRed
	case PhaseNSGreen:
		return SignalRed, SignalRed, SignalGreen, SignalRed
	case PhaseNSYellow:
		return SignalRed, SignalRed, SignalYellow, SignalRed
	case PhaseNSLeftGreen:
		return SignalRed, SignalRed, SignalRed, SignalGreen
	case PhaseNSLeftYellow:
		return SignalRed, SignalRed, SignalRed, SignalYellow
	default:
		return SignalRed, SignalRed, SignalRed, SignalRed
	}
}

// GetSignals returns the primary through signal states for (East-West, North-South)
func (inter *TrafficIntersection) GetSignals() (ew LightSignal, ns LightSignal) {
	ewThru, _, nsThru, _ := inter.GetDetailedSignals()
	return ewThru, nsThru
}

// GetSignalForApproach returns the appropriate through and left-turn signals for vehicle's heading
func (inter *TrafficIntersection) GetSignalForApproach(heading float64) (throughSig LightSignal, leftSig LightSignal) {
	ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
	cosH := math.Abs(math.Cos(heading))
	sinH := math.Abs(math.Sin(heading))
	if cosH > sinH {
		return ewThru, ewL
	}
	return nsThru, nsL
}

// GetSignalForHeading returns the traffic signal for a vehicle facing heading (optional isLeft flag)
func (inter *TrafficIntersection) GetSignalForHeading(heading float64, isLeft ...bool) LightSignal {
	thru, l := inter.GetSignalForApproach(heading)
	if len(isLeft) > 0 && isLeft[0] {
		return l
	}
	return thru
}

// CheckPlayerViolation checks if player's bus is running a red light.
// Core rule: As long as the bus crosses the stop line before yellow (during green) or during yellow,
// it has lawful right of way and will not be fined even if the light turns yellow or red while traversing.
func (tls *TrafficLightSystem) CheckPlayerViolation(bus *Bus, laneWidth ...float64) (violated bool, fineNotice string) {
	lw := 52.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}
	stopLineDist := lw*1.6 + 18.0

	halfLen := bus.Length * 0.5
	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	frontPos := Vec2{
		X: bus.Pos.X + cosH*halfLen,
		Y: bus.Pos.Y + sinH*halfLen,
	}
	rearPos := Vec2{
		X: bus.Pos.X - cosH*halfLen,
		Y: bus.Pos.Y - sinH*halfLen,
	}

	leadingPos := frontPos
	trailingPos := rearPos
	if bus.Speed < -1.0 {
		leadingPos = rearPos
		trailingPos = frontPos
	}

	for _, inter := range tls.Intersections {
		dist := bus.Pos.Distance(inter.Center)
		maxZone := stopLineDist + bus.Length + 50.0

		// If bus is outside this intersection zone, reset tracking for this intersection
		if dist > maxZone {
			if inter.PlayerInside || inter.PassageCompleted {
				inter.PlayerInside = false
				inter.PassageCompleted = false
				inter.PlayerCrossedBeforeYellow = false
				inter.PlayerCrossedOnRed = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false
				inter.FinedThisPassage = false
			}
			continue
		}

		// Effective moving heading of vehicle
		effHeading := bus.Heading
		if bus.Speed < -1.0 {
			effHeading = NormalizeAngle(bus.Heading + math.Pi)
		}

		// Direction vectors based on current heading (or entry heading if already inside)
		refHeading := effHeading
		if inter.PlayerInside {
			refHeading = inter.PlayerEntryHeading
		}

		uFwd := Vec2{X: math.Cos(refHeading), Y: math.Sin(refHeading)}
		vRight := Vec2{X: -math.Sin(refHeading), Y: math.Cos(refHeading)}

		// Projected position along forward entry axis (origin = intersection center)
		rLead := leadingPos.Sub(inter.Center)
		sLead := rLead.X*uFwd.X + rLead.Y*uFwd.Y

		rTrail := trailingPos.Sub(inter.Center)
		sTrail := rTrail.X*uFwd.X + rTrail.Y*uFwd.Y

		// Motion vector towards intersection center:
		toCenter := inter.Center.Sub(bus.Pos)
		fwdDir := Vec2{X: math.Cos(effHeading), Y: math.Sin(effHeading)}
		movingTowardsCenter := toCenter.X*fwdDir.X + toCenter.Y*fwdDir.Y

		// Lateral offset from approach road corridor centerline (checked only before entry)
		latDist := math.Abs(rLead.X*vRight.X + rLead.Y*vRight.Y)
		roadHalfWidth := lw*2.5 + 35.0
		if !inter.PlayerInside && latDist > roadHalfWidth {
			continue
		}

		isPastStopLine := sLead >= -stopLineDist
		isPastExit := sTrail > stopLineDist

		// If the vehicle is before the stop line approaching the intersection,
		// reset PassageCompleted so it can be monitored for this new approach:
		if !isPastStopLine {
			inter.PassageCompleted = false
		}

		// Intersection entry tracking
		if !inter.PlayerInside && !inter.PassageCompleted {
			// A vehicle can only enter if it is moving towards the intersection center.
			// Departing/exiting vehicles (movingTowardsCenter < -5) cannot trigger re-entry!
			if movingTowardsCenter <= -5.0 && sLead > 0 {
				continue
			}

			if isPastStopLine && !isPastExit {
				// Crossing event: Leading bumper crosses stop line into intersection
				inter.PlayerInside = true
				inter.FinedThisPassage = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false

				inter.PlayerEntryHeading = effHeading
				inter.PlayerEntryPos = bus.Pos

				// Robust approach corridor determination:
				// If heading is clearly aligned (|cos| > 0.707 or |sin| > 0.707), use heading.
				// If vehicle is already steering into a turn at the stop line, use approach position!
				cosEntry := math.Abs(math.Cos(effHeading))
				sinEntry := math.Abs(math.Sin(effHeading))
				relPos := bus.Pos.Sub(inter.Center)
				isEW := true
				if cosEntry > 0.707 {
					isEW = true
				} else if sinEntry > 0.707 {
					isEW = false
				} else {
					isEW = math.Abs(relPos.X) >= math.Abs(relPos.Y)
				}
				inter.PlayerEntryIsEW = isEW

				// Determine entry lane on multi-lane avenues:
				// 0 = inner (dedicated left-turn lane), 1 = outer (straight/right lane)
				// Lenient detection: Bus is large. If vehicle is in left lane, slightly straddling
				// the straight lane (distFromCenter < lw*1.45), cutting corners near the centerline,
				// or actively steering left at entry (SteeringAngle < -0.015), treat as left-turn lane (0)!
				inter.PlayerEntryLane = 1
				if inter.IsMajor {
					distFromCenter := 0.0
					if isEW {
						distFromCenter = math.Abs(bus.Pos.Y - inter.Center.Y)
					} else {
						distFromCenter = math.Abs(bus.Pos.X - inter.Center.X)
					}
					if distFromCenter < lw*1.45 || bus.SteeringAngle < -0.015 {
						inter.PlayerEntryLane = 0
					}
				}

				// Check initial steering intent at entry
				if bus.SteeringAngle > 0.02 {
					inter.PlayerTurnedRight = true
				}
				if bus.SteeringAngle < -0.015 {
					inter.PlayerTurnedLeft = true
				}

				// Retrieve signals of the approach corridor the vehicle entered from:
				ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
				thruSig := nsThru
				leftSig := nsL
				if inter.PlayerEntryIsEW {
					thruSig = ewThru
					leftSig = ewL
				}

				movementSig := thruSig
				if (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) && inter.IsMajor {
					movementSig = leftSig
				}

				// Lawful entry evaluation:
				// 1. Right turn is ALWAYS lawful and exempt from red light violations!
				// 2. Left turn when left signal is Green/Yellow gives lawful right-of-way!
				// 3. Green or Yellow entry gives lawful right-of-way!
				if inter.PlayerTurnedRight || movementSig == SignalGreen || movementSig == SignalYellow {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else if inter.IsMajor && (leftSig == SignalGreen || leftSig == SignalYellow) && (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else if !inter.IsMajor && (thruSig == SignalGreen || thruSig == SignalYellow) {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else {
					inter.PlayerCrossedBeforeYellow = false
					inter.PlayerCrossedOnRed = true
				}
			}
		}

		dHead := NormalizeAngle(effHeading - inter.PlayerEntryHeading)

		// Relative lateral movement towards the cross-streets:
		entryVRight := Vec2{X: -math.Sin(inter.PlayerEntryHeading), Y: math.Cos(inter.PlayerEntryHeading)}
		disp := bus.Pos.Sub(inter.PlayerEntryPos)
		latRight := disp.X*entryVRight.X + disp.Y*entryVRight.Y
		latLeft := -latRight

		// Retrieve signals of the approach corridor entered:
		ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
		thruSig := nsThru
		leftSig := nsL
		if inter.PlayerEntryIsEW {
			thruSig = ewThru
			leftSig = ewL
		}

		// Right-Turn Detection:
		// Right turn is completely lawful and never fined!
		// If steering right, heading turned clockwise, or drifted right:
		if bus.SteeringAngle > 0.02 || dHead > 0.05 || (latRight > 5.0 && dHead > -0.05) {
			inter.PlayerTurnedRight = true
			inter.PlayerCrossedOnRed = false
			inter.PlayerCrossedBeforeYellow = true
		}

		// Left-Turn Detection (Dynamic & Lenient):
		// If steering left, heading turned counter-clockwise, or drifted left towards cross street:
		if bus.SteeringAngle < -0.015 || dHead < -0.05 || (latLeft > 5.0 && dHead < 0.05) {
			inter.PlayerTurnedLeft = true
			turnSig := thruSig
			if inter.IsMajor {
				turnSig = leftSig
			}
			// If the left turn signal is Green or Yellow, the vehicle has lawful right-of-way!
			if turnSig == SignalGreen || turnSig == SignalYellow || (!inter.IsMajor && (thruSig == SignalGreen || thruSig == SignalYellow)) {
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			}
		}

		// 1. Red Light Violation Check (闯红灯抓拍 - 对应方向车道后轮过线判定):
		// Core rules (宽松友好设计):
		// - 右转永远合法，不抓拍 (PlayerTurnedRight).
		// - 绿灯/黄灯进入路口合法通行，绝不抓拍 (PlayerCrossedBeforeYellow).
		// - 绿灯左右转弯在路口清空或出路口时绝不误判为闯红灯 (PassageCompleted).
		// - 只有当对应进道方向为红灯、非合法转向、且后轮越过停止线 (sTrail >= -stopLineDist) 时才抓拍。
		if inter.PlayerInside && inter.PlayerCrossedOnRed && !inter.PlayerTurnedRight && !inter.PassageCompleted && !inter.FinedThisPassage {
			reqSig := thruSig
			if (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) && inter.IsMajor {
				reqSig = leftSig
			}

			// Left turn immunity if left signal is currently Green or Yellow:
			if inter.PlayerTurnedLeft && (leftSig == SignalGreen || leftSig == SignalYellow || (!inter.IsMajor && (thruSig == SignalGreen || thruSig == SignalYellow))) {
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			} else if reqSig == SignalGreen || reqSig == SignalYellow || (!inter.IsMajor && (thruSig == SignalGreen || thruSig == SignalYellow)) {
				// If signal turned Green/Yellow before rear wheels crossed, forgive!
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			} else if reqSig == SignalRed && math.Abs(bus.Speed) >= 10.0 && inter.Cooldown <= 0 {
				// 后轮过线判定:
				if sTrail >= -stopLineDist {
					inter.FinedThisPassage = true
					inter.Cooldown = 6.0
					return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 闯红灯（后轮越过停止线）！", inter.ID+1)
				}
			}
		}

		// 2. Lane Discipline Violation Check (不按导向车道行驶抓拍 - 到达对侧正式判定):
		// Core rule: 路口内部绝不误判罚款。必须等车辆到达对侧线时，才能确定是否没有左转或直行或右转再行罚款！
		if inter.PlayerInside && inter.IsMajor && !inter.FinedThisPassage && inter.Cooldown <= 0 {
			vLeft := Vec2{X: math.Sin(inter.PlayerEntryHeading), Y: -math.Cos(inter.PlayerEntryHeading)}
			sLeft := rLead.X*vLeft.X + rLead.Y*vLeft.Y
			sRight := rLead.X*vRight.X + rLead.Y*vRight.Y

			// Determine if vehicle has reached an exit on the opposite side:
			reachedStraightExit := sLead >= stopLineDist || isPastExit
			reachedLeftExit := sLeft >= stopLineDist-15.0 && dHead < -0.35
			reachedRightExit := sRight >= stopLineDist-15.0 && dHead > 0.35

			if reachedStraightExit || reachedLeftExit || reachedRightExit {
				// Vehicle has reached the opposite side! Now evaluate whether the vehicle obeyed the lane arrow:
				// Case A: Left-Turn Lane (0) must turn left!
				if inter.PlayerEntryLane == 0 {
					// Only fine if vehicle truly went straight without turning left
					if reachedStraightExit && math.Abs(dHead) <= 0.35 && !inter.PlayerTurnedLeft {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（左转车道违规直行）！", inter.ID+1)
					} else if (reachedRightExit || dHead > 0.40) && !inter.PlayerTurnedLeft {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（左转车道违规右转）！", inter.ID+1)
					}
				}

				// Case B: Straight/Right Lane (1) must NOT turn left!
				// 宽松判定：如果左转灯为绿灯/黄灯，或者只是借道转弯/压线的大巴车，不误判为违规左转。
				// 只有在左转为红灯（禁止左转）时从直行车道左转，才判定违规！
				if inter.PlayerEntryLane == 1 {
					if (reachedLeftExit || dHead < -0.35) && leftSig != SignalGreen && leftSig != SignalYellow {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（直行车道违规左转）！", inter.ID+1)
					}
				}

				// If legally exited, mark passage completed and clear inside tracking
				inter.PassageCompleted = true
				inter.PlayerInside = false
				inter.PlayerCrossedBeforeYellow = true
				inter.PlayerCrossedOnRed = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false
				inter.FinedThisPassage = false
			}
		}

		// Exit clearing if vehicle cleared the intersection (for both major and minor intersections)
		if inter.PlayerInside && isPastExit {
			inter.PassageCompleted = true
			inter.PlayerInside = false
			inter.PlayerCrossedBeforeYellow = true
			inter.PlayerCrossedOnRed = false
			inter.PlayerTurnedRight = false
			inter.PlayerTurnedLeft = false
			inter.FinedThisPassage = false
		}
	}

	return false, ""
}

// CheckStopForVehicle checks whether an approaching vehicle should stop for a red/yellow light,
// and returns signed distance to the stop line along the vehicle's heading.
// If distToStopLine <= -12 (vehicle already past stop line into intersection), it will NOT stop,
// ensuring the vehicle clears the intersection and does not get stuck in the middle of the road.
func (tls *TrafficLightSystem) CheckStopForVehicle(pos Vec2, heading float64, lookAhead float64, laneWidth float64, isTurningLeft ...bool) (shouldStop bool, distToStopLine float64) {
	stopLineDist := laneWidth*1.6 + 18.0
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	for _, inter := range tls.Intersections {
		toInter := inter.Center.Sub(pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

		// Must be on the approaching corridor heading towards the intersection
		if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookAhead+60 && lateralDist < laneWidth*2.5 {
			distToLine := forwardDist - stopLineDist

			// If already at or past the stop line into the intersection (distToLine <= 0):
			// NEVER stop! Must proceed and clear intersection to avoid blocking zebra crossings!
			if distToLine <= 0 {
				continue
			}

			// Determine which signal to respect (Left-turn vs Through):
			wantLeft := len(isTurningLeft) > 0 && isTurningLeft[0]
			if !wantLeft && inter.IsMajor {
				// Check if vehicle is in the inner lane (dedicated left-turn lane)
				if math.Abs(cosH) > math.Abs(sinH) {
					// EW avenue
					dy := math.Abs(pos.Y - inter.Center.Y)
					if dy < laneWidth*1.0 {
						wantLeft = true
					}
				} else {
					// NS avenue
					dx := math.Abs(pos.X - inter.Center.X)
					if dx < laneWidth*1.0 {
						wantLeft = true
					}
				}
			}

			sig := inter.GetSignalForHeading(heading, wantLeft)

			// If approaching before the stop line within lookAhead
			if distToLine > 0 && distToLine < lookAhead {
				if sig == SignalRed || sig == SignalYellow {
					return true, distToLine
				}
			}
		}
	}
	return false, 999.0
}

// ShouldVehicleStop checks if an approaching vehicle should stop before the intersection stop line
func (tls *TrafficLightSystem) ShouldVehicleStop(pos Vec2, heading float64, lookAhead float64) bool {
	stop, _ := tls.CheckStopForVehicle(pos, heading, lookAhead, 42.0)
	return stop
}

// GetApproachingSignalInfo returns signal info for a vehicle approaching an intersection within lookAhead
func (tls *TrafficLightSystem) GetApproachingSignalInfo(pos Vec2, heading float64, laneWidth float64) (hasInter bool, thruSig LightSignal, leftSig LightSignal, isMajor bool, remainingSec float64, distToLine float64) {
	stopLineDist := laneWidth*1.6 + 18.0
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	for _, inter := range tls.Intersections {
		toInter := inter.Center.Sub(pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

		if forwardDist > 0 && forwardDist < stopLineDist+260 && lateralDist < laneWidth*2.5 {
			dist := forwardDist - stopLineDist
			th, lf := inter.GetSignalForApproach(heading)
			rem := 0.0
			switch inter.Phase {
			case PhaseEWGreen, PhaseNSGreen:
				rem = math.Max(0, inter.GreenDuration-inter.Timer)
			case PhaseEWYellow, PhaseNSYellow:
				rem = math.Max(0, inter.YellowDuration-inter.Timer)
			case PhaseEWLeftGreen, PhaseNSLeftGreen:
				rem = math.Max(0, inter.LeftGreenDuration-inter.Timer)
			case PhaseEWLeftYellow, PhaseNSLeftYellow:
				rem = math.Max(0, inter.YellowDuration-inter.Timer)
			}
			return true, th, lf, inter.IsMajor, rem, dist
		}
	}
	return false, SignalGreen, SignalRed, false, 0, 999.0
}

// Draw renders traffic lights and stop lines in the world with dynamic lane width
func (tls *TrafficLightSystem) Draw(screen *ebiten.Image, cam *Camera, laneWidth float64) {
	for _, inter := range tls.Intersections {
		// 1. Draw Stop Lines on 4 approaches scaled to current lane width
		drawIntersectionStopLines(screen, cam, inter.Center, laneWidth)

		ewThru, ewLeft, nsThru, nsLeft := inter.GetDetailedSignals()
		stopLineDist := laneWidth*1.6 + 18.0
		crosswalkDist := stopLineDist - 25.0

		isEW4Lane := (inter.Center.Y == -1200 || inter.Center.Y == 0 || inter.Center.Y == 1200)
		isNS4Lane := (inter.Center.X == 0)

		// 2. Draw traffic lights directly mounted at each of the 4 zebra crossings:
		// West Approach (Eastbound traffic, oncoming from -X to +X, right curb on +Y side):
		drawApproachSignal(screen, cam, inter.Center, 0, ewThru, ewLeft, isEW4Lane, laneWidth, crosswalkDist)

		// East Approach (Westbound traffic, oncoming from +X to -X, right curb on -Y side):
		drawApproachSignal(screen, cam, inter.Center, 1, ewThru, ewLeft, isEW4Lane, laneWidth, crosswalkDist)

		// North Approach (Southbound traffic, oncoming from -Y to +Y, right curb on -X side):
		drawApproachSignal(screen, cam, inter.Center, 2, nsThru, nsLeft, isNS4Lane, laneWidth, crosswalkDist)

		// South Approach (Northbound traffic, oncoming from +Y to -Y, right curb on +X side):
		drawApproachSignal(screen, cam, inter.Center, 3, nsThru, nsLeft, isNS4Lane, laneWidth, crosswalkDist)
	}

	// 3. Draw straight road surveillance camera gantries
	tls.DrawRoadCameras(screen, cam, laneWidth)
}

// CheckRoadCameraViolation checks if the player bus is driving the wrong way (逆行) near straight road surveillance cameras
func (tls *TrafficLightSystem) CheckRoadCameraViolation(bus *Bus) (violated bool, fineNotice string) {
	for _, cam := range tls.RoadCameras {
		if cam.Cooldown > 0 {
			continue
		}
		dist := bus.Pos.Distance(cam.Pos)
		if dist > 110.0 {
			continue
		}
		// Vehicle must be moving at driving speed
		if math.Abs(bus.Speed) < 15.0 {
			continue
		}

		effHeading := bus.Heading
		if bus.Speed < -1.0 {
			effHeading = NormalizeAngle(bus.Heading + math.Pi)
		}

		isWrongWay := false
		if cam.IsEW {
			// East-West road: centerline at Y = cam.RoadCoord
			// South of centerline (Y > cam.RoadCoord + 18.0): legal flow is East (heading ~ 0, cos > 0)
			// North of centerline (Y < cam.RoadCoord - 18.0): legal flow is West (heading ~ pi, cos < 0)
			if bus.Pos.Y > cam.RoadCoord+18.0 {
				if math.Cos(effHeading) < -0.35 {
					isWrongWay = true
				}
			} else if bus.Pos.Y < cam.RoadCoord-18.0 {
				if math.Cos(effHeading) > 0.35 {
					isWrongWay = true
				}
			}
		} else {
			// North-South road: centerline at X = cam.RoadCoord
			// West of centerline (X < cam.RoadCoord - 18.0): legal flow is South (heading ~ pi/2, sin > 0)
			// East of centerline (X > cam.RoadCoord + 18.0): legal flow is North (heading ~ -pi/2, sin < 0)
			if bus.Pos.X < cam.RoadCoord-18.0 {
				if math.Sin(effHeading) < -0.35 {
					isWrongWay = true
				}
			} else if bus.Pos.X > cam.RoadCoord+18.0 {
				if math.Sin(effHeading) > 0.35 {
					isWrongWay = true
				}
			}
		}

		if isWrongWay {
			cam.Cooldown = 8.0
			cam.FlashTimer = 0.5
			return true, fmt.Sprintf("[交通违章抓拍] 车辆在【%s】监控路段逆向行驶（逆行抓拍）！", cam.RoadName)
		}
	}
	return false, ""
}

// DrawRoadCameras renders surveillance gantries, cameras, pulsing LEDs, and violation flash effects
func (tls *TrafficLightSystem) DrawRoadCameras(screen *ebiten.Image, cam *Camera, laneWidth float64) {
	roadHalfWidth := laneWidth*2.0 + 12.0

	for _, rc := range tls.RoadCameras {
		sc := cam.WorldToScreen(rc.Pos)
		// View culling
		sw := float64(screen.Bounds().Dx())
		sh := float64(screen.Bounds().Dy())
		if sc.X < -150 || sc.X > sw+150 || sc.Y < -150 || sc.Y > sh+150 {
			continue
		}

		// Draw Camera Flash Flare effect when capturing a violation
		if rc.FlashTimer > 0 {
			alpha := float32(math.Min(1.0, rc.FlashTimer/0.4))
			vector.DrawFilledCircle(screen, float32(sc.X), float32(sc.Y), float32(65*cam.Zoom),
				RGBA(255, 255, 255, uint8(alpha*160)), true)
			vector.DrawFilledCircle(screen, float32(sc.X), float32(sc.Y), float32(35*cam.Zoom),
				RGBA(56, 189, 248, uint8(alpha*220)), true)
		}

		if rc.IsEW {
			// East-West road: Gantry spans across Y from (Pos.X, RoadCoord - roadHalfWidth) to (Pos.X, RoadCoord + roadHalfWidth)
			pNorth := cam.WorldToScreen(Vec2{X: rc.Pos.X, Y: rc.RoadCoord - roadHalfWidth})
			pSouth := cam.WorldToScreen(Vec2{X: rc.Pos.X, Y: rc.RoadCoord + roadHalfWidth})

			// Gantry main overhead beam
			beamThick := float32(math.Max(2.5, 4.0*cam.Zoom))
			vector.StrokeLine(screen, float32(pNorth.X), float32(pNorth.Y), float32(pSouth.X), float32(pSouth.Y),
				beamThick, RGBA(71, 85, 105, 240), true)
			vector.StrokeLine(screen, float32(pNorth.X+1.5*cam.Zoom), float32(pNorth.Y), float32(pSouth.X+1.5*cam.Zoom), float32(pSouth.Y),
				float32(math.Max(1.0, 1.5*cam.Zoom)), RGBA(148, 163, 184, 200), true)

			// Support footings with hazard stripes
			footW := float32(math.Max(4, 7*cam.Zoom))
			footH := float32(math.Max(5, 9*cam.Zoom))
			vector.DrawFilledRect(screen, float32(pNorth.X)-footW/2, float32(pNorth.Y)-footH/2, footW, footH, RGBA(234, 179, 8, 255), true)
			vector.DrawFilledRect(screen, float32(pSouth.X)-footW/2, float32(pSouth.Y)-footH/2, footW, footH, RGBA(234, 179, 8, 255), true)

			// Camera boxes mounted across the beam (South lane cam, North lane cam)
			camOffsets := []float64{-laneWidth * 1.0, laneWidth * 1.0}
			for _, off := range camOffsets {
				cp := cam.WorldToScreen(Vec2{X: rc.Pos.X, Y: rc.RoadCoord + off})
				boxW := float32(math.Max(5, 9*cam.Zoom))
				boxH := float32(math.Max(4, 7*cam.Zoom))

				// Camera housing body
				vector.DrawFilledRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, RGBA(15, 23, 42, 255), true)
				vector.StrokeRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, 1.0, RGBA(148, 163, 184, 230), true)

				// Dark camera lens
				vector.DrawFilledCircle(screen, float32(cp.X), float32(cp.Y), float32(math.Max(1.5, 2.5*cam.Zoom)), RGBA(2, 6, 23, 255), true)
				// Pulsing cyan surveillance LED
				vector.DrawFilledCircle(screen, float32(cp.X)+boxW*0.3, float32(cp.Y)-boxH*0.25, float32(math.Max(1.0, 1.8*cam.Zoom)), RGBA(56, 189, 248, 240), true)
			}

			// Road asphalt marking: "📸 电子监控"
			if cam.Zoom >= 0.75 {
				DrawText(screen, "📸 电子监控", sc.X-36*cam.Zoom, sc.Y-18*cam.Zoom, 10, RGBA(255, 255, 255, 180))
			}
		} else {
			// North-South road: Gantry spans across X from (RoadCoord - roadHalfWidth, Pos.Y) to (RoadCoord + roadHalfWidth, Pos.Y)
			pWest := cam.WorldToScreen(Vec2{X: rc.RoadCoord - roadHalfWidth, Y: rc.Pos.Y})
			pEast := cam.WorldToScreen(Vec2{X: rc.RoadCoord + roadHalfWidth, Y: rc.Pos.Y})

			beamThick := float32(math.Max(2.5, 4.0*cam.Zoom))
			vector.StrokeLine(screen, float32(pWest.X), float32(pWest.Y), float32(pEast.X), float32(pEast.Y),
				beamThick, RGBA(71, 85, 105, 240), true)
			vector.StrokeLine(screen, float32(pWest.X), float32(pWest.Y+1.5*cam.Zoom), float32(pEast.X), float32(pEast.Y+1.5*cam.Zoom),
				float32(math.Max(1.0, 1.5*cam.Zoom)), RGBA(148, 163, 184, 200), true)

			footW := float32(math.Max(5, 9*cam.Zoom))
			footH := float32(math.Max(4, 7*cam.Zoom))
			vector.DrawFilledRect(screen, float32(pWest.X)-footW/2, float32(pWest.Y)-footH/2, footW, footH, RGBA(234, 179, 8, 255), true)
			vector.DrawFilledRect(screen, float32(pEast.X)-footW/2, float32(pEast.Y)-footH/2, footW, footH, RGBA(234, 179, 8, 255), true)

			camOffsets := []float64{-laneWidth * 1.0, laneWidth * 1.0}
			for _, off := range camOffsets {
				cp := cam.WorldToScreen(Vec2{X: rc.RoadCoord + off, Y: rc.Pos.Y})
				boxW := float32(math.Max(4, 7*cam.Zoom))
				boxH := float32(math.Max(5, 9*cam.Zoom))

				vector.DrawFilledRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, RGBA(15, 23, 42, 255), true)
				vector.StrokeRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, 1.0, RGBA(148, 163, 184, 230), true)

				vector.DrawFilledCircle(screen, float32(cp.X), float32(cp.Y), float32(math.Max(1.5, 2.5*cam.Zoom)), RGBA(2, 6, 23, 255), true)
				vector.DrawFilledCircle(screen, float32(cp.X)-boxW*0.25, float32(cp.Y)+boxH*0.3, float32(math.Max(1.0, 1.8*cam.Zoom)), RGBA(56, 189, 248, 240), true)
			}

			if cam.Zoom >= 0.75 {
				DrawText(screen, "📸 电子监控", sc.X-36*cam.Zoom, sc.Y-18*cam.Zoom, 10, RGBA(255, 255, 255, 180))
			}
		}
	}
}

func drawIntersectionStopLines(screen *ebiten.Image, cam *Camera, c Vec2, laneWidth float64) {
	stopLineDist := laneWidth*1.6 + 18.0
	roadHalf := laneWidth*2.0 + 12.0 // cover full incoming width of multi-lane road

	stopLineColor := RGBA(248, 250, 252, 235) // Crisp solid white line
	lineWidth := float32(3.5 * cam.Zoom)

	// West approach stop line (heading East, incoming lane on +Y)
	w1 := cam.WorldToScreen(Vec2{X: c.X - stopLineDist, Y: c.Y + 3})
	w2 := cam.WorldToScreen(Vec2{X: c.X - stopLineDist, Y: c.Y + roadHalf})
	vector.StrokeLine(screen, float32(w1.X), float32(w1.Y), float32(w2.X), float32(w2.Y), lineWidth, stopLineColor, true)

	// East approach stop line (heading West, incoming lane on -Y)
	e1 := cam.WorldToScreen(Vec2{X: c.X + stopLineDist, Y: c.Y - 3})
	e2 := cam.WorldToScreen(Vec2{X: c.X + stopLineDist, Y: c.Y - roadHalf})
	vector.StrokeLine(screen, float32(e1.X), float32(e1.Y), float32(e2.X), float32(e2.Y), lineWidth, stopLineColor, true)

	// North approach stop line (heading South, incoming lane on -X)
	n1 := cam.WorldToScreen(Vec2{X: c.X - 3, Y: c.Y - stopLineDist})
	n2 := cam.WorldToScreen(Vec2{X: c.X - roadHalf, Y: c.Y - stopLineDist})
	vector.StrokeLine(screen, float32(n1.X), float32(n1.Y), float32(n2.X), float32(n2.Y), lineWidth, stopLineColor, true)

	// South approach stop line (heading North, incoming lane on +X)
	s1 := cam.WorldToScreen(Vec2{X: c.X + 3, Y: c.Y + stopLineDist})
	s2 := cam.WorldToScreen(Vec2{X: c.X + roadHalf, Y: c.Y + stopLineDist})
	vector.StrokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lineWidth, stopLineColor, true)
}

func drawApproachSignal(screen *ebiten.Image, cam *Camera, c Vec2, approach int, thruSig, leftSig LightSignal, is4Lane bool, laneWidth float64, crosswalkDist float64) {
	scale := cam.Zoom
	if scale < 0.20 {
		return
	}

	var curbPos Vec2
	var armEnd Vec2
	var thruPos Vec2
	var leftPos Vec2
	isHorizontalHead := true

	switch approach {
	case 0: // West approach (Eastbound, right curb on +Y side)
		curbY := c.Y + laneWidth*1.0 + 8.0
		if is4Lane {
			curbY = c.Y + laneWidth*2.0 + 12.0
		}
		curbPos = Vec2{X: c.X - crosswalkDist, Y: curbY}
		armEnd = Vec2{X: c.X - crosswalkDist, Y: c.Y + 4.0}
		thruPos = Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*1.48}
		leftPos = Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*0.52}
		isHorizontalHead = true

	case 1: // East approach (Westbound, right curb on -Y side)
		curbY := c.Y - (laneWidth*1.0 + 8.0)
		if is4Lane {
			curbY = c.Y - (laneWidth*2.0 + 12.0)
		}
		curbPos = Vec2{X: c.X + crosswalkDist, Y: curbY}
		armEnd = Vec2{X: c.X + crosswalkDist, Y: c.Y - 4.0}
		thruPos = Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*1.48}
		leftPos = Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*0.52}
		isHorizontalHead = true

	case 2: // North approach (Southbound, right curb on -X side)
		curbX := c.X - (laneWidth*1.0 + 8.0)
		if is4Lane {
			curbX = c.X - (laneWidth*2.0 + 12.0)
		}
		curbPos = Vec2{X: curbX, Y: c.Y - crosswalkDist}
		armEnd = Vec2{X: c.X - 4.0, Y: c.Y - crosswalkDist}
		thruPos = Vec2{X: c.X - laneWidth*1.48, Y: c.Y - crosswalkDist}
		leftPos = Vec2{X: c.X - laneWidth*0.52, Y: c.Y - crosswalkDist}
		isHorizontalHead = false

	case 3: // South approach (Northbound, right curb on +X side)
		curbX := c.X + (laneWidth*1.0 + 8.0)
		if is4Lane {
			curbX = c.X + (laneWidth*2.0 + 12.0)
		}
		curbPos = Vec2{X: curbX, Y: c.Y + crosswalkDist}
		armEnd = Vec2{X: c.X + 4.0, Y: c.Y + crosswalkDist}
		thruPos = Vec2{X: c.X + laneWidth*1.48, Y: c.Y + crosswalkDist}
		leftPos = Vec2{X: c.X + laneWidth*0.52, Y: c.Y + crosswalkDist}
		isHorizontalHead = false
	}

	// 1. Draw Curb Post Base on the sidewalk
	spCurb := cam.WorldToScreen(curbPos)
	baseR := float32(4.5 * scale)
	if baseR < 2.0 {
		baseR = 2.0
	}
	vector.DrawFilledCircle(screen, float32(spCurb.X), float32(spCurb.Y), baseR, RGBA(71, 85, 105, 255), true)
	vector.StrokeCircle(screen, float32(spCurb.X), float32(spCurb.Y), baseR, 1.0, RGBA(148, 163, 184, 255), true)

	if is4Lane {
		// 2. Draw Overhead Cantilever Arm spanning across the incoming lanes
		spEnd := cam.WorldToScreen(armEnd)
		armLw := float32(3.5 * scale)
		if armLw < 1.5 {
			armLw = 1.5
		}
		// Mast shadow
		vector.StrokeLine(screen, float32(spCurb.X+3), float32(spCurb.Y+3), float32(spEnd.X+3), float32(spEnd.Y+3), armLw, RGBA(2, 6, 23, 100), true)
		// Mast steel truss beam
		vector.StrokeLine(screen, float32(spCurb.X), float32(spCurb.Y), float32(spEnd.X), float32(spEnd.Y), armLw, RGBA(51, 65, 85, 240), true)
		vector.StrokeLine(screen, float32(spCurb.X), float32(spCurb.Y), float32(spEnd.X), float32(spEnd.Y), armLw*0.35, RGBA(148, 163, 184, 200), true)

		// 3. Mount Left-Turn Signal Head over Inner Lane
		drawSignalHead(screen, cam, leftPos, leftSig, true, isHorizontalHead)

		// 4. Mount Through Signal Head over Outer Lane
		drawSignalHead(screen, cam, thruPos, thruSig, false, isHorizontalHead)
	} else {
		// Minor 2-lane street: Signal head mounted directly at the curb post at the zebra crossing
		drawSignalHead(screen, cam, curbPos, thruSig, false, isHorizontalHead)
	}
}

func drawSignalHead(screen *ebiten.Image, cam *Camera, pos Vec2, currentSig LightSignal, isLeftArrow bool, isHorizontal bool) {
	sp := cam.WorldToScreen(pos)
	scale := cam.Zoom
	if scale < 0.22 {
		return
	}

	lampR := float32(2.8 * scale)
	if lampR < 1.5 {
		lampR = 1.5
	}

	var boxW, boxH float32
	if isHorizontal {
		boxW = float32(25 * scale)
		boxH = float32(11 * scale)
	} else {
		boxW = float32(11 * scale)
		boxH = float32(25 * scale)
	}

	boxX := float32(sp.X) - boxW/2
	boxY := float32(sp.Y) - boxH/2

	// Head housing frosted glass: shadow, translucent base, specular sheen & crisp glass border
	vector.DrawFilledRect(screen, boxX-1, boxY-1, boxW+2, boxH+2, RGBA(2, 6, 23, 110), true)
	vector.DrawFilledRect(screen, boxX, boxY, boxW, boxH, RGBA(15, 23, 42, 215), true)
	vector.DrawFilledRect(screen, boxX+0.5, boxY+0.5, boxW-1, boxH*0.45, RGBA(255, 255, 255, 25), true)
	vector.StrokeRect(screen, boxX, boxY, boxW, boxH, 1.0, RGBA(255, 255, 255, 75), true)

	if isHorizontal {
		cy := boxY + boxH/2
		drawSignalLamp(screen, boxX+boxW*0.20, cy, lampR, SignalRed, currentSig, isLeftArrow, scale)
		drawSignalLamp(screen, boxX+boxW*0.50, cy, lampR, SignalYellow, currentSig, isLeftArrow, scale)
		drawSignalLamp(screen, boxX+boxW*0.80, cy, lampR, SignalGreen, currentSig, isLeftArrow, scale)
	} else {
		cx := boxX + boxW/2
		drawSignalLamp(screen, cx, boxY+boxH*0.20, lampR, SignalRed, currentSig, isLeftArrow, scale)
		drawSignalLamp(screen, cx, boxY+boxH*0.50, lampR, SignalYellow, currentSig, isLeftArrow, scale)
		drawSignalLamp(screen, cx, boxY+boxH*0.80, lampR, SignalGreen, currentSig, isLeftArrow, scale)
	}
}

func drawSignalLamp(screen *ebiten.Image, cx, cy, radius float32, lampKind LightSignal, currentSig LightSignal, isLeftArrow bool, scale float64) {
	isActive := (lampKind == currentSig)

	var lampColor, glowColor color.RGBA
	switch lampKind {
	case SignalRed:
		if isActive {
			lampColor = RGBA(248, 40, 40, 255)
			glowColor = RGBA(248, 40, 40, 80)
		} else {
			lampColor = RGBA(65, 15, 15, 230)
		}
	case SignalYellow:
		if isActive {
			lampColor = RGBA(250, 204, 21, 255)
			glowColor = RGBA(250, 204, 21, 80)
		} else {
			lampColor = RGBA(65, 55, 15, 230)
		}
	case SignalGreen:
		if isActive {
			lampColor = RGBA(34, 197, 94, 255)
			glowColor = RGBA(34, 197, 94, 80)
		} else {
			lampColor = RGBA(15, 55, 25, 230)
		}
	}

	// Glowing halo for active lamp
	if isActive {
		vector.DrawFilledCircle(screen, cx, cy, radius*1.8, glowColor, true)
	}

	// Main lamp circle
	vector.DrawFilledCircle(screen, cx, cy, radius, lampColor, true)

	// Arrow glyph for directional lenses
	if scale >= 0.38 {
		arrowCol := RGBA(255, 255, 255, 240)
		if !isActive {
			arrowCol = RGBA(100, 116, 139, 120)
		}
		aLw := float32(1.2 * scale)
		if aLw < 1.0 {
			aLw = 1.0
		}

		if isLeftArrow {
			// Left arrow (⇦) inside lamp
			vector.StrokeLine(screen, cx+radius*0.4, cy, cx-radius*0.4, cy, aLw, arrowCol, true)
			vector.StrokeLine(screen, cx-radius*0.4, cy, cx-radius*0.1, cy-radius*0.4, aLw, arrowCol, true)
			vector.StrokeLine(screen, cx-radius*0.4, cy, cx-radius*0.1, cy+radius*0.4, aLw, arrowCol, true)
		} else if isActive && lampKind == SignalGreen {
			// Straight arrow (⬆) inside green lamp
			vector.StrokeLine(screen, cx, cy+radius*0.4, cx, cy-radius*0.4, aLw, arrowCol, true)
			vector.StrokeLine(screen, cx, cy-radius*0.4, cx-radius*0.4, cy-radius*0.1, aLw, arrowCol, true)
			vector.StrokeLine(screen, cx, cy-radius*0.4, cx+radius*0.4, cy-radius*0.1, aLw, arrowCol, true)
		}
	}
}
