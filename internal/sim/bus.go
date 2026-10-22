package sim

type Gear int

const (
	GearReverse Gear = -1 // 倒挡 (R)
	GearNeutral Gear = 0  // 空挡 (N)
	GearDrive   Gear = 1  // 前进挡 (D)

	// 预留手动挡位 (Reserved for manual transmission):
	Gear1 Gear = 2 // 手动 1 挡 (M1)
	Gear2 Gear = 3 // 手动 2 挡 (M2)
	Gear3 Gear = 4 // 手动 3 挡 (M3)
	Gear4 Gear = 5 // 手动 4 挡 (M4)
	Gear5 Gear = 6 // 手动 5 挡 (M5)
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
	TransmissionAuto   TransmissionMode = iota // 自动挡 (D, N, R)
	TransmissionManual                         // 手动挡模式 (预留 1~5, N, R)
)

const (
	// DefaultBusCapacity specifies standard passenger capacity for all city buses (Player & AI)
	DefaultBusCapacity = 45
)

// Bus represents our driveable city bus
type Bus struct {
	StationMergeRequest *BusStationMergeRequest
	LaneMergeRequest    *BusLaneMergeRequest
	YieldingToBus       *Bus // Acknowledged priority for a bus returning ahead of this route bus.
	RoundaboutEntry     *BusRoundaboutEntry
	JunctionCorridor    *JunctionCorridor
	Pos                 Vec2    // Center position in world coordinates
	Heading             float64 // Direction in radians (0 = facing +X / East)
	Speed               float64 // Current speed (pixels per second, positive = forward, negative = reverse)
	SteeringAngle       float64 // Current front wheel angle in radians

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
	Passengers     []Passenger

	// Vehicle Health & Collision
	Durability    float64 // 0.0 to 100.0%
	OnShoulder    bool    // true when wheels are on roadside shoulder (slowdown)
	CrashCooldown float64 // Invulnerability timer after impact

	// Skidmarks and trail
	TireTracks []TireMark
}

// Passenger represents an onboard passenger with an intended destination stop ID
type Passenger struct {
	PaidFare     int // Charged once at boarding; later tariff edits do not reprice this trip.
	TargetStopID int
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
		Gear:             GearDrive,        // Default in Drive
		TransmissionMode: TransmissionAuto, // Default Automatic
		Length:           96,
		Width:            36,
		WheelBase:        60,
		MaxSpeedFwd:      460,
		MaxSpeedRev:      100,
		AccelRate:        230,
		BrakeRate:        380,
		HandbrakeRate:    600,
		FrictionRate:     70,
		MaxSteerAngle:    0.68, // ~39.0 degrees (agile cornering at intersections)
		SteerTurnSpeed:   2.4,  // steering response speed
		SteerReturnSpd:   3.5,  // auto-center return speed
		Capacity:         DefaultBusCapacity,
		PassengerCount:   0,
		Passengers:       make([]Passenger, 0, DefaultBusCapacity),
		Durability:       100.0,
		OnShoulder:       false,
		CrashCooldown:    0,
		TireTracks:       make([]TireMark, 0, 200),
	}
}

// CountPassengersAlightingAt returns the number of passengers whose destination is stopID
func (b *Bus) CountPassengersAlightingAt(stopID int) int {
	count := 0
	for _, p := range b.Passengers {
		if p.TargetStopID == stopID {
			count++
		}
	}
	return count
}

// RemovePassengersAlightingAt removes all passengers destined for stopID and returns the count removed
func (b *Bus) RemovePassengersAlightingAt(stopID int) int {
	if len(b.Passengers) == 0 {
		b.PassengerCount = 0
		return 0
	}
	remaining := make([]Passenger, 0, len(b.Passengers))
	removed := 0
	for _, p := range b.Passengers {
		if p.TargetStopID == stopID {
			removed++
		} else {
			remaining = append(remaining, p)
		}
	}
	b.Passengers = remaining
	b.PassengerCount = len(b.Passengers)
	return removed
}

// ClearPassengers removes all passengers from the bus and returns the count removed
func (b *Bus) ClearPassengers() int {
	removed := len(b.Passengers)
	b.Passengers = make([]Passenger, 0, b.Capacity)
	b.PassengerCount = 0
	return removed
}

// BoardPassengers adds up to count passengers destined for randomly chosen stops from candidateDestStopIDs.
// Returns the actual number of passengers boarded.
func (b *Bus) BoardPassengers(count int, candidateDestStopIDs []int) int {
	if count <= 0 {
		return 0
	}
	availableSpace := b.Capacity - len(b.Passengers)
	if count > availableSpace {
		count = availableSpace
	}
	if count <= 0 {
		return 0
	}

	for i := 0; i < count; i++ {
		targetID := -1
		if len(candidateDestStopIDs) > 0 {
			targetID = candidateDestStopIDs[RandomIntn(len(candidateDestStopIDs))]
		}
		b.Passengers = append(b.Passengers, Passenger{
			TargetStopID: targetID,
		})
	}
	b.PassengerCount = len(b.Passengers)
	return count
}

// GetDestinationsCountMap returns a map of destination stop IDs to passenger counts
func (b *Bus) GetDestinationsCountMap() map[int]int {
	m := make(map[int]int)
	for _, p := range b.Passengers {
		m[p.TargetStopID]++
	}
	return m
}
