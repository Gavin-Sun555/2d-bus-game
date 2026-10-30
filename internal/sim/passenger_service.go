package sim

import "math"

// PassengerService tracks independent front/rear door work in simulation seconds.
// It is serialized with the bus so saving/loading cannot bypass minimum dwell.
type PassengerService struct {
	BoardingSeconds    float64 `json:",omitempty"`
	AlightingSeconds   float64 `json:",omitempty"`
	StopID             int     `json:",omitempty"`
	RouteIndex         int     `json:",omitempty"`
	Active             bool    `json:",omitempty"`
	IsTerminus         bool    `json:",omitempty"`
	Candidates         []int   `json:",omitempty"`
	BoardingTarget     int     `json:",omitempty"`
	BoardingRemaining  float64
	AlightingTarget    int `json:",omitempty"`
	AlightingRemaining float64
	BoardedSoFar       int `json:",omitempty"`
	AlightedSoFar      int `json:",omitempty"`
	FareSoFar          int `json:",omitempty"`
}

func (s PassengerService) Remaining() float64 {
	return math.Max(s.BoardingRemaining, s.AlightingRemaining)
}
func (s PassengerService) Valid() bool {
	if (s.BoardingSeconds != 0 || s.AlightingSeconds != 0) && !(PassengerRates{s.BoardingSeconds, s.AlightingSeconds}).Valid() {
		return false
	}
	if s.BoardingTarget < 0 || s.BoardingTarget > 1000 || s.AlightingTarget < 0 || s.AlightingTarget > 1000 || s.BoardedSoFar < 0 || s.AlightedSoFar < 0 || s.FareSoFar < 0 {
		return false
	}
	for _, v := range []float64{s.BoardingRemaining, s.AlightingRemaining} {
		if math.IsNaN(v) || math.IsInf(v, 0) || v < 0 || v > 3600 {
			return false
		}
	}
	return true
}

func PendingSteps(remaining, stepSeconds float64) int {
	if remaining <= 1e-9 || stepSeconds <= 0 {
		return 0
	}
	return int(math.Ceil((remaining - 1e-9) / stepSeconds))
}

func (b *Bus) AddPassengerService(boarded, alighted int) {
	if !b.PassengerService.Active && b.PassengerService.BoardingSeconds == 0 && b.PassengerService.AlightingSeconds == 0 {
		b.SetStationServiceRates(DefaultPassengerRates(b.Model))
	}
	if boarded > 0 {
		b.PassengerService.BoardingTarget += boarded
		b.PassengerService.BoardingRemaining += float64(boarded) * b.PassengerService.rates().Boarding
	}
	if alighted > 0 {
		b.PassengerService.AlightingTarget += alighted
		b.PassengerService.AlightingRemaining += float64(alighted) * b.PassengerService.rates().Alighting
	}
}
func (b *Bus) PassengerServiceRemaining() float64 { return b.PassengerService.Remaining() }
func (b *Bus) advancePassengerService(dt float64) {
	if !b.DoorsOpen || math.Abs(b.Speed) > 3 || dt <= 0 || math.IsNaN(dt) || math.IsInf(dt, 0) {
		return
	}
	if len(b.Passengers) < b.Capacity {
		b.PassengerService.BoardingRemaining = math.Max(0, b.PassengerService.BoardingRemaining-dt)
	}
	b.PassengerService.AlightingRemaining = math.Max(0, b.PassengerService.AlightingRemaining-dt)
	// Avoid a one-tick residue from subtracting fixed 1/60 steps.
	if b.PassengerService.BoardingRemaining < 1e-9 {
		b.PassengerService.BoardingRemaining = 0
	}
	if b.PassengerService.AlightingRemaining < 1e-9 {
		b.PassengerService.AlightingRemaining = 0
	}
}

func (b *Bus) BeginStationService(stopID, routeIndex, alighting int, isTerminus bool, candidates []int, configured ...PassengerRates) {
	rates := DefaultPassengerRates(b.Model)
	if len(configured) > 0 && configured[0].Valid() {
		rates = configured[0]
	}
	alighting = max(0, alighting)
	b.PassengerService = PassengerService{
		BoardingSeconds:    rates.Boarding,
		AlightingSeconds:   rates.Alighting,
		StopID:             stopID,
		RouteIndex:         routeIndex,
		Active:             true,
		IsTerminus:         isTerminus,
		Candidates:         append([]int(nil), candidates...),
		AlightingTarget:    alighting,
		AlightingRemaining: float64(alighting) * rates.Alighting,
	}
}

func (b *Bus) setBoardingTarget(target int) {
	oldTarget := b.PassengerService.BoardingTarget
	if target <= 0 {
		b.PassengerService.BoardingTarget = 0
		b.PassengerService.BoardingRemaining = 0
		return
	}
	if target == oldTarget {
		return
	}
	step := b.PassengerService.rates().Boarding
	timer := step
	if oldTarget > 0 {
		timer = b.PassengerService.BoardingRemaining - float64(oldTarget-1)*step
		timer = math.Min(step, math.Max(0, timer))
	}
	b.PassengerService.BoardingTarget = target
	b.PassengerService.BoardingRemaining = float64(target-1)*step + timer
}

func (b *Bus) removeAlightingPassengers(stopID, count int, isTerminus bool) int {
	if count <= 0 || len(b.Passengers) == 0 {
		b.PassengerCount = len(b.Passengers)
		return 0
	}
	remaining := make([]Passenger, 0, len(b.Passengers))
	removed := 0
	for _, p := range b.Passengers {
		if removed < count && p.TargetStopID == stopID {
			removed++
		} else {
			remaining = append(remaining, p)
		}
	}
	if isTerminus && removed < count && len(remaining) > 0 {
		extra := min(count-removed, len(remaining))
		removed += extra
		remaining = remaining[extra:]
	}
	b.Passengers = remaining
	b.PassengerCount = len(b.Passengers)
	return removed
}

func (b *Bus) StepStationService(busID int, stop *BusStop, route *Route, ops *TransitOperations) (alightedNow, boardedNow, fareNow int) {
	if !b.PassengerService.Active || stop == nil {
		return 0, 0, 0
	}
	stillAlighting := PendingSteps(b.PassengerService.AlightingRemaining, b.PassengerService.rates().Alighting)
	if b.PassengerService.AlightingTarget > stillAlighting {
		due := b.PassengerService.AlightingTarget - stillAlighting
		alightedNow = b.removeAlightingPassengers(stop.ID, due, b.PassengerService.IsTerminus)
		b.PassengerService.AlightingTarget = stillAlighting
		b.PassengerService.AlightedSoFar += alightedNow
		if ops != nil && alightedNow > 0 {
			ops.Alight(busID, stop.ID, alightedNow)
		}
	}
	stillBoarding := PendingSteps(b.PassengerService.BoardingRemaining, b.PassengerService.rates().Boarding)
	if b.PassengerService.BoardingTarget > stillBoarding {
		due := b.PassengerService.BoardingTarget - stillBoarding
		if stop.WaitingPassengers > 0 {
			take := min(due, stop.WaitingPassengers)
			if space := b.Capacity - len(b.Passengers); take > space {
				take = max(0, space)
			}
			if take > 0 {
				savedIdx := 0
				if route != nil {
					savedIdx = route.CurrentIndex
					route.CurrentIndex = b.PassengerService.RouteIndex
				}
				boardedNow, fareNow = ops.Board(busID, b, route, stop.ID, take, b.PassengerService.Candidates)
				if route != nil {
					route.CurrentIndex = savedIdx
				}
				stop.WaitingPassengers -= boardedNow
				b.PassengerService.BoardedSoFar += boardedNow
				b.PassengerService.FareSoFar += fareNow
			}
		}
		b.PassengerService.BoardingTarget = stillBoarding
		if stop.WaitingPassengers <= 0 || len(b.Passengers) >= b.Capacity {
			b.PassengerService.BoardingTarget = 0
			b.PassengerService.BoardingRemaining = 0
		}
	} else if stop.WaitingPassengers <= 0 && b.PassengerService.BoardingTarget > 0 {
		b.PassengerService.BoardingTarget = 0
		b.PassengerService.BoardingRemaining = 0
	}
	return alightedNow, boardedNow, fareNow
}

// RebalanceStationBoarding refreshes each door's work estimate against one
// shared queue. Targets are not reservations: every bus keeps boarding at its
// own rate while passengers remain, even if another bus is busy alighting.
// The last argument is retained for callers joining/leaving a platform.
func RebalanceStationBoarding(stop *BusStop, buses []*Bus, _ bool) {
	if stop == nil {
		return
	}
	for _, b := range buses {
		staying := max(0, len(b.Passengers)-b.PassengerService.AlightingTarget)
		b.setBoardingTarget(min(max(0, stop.WaitingPassengers), max(0, b.Capacity-staying)))
	}
}

func (s PassengerService) rates() PassengerRates {
	r := PassengerRates{s.BoardingSeconds, s.AlightingSeconds}
	if !r.Valid() {
		return DefaultPassengerRates(BusStandard)
	}
	return r
}

// A terminal replacement uses the new model's doors for remaining work.
func (b *Bus) SetStationServiceRates(r PassengerRates) {
	if !r.Valid() {
		return
	}
	old := b.PassengerService.rates()
	b.PassengerService.BoardingRemaining *= r.Boarding / old.Boarding
	b.PassengerService.AlightingRemaining *= r.Alighting / old.Alighting
	b.PassengerService.BoardingSeconds, b.PassengerService.AlightingSeconds = r.Boarding, r.Alighting
}
