package sim

import "math"

// BusModel zero is the original bus, including in saves made before models existed.
type BusModel int

const (
	BusStandard BusModel = iota
	BusDoubleDecker
	BusArticulated
	BusModelCount
)

func (m BusModel) Valid() bool { return m >= BusStandard && m < BusModelCount }
func (m BusModel) Capacity() int {
	switch m {
	case BusDoubleDecker:
		return 80
	case BusArticulated:
		return 85
	default:
		return DefaultBusCapacity
	}
}

// CapacityWithOverload floors fractional riders after applying the allowance.
func (m BusModel) CapacityWithOverload(percent int) int {
	return m.Capacity() * (100 + max(0, min(50, percent))) / 100
}
func (m BusModel) MaxPassengerCapacity() int { return m.CapacityWithOverload(50) }

func (m BusModel) Acceleration() float64 {
	switch m {
	case BusDoubleDecker:
		return 230 * .9
	case BusArticulated:
		return 230 * .85
	default:
		return 230
	}
}
func (m BusModel) Name() string {
	switch m {
	case BusDoubleDecker:
		return "双层车"
	case BusArticulated:
		return "铰接车"
	default:
		return "普通车"
	}
}
func (m BusModel) EnglishName() string {
	switch m {
	case BusDoubleDecker:
		return "Double-decker"
	case BusArticulated:
		return "Articulated"
	default:
		return "Standard"
	}
}

// SetModel initializes a vehicle before dispatch. In-service changes must use
// RequestModel and wait for the terminal passenger exchange.
func (b *Bus) SetModel(model BusModel) bool {
	return b.SetModelWithCapacity(model, model.Capacity())
}

func (b *Bus) SetModelWithCapacity(model BusModel, capacity int) bool {
	if !model.Valid() || capacity < 1 || capacity > model.MaxPassengerCapacity() || max(b.PassengerCount, len(b.Passengers)) > capacity {
		return false
	}
	b.GeometryVersion = 1
	b.Model, b.Capacity, b.AccelRate = model, capacity, model.Acceleration()
	b.MaxSteerAngle = .68
	if model == BusArticulated {
		b.MaxSteerAngle = .54
	}
	b.PendingModel = nil
	b.ResetArticulation()
	return true
}

// RequestModel leaves the current vehicle intact, even when empty or upgrading.
// Selecting the current model cancels a queued replacement.
func (b *Bus) RequestModel(model BusModel) bool {
	if !model.Valid() {
		return false
	}
	if model == b.Model && !(model == BusArticulated && b.GeometryVersion == 0) {
		b.PendingModel = nil
		b.TerminalReplacementHold = false
		return true
	}
	if b.PendingModel == nil || *b.PendingModel != model {
		b.PendingModel = &model
	}
	return true
}

// ReplaceModelAtTerminus exchanges the old vehicle for a fresh vehicle after
// terminal alighting and before boarding. Keep the service slot's pointer stable
// for traffic priority links and operating statistics; reset vehicle condition.
func (b *Bus) ReplaceModelAtTerminus(route *Route, stop int, sites ...VehicleHandoverSite) bool {
	if b.PendingModel == nil || !b.PendingModel.Valid() || route == nil || !b.DoorsOpen || math.Abs(b.Speed) > 3 || b.PassengerCount != 0 || len(b.Passengers) != 0 {
		return false
	}
	isTerminus := (route.IsTerminusIndex(route.CurrentIndex) && route.GetCurrentTargetStopID() == stop) ||
		(b.PassengerService.IsTerminus && (b.PassengerService.StopID == stop || route.IsTerminus(stop))) ||
		(route.IsTerminusIndex(b.PassengerService.RouteIndex) && (b.PassengerService.StopID == stop || route.IsTerminus(stop))) ||
		(route.IsTerminus(stop) && (b.PassengerService.StopID == stop || b.PassengerService.RouteIndex == 0))
	if !isTerminus {
		return false
	}
	fresh := NewBus(b.Pos.X, b.Pos.Y, b.Heading)
	fresh.SetModel(*b.PendingModel)
	if len(sites) > 0 && !placeReplacement(fresh, sites[0]) {
		return false
	}
	fresh.HeadlightsOn = b.HeadlightsOn
	fresh.MaxSpeedFwd = b.MaxSpeedFwd
	fresh.TransmissionMode = b.TransmissionMode
	fresh.DoorsOpen, fresh.DoorProgress = b.DoorsOpen, b.DoorProgress
	fresh.PassengerService = b.PassengerService
	*b = *fresh
	return true
}

func (b *Bus) PlannedModel() BusModel {
	if b.PendingModel != nil {
		return *b.PendingModel
	}
	return b.Model
}
