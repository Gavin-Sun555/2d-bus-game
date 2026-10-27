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
	if !model.Valid() || max(b.PassengerCount, len(b.Passengers)) > model.Capacity() {
		return false
	}
	b.GeometryVersion = 1
	b.Model, b.Capacity, b.AccelRate = model, model.Capacity(), model.Acceleration()
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
	if b.PendingModel == nil || !b.PendingModel.Valid() || route == nil || !route.IsTerminusIndex(route.CurrentIndex) || route.GetCurrentTargetStopID() != stop || !b.DoorsOpen || math.Abs(b.Speed) > 3 || b.PassengerCount != 0 || len(b.Passengers) != 0 {
		return false
	}
	fresh := NewBus(b.Pos.X, b.Pos.Y, b.Heading)
	fresh.SetModel(*b.PendingModel)
	if len(sites) > 0 && !placeReplacement(fresh, sites[0]) {
		return false
	}
	fresh.MaxSpeedFwd = b.MaxSpeedFwd
	fresh.TransmissionMode = b.TransmissionMode
	fresh.DoorsOpen, fresh.DoorProgress = b.DoorsOpen, b.DoorProgress
	*b = *fresh
	return true
}

func (b *Bus) PlannedModel() BusModel {
	if b.PendingModel != nil {
		return *b.PendingModel
	}
	return b.Model
}
