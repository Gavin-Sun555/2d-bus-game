package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"slices"
)

const (
	MaxBusesPerLine  = sim.MaxBusesPerLine
	MaxFleetBuses    = 6 * MaxBusesPerLine
	legacyBusSlots   = 8
	legacyFleetBuses = 6 * legacyBusSlots
)

// Keep IDs 1–48 assigned to their original lines so existing saves and
// references to buses and their statistics need no migration.
func lineBusID(line, slot int) int {
	if slot < legacyBusSlots {
		return line*legacyBusSlots + slot + 1
	}
	return legacyFleetBuses + line*(MaxBusesPerLine-legacyBusSlots) + slot - legacyBusSlots + 1
}

// FleetSlotID is the stable configuration key, separate from the ID of the
// individual bus assigned to that slot.
func (b *AIBus) FleetSlotID() int {
	if b.LineIndex < 0 || b.LineIndex >= 6 || b.ID < 1 {
		return 0
	}
	if b.FleetSlot != 0 {
		if b.FleetSlot < 1 || b.FleetSlot > MaxBusesPerLine {
			return 0
		}
		slotID := lineBusID(b.LineIndex, b.FleetSlot-1)
		if b.ID > MaxFleetBuses || b.ID == slotID {
			return slotID
		}
		return 0
	}
	for slot := 0; slot < MaxBusesPerLine; slot++ {
		if b.ID == lineBusID(b.LineIndex, slot) {
			return b.ID
		}
	}
	return 0
}

type LineFleetSettings struct {
	BusCount    int
	SpeedRange  sim.SpeedRange
	Model       sim.BusModel
	PlayerModel sim.BusModel
	MixedModels bool
	ModelCounts [sim.BusModelCount]int
}

func (c LineFleetSettings) Normalize() LineFleetSettings {
	if !c.Model.Valid() {
		c.Model = sim.BusStandard
	}
	if !c.PlayerModel.Valid() {
		c.PlayerModel = sim.BusStandard
	}
	c.BusCount = max(0, min(MaxBusesPerLine, c.BusCount))
	if c.MixedModels {
		remaining := MaxBusesPerLine
		c.BusCount = 0
		for i, n := range c.ModelCounts {
			n = max(0, min(remaining, n))
			c.ModelCounts[i] = n
			c.BusCount += n
			remaining -= n
		}
	} else {
		c.ModelCounts = [sim.BusModelCount]int{}
	}
	c.SpeedRange = c.SpeedRange.Normalize(50, sim.TrafficMaxSpeedKPH)
	return c
}

// Counts exposes the planned mix, including legacy uniform fleet settings.
func (c LineFleetSettings) Counts() [sim.BusModelCount]int {
	c = c.Normalize()
	if c.MixedModels {
		return c.ModelCounts
	}
	var counts [sim.BusModelCount]int
	counts[c.Model] = c.BusCount
	return counts
}
func (c LineFleetSettings) ChangeModelCount(model sim.BusModel, delta int) LineFleetSettings {
	if !model.Valid() {
		return c.Normalize()
	}
	c = c.Normalize()
	counts := c.Counts()
	counts[model] = max(0, min(MaxBusesPerLine-c.BusCount+counts[model], counts[model]+delta))
	c.MixedModels, c.ModelCounts = true, counts
	return c.Normalize()
}

type LineFleetPlan struct {
	Route    *sim.Route
	Settings LineFleetSettings
}

// ConfigureLineFleets reconciles stable per-line vehicle slots. Changing one
// service never respawns vehicles on another service or reassigns their fares.
func (m *AIBusManager) ConfigureLineFleets(plans []LineFleetPlan, w *sim.World, lw float64, player *sim.Bus, reset bool, traffic ...[]*sim.TrafficCar) {
	previous := map[int]*AIBus{}
	for _, b := range m.Buses {
		previous[b.FleetSlotID()] = b
		m.LastBusID = max(m.LastBusID, b.ID)
	}
	initialDispatch := m.LastBusID == 0
	buses := make([]*AIBus, 0)
	pending := []*AIBus{}
	initialLines := map[int]bool{}
	m.Routes = nil
	m.BusCount = 0
	for line, p := range plans {
		if p.Route == nil || len(p.Route.StopIDs) == 0 {
			continue
		}
		c := p.Settings.Normalize()
		initialLines[line] = reset
		if !reset {
			initialLines[line] = true
			for _, old := range previous {
				if old.LineIndex == line && slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
					initialLines[line] = false
					break
				}
			}
		}
		if c.Counts()[sim.BusArticulated] > 0 || c.PlayerModel == sim.BusArticulated {
			sim.EnsureLongBusBays(w, p.Route)
		}
		m.Routes = append(m.Routes, p.Route)
		// Keep matching models in their existing slots when the mix changes.
		// In particular, adding a standard bus does not turn a double-decker
		// already in service into a different vehicle just to sort the roster.
		var slotModels [MaxBusesPerLine]sim.BusModel
		for slot := range slotModels {
			slotModels[slot] = -1
		}
		remaining := c.Counts()
		if !reset {
			for slot := range slotModels {
				old := previous[lineBusID(line, slot)]
				if old == nil || old.LineIndex != line || !slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
					continue
				}
				model := old.Bus.PlannedModel()
				if model.Valid() && remaining[model] > 0 {
					slotModels[slot] = model
					remaining[model]--
				}
			}
		}
		for slot := range slotModels {
			if slotModels[slot].Valid() {
				continue
			}
			for model, n := range remaining {
				if n > 0 {
					slotModels[slot] = sim.BusModel(model)
					remaining[model]--
					break
				}
			}
		}
		var freshBuses []*AIBus
		for slot, model := range slotModels {
			if !model.Valid() {
				continue
			}
			slotID := lineBusID(line, slot)
			old := previous[slotID]
			sameService := old != nil && old.LineIndex == line && slices.Equal(old.Route.StopIDs, p.Route.StopIDs)
			if !reset && sameService {
				old.Retiring = false
				if old.Bus.PlannedModel() != model {
					old.Bus.RequestModel(model)
				}
				if old.AssignedSpeedRange != c.SpeedRange {
					old.assignLineSpeed(c.SpeedRange, c.SpeedRange.Sample())
					m.Operations.Track(old.ID, old.Bus, old.Route, 0)
				}
				buses = append(buses, old)
			} else {
				if freshBuses == nil {
					// The dispatcher supplies legal bay positions and initializes drivers.
					builder := NewAIBusManager()
					builder.stationOnlyBuild = true
					builder.ActiveRoute = p.Route
					builder.BuildFleet(w, c.BusCount, lw, player)
					freshBuses = builder.Buses
				}
				fresh := freshBuses[0]
				freshBuses = freshBuses[1:]
				fresh.FleetSlot = slot + 1
				fresh.Bus.SetModel(model)
				fresh.Color = p.Route.Color
				fresh.LineIndex = line
				if sameService {
					fresh.ID = old.ID
					fresh.Bus.SetModel(old.Bus.Model)
					fresh.Bus.RequestModel(model)
				} else if initialDispatch {
					fresh.ID = slotID
					m.LastBusID = max(m.LastBusID, fresh.ID)
				} else {
					m.LastBusID = max(m.LastBusID, MaxFleetBuses) + 1
					fresh.ID = m.LastBusID
				}
				if sameService && old.AssignedSpeedRange == c.SpeedRange && old.MaxSpeedKPH > 0 {
					fresh.assignLineSpeed(c.SpeedRange, old.MaxSpeedKPH)
				} else {
					fresh.assignLineSpeed(c.SpeedRange, c.SpeedRange.Sample())
				}
				m.ApplyPassengerCapacity(fresh.Bus)
				pending = append(pending, fresh)
			}
		}
		if !reset {
			for slot, model := range slotModels {
				if model.Valid() {
					continue
				}
				old := previous[lineBusID(line, slot)]
				if old == nil || old.LineIndex != line || !slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
					continue
				}
				old.Retiring = true
				old.Bus.RequestModel(old.Bus.Model)
				if old.AssignedSpeedRange != c.SpeedRange {
					old.assignLineSpeed(c.SpeedRange, c.SpeedRange.Sample())
					m.Operations.Track(old.ID, old.Bus, old.Route, 0)
				}
				buses = append(buses, old)
			}
		}
	}
	var cars []*sim.TrafficCar
	if len(traffic) > 0 {
		cars = traffic[0]
	}
	others := make([]*sim.Bus, 0, 1+len(previous)+len(pending))
	if player != nil {
		others = append(others, player)
	}
	for _, b := range buses {
		others = append(others, b.Bus)
	}
	for _, b := range pending {
		if initialLines[b.LineIndex] {
			continue
		}
		if !PlaceInitialBusOnRoute(b.Bus, b.Driver, b.Route, w, lw, others, cars) {
			continue
		}
		others = append(others, b.Bus)
		buses = append(buses, b)
	}
	for line := range plans {
		if !initialLines[line] {
			continue
		}
		var fresh []*AIBus
		for _, b := range pending {
			if b.LineIndex == line {
				fresh = append(fresh, b)
			}
		}
		fresh = distributeInitialLine(fresh, w, lw, others, cars)
		for _, b := range fresh {
			others = append(others, b.Bus)
			buses = append(buses, b)
		}
	}

	slices.SortFunc(buses, func(a, b *AIBus) int { return a.ID - b.ID })
	for _, old := range m.Buses {
		_, kept := slices.BinarySearchFunc(buses, old.ID, func(b *AIBus, id int) int { return b.ID - id })
		if !kept {
			m.Operations.EndVehicleService(old.ID)
		}
	}
	for _, fresh := range pending {
		m.Operations.Track(fresh.ID, fresh.Bus, fresh.Route, 0)
	}
	m.Buses = buses
	m.BusCount = len(buses)
	m.CurrentMap = w.Theme
	m.CurrentLane = lw
}
func (b *AIBus) assignLineSpeed(r sim.SpeedRange, kph int) {
	b.AssignedSpeedRange = r
	b.MaxSpeedKPH = kph
	b.CanOvertake = kph > r.MinKPH
	b.Bus.MaxSpeedFwd = sim.SpeedKPH(kph)
	b.Bus.ApplyCatalogLimits()
	b.Bus.Speed = min(b.Bus.Speed, b.Bus.MaxSpeedFwd)
	b.Driver.BerserkMode = b.CanOvertake
	b.Driver.BerserkMaxSpeed = b.Bus.MaxSpeedFwd
	b.Driver.CruiseSpeedLimit = b.Bus.MaxSpeedFwd
}
