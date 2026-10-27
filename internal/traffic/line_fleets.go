package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"slices"
)

const MaxBusesPerLine = 8

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
	c.SpeedRange = c.SpeedRange.Normalize(50, 80)
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
func (m *AIBusManager) ConfigureLineFleets(plans []LineFleetPlan, w *sim.World, lw float64, player *sim.Bus, reset bool) {
	previous := map[int]*AIBus{}
	for _, b := range m.Buses {
		previous[b.ID] = b
	}
	buses := make([]*AIBus, 0)
	pending := []*AIBus{}
	m.Routes = nil
	m.BusCount = 0
	for line, p := range plans {
		if p.Route == nil || len(p.Route.StopIDs) == 0 {
			continue
		}
		c := p.Settings.Normalize()
		if c.Counts()[sim.BusArticulated] > 0 || c.PlayerModel == sim.BusArticulated {
			sim.EnsureLongBusBays(w, p.Route)
		}
		m.Routes = append(m.Routes, p.Route)
		builder := NewAIBusManager()
		builder.ActiveRoute = p.Route
		builder.BusSpeedRange = c.SpeedRange
		// The existing dispatcher supplies legal bay positions and driver initialization.
		builder.BuildFleet(w, c.BusCount, lw, player)
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
				old := previous[line*MaxBusesPerLine+slot+1]
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
		freshIndex := 0
		for slot, model := range slotModels {
			if !model.Valid() {
				continue
			}
			fresh := builder.Buses[freshIndex]
			freshIndex++
			id := line*MaxBusesPerLine + slot + 1
			fresh.ID = id
			fresh.Bus.SetModel(model)
			fresh.Color = p.Route.Color
			fresh.LineIndex = line
			fresh.AssignedSpeedRange = c.SpeedRange
			old := previous[id]
			if !reset && old != nil && old.LineIndex == line && slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
				old.Retiring = false
				if old.Bus.PlannedModel() != model {
					old.Bus.RequestModel(model)
				}
				if old.AssignedSpeedRange != c.SpeedRange {
					setLineSpeed(old, c.SpeedRange)
				}
				buses = append(buses, old)
			} else {
				if reset && old != nil && old.LineIndex == line && slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
					fresh.Bus.SetModel(old.Bus.Model)
					fresh.Bus.RequestModel(model)
				}
				pending = append(pending, fresh)
			}
		}
		if !reset {
			for slot, model := range slotModels {
				if model.Valid() {
					continue
				}
				old := previous[line*MaxBusesPerLine+slot+1]
				if old == nil || old.LineIndex != line || !slices.Equal(old.Route.StopIDs, p.Route.StopIDs) {
					continue
				}
				old.Retiring = true
				old.Bus.RequestModel(old.Bus.Model)
				if old.AssignedSpeedRange != c.SpeedRange {
					setLineSpeed(old, c.SpeedRange)
				}
				buses = append(buses, old)
			}
		}
	}
	others := make([]*sim.Bus, 0, 1+len(previous)+len(pending))
	if player != nil {
		others = append(others, player)
	}
	for _, b := range buses {
		others = append(others, b.Bus)
	}
	for _, b := range pending {
		stop := w.Stops[b.Route.GetCurrentTargetStopID()]
		sim.PlaceBusInStationQueue(b.Bus, stop, others, lw)
		b.Driver.QueueAtStation(stop.ID)
		others = append(others, b.Bus)
		buses = append(buses, b)
	}

	slices.SortFunc(buses, func(a, b *AIBus) int { return a.ID - b.ID })
	m.Buses = buses
	m.BusCount = len(buses)
	m.CurrentMap = w.Theme
	m.CurrentLane = lw
}
func setLineSpeed(b *AIBus, r sim.SpeedRange) {
	kph := r.Sample()
	b.AssignedSpeedRange = r
	b.MaxSpeedKPH = kph
	b.CanOvertake = kph > r.MinKPH
	b.Bus.MaxSpeedFwd = sim.SpeedKPH(kph)
	b.Driver.BerserkMode = b.CanOvertake
	b.Driver.BerserkMaxSpeed = sim.SpeedKPH(kph)
	b.Driver.CruiseSpeedLimit = sim.SpeedKPH(kph)
}
