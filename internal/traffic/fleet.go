package traffic

import (
	"image/color"
	"math"
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// AIBus represents an autonomous transit bus operating along the city's bus route
type AIBus struct {
	Retiring            bool // Finish this lap, alight at the terminal, then leave service.
	retireReady         bool
	lateArrivalWait     float64
	LineIndex           int
	AssignedSpeedRange  sim.SpeedRange
	ID                  int
	MaxSpeedKPH         int
	CanOvertake         bool
	LineName            string
	Bus                 *sim.Bus
	Route               *sim.Route
	Driver              *autopilot.AutopilotDriver
	Color               color.RGBA
	PassengersCarried   int
	DeadlockTimer       float64 // Time accumulated while motionless and jammed against another non-player vehicle
	DeadlockTracker     sim.StationaryTracker
	LastExchangedStopID int // Prevents re-exchanging passengers on every frame of the same station dwell
}

// ResetToStartingStation resets an AI bus back to its route's starting station (StopIDs[0])
func (ab *AIBus) ResetToStartingStation(world *sim.World) {
	if ab == nil || ab.Bus == nil || ab.Route == nil || len(ab.Route.StopIDs) == 0 || world == nil || len(world.Stops) == 0 {
		return
	}
	ab.DeadlockTimer = 0
	ab.DeadlockTracker = sim.StationaryTracker{}
	ab.Route.CurrentIndex = 0
	startID := ab.Route.StopIDs[0]
	if startID >= 0 && startID < len(world.Stops) {
		st := world.Stops[startID]
		ab.Bus.Pos = st.Pos
		ab.Bus.Heading = st.Heading
		ab.Bus.Speed = 0
		ab.Bus.SteeringAngle = 0
		ab.Bus.JunctionCorridor = nil
		ab.Bus.YieldingToBus = nil
		ab.Bus.ClearPassengers()
		ab.Bus.PassengerService = sim.PassengerService{}
		ab.Bus.ResetArticulation()
		ab.LastExchangedStopID = -1
		if ab.Driver != nil {
			ab.Driver.Reset()
			ab.Driver.QueueAtStation(startID)
		}
	}
}

// updateDeadlock applies this bus's independent stationary recovery clock.
func (ab *AIBus) updateDeadlock(dt float64, jammed bool, tls *sim.TrafficLightSystem, world *sim.World) bool {
	ab.DeadlockTimer = ab.DeadlockTracker.Update(ab.DeadlockTimer, dt, ab.Bus.Pos, ab.Bus.Speed, jammed)
	if ab.DeadlockTimer >= tls.DeadlockTimeoutAt(ab.Bus.Pos) {
		ab.ResetToStartingStation(world)
		return true
	}
	return false
}

// AIBusManager oversees all AI buses operating on the active city route
type AIBusManager struct {
	stationOnlyBuild bool // Per-line builder defers distribution until models are assigned.
	PassengerRates   [sim.BusModelCount]sim.PassengerRates
	Operations       *sim.TransitOperations
	baseObstacles    []autopilot.VehicleObstacle
	obstacles        []autopilot.VehicleObstacle
	Buses            []*AIBus
	BusSpeedRange    sim.SpeedRange
	BusCount         int // Number of AI buses operating on the route (0 ~ 8)
	CurrentMap       sim.MapTheme
	CurrentLane      float64
	ActiveRoute      *sim.Route
	Routes           []*sim.Route
}

// NewAIBusManager creates the AI bus manager with default 0 AI buses
func NewAIBusManager() *AIBusManager {
	return &AIBusManager{
		Buses:       make([]*AIBus, 0),
		BusCount:    0,
		CurrentLane: 52.0,
	}
}

// GetLineCount returns the configured number of AI buses
func (abm *AIBusManager) GetLineCount(_ ...int) int {
	return abm.BusCount
}

// SetLineCount updates the count of AI buses and rebuilds fleet
func (abm *AIBusManager) SetLineCount(count int, world *sim.World, laneWidth float64, playerBus ...*sim.Bus) {
	if count < 0 {
		count = 0
	} else if count > 8 {
		count = 8
	}
	abm.BusCount = count
	abm.RebuildFleet(world, laneWidth, playerBus...)
}

// CycleLineCount increments and wraps the count of AI buses (0 -> 1 -> ... -> 8 -> 0)
func (abm *AIBusManager) CycleLineCount(world *sim.World, laneWidth float64, playerBus ...*sim.Bus) int {
	next := (abm.BusCount + 1) % 9
	abm.SetLineCount(next, world, laneWidth, playerBus...)
	return next
}

// BuildFleet sets the AI bus count and reconstructs fleet dispatch
func (abm *AIBusManager) BuildFleet(world *sim.World, count int, laneWidth float64, playerBus ...*sim.Bus) {
	abm.SetLineCount(count, world, laneWidth, playerBus...)
}

// RebuildFleetForRoute updates the active route and reconstructs the autonomous bus fleet
func (abm *AIBusManager) RebuildFleetForRoute(world *sim.World, route *sim.Route, laneWidth float64, playerBus ...*sim.Bus) {
	abm.ActiveRoute = route
	abm.RebuildFleet(world, laneWidth, playerBus...)
}

// RebuildFleet reconstructs and distributes a new fleet along its routes.
func (abm *AIBusManager) RebuildFleet(world *sim.World, laneWidth float64, playerBus ...*sim.Bus) {
	if world == nil || len(world.Stops) == 0 || abm.BusCount <= 0 {
		abm.Buses = make([]*AIBus, 0)
		return
	}

	abm.CurrentMap = world.Theme
	abm.CurrentLane = laneWidth

	targetRoute := abm.ActiveRoute
	if targetRoute == nil || len(targetRoute.StopIDs) == 0 {
		targetRoute = sim.GetStandardRouteForMap(world.Theme, world)
	}

	if len(targetRoute.StopIDs) < 1 {
		abm.Buses = make([]*AIBus, 0)
		return
	}

	startStopID := 0
	if len(targetRoute.StopIDs) > 0 {
		startStopID = targetRoute.StopIDs[0]
	}
	if startStopID < 0 || startStopID >= len(world.Stops) {
		startStopID = 0
	}
	startStop := world.Stops[startStopID]

	// Check if player bus is occupying slot 0 inside the starting station bay
	playerAtStart := true
	if len(playerBus) > 0 && playerBus[0] != nil {
		if playerBus[0].Pos.Distance(startStop.Pos) > 120.0 {
			playerAtStart = false
		}
	}

	baseSlot := 0
	if playerAtStart {
		baseSlot = 1 // Player occupies slot 0; AI fleet queues sequentially behind player
	}
	queueSpacing := 120.0 // Safe headway: bus length 96px + 24px bumper-to-bumper buffer

	newBuses := make([]*AIBus, 0, abm.BusCount)
	busColors := []color.RGBA{
		sim.RGBA(16, 185, 129, 255), // Emerald Green
		sim.RGBA(245, 158, 11, 255), // Amber Orange
		sim.RGBA(168, 85, 247, 255), // Royal Violet
		sim.RGBA(6, 182, 212, 255),  // Vivid Cyan
		sim.RGBA(236, 72, 153, 255), // Rose Pink
		sim.RGBA(59, 130, 246, 255), // Cobalt Blue
		sim.RGBA(234, 179, 8, 255),  // Golden Yellow
		sim.RGBA(20, 184, 166, 255), // Teal Mint
	}

	routeStops := targetRoute.StopIDs
	if len(routeStops) > 1 && routeStops[len(routeStops)-1] == routeStops[0] {
		routeStops = routeStops[:len(routeStops)-1]
	}

	lineSlots := make(map[int]int)
	for i := 0; i < abm.BusCount; i++ {
		if len(abm.Routes) > 1 {
			targetRoute = abm.Routes[i%len(abm.Routes)]
			if targetRoute == nil || len(targetRoute.StopIDs) == 0 {
				continue
			}
			startStop = world.Stops[targetRoute.StopIDs[0]]
			routeStops = targetRoute.StopIDs
			baseSlot = 0
			if len(playerBus) > 0 && playerBus[0] != nil && playerBus[0].Pos.Distance(startStop.Pos) < 120 {
				baseSlot = 1
			}
		}
		var st *sim.BusStop
		var slot int
		var currIdx int
		if i < 4 || len(routeStops) <= 1 {
			st = startStop
			slot = baseSlot + i
			currIdx = 0
		} else {
			downstream := routeStops[1:]
			idxInDownstream := (i - 4) % len(downstream)
			slotInStop := (i - 4) / len(downstream)
			currIdx = 1 + idxInDownstream
			targetID := downstream[idxInDownstream]
			if targetID >= 0 && targetID < len(world.Stops) {
				st = world.Stops[targetID]
			} else {
				st = startStop
			}
			slot = slotInStop
		}

		if len(abm.Routes) > 1 {
			st = startStop
			currIdx = 0
			slot = baseSlot + lineSlots[st.ID]
			lineSlots[st.ID]++
		}
		stSin, stCos := math.Sincos(st.Heading)
		stBack := sim.Vec2{X: -stCos, Y: -stSin}
		spawnPos := st.FrontBerthPosition().Add(stBack.Mul(float64(slot) * queueSpacing))

		busRoute := &sim.Route{
			Name:         targetRoute.Name,
			Color:        targetRoute.Color,
			StopIDs:      append([]int(nil), targetRoute.StopIDs...),
			CurrentIndex: currIdx,
		}

		bus := sim.NewBus(spawnPos.X, spawnPos.Y, st.Heading)
		bus.Gear = sim.GearDrive
		bus.TransmissionMode = sim.TransmissionAuto
		bus.Speed = 0
		bus.SteeringAngle = 0
		bus.Capacity = sim.DefaultBusCapacity

		rangeKPH := abm.BusSpeedRange.Normalize(50, 80)
		speedKPH := rangeKPH.Sample()
		driver := autopilot.NewAutopilotDriver()
		driver.QueueAtStation(st.ID)
		driver.DisallowOncomingOvertake = true
		driver.BerserkMode = speedKPH > rangeKPH.MinKPH
		driver.BerserkMaxSpeed = sim.SpeedKPH(speedKPH)
		driver.CruiseSpeedLimit = sim.SpeedKPH(speedKPH)
		bus.MaxSpeedFwd = sim.SpeedKPH(speedKPH)
		queuedBuses := make([]*sim.Bus, 0, len(newBuses)+1)
		if len(playerBus) > 0 && playerBus[0] != nil {
			queuedBuses = append(queuedBuses, playerBus[0])
		}
		for _, placed := range newBuses {
			queuedBuses = append(queuedBuses, placed.Bus)
		}
		sim.PlaceBusInStationQueue(bus, st, queuedBuses, laneWidth)
		aiColor := busColors[i%len(busColors)]
		newBuses = append(newBuses, &AIBus{
			ID:          i + 1,
			LineName:    targetRoute.Name,
			Bus:         bus,
			Route:       busRoute,
			Driver:      driver,
			MaxSpeedKPH: speedKPH, CanOvertake: speedKPH > rangeKPH.MinKPH,
			Color:               aiColor,
			LastExchangedStopID: -1,
		})
	}

	if !abm.stationOnlyBuild {
		var others []*sim.Bus
		if len(playerBus) > 0 && playerBus[0] != nil {
			others = append(others, playerBus[0])
		}
		used := map[int]bool{}
		for _, b := range newBuses {
			if used[b.ID] {
				continue
			}
			var group []*AIBus
			for _, other := range newBuses {
				if !used[other.ID] && other.Route.Name == b.Route.Name && slices.Equal(other.Route.StopIDs, b.Route.StopIDs) {
					group = append(group, other)
					used[other.ID] = true
				}
			}
			distributeInitialLine(group, world, laneWidth, others, nil)
			for _, placed := range group {
				others = append(others, placed.Bus)
			}
		}
	}
	abm.Buses = newBuses
}

// GetObstacles returns all AI buses as obstacles for forward collision detection
func (abm *AIBusManager) GetObstacles(excludeID int) []autopilot.VehicleObstacle {
	obs := make([]autopilot.VehicleObstacle, 0, len(abm.Buses))
	for _, b := range abm.Buses {
		if b.ID == excludeID {
			continue
		}
		obs = append(obs, autopilot.ObserveBus(b.Bus))
	}
	return obs
}

// TotalActiveBuses returns total number of active AI buses
func (abm *AIBusManager) TotalActiveBuses() int {
	return len(abm.Buses)
}
