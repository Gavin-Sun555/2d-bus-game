package game

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// AIBus represents an autonomous transit bus operating along the city's bus route
type AIBus struct {
	ID                int
	LineName          string
	Bus               *Bus
	Route             *Route
	Driver            *AutopilotDriver
	Color             color.RGBA
	PassengersCarried int
}

// AIBusManager oversees all AI buses operating on the active city route
type AIBusManager struct {
	Buses       []*AIBus
	BusCount    int // Number of AI buses operating on the route (0 ~ 4)
	CurrentMap  MapTheme
	CurrentLane float64
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
func (abm *AIBusManager) SetLineCount(count int, world *World, laneWidth float64) {
	if count < 0 {
		count = 0
	} else if count > 4 {
		count = 4
	}
	abm.BusCount = count
	abm.RebuildFleet(world, laneWidth)
}

// CycleLineCount increments and wraps the count of AI buses (0 -> 1 -> 2 -> 3 -> 4 -> 0)
func (abm *AIBusManager) CycleLineCount(world *World, laneWidth float64) int {
	next := (abm.BusCount + 1) % 5
	abm.SetLineCount(next, world, laneWidth)
	return next
}

// BuildFleet sets the AI bus count and reconstructs fleet dispatch
func (abm *AIBusManager) BuildFleet(world *World, count int, laneWidth float64) {
	abm.SetLineCount(count, world, laneWidth)
}

// GetStandardRouteForMap returns the single primary continuous loop bus route for the active city map
func GetStandardRouteForMap(theme MapTheme, world *World) *Route {
	route := &Route{
		Name:            "Line 1 (城市主干环线)",
		Color:           RGBA(16, 185, 129, 255), // Emerald green
		CurrentIndex:    0,
		cachedDescDirty: true,
	}

	numStops := len(world.Stops)
	if numStops == 0 {
		return route
	}

	switch theme {
	case MapThemeMetropolis:
		// Central Terminal (0) -> Civic Center (2) -> Creative District (10) -> City Hospital (8) -> 0
		route.StopIDs = []int{0, 2, 10, 8, 0}
	case MapThemeCoastalBay:
		// Coastal Plaza (0) -> Ocean Pier (1) -> Golden Beach (3) -> Marina (5) -> 0
		route.StopIDs = []int{0, 1, 3, 5, 0}
	case MapThemeRiverValley:
		// Valley Hub (0) -> Riverside Ave (3) -> Suspension Bridge (6) -> High-Tech (9) -> 0
		route.StopIDs = []int{0, 3, 6, 9, 0}
	case MapThemeEmeraldLake:
		// Lake Hub (0) -> East Causeway (9) -> Ancient Town (3) -> West Wetlands (6) -> 0
		route.StopIDs = []int{0, 9, 3, 6, 0}
	case MapThemeTwinCities:
		// Logistics Hub (0) -> Smart City Center (4) -> Interchange East (7) -> West Industrial (2) -> 0
		route.StopIDs = []int{0, 4, 7, 2, 0}
	case MapThemeCustom:
		if numStops >= 5 {
			route.StopIDs = []int{0, 1, 3, 4, 0}
		} else if numStops >= 3 {
			route.StopIDs = []int{0, 1, 2, 0}
		} else if numStops >= 2 {
			route.StopIDs = []int{0, 1, 0}
		} else {
			route.StopIDs = []int{0}
		}
	}
	return route
}

// RebuildFleet reconstructs the autonomous bus fleet according to route definitions and count
func (abm *AIBusManager) RebuildFleet(world *World, laneWidth float64) {
	if world == nil || len(world.Stops) == 0 || abm.BusCount <= 0 {
		abm.Buses = make([]*AIBus, 0)
		return
	}

	abm.CurrentMap = world.Theme
	abm.CurrentLane = laneWidth
	stdRoute := GetStandardRouteForMap(world.Theme, world)

	if len(stdRoute.StopIDs) < 2 {
		abm.Buses = make([]*AIBus, 0)
		return
	}

	newBuses := make([]*AIBus, 0, abm.BusCount)
	numStopsInLoop := len(stdRoute.StopIDs) - 1
	if numStopsInLoop < 1 {
		numStopsInLoop = len(stdRoute.StopIDs)
	}

	for i := 0; i < abm.BusCount; i++ {
		// Distribute spawn stations evenly around the loop to ensure consistent timetable headway
		stopIdx := (i + 1) * numStopsInLoop / (abm.BusCount + 1)
		if stopIdx >= len(stdRoute.StopIDs) {
			stopIdx = 0
		}
		spawnStopID := stdRoute.StopIDs[stopIdx]
		if spawnStopID < 0 || spawnStopID >= len(world.Stops) {
			spawnStopID = 0
		}
		spawnStop := world.Stops[spawnStopID]

		// Independent route copy for this bus
		busRoute := &Route{
			Name:         stdRoute.Name,
			Color:        stdRoute.Color,
			StopIDs:      append([]int(nil), stdRoute.StopIDs...),
			CurrentIndex: (stopIdx + 1) % len(stdRoute.StopIDs),
		}

		bus := NewBus(spawnStop.Pos.X, spawnStop.Pos.Y, spawnStop.Heading)
		bus.Gear = GearDrive
		bus.TransmissionMode = TransmissionAuto

		// Distinguishable palette tints across the fleet
		busColors := []color.RGBA{
			RGBA(16, 185, 129, 255), // Emerald
			RGBA(245, 158, 11, 255), // Amber
			RGBA(168, 85, 247, 255), // Violet
			RGBA(6, 182, 212, 255),  // Cyan
		}
		aiColor := busColors[i%len(busColors)]

		newBuses = append(newBuses, &AIBus{
			ID:       i + 1,
			LineName: stdRoute.Name,
			Bus:      bus,
			Route:    busRoute,
			Driver:   NewAutopilotDriver(),
			Color:    aiColor,
		})
	}

	abm.Buses = newBuses
}

// GetObstacles returns all AI buses as obstacles for forward collision detection
func (abm *AIBusManager) GetObstacles(excludeID int) []VehicleObstacle {
	obs := make([]VehicleObstacle, 0, len(abm.Buses))
	for _, b := range abm.Buses {
		if b.ID == excludeID {
			continue
		}
		obs = append(obs, VehicleObstacle{
			Pos:     b.Bus.Pos,
			Heading: b.Bus.Heading,
			Speed:   b.Bus.Speed,
			Length:  b.Bus.Length,
			Width:   b.Bus.Width,
		})
	}
	return obs
}

// Update advances all AI route buses per frame
func (abm *AIBusManager) Update(
	dt float64,
	playerBus *Bus,
	world *World,
	tls *TrafficLightSystem,
	tm *TrafficManager,
	laneWidth float64,
	densityLevel ...int,
) {
	if world == nil || len(abm.Buses) == 0 {
		return
	}

	var activeCars []*TrafficCar
	if tm != nil {
		density := 2
		if len(densityLevel) > 0 {
			density = densityLevel[0]
		}
		activeCars = tm.GetActiveCars(density)
	}

	baseObstacles := make([]VehicleObstacle, 0, 1+len(activeCars))
	if playerBus != nil {
		baseObstacles = append(baseObstacles, VehicleObstacle{
			Pos:     playerBus.Pos,
			Heading: playerBus.Heading,
			Speed:   playerBus.Speed,
			Length:  playerBus.Length,
			Width:   playerBus.Width,
		})
	}
	for _, c := range activeCars {
		baseObstacles = append(baseObstacles, VehicleObstacle{
			Pos:     c.Pos,
			Heading: c.Heading,
			Speed:   c.Speed,
			Length:  c.Length,
			Width:   c.Width,
		})
	}

	for _, ab := range abm.Buses {
		obstacles := append([]VehicleObstacle(nil), baseObstacles...)
		for _, other := range abm.Buses {
			if other.ID == ab.ID {
				continue
			}
			obstacles = append(obstacles, VehicleObstacle{
				Pos:     other.Bus.Pos,
				Heading: other.Bus.Heading,
				Speed:   other.Bus.Speed,
				Length:  other.Bus.Length,
				Width:   other.Bus.Width,
			})
		}

		throttle, brake, steer, handbrake, toggleDoors, isDocked := ab.Driver.ComputeControls(
			ab.Bus,
			ab.Route,
			world,
			tls,
			obstacles,
			laneWidth,
			dt,
		)

		ab.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Passenger exchange simulation when parked at station
		if isDocked && ab.Bus.DoorsOpen {
			targetStopID := ab.Route.GetCurrentTargetStopID()
			if targetStopID >= 0 && targetStopID < len(world.Stops) {
				st := world.Stops[targetStopID]
				if st.WaitingPassengers > 0 && ab.Bus.PassengerCount < ab.Bus.Capacity {
					take := int(math.Min(float64(st.WaitingPassengers), 2.0))
					st.WaitingPassengers -= take
					ab.Bus.PassengerCount += take
					ab.PassengersCarried += take
				}
			}
		}

		// Boundary clearance check to keep AI bus safely on road
		bRes := world.CheckBoundaries(ab.Bus.Pos, ab.Bus.Width/2, laneWidth)
		if bRes.Zone == ZoneHardBarrier {
			ab.Bus.Pos = ab.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 4.0))
			ab.Bus.Speed = 0
		}
	}
}

// Draw renders all AI route buses with customized transit liveries and roof badges
func (abm *AIBusManager) Draw(screen *ebiten.Image, cam *Camera) {
	if cam == nil || len(abm.Buses) == 0 {
		return
	}

	for _, ab := range abm.Buses {
		if !cam.IsPointVisible(ab.Bus.Pos, 140) {
			continue
		}

		drawBusBody(screen, cam, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Length, ab.Bus.Width, ab.Color, true)
		ab.Bus.drawWheels(screen, cam)
		ab.Bus.drawBusRoof(screen, cam)
		ab.Bus.drawLights(screen, cam)
		ab.Bus.drawDoors(screen, cam)

		// Overhead glowing LED Route Badge
		if cam.Zoom >= 0.28 {
			sPos := cam.WorldToScreen(ab.Bus.Pos)
			badgeW := 90.0
			badgeH := 20.0
			badgeX := sPos.X - badgeW/2
			badgeY := sPos.Y - 32.0*cam.Zoom - badgeH

			vector.DrawFilledRect(screen, float32(badgeX), float32(badgeY), float32(badgeW), float32(badgeH),
				RGBA(15, 23, 42, 230), true)
			vector.StrokeRect(screen, float32(badgeX), float32(badgeY), float32(badgeW), float32(badgeH),
				1.2, ab.Color, true)

			badgeText := fmt.Sprintf("[1号线] AI#%d 👥%d", ab.ID, ab.Bus.PassengerCount)
			DrawText(screen, badgeText, badgeX+6, badgeY+4, 10, color.White)
		}
	}
}

// TotalActiveBuses returns total number of active AI buses
func (abm *AIBusManager) TotalActiveBuses() int {
	return len(abm.Buses)
}
