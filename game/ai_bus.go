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
	PassengersCarried   int
	DeadlockTimer       float64 // Time accumulated while motionless and jammed against another non-player vehicle
	LastExchangedStopID int     // Prevents re-exchanging passengers on every frame of the same station dwell
}

// AIBusManager oversees all AI buses operating on the active city route
type AIBusManager struct {
	Buses       []*AIBus
	BusCount    int // Number of AI buses operating on the route (0 ~ 8)
	CurrentMap  MapTheme
	CurrentLane float64
	ActiveRoute *Route
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
func (abm *AIBusManager) SetLineCount(count int, world *World, laneWidth float64, playerBus ...*Bus) {
	if count < 0 {
		count = 0
	} else if count > 8 {
		count = 8
	}
	abm.BusCount = count
	abm.RebuildFleet(world, laneWidth, playerBus...)
}

// CycleLineCount increments and wraps the count of AI buses (0 -> 1 -> ... -> 8 -> 0)
func (abm *AIBusManager) CycleLineCount(world *World, laneWidth float64, playerBus ...*Bus) int {
	next := (abm.BusCount + 1) % 9
	abm.SetLineCount(next, world, laneWidth, playerBus...)
	return next
}

// BuildFleet sets the AI bus count and reconstructs fleet dispatch
func (abm *AIBusManager) BuildFleet(world *World, count int, laneWidth float64, playerBus ...*Bus) {
	abm.SetLineCount(count, world, laneWidth, playerBus...)
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

// RebuildFleetForRoute updates the active route and reconstructs the autonomous bus fleet
func (abm *AIBusManager) RebuildFleetForRoute(world *World, route *Route, laneWidth float64, playerBus ...*Bus) {
	abm.ActiveRoute = route
	abm.RebuildFleet(world, laneWidth, playerBus...)
}

// RebuildFleet reconstructs the autonomous bus fleet with front-and-back queuing at the starting station
func (abm *AIBusManager) RebuildFleet(world *World, laneWidth float64, playerBus ...*Bus) {
	if world == nil || len(world.Stops) == 0 || abm.BusCount <= 0 {
		abm.Buses = make([]*AIBus, 0)
		return
	}

	abm.CurrentMap = world.Theme
	abm.CurrentLane = laneWidth

	targetRoute := abm.ActiveRoute
	if targetRoute == nil || len(targetRoute.StopIDs) == 0 {
		targetRoute = GetStandardRouteForMap(world.Theme, world)
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
		RGBA(16, 185, 129, 255), // Emerald Green
		RGBA(245, 158, 11, 255), // Amber Orange
		RGBA(168, 85, 247, 255), // Royal Violet
		RGBA(6, 182, 212, 255),  // Vivid Cyan
		RGBA(236, 72, 153, 255), // Rose Pink
		RGBA(59, 130, 246, 255), // Cobalt Blue
		RGBA(234, 179, 8, 255),  // Golden Yellow
		RGBA(20, 184, 166, 255), // Teal Mint
	}

	routeStops := targetRoute.StopIDs
	if len(routeStops) > 1 && routeStops[len(routeStops)-1] == routeStops[0] {
		routeStops = routeStops[:len(routeStops)-1]
	}

	for i := 0; i < abm.BusCount; i++ {
		var st *BusStop
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

		stCos := math.Cos(st.Heading)
		stSin := math.Sin(st.Heading)
		stBack := Vec2{X: -stCos, Y: -stSin}
		spawnPos := st.Pos.Add(stBack.Mul(float64(slot) * queueSpacing))

		busRoute := &Route{
			Name:         targetRoute.Name,
			Color:        targetRoute.Color,
			StopIDs:      append([]int(nil), targetRoute.StopIDs...),
			CurrentIndex: currIdx,
		}

		bus := NewBus(spawnPos.X, spawnPos.Y, st.Heading)
		bus.Gear = GearDrive
		bus.TransmissionMode = TransmissionAuto
		bus.Speed = 0
		bus.SteeringAngle = 0

		aiColor := busColors[i%len(busColors)]
		newBuses = append(newBuses, &AIBus{
			ID:                  i + 1,
			LineName:            targetRoute.Name,
			Bus:                 bus,
			Route:               busRoute,
			Driver:              NewAutopilotDriver(),
			Color:               aiColor,
			LastExchangedStopID: -1,
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
			if targetStopID >= 0 && targetStopID < len(world.Stops) && ab.LastExchangedStopID != targetStopID {
				ab.LastExchangedStopID = targetStopID
				st := world.Stops[targetStopID]

				// 1. Alight passengers destined for this stop
				ab.Bus.RemovePassengersAlightingAt(targetStopID)
				if ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) {
					ab.Bus.ClearPassengers()
				}

				// 2. Board waiting passengers with downstream destinations
				if st.WaitingPassengers > 0 && len(ab.Bus.Passengers) < ab.Bus.Capacity {
					candidates := ab.Route.GetUpcomingCandidateStopIDs(ab.Route.CurrentIndex)
					boardSpace := ab.Bus.Capacity - len(ab.Bus.Passengers)
					take := int(math.Min(float64(st.WaitingPassengers), float64(boardSpace)))
					if take > 6 {
						take = 6
					}
					actualBoarded := ab.Bus.BoardPassengers(take, candidates)
					st.WaitingPassengers -= actualBoarded
					ab.PassengersCarried += actualBoarded
				}
			}
		} else if !isDocked {
			ab.LastExchangedStopID = -1
		}

		// Boundary clearance check to keep AI bus safely on road
		bRes := world.CheckBoundaries(ab.Bus.Pos, ab.Bus.Width/2, laneWidth)
		if bRes.Zone == ZoneHardBarrier {
			ab.Bus.Pos = ab.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 4.0))
			ab.Bus.Speed = 0
		}

		// Deadlock prevention between motionless AI buses (> 1 traffic light cycle threshold)
		if math.Abs(ab.Bus.Speed) < 3.0 {
			isBusDeadlocked := false
			playerBlocking := false
			if playerBus != nil && ab.Bus.Pos.Distance(playerBus.Pos) < 180.0 {
				toBus := playerBus.Pos.Sub(ab.Bus.Pos)
				cosH := math.Cos(ab.Bus.Heading)
				sinH := math.Sin(ab.Bus.Heading)
				fwd := toBus.X*cosH + toBus.Y*sinH
				lat := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
				if fwd > -15.0 && lat < laneWidth*1.5 {
					playerBlocking = true
				}
			}

			if !playerBlocking {
				for _, other := range abm.Buses {
					if other.ID == ab.ID || other.Bus == nil || math.Abs(other.Bus.Speed) >= 3.0 {
						continue
					}
					// If facing the same direction and the other bus is not deadlocked (e.g. legitimately waiting in queue),
					// this is normal queue following, not a deadlock
					headDiff := math.Abs(NormalizeAngle(ab.Bus.Heading - other.Bus.Heading))
					if headDiff < 0.5 && other.DeadlockTimer == 0 {
						continue
					}

					dist := ab.Bus.Pos.Distance(other.Bus.Pos)
					if dist < (ab.Bus.Length+other.Bus.Length)*0.5+40.0 {
						toOther := other.Bus.Pos.Sub(ab.Bus.Pos)
						cosH := math.Cos(ab.Bus.Heading)
						sinH := math.Sin(ab.Bus.Heading)
						fwd := toOther.X*cosH + toOther.Y*sinH
						if fwd > -15.0 || dist < (ab.Bus.Length+other.Bus.Length)*0.5+15.0 {
							isBusDeadlocked = true
							break
						}
					}
				}
			}

			if isBusDeadlocked {
				ab.DeadlockTimer += dt
			} else {
				if ab.DeadlockTimer > 0 {
					ab.DeadlockTimer = math.Max(0, ab.DeadlockTimer-dt*2.0)
				}
			}

			if ab.DeadlockTimer >= DeadlockDurationThreshold {
				// Disappear from deadlock zone and reposition to next route stop
				ab.DeadlockTimer = 0
				if len(ab.Route.StopIDs) > 0 && len(world.Stops) > 0 {
					ab.Route.AdvanceToNextStop()
					nextID := ab.Route.GetCurrentTargetStopID()
					if nextID >= 0 && nextID < len(world.Stops) {
						st := world.Stops[nextID]
						ab.Bus.Pos = st.Pos
						ab.Bus.Heading = st.Heading
						ab.Bus.Speed = 0
						if ab.Driver != nil {
							ab.Driver.Reset()
						}
					}
				}
			}
		} else {
			ab.DeadlockTimer = 0
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
