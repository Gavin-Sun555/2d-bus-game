package game

import (
	"encoding/json"
	"fmt"
	"math"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

// Version 1 originally allowed 1152 civilian cars. Validate the whole legacy
// payload before adapting only the ambient population to the current limit.
const maxCompatibleSavedCars = 1152

type savedBus struct {
	Bus        sim.Bus
	YieldingTo int
	Driver     autopilot.DriverSnapshot
}
type savedFleetBus struct {
	Vehicle traffic.AIBus
	State   savedBus
}
type savedCar struct {
	Car                     sim.TrafficCar
	YieldingTo, PassingLead int
}
type savedExchange struct {
	Boarded, Bell, BellToast bool
	Alighting                int
	SkipCooldown             float64
	LastSkipped              int
}
type citySave struct {
	World                          *sim.World
	Lines                          []*sim.Route
	LineCount, PlayerLine          int
	LineFleets                     [6]traffic.LineFleetSettings
	Player                         savedBus
	Fleet                          []savedFleetBus
	LastAIBusID                    int `json:",omitempty"`
	Cars                           []savedCar
	Population                     traffic.PopulationSnapshot
	Lights                         *sim.TrafficLightSystem
	Operations                     sim.OperationsSnapshot
	Camera                         sim.Camera
	Exchange                       savedExchange
	Planning                       bool
	PlanningLine                   int
	InitialStops                   []int
	InitialIndex                   *int `json:",omitempty"`
	RouteModified, HideOtherRoutes bool
}

type gameSave struct {
	Mode                           PlayMode                  `json:",omitempty"`
	Career                         *sim.CareerProfile        `json:",omitempty"`
	Cities                         map[sim.MapTheme]citySave `json:",omitempty"`
	World                          *sim.World
	Settings                       ui.Settings
	Lines                          []*sim.Route
	LineCount, PlayerLine          int
	LineFleets                     [6]traffic.LineFleetSettings
	Player                         savedBus
	Fleet                          []savedFleetBus
	LastAIBusID                    int `json:",omitempty"`
	Cars                           []savedCar
	Population                     traffic.PopulationSnapshot
	Lights                         *sim.TrafficLightSystem
	Operations                     sim.OperationsSnapshot
	Cash                           int
	Camera                         sim.Camera
	Exchange                       savedExchange
	Planning                       bool
	PlanningLine                   int
	InitialStops                   []int
	InitialIndex                   *int `json:",omitempty"`
	RouteModified, HideOtherRoutes bool
}

func (s *gameSave) citySave() citySave {
	return citySave{
		World:           s.World,
		Lines:           s.Lines,
		LineCount:       s.LineCount,
		PlayerLine:      s.PlayerLine,
		LineFleets:      s.LineFleets,
		Player:          s.Player,
		Fleet:           s.Fleet,
		LastAIBusID:     s.LastAIBusID,
		Cars:            s.Cars,
		Population:      s.Population,
		Lights:          s.Lights,
		Operations:      s.Operations,
		Camera:          s.Camera,
		Exchange:        s.Exchange,
		Planning:        s.Planning,
		PlanningLine:    s.PlanningLine,
		InitialStops:    s.InitialStops,
		InitialIndex:    s.InitialIndex,
		RouteModified:   s.RouteModified,
		HideOtherRoutes: s.HideOtherRoutes,
	}
}

func (s *gameSave) setCitySave(cs citySave) {
	s.World = cs.World
	s.Lines = cs.Lines
	s.LineCount = cs.LineCount
	s.PlayerLine = cs.PlayerLine
	s.LineFleets = cs.LineFleets
	s.Player = cs.Player
	s.Fleet = cs.Fleet
	s.LastAIBusID = cs.LastAIBusID
	s.Cars = cs.Cars
	s.Population = cs.Population
	s.Lights = cs.Lights
	s.Operations = cs.Operations
	s.Camera = cs.Camera
	s.Exchange = cs.Exchange
	s.Planning = cs.Planning
	s.PlanningLine = cs.PlanningLine
	s.InitialStops = cs.InitialStops
	s.InitialIndex = cs.InitialIndex
	s.RouteModified = cs.RouteModified
	s.HideOtherRoutes = cs.HideOtherRoutes
}

func cloneCitySave(cs citySave) citySave {
	raw, err := json.Marshal(cs)
	if err != nil {
		return cs
	}
	var out citySave
	if err := json.Unmarshal(raw, &out); err != nil {
		return cs
	}
	return out
}

func cloneCareerCities(cities map[sim.MapTheme]citySave) map[sim.MapTheme]citySave {
	if len(cities) == 0 {
		return nil
	}
	out := make(map[sim.MapTheme]citySave, len(cities))
	for k, v := range cities {
		out[k] = cloneCitySave(v)
	}
	return out
}

func (g *Game) savePath() (string, error) {
	return g.saveSlotPath(g.activeSaveSlot)
}

func snapshotBus(bus *sim.Bus, driver *autopilot.AutopilotDriver, ids map[*sim.Bus]int) savedBus {
	state := savedBus{Bus: *bus, YieldingTo: -1, Driver: driver.Snapshot()}
	if id, ok := ids[bus.YieldingToBus]; ok {
		state.YieldingTo = id
	}
	state.Bus.YieldingToBus = nil
	state.Bus.TireTracks = nil
	return state
}

func (g *Game) snapshotCity() citySave {
	g.World.Clock.Rate = g.Settings.ClockRate
	g.syncAutoHeadlights()
	g.syncCareerCondition()
	ids := make(map[*sim.Bus]int, 1+len(g.AIBusMgr.Buses))
	ids[g.Bus] = 0
	for _, b := range g.AIBusMgr.Buses {
		ids[b.Bus] = b.ID
	}
	cs := citySave{
		World:           g.World,
		Lines:           append([]*sim.Route(nil), g.Lines...),
		LineCount:       g.LineCount,
		PlayerLine:      g.PlayerLine,
		LineFleets:      g.LineFleets,
		LastAIBusID:     g.AIBusMgr.LastBusID,
		Population:      g.TrafficMgr.PopulationSnapshot(),
		Lights:          g.TrafficLights,
		Operations:      g.Operations.Snapshot(),
		Camera:          *g.Camera,
		Exchange:        savedExchange{g.boardedThisStop, g.stopBellRung, g.bellToastShown, g.passengersAlighting, g.skipCooldown, g.lastSkippedStopID},
		Planning:        g.IsPlanMode,
		PlanningLine:    g.PlayerLine,
		InitialStops:    g.planModeInitialStops,
		RouteModified:   g.planModeRouteModified,
		HideOtherRoutes: g.HideOtherRoutes,
	}
	cs.Lines[g.PlayerLine] = g.Route
	if g.IsPlanMode {
		cs.InitialIndex = &g.planModeInitialIndex
	}
	for i, r := range cs.Lines {
		if r == g.planningRoute() {
			cs.PlanningLine = i
		}
	}
	cs.Player = snapshotBus(g.Bus, g.AutopilotDriver, ids)
	if n := len(g.AIBusMgr.Buses); n > 0 {
		cs.Fleet = make([]savedFleetBus, 0, n)
	}
	if n := len(g.TrafficMgr.Cars); n > 0 {
		cs.Cars = make([]savedCar, 0, n)
	}
	for _, b := range g.AIBusMgr.Buses {
		vehicle := *b
		vehicle.Bus = nil
		vehicle.Driver = nil
		cs.Fleet = append(cs.Fleet, savedFleetBus{Vehicle: vehicle, State: snapshotBus(b.Bus, b.Driver, ids)})
	}
	for _, c := range g.TrafficMgr.Cars {
		sc := savedCar{Car: *c, YieldingTo: -1, PassingLead: -1}
		if id, ok := ids[c.YieldingToBus]; ok {
			sc.YieldingTo = id
		}
		if id, ok := ids[c.PassingLeadBus]; ok {
			sc.PassingLead = id
		}
		sc.Car.YieldingToBus = nil
		sc.Car.PassingLeadBus = nil
		cs.Cars = append(cs.Cars, sc)
	}
	return cs
}

func (g *Game) snapshot() gameSave {
	cs := g.snapshotCity()
	s := gameSave{Mode: g.Mode, Career: g.Career, Settings: *g.Settings, Cash: g.Cash}
	s.setCitySave(cs)
	if g.isCareer() && g.World != nil {
		if g.careerCities == nil {
			g.careerCities = make(map[sim.MapTheme]citySave)
		}
		g.careerCities[g.World.Theme] = cs
		s.Cities = g.careerCities
	}
	s.Settings.SaveRequested = false
	s.Settings.LoadRequested = false
	s.Settings.IsOpen = false
	s.Settings.OpenNetworkRequested = false
	s.Settings.OpenOperationsRequested = false
	s.Settings.ReturnToMenuRequested = false
	s.Settings.ThemeChanged = false
	s.Settings.AIBusFleetChanged = false
	s.Settings.CameraViewMode = g.Camera.Mode
	return s
}

// SaveGame writes one local slot atomically; failed writes retain the old slot.
func (g *Game) SaveGame() error {
	if g.isCareer() {
		if g.State != StateMainMenu && g.State != StatePlaying {
			return fmt.Errorf("no active game to save")
		}
		g.prepareCareerSave()
	} else if g.State != StatePlaying {
		return fmt.Errorf("no active game to save")
	}
	path, err := g.savePath()
	if err != nil {
		return err
	}
	name := readSaveSlotName(path)
	if g.pendingSaveName != nil {
		name, err = validateSaveSlotName(*g.pendingSaveName)
		if err != nil {
			return err
		}
	}
	return g.saveSnapshotFile(path, name, g.saveMetadata("manual", ""), time.Now())
}

func readSave(path string) (gameSave, error) {
	var s gameSave
	envelope, err := savefile.Read[SaveMetadata](path)
	if err != nil {
		return s, err
	}
	if err = json.Unmarshal(envelope.Data, &s); err != nil {
		return s, fmt.Errorf("invalid save data: %w", err)
	}
	return s, nil
}

func validSavedRoute(r *sim.Route, stops int, empty bool) bool {
	if r == nil || len(r.StopIDs) > 10000 || r.CurrentIndex < 0 || r.TotalTrips < 0 {
		return false
	}
	if len(r.StopIDs) == 0 {
		return empty && r.CurrentIndex == 0
	}
	if r.CurrentIndex >= len(r.StopIDs) {
		return false
	}
	for _, id := range r.StopIDs {
		if id < 0 || id >= stops {
			return false
		}
	}
	return true
}
func validSavedBus(b *sim.Bus, stops int) bool {
	if b == nil || !b.PassengerService.Valid() || !b.Model.Valid() || b.GeometryVersion < 0 || b.GeometryVersion > 1 || (b.PendingModel != nil && !b.PendingModel.Valid()) || b.Length <= 0 || b.Width <= 0 || b.WheelBase <= 0 || b.Capacity < 1 || b.Capacity > 1000 || b.PassengerCount != len(b.Passengers) || len(b.Passengers) > max(b.Capacity, b.Model.MaxPassengerCapacity()) || b.Durability < 0 || b.Durability > 100 || b.MaxSpeedFwd <= 0 {
		return false
	}
	for _, p := range b.Passengers {
		if p.TargetStopID < 0 || p.TargetStopID >= stops || p.PaidFare < 0 || p.PaidFare > 10 {
			return false
		}
	}
	return true
}
func (cs *citySave) validateCity() error {
	bad := func() error { return fmt.Errorf("invalid or incompatible save data") }
	if cs.World == nil || !cs.World.Clock.Valid() || cs.Lights == nil || len(cs.World.Roads) == 0 || len(cs.World.Stops) == 0 || len(cs.World.Stops) > 10000 || cs.World.Theme < sim.MapThemeMetropolis || cs.World.Theme > sim.MapThemeCustom {
		return bad()
	}
	n := len(cs.World.Stops)
	for i, stop := range cs.World.Stops {
		if stop == nil || stop.ID != i || stop.WaitingPassengers < 0 {
			return bad()
		}
		for _, dest := range stop.WaitingDestinations {
			if dest != -1 && (dest < 0 || dest >= n) {
				return bad()
			}
		}
	}
	if len(cs.Lines) != 6 || cs.LineCount < 1 || cs.LineCount > 6 || cs.PlayerLine < 0 || cs.PlayerLine >= cs.LineCount || cs.PlanningLine < 0 || cs.PlanningLine >= cs.LineCount {
		return bad()
	}
	for i, r := range cs.Lines {
		if !validSavedRoute(r, n, i >= cs.LineCount || (cs.Planning && i == cs.PlanningLine)) {
			return bad()
		}
	}
	if cs.Planning && !validSavedRoute(&sim.Route{StopIDs: cs.InitialStops}, n, true) {
		return bad()
	}
	if !validSavedBus(&cs.Player.Bus, n) || len(cs.Fleet) > traffic.MaxFleetBuses || len(cs.Cars) > maxCompatibleSavedCars || len(cs.Population.PendingExits)+len(cs.Cars) > maxCompatibleSavedCars || cs.Camera.Zoom <= 0 || cs.Camera.TargetZoom <= 0 {
		return bad()
	}
	for _, c := range cs.LineFleets {
		if c != c.Normalize() {
			return bad()
		}
	}
	ids := map[int]bool{0: true}
	slots := map[int]bool{}
	counts := [6]int{}
	const maxSavedBusID = math.MaxInt - traffic.MaxFleetBuses
	if cs.LastAIBusID < 0 || cs.LastAIBusID > maxSavedBusID {
		return bad()
	}
	for _, b := range cs.Fleet {
		v := b.Vehicle
		slot := v.FleetSlotID()
		if slot == 0 || slots[slot] || ids[v.ID] || v.LineIndex >= cs.LineCount || (v.ID > traffic.MaxFleetBuses && v.ID > cs.LastAIBusID) || !validSavedBus(&b.State.Bus, n) || !validSavedRoute(v.Route, n, false) {
			return bad()
		}
		slots[slot] = true
		ids[v.ID] = true
		if !v.Retiring {
			counts[v.LineIndex]++
		}
	}
	for i := 0; i < cs.LineCount; i++ {
		if len(cs.Lines[i].StopIDs) > 0 && counts[i] != cs.LineFleets[i].BusCount {
			return bad()
		}
	}
	seenCars := map[int]bool{}
	for _, c := range cs.Cars {
		if c.Car.ID < 0 || seenCars[c.Car.ID] || c.Car.Length <= 0 || c.Car.Width <= 0 || c.Car.ID >= cs.Population.NextCarID {
			return bad()
		}
		seenCars[c.Car.ID] = true
		if t := c.Car.JunctionTurn; t != nil && (len(t.Points) < 2 || t.Segment < 0 || t.Segment >= len(t.Points)) {
			return bad()
		}
	}
	for _, i := range cs.Lights.Intersections {
		if i == nil || i.Phase < sim.PhaseEWGreen || i.Phase > sim.PhaseNSLeftYellow {
			return bad()
		}
	}
	for _, c := range cs.Lights.RoadCameras {
		if c == nil {
			return bad()
		}
	}
	for id, v := range cs.Operations.Vehicles {
		if id < 0 || id > maxSavedBusID || (v.Public.Route != nil && !validSavedRoute(v.Public.Route, n, true)) {
			return bad()
		}
	}
	return nil
}

func (s *gameSave) validate() error {
	bad := func() error { return fmt.Errorf("invalid or incompatible save data") }
	if s.Mode != ModeFast && s.Mode != ModeCareer {
		return bad()
	}
	if s.Mode == ModeFast && s.Career != nil {
		return bad()
	}
	if s.Mode == ModeCareer {
		if err := s.Career.Validate(); err != nil {
			return err
		}
		if s.World == nil || !s.Career.MapUnlocked(s.World.Theme) || s.Cash != int(math.Floor(s.Career.Balance)) {
			return bad()
		}
		seen := map[int]bool{}
		validOwned := func(b *sim.Bus, player bool, line int) bool {
			v := s.Career.Vehicle(b.OwnedVehicleID)
			if v == nil || seen[v.ID] || v.SpecID != b.CatalogID {
				return false
			}
			spec, _ := sim.FindVehicleSpec(v.SpecID)
			if b.Model != spec.Model || b.PendingModel != nil || b.Durability != v.Condition || (player && v.ID != s.Career.PlayerVehicleID) || (!player && (v.ID == s.Career.PlayerVehicleID || v.Line != line)) {
				return false
			}
			seen[v.ID] = true
			return true
		}
		if !validOwned(&s.Player.Bus, true, s.PlayerLine) {
			return bad()
		}
		for _, v := range s.Fleet {
			if !validOwned(&v.State.Bus, false, v.Vehicle.LineIndex) {
				return bad()
			}
		}
		for theme, city := range s.Cities {
			if city.World == nil || city.World.Theme != theme || !s.Career.MapUnlocked(theme) {
				return bad()
			}
			if err := city.validateCity(); err != nil {
				return err
			}
		}
	}
	if s.Settings.PassengerCapacities != nil {
		for model, capacity := range s.Settings.PassengerCapacities {
			if capacity < 0 || capacity > sim.BusModel(model).MaxPassengerCapacity() {
				return bad()
			}
		}
	}
	if s.Settings.PassengerOverloadPercents != nil {
		for _, percent := range s.Settings.PassengerOverloadPercents {
			if percent < 0 || percent > 50 || percent%10 != 0 {
				return bad()
			}
		}
	}
	if s.Settings.PassengerRates != nil {
		for _, rates := range s.Settings.PassengerRates {
			if !rates.Valid() {
				return bad()
			}
		}
	}
	if !s.Settings.ClockRate.Valid() || s.World == nil || s.Settings.MapTheme != s.World.Theme {
		return bad()
	}
	if !s.Settings.AnnouncementVoice.Valid() || !s.Settings.Announcements.Valid() || s.Settings.TrafficDensity < 1 || s.Settings.TrafficDensity > sim.TrafficDensityLevels || s.Settings.Language < locale.LanguageZH || s.Settings.Language > locale.LanguageEN || !s.Settings.PassengerSpawnRate.Valid() {
		return bad()
	}
	if s.Settings.CameraViewMode < sim.CameraModeNorthUp || s.Settings.CameraViewMode > sim.CameraModeSouthUp {
		return bad()
	}
	carRange := sim.SpeedRange{MinKPH: s.Settings.AICarMinSpeed, MaxKPH: s.Settings.AICarMaxSpeed}
	if carRange != carRange.Normalize(60, sim.TrafficMaxSpeedKPH) || s.Settings.Fare != s.Settings.Fare.Normalize() {
		return bad()
	}
	cs := s.citySave()
	return cs.validateCity()
}

// LoadGame prepares and validates a separate game before replacing the live one.
func (g *Game) LoadGame() error {
	path, err := g.savePath()
	if err != nil {
		return err
	}
	return g.loadGamePath(path)
}

func (g *Game) restoreCareerCity(saved citySave) error {
	cloned := cloneCitySave(saved)
	s := gameSave{
		Mode:     g.Mode,
		Career:   g.Career,
		Cities:   g.careerCities,
		Settings: *g.Settings,
		Cash:     int(math.Floor(g.Career.Balance)),
	}
	s.setCitySave(cloned)
	if s.World != nil {
		s.Settings.MapTheme = s.World.Theme
		s.Settings.LargeRoadNetwork = s.World.LargeGrid
		s.Settings.CenterBRT = s.World.CenterBRT
	}
	if err := g.restoreFromSave(s); err != nil {
		return err
	}
	g.reconcileCareerCityFleet()
	return nil
}

func (g *Game) loadGamePath(path string) error {
	s, err := readSave(path)
	if err != nil {
		return err
	}
	if s.Mode != g.Mode {
		return fmt.Errorf("存档属于另一种游戏模式 / Save belongs to another play mode")
	}
	if err = s.validate(); err != nil {
		return err
	}
	return g.restoreFromSave(s)
}

func (g *Game) restoreFromSave(s gameSave) error {
	var err error
	s.Settings.AutopilotBerserkMaxSpeed = s.Settings.BerserkSpeedKPH()
	s.limitAmbientPopulation()
	// This is a game-wide guardrail, not a saved passenger-growth setting.
	// Upgrade old saves without resetting their waiting counts or spawn clock.
	for _, stop := range s.World.Stops {
		stop.MaxWaiting = sim.DefaultStopMaxWaiting
		stop.SyncWaitingQueue(nil)
	}
	s.World.RefreshGeneratedStopNames()
	s.World.ConfigureCenterBRT(s.Settings.GetLaneWidth())
	sim.StandardizeStopBays(s.World, s.Settings.GetLaneWidth())
	var careerCities map[sim.MapTheme]citySave
	if s.Mode == ModeCareer {
		careerCities = cloneCareerCities(s.Cities)
		if careerCities == nil {
			careerCities = make(map[sim.MapTheme]citySave)
		}
		if s.World != nil {
			careerCities[s.World.Theme] = cloneCitySave(s.citySave())
		}
	}
	next := &Game{trafficDebug: g.trafficDebug, autosavesEnabled: g.autosavesEnabled, sessionStarted: true, lastAutosaveAt: time.Now(), Mode: s.Mode, Career: s.Career, careerCities: careerCities, modeSessions: g.modeSessions, Width: g.Width, Height: g.Height, State: StatePlaying, SelectedMapTheme: s.World.Theme, World: s.World, Settings: &s.Settings, Lines: s.Lines,
		LineCount: s.LineCount, PlayerLine: s.PlayerLine, LineFleets: s.LineFleets, Bus: &s.Player.Bus, Camera: &s.Camera, HUD: ui.NewHUD(), TrafficLights: s.Lights,
		TrafficMgr: &traffic.TrafficManager{}, AIBusMgr: traffic.NewAIBusManager(), OperationsPanel: ui.NewOperationsPanel(), Cash: s.Cash,
		IsPlanMode: s.Planning, HideOtherRoutes: s.HideOtherRoutes, planModeInitialStops: s.InitialStops, planModeRouteModified: s.RouteModified, savePathOverride: g.savePathOverride, activeSaveSlot: g.activeSaveSlot}
	next.Settings.CareerMode = next.isCareer()
	if next.World != nil {
		next.World.PassengerSpawnRate = s.Settings.PassengerSpawnRate
		next.Settings.MapTheme = next.World.Theme
		next.Settings.SetCenterBRT(next.World.Theme, next.World.CenterBRT)
		next.SelectedMapTheme = next.World.Theme
		if next.Settings.LargeRoadNetworks != nil {
			next.Settings.LargeRoadNetworks[next.World.Theme] = next.World.LargeGrid
		} else if next.World.LargeGrid {
			next.Settings.SetLargeRoadNetwork(next.World.Theme, true)
		}
		next.Settings.LargeRoadNetwork = next.World.LargeGrid
	}
	next.Route = next.Lines[next.PlayerLine]
	next.NetworkSelected = next.PlayerLine
	if next.IsPlanMode {
		next.planRoute = next.Lines[s.PlanningLine]
		next.NetworkSelected = s.PlanningLine
		next.planModeInitialIndex = next.planRoute.CurrentIndex
		if s.InitialIndex != nil {
			next.planModeInitialIndex = max(0, min(*s.InitialIndex, len(s.InitialStops)-1))
		}
	}
	next.AutopilotDriver, err = s.Player.Driver.Restore(next.World)
	if err != nil {
		return err
	}
	buses := map[int]*sim.Bus{0: next.Bus}
	routes := map[int]*sim.Route{0: next.Route}
	next.AIBusMgr.LastBusID = s.LastAIBusID
	for _, saved := range s.Fleet {
		v := saved.Vehicle
		v.Bus = &saved.State.Bus
		// Older saves can have a speed cap without its source range.
		if v.AssignedSpeedRange == (sim.SpeedRange{}) {
			v.AssignedSpeedRange = next.LineFleets[v.LineIndex].SpeedRange
		}
		if v.MaxSpeedKPH == 0 {
			v.MaxSpeedKPH = int(math.Round(v.Bus.MaxSpeedFwd * sim.MetersPerPixel * 3.6))
		}
		v.Driver, err = saved.State.Driver.Restore(next.World)
		if err != nil {
			return err
		}
		next.AIBusMgr.Buses = append(next.AIBusMgr.Buses, &v)
		next.AIBusMgr.LastBusID = max(next.AIBusMgr.LastBusID, v.ID)
		buses[v.ID] = v.Bus
		routes[v.ID] = v.Route
	}
	// Older articulated saves used the short body. Finish their current lap
	// before fitting the longer chassis, rather than spawning a trailer in traffic.
	for id, b := range buses {
		if b.Model == sim.BusArticulated && b.GeometryVersion == 0 && b.PendingModel == nil {
			b.RequestModel(sim.BusArticulated)
		}
		if b.Model == sim.BusArticulated || (b.PendingModel != nil && *b.PendingModel == sim.BusArticulated) {
			sim.EnsureLongBusBays(next.World, routes[id])
		}
	}
	next.Bus.YieldingToBus = buses[s.Player.YieldingTo]
	for i, v := range next.AIBusMgr.Buses {
		v.Bus.YieldingToBus = buses[s.Fleet[i].State.YieldingTo]
	}
	for _, saved := range s.Cars {
		car := saved.Car
		car.YieldingToBus = buses[saved.YieldingTo]
		car.PassingLeadBus = buses[saved.PassingLead]
		next.TrafficMgr.Cars = append(next.TrafficMgr.Cars, &car)
	}
	next.TrafficMgr.RestorePopulation(s.Population)
	next.TrafficMgr.AIBusMgr = next.AIBusMgr
	next.AIBusMgr.ActiveRoute = next.Route
	next.AIBusMgr.Routes = next.activeLines()
	next.AIBusMgr.BusCount = len(next.AIBusMgr.Buses)
	next.AIBusMgr.CurrentMap = next.World.Theme
	next.AIBusMgr.CurrentLane = next.Settings.GetLaneWidth()
	sim.ClearRoutePathCache()
	next.Operations = sim.RestoreOperations(s.Operations, next.World, next.Route, next.Settings.GetLaneWidth(), next.Settings.Fare, buses, routes)
	for id := range next.Operations.Vehicles {
		next.AIBusMgr.LastBusID = max(next.AIBusMgr.LastBusID, id)
	}
	next.AIBusMgr.Operations = next.Operations
	next.careerRevenueMark = next.Operations.Revenue
	next.operationsRoute = append([]int(nil), next.operationsStops()...)
	next.operationsLaneWidth = next.Settings.GetLaneWidth()
	next.lastLaneWidth = next.Settings.GetLaneWidth()
	next.boardedThisStop = s.Exchange.Boarded
	next.stopBellRung = s.Exchange.Bell
	next.bellToastShown = s.Exchange.BellToast
	next.passengersAlighting = s.Exchange.Alighting
	next.skipCooldown = s.Exchange.SkipCooldown
	next.lastSkippedStopID = s.Exchange.LastSkipped
	next.Camera.ScreenWidth = g.Width
	next.Camera.ScreenHeight = g.Height
	next.showPlayerLineSettings()
	next.syncAISpeedRanges()
	next.syncPassengerCapacities()
	next.Settings.IsOpen = false
	next.Settings.OpenOperationsRequested = false
	next.Settings.OpenNetworkRequested = false
	next.Settings.SaveRequested = false
	next.Settings.LoadRequested = false
	next.Settings.ReturnToMenuRequested = false
	next.repairSharedTerminusQueues()
	next.repairInvalidInitialPlayerSpawn()
	next.World.Clock.Rate = next.Settings.ClockRate
	next.syncAutoHeadlights()
	// Device/player ownership belongs to this running session, not the save.
	g.resetAnnouncements()
	next.announcements = g.announcements
	next.vehicleAudio = g.vehicleAudio
	next.stopNames = g.stopNames
	g.closeStopNameEditor()
	g.closeStationSearch()
	g.finishIntervalAutosave(true)
	*g = *next
	g.applyStopNames()
	sim.ActiveWorldForRoute = g.World
	sim.ClearRoutePathCache()
	locale.CurrentLanguage = g.Settings.Language
	return nil
}

// Keep the retained cars in their original order, along with stable IDs. Only
// old ambient cars/replacement requests beyond the new capacity are removed.
func (s *gameSave) limitAmbientPopulation() {
	if len(s.Cars) > sim.MaxTrafficCars {
		s.Cars = s.Cars[:sim.MaxTrafficCars]
	}
	available := sim.MaxTrafficCars - len(s.Cars)
	if len(s.Population.PendingExits) > available {
		s.Population.PendingExits = s.Population.PendingExits[:available]
	}
}
