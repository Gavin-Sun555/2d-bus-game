package game

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

const saveVersion = 1
const maxSaveBytes = 32 << 20

// Version 1 originally allowed 1152 civilian cars. Validate the whole legacy
// payload before adapting only the ambient population to the current limit.
const maxCompatibleSavedCars = 1152

type saveEnvelope struct {
	Version  int             `json:"version"`
	SavedAt  time.Time       `json:"saved_at"`
	Name     string          `json:"name,omitempty"`
	Checksum string          `json:"checksum"`
	Data     json.RawMessage `json:"data"`
}
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
type gameSave struct {
	World                          *sim.World
	Settings                       ui.Settings
	Lines                          []*sim.Route
	LineCount, PlayerLine          int
	LineFleets                     [6]traffic.LineFleetSettings
	Player                         savedBus
	Fleet                          []savedFleetBus
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
	RouteModified, HideOtherRoutes bool
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
func (g *Game) snapshot() gameSave {
	ids := map[*sim.Bus]int{g.Bus: 0}
	for _, b := range g.AIBusMgr.Buses {
		ids[b.Bus] = b.ID
	}
	s := gameSave{World: g.World, Settings: *g.Settings, Lines: append([]*sim.Route(nil), g.Lines...), LineCount: g.LineCount, PlayerLine: g.PlayerLine, LineFleets: g.LineFleets,
		Population: g.TrafficMgr.PopulationSnapshot(), Lights: g.TrafficLights, Operations: g.Operations.Snapshot(), Cash: g.Cash, Camera: *g.Camera,
		Exchange: savedExchange{g.boardedThisStop, g.stopBellRung, g.bellToastShown, g.passengersAlighting, g.skipCooldown, g.lastSkippedStopID},
		Planning: g.IsPlanMode, PlanningLine: g.PlayerLine, InitialStops: g.planModeInitialStops, RouteModified: g.planModeRouteModified, HideOtherRoutes: g.HideOtherRoutes}
	s.Lines[g.PlayerLine] = g.Route
	for i, r := range s.Lines {
		if r == g.planningRoute() {
			s.PlanningLine = i
		}
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
	s.Player = snapshotBus(g.Bus, g.AutopilotDriver, ids)
	for _, b := range g.AIBusMgr.Buses {
		vehicle := *b
		vehicle.Bus = nil
		vehicle.Driver = nil
		s.Fleet = append(s.Fleet, savedFleetBus{Vehicle: vehicle, State: snapshotBus(b.Bus, b.Driver, ids)})
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
		s.Cars = append(s.Cars, sc)
	}
	return s
}

// SaveGame writes one local slot atomically; failed writes retain the old slot.
func (g *Game) SaveGame() error {
	if g.State != StatePlaying {
		return fmt.Errorf("no active game to save")
	}
	payload, err := json.Marshal(g.snapshot())
	if err != nil {
		return err
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
	digest := sha256.Sum256(payload)
	data, err := json.Marshal(saveEnvelope{
		Version:  saveVersion,
		SavedAt:  time.Now(),
		Name:     name,
		Checksum: hex.EncodeToString(digest[:]),
		Data:     payload,
	})
	if err != nil {
		return err
	}
	if len(data) > maxSaveBytes {
		return fmt.Errorf("save exceeds size limit")
	}
	return writeSaveFileAtomically(path, data)
}

func writeSaveFileAtomically(path string, data []byte) error {
	if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
		return err
	}
	f, err := os.CreateTemp(filepath.Dir(path), ".save-*.tmp")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	if _, err = f.Write(data); err != nil {
		f.Close()
		return err
	}
	if err = f.Sync(); err != nil {
		f.Close()
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	return os.Rename(f.Name(), path)
}

func readSaveEnvelope(path string) (saveEnvelope, error) {
	var envelope saveEnvelope
	f, err := os.Open(path)
	if err != nil {
		return envelope, err
	}
	defer f.Close()
	raw, err := io.ReadAll(io.LimitReader(f, maxSaveBytes+1))
	if err != nil {
		return envelope, err
	}
	if len(raw) > maxSaveBytes {
		return envelope, fmt.Errorf("save exceeds size limit")
	}
	if err = json.Unmarshal(raw, &envelope); err != nil {
		return envelope, fmt.Errorf("invalid save file: %w", err)
	}
	if envelope.Version != saveVersion {
		return envelope, fmt.Errorf("unsupported save version %d", envelope.Version)
	}
	digest := sha256.Sum256(envelope.Data)
	if hex.EncodeToString(digest[:]) != envelope.Checksum {
		return envelope, fmt.Errorf("save checksum mismatch")
	}
	return envelope, nil
}

func readSave(path string) (gameSave, error) {
	var s gameSave
	envelope, err := readSaveEnvelope(path)
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
	if b == nil || !b.PassengerService.Valid() || !b.Model.Valid() || b.GeometryVersion < 0 || b.GeometryVersion > 1 || (b.PendingModel != nil && !b.PendingModel.Valid()) || b.Length <= 0 || b.Width <= 0 || b.WheelBase <= 0 || b.Capacity < 1 || b.Capacity > 1000 || b.PassengerCount != len(b.Passengers) || len(b.Passengers) > b.Capacity || b.Durability < 0 || b.Durability > 100 || b.MaxSpeedFwd <= 0 {
		return false
	}
	for _, p := range b.Passengers {
		if p.TargetStopID < 0 || p.TargetStopID >= stops || p.PaidFare < 0 || p.PaidFare > 10 {
			return false
		}
	}
	return true
}
func (s *gameSave) validate() error {
	bad := func() error { return fmt.Errorf("invalid or incompatible save data") }
	if s.Settings.PassengerRates != nil {
		for _, rates := range s.Settings.PassengerRates {
			if !rates.Valid() {
				return bad()
			}
		}
	}
	if s.World == nil || s.Lights == nil || len(s.World.Roads) == 0 || len(s.World.Stops) == 0 || len(s.World.Stops) > 10000 || s.World.Theme < sim.MapThemeMetropolis || s.World.Theme > sim.MapThemeCustom || s.Settings.MapTheme != s.World.Theme {
		return bad()
	}
	for i, stop := range s.World.Stops {
		if stop == nil || stop.ID != i || stop.WaitingPassengers < 0 {
			return bad()
		}
	}
	if len(s.Lines) != 6 || s.LineCount < 1 || s.LineCount > 6 || s.PlayerLine < 0 || s.PlayerLine >= s.LineCount || s.PlanningLine < 0 || s.PlanningLine >= s.LineCount {
		return bad()
	}
	n := len(s.World.Stops)
	for i, r := range s.Lines {
		if !validSavedRoute(r, n, i >= s.LineCount || (s.Planning && i == s.PlanningLine)) {
			return bad()
		}
	}
	if !validSavedBus(&s.Player.Bus, n) || len(s.Fleet) > 48 || len(s.Cars) > maxCompatibleSavedCars || len(s.Population.PendingExits)+len(s.Cars) > maxCompatibleSavedCars {
		return bad()
	}
	if !s.Settings.AnnouncementVoice.Valid() || !s.Settings.Announcements.Valid() || s.Settings.TrafficDensity < 1 || s.Settings.TrafficDensity > sim.TrafficDensityLevels || s.Camera.Zoom <= 0 || s.Camera.TargetZoom <= 0 || s.Settings.Language < locale.LanguageZH || s.Settings.Language > locale.LanguageEN {
		return bad()
	}
	if s.Settings.CameraViewMode < sim.CameraModeNorthUp || s.Settings.CameraViewMode > sim.CameraModeSouthUp {
		return bad()
	}
	carRange := sim.SpeedRange{MinKPH: s.Settings.AICarMinSpeed, MaxKPH: s.Settings.AICarMaxSpeed}
	if carRange != carRange.Normalize(60, 100) || s.Settings.Fare != s.Settings.Fare.Normalize() {
		return bad()
	}
	for _, c := range s.LineFleets {
		if c != c.Normalize() {
			return bad()
		}
	}
	ids := map[int]bool{0: true}
	counts := [6]int{}
	for _, b := range s.Fleet {
		v := b.Vehicle
		if v.ID < 1 || v.ID > 48 || ids[v.ID] || v.LineIndex < 0 || v.LineIndex >= s.LineCount || ((v.ID-1)/traffic.MaxBusesPerLine) != v.LineIndex || !validSavedBus(&b.State.Bus, n) || !validSavedRoute(v.Route, n, false) {
			return bad()
		}
		ids[v.ID] = true
		if !v.Retiring {
			counts[v.LineIndex]++
		}
	}
	for i := 0; i < s.LineCount; i++ {
		if len(s.Lines[i].StopIDs) > 0 && counts[i] != s.LineFleets[i].BusCount {
			return bad()
		}
	}
	seenCars := map[int]bool{}
	for _, c := range s.Cars {
		if c.Car.ID < 0 || seenCars[c.Car.ID] || c.Car.Length <= 0 || c.Car.Width <= 0 || c.Car.ID >= s.Population.NextCarID {
			return bad()
		}
		seenCars[c.Car.ID] = true
		if t := c.Car.JunctionTurn; t != nil && (len(t.Points) < 2 || t.Segment < 0 || t.Segment >= len(t.Points)) {
			return bad()
		}
	}
	for _, i := range s.Lights.Intersections {
		if i == nil || i.Phase < sim.PhaseEWGreen || i.Phase > sim.PhaseNSLeftYellow {
			return bad()
		}
	}
	for _, c := range s.Lights.RoadCameras {
		if c == nil {
			return bad()
		}
	}
	for _, v := range s.Operations.Vehicles {
		if v.Public.Route != nil && !validSavedRoute(v.Public.Route, n, true) {
			return bad()
		}
	}
	return nil
}

// LoadGame prepares and validates a separate game before replacing the live one.
func (g *Game) LoadGame() error {
	path, err := g.savePath()
	if err != nil {
		return err
	}
	s, err := readSave(path)
	if err != nil {
		return err
	}
	if err = s.validate(); err != nil {
		return err
	}
	s.limitAmbientPopulation()
	// This is a game-wide guardrail, not a saved passenger-growth setting.
	// Upgrade old saves without resetting their waiting counts or spawn clock.
	for _, stop := range s.World.Stops {
		stop.MaxWaiting = sim.DefaultStopMaxWaiting
	}
	sim.StandardizeStopBays(s.World, s.Settings.GetLaneWidth())
	next := &Game{Width: g.Width, Height: g.Height, State: StatePlaying, SelectedMapTheme: s.World.Theme, World: s.World, Settings: &s.Settings, Lines: s.Lines,
		LineCount: s.LineCount, PlayerLine: s.PlayerLine, LineFleets: s.LineFleets, Bus: &s.Player.Bus, Camera: &s.Camera, HUD: ui.NewHUD(), TrafficLights: s.Lights,
		TrafficMgr: &traffic.TrafficManager{}, AIBusMgr: traffic.NewAIBusManager(), OperationsPanel: ui.NewOperationsPanel(), Cash: s.Cash,
		IsPlanMode: s.Planning, HideOtherRoutes: s.HideOtherRoutes, planModeInitialStops: s.InitialStops, planModeRouteModified: s.RouteModified, savePathOverride: g.savePathOverride, activeSaveSlot: g.activeSaveSlot}
	next.Route = next.Lines[next.PlayerLine]
	next.NetworkSelected = next.PlayerLine
	if next.IsPlanMode {
		next.planRoute = next.Lines[s.PlanningLine]
		next.NetworkSelected = s.PlanningLine
	}
	next.AutopilotDriver, err = s.Player.Driver.Restore(next.World)
	if err != nil {
		return err
	}
	buses := map[int]*sim.Bus{0: next.Bus}
	routes := map[int]*sim.Route{0: next.Route}
	for _, saved := range s.Fleet {
		v := saved.Vehicle
		v.Bus = &saved.State.Bus
		v.Driver, err = saved.State.Driver.Restore(next.World)
		if err != nil {
			return err
		}
		next.AIBusMgr.Buses = append(next.AIBusMgr.Buses, &v)
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
	next.AIBusMgr.Operations = next.Operations
	next.operationsRoute = append([]int(nil), next.Route.StopIDs...)
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
	next.Settings.IsOpen = false
	next.Settings.OpenOperationsRequested = false
	next.Settings.OpenNetworkRequested = false
	next.Settings.SaveRequested = false
	next.Settings.LoadRequested = false
	next.Settings.ReturnToMenuRequested = false
	next.repairSharedTerminusQueues()
	// Device/player ownership belongs to this running session, not the save.
	g.resetAnnouncements()
	next.announcements = g.announcements
	next.vehicleAudio = g.vehicleAudio
	next.stopNames = g.stopNames
	g.closeStopNameEditor()
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
