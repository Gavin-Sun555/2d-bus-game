package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

type PlayMode int

const (
	ModeFast PlayMode = iota
	ModeCareer
)

func (m PlayMode) String() string {
	if m == ModeCareer {
		return "Career"
	}
	return "Fast"
}
func (g *Game) isCareer() bool     { return g.Mode == ModeCareer && g.Career != nil }
func (g *Game) ShowModeSelection() { g.State = StateModeSelect; g.careerOffice.open = false }

// Keep unsaved sessions separate in memory as well as on disk.
func (g *Game) SelectMode(mode PlayMode) {
	if mode != ModeFast && mode != ModeCareer {
		return
	}
	if g.modeSessions == nil {
		g.modeSessions = map[PlayMode]*Game{}
	}
	sessions := g.modeSessions
	if g.isCareer() {
		g.leaveCurrentCareerCity()
	}
	previous := *g
	previous.State = StateMainMenu
	sessions[g.Mode] = &previous
	target := sessions[mode]
	if target == nil {
		target = NewGame(g.Width, g.Height)
		target.Mode = mode
		target.savePathOverride = g.savePathOverride
		if mode == ModeCareer {
			target.Career = sim.NewCareerProfile()
			target.Career.DriverName = locale.T("新任司机", "New driver")
			target.careerCities = make(map[sim.MapTheme]citySave)
			target.Cash = sim.CareerStartingCash
			target.Settings.AIBusCount = 0
			target.Settings.PlayerModel = sim.BusStandard
			target.initializeLineFleets()
			target.configureCareerPlans()
			target.buildLineFleets(true)
			target.bindCareerVehicles()
		}
	}
	g.resetAnnouncements()
	debug := g.trafficDebug
	autosavesEnabled := g.autosavesEnabled
	audio, announcements, names := g.vehicleAudio, g.announcements, g.stopNames
	*g = *target
	g.modeSessions = sessions
	g.trafficDebug = debug
	g.autosavesEnabled = autosavesEnabled
	g.State = StateMainMenu
	g.Settings.CareerMode = g.isCareer()
	g.vehicleAudio, g.announcements, g.stopNames = audio, announcements, names
	g.careerOffice.open = false
	sim.ActiveWorldForRoute = g.World
	sim.ClearRoutePathCache()
	locale.CurrentLanguage = g.Settings.Language
}
func (g *Game) careerMapAllowed(theme sim.MapTheme) bool {
	return !g.isCareer() || g.Career.MapUnlocked(theme)
}
func (g *Game) syncCareerCondition() {
	if !g.isCareer() {
		return
	}
	for _, b := range g.careerLiveBuses() {
		if v := g.Career.Vehicle(b.OwnedVehicleID); v != nil {
			v.Condition = b.Durability
		}
	}
}
func (g *Game) careerLiveBuses() []*sim.Bus {
	buses := []*sim.Bus{g.Bus}
	if g.AIBusMgr != nil {
		for _, ab := range g.AIBusMgr.Buses {
			buses = append(buses, ab.Bus)
		}
	}
	return buses
}
func (g *Game) careerBoarded() int {
	n := 0
	if g.Operations != nil {
		for _, v := range g.Operations.Vehicles {
			n += v.Boarded
		}
	}
	return n
}
func (g *Game) settleCareer(dt float64, cashBefore, boardedBefore int) {
	if !g.isCareer() {
		return
	}
	c := g.Career
	c.Balance += float64(g.Cash - cashBefore)
	boarded := max(0, g.careerBoarded()-boardedBefore)
	c.Passengers += boarded
	// Revenue is separate from fines, maintenance, sales and achievement grants.
	if g.Operations != nil {
		before := g.careerRevenueMark
		if g.Operations.Revenue >= before {
			c.Revenue += float64(g.Operations.Revenue - before)
		}
		g.careerRevenueMark = g.Operations.Revenue
	}
	for _, b := range g.careerLiveBuses() {
		v := c.Vehicle(b.OwnedVehicleID)
		if v == nil {
			continue
		}
		spec, _ := sim.FindVehicleSpec(v.SpecID)
		km := math.Abs(b.Speed) * dt * sim.MetersPerPixel / 1000
		v.KM += km
		c.TotalKM += km
		cost := km * spec.CostPerKM
		c.OperatingCost += cost
		c.Balance -= cost
		b.Durability = math.Max(0, b.Durability-km*.015)
		v.Condition = b.Durability
	}
	g.Cash = int(math.Floor(c.Balance))
}
func (g *Game) configureCareerPlans() {
	if !g.isCareer() {
		return
	}
	player := g.Career.Vehicle(g.Career.PlayerVehicleID)
	spec, _ := sim.FindVehicleSpec(player.SpecID)
	g.Settings.PlayerModel = spec.Model
	g.Settings.AIBusFleetChanged = false
	for i := range g.LineFleets {
		c := traffic.LineFleetSettings{MixedModels: true, PlayerModel: spec.Model, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}}
		for _, v := range g.Career.Vehicles {
			if v.ID != player.ID && v.Line == i {
				s, _ := sim.FindVehicleSpec(v.SpecID)
				c.ModelCounts[s.Model]++
			}
		}
		g.LineFleets[i] = c.Normalize()
	}
	g.showPlayerLineSettings()
}
func (g *Game) bindCareerVehicles() {
	if !g.isCareer() {
		return
	}
	used := map[int]bool{}
	bind := func(b *sim.Bus, v *sim.OwnedVehicle) {
		s, _ := sim.FindVehicleSpec(v.SpecID)
		b.CatalogID = s.ID
		b.OwnedVehicleID = v.ID
		b.PendingModel = nil
		if b.Model != s.Model {
			capacity := g.Settings.ModelPassengerCapacity(s.Model)
			if len(b.Passengers) > capacity {
				b.Passengers = b.Passengers[:capacity]
				b.PassengerCount = len(b.Passengers)
			}
			b.SetModelWithCapacity(s.Model, capacity)
		}
		b.Durability = v.Condition
		b.ApplyCatalogLimits()
		used[v.ID] = true
	}
	bind(g.Bus, g.Career.Vehicle(g.Career.PlayerVehicleID))
	// First retain identities on existing matching vehicles.
	for _, a := range g.AIBusMgr.Buses {
		v := g.Career.Vehicle(a.Bus.OwnedVehicleID)
		if v != nil && !used[v.ID] && v.Line == a.LineIndex {
			s, _ := sim.FindVehicleSpec(v.SpecID)
			if s.Model == a.Bus.Model {
				bind(a.Bus, v)
			}
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		if used[a.Bus.OwnedVehicleID] && a.Bus.OwnedVehicleID != g.Career.PlayerVehicleID {
			continue
		}
		for i := range g.Career.Vehicles {
			v := &g.Career.Vehicles[i]
			s, _ := sim.FindVehicleSpec(v.SpecID)
			if !used[v.ID] && v.Line == a.LineIndex && s.Model == a.Bus.Model {
				bind(a.Bus, v)
				break
			}
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		if s, ok := sim.FindVehicleSpec(a.Bus.CatalogID); ok {
			a.Color = sim.RGBA(s.Color[0], s.Color[1], s.Color[2], 255)
			a.Driver.CruiseSpeedLimit = math.Min(a.Driver.CruiseSpeedLimit, sim.SpeedKPH(int(s.TopSpeedKPH)))
		}
	}
}
func (g *Game) careerFleetEditable() error {
	if !g.isCareer() {
		return fmt.Errorf("Career only")
	}
	if g.State != StateMainMenu {
		return fmt.Errorf("请返回城市选择页调度车辆 / Dispatch from the city menu")
	}
	return nil
}
func (g *Game) assignCareerVehicle(id, line int) error {
	if err := g.careerFleetEditable(); err != nil {
		return err
	}
	v := g.Career.Vehicle(id)
	if v == nil || line < -1 || line > 5 {
		return fmt.Errorf("invalid vehicle or line")
	}
	if id == g.Career.PlayerVehicleID {
		return fmt.Errorf("玩家车由驾驶按钮切换 / Use Drive for player bus")
	}
	n := 0
	for _, other := range g.Career.Vehicles {
		if other.ID != id && other.ID != g.Career.PlayerVehicleID && other.Line == line {
			n++
		}
	}
	if line >= 0 && n >= sim.MaxBusesPerLine {
		return fmt.Errorf("每条线路最多 %d 辆 AI 公交 / Maximum %d AI buses per line", sim.MaxBusesPerLine, sim.MaxBusesPerLine)
	}
	v.Line = line
	return nil
}
func (g *Game) driveCareerVehicle(id int) error {
	if err := g.careerFleetEditable(); err != nil {
		return err
	}
	v := g.Career.Vehicle(id)
	if v == nil {
		return fmt.Errorf("vehicle missing")
	}
	if id == g.Career.PlayerVehicleID {
		return nil
	}
	g.Career.Vehicle(g.Career.PlayerVehicleID).Line = -1
	v.Line = 0
	g.Career.PlayerVehicleID = id
	return nil
}
func (g *Game) repairCareerVehicle(id int) error {
	if !g.isCareer() {
		return fmt.Errorf("Career only")
	}
	g.syncCareerCondition()
	for _, b := range g.careerLiveBuses() {
		if b.OwnedVehicleID == id && g.State == StatePlaying && (math.Abs(b.Speed) > 1 || b.PassengerCount > 0) {
			return fmt.Errorf("维修前请停车并清客 / Stop and unload before repair")
		}
	}
	_, err := g.Career.Repair(id)
	if err != nil {
		return err
	}
	for _, b := range g.careerLiveBuses() {
		if b.OwnedVehicleID == id {
			b.Durability = 100
		}
	}
	g.Cash = int(math.Floor(g.Career.Balance))
	return nil
}
func (g *Game) emergencyRepairCost() int {
	if g.isCareer() {
		if s, ok := sim.FindVehicleSpec(g.Bus.CatalogID); ok {
			cost := int(50 * s.RepairPerPoint)
			g.Career.RepairCost += float64(cost)
			return cost
		}
	}
	return 200
}

// OpenCareerOffice opens the paused company panel from menus or driving.
func (g *Game) OpenCareerOffice(tab int) {
	if !g.isCareer() {
		return
	}
	g.syncCareerCondition()
	g.careerOffice.open = true
	g.careerOffice.tab = max(0, min(2, tab))
}

func (g *Game) leaveCurrentCareerCity() {
	if !g.isCareer() || !g.sessionStarted || g.World == nil {
		return
	}
	if g.IsPlanMode && !g.finishPlannedRoute() {
		if len(g.planModeInitialStops) > 0 {
			g.planningRoute().StopIDs = append([]int(nil), g.planModeInitialStops...)
			g.planningRoute().CurrentIndex = max(0, min(g.planModeInitialIndex, len(g.planModeInitialStops)-1))
		}
		g.IsPlanMode = false
		g.planRoute = nil
	}
	g.saveCurrentCareerCity()
}

func (g *Game) saveCurrentCareerCity() {
	if !g.isCareer() || g.World == nil {
		return
	}
	g.syncCareerCondition()
	if g.careerCities == nil {
		g.careerCities = make(map[sim.MapTheme]citySave)
	}
	g.careerCities[g.World.Theme] = cloneCitySave(g.snapshotCity())
}

func (g *Game) prepareCareerSave() {
	if !g.isCareer() {
		return
	}
	if g.State != StatePlaying {
		g.reconcileCareerCityFleet()
	}
	g.saveCurrentCareerCity()
}

func (g *Game) reconcileCareerCityFleet() {
	if !g.isCareer() || g.World == nil {
		return
	}
	if g.IsPlanMode && len(g.planningRoute().StopIDs) == 0 {
		if len(g.planModeInitialStops) > 0 {
			g.planningRoute().StopIDs = append([]int(nil), g.planModeInitialStops...)
			g.planningRoute().CurrentIndex = max(0, min(g.planModeInitialIndex, len(g.planModeInitialStops)-1))
		}
		g.IsPlanMode = false
		g.planRoute = nil
	}
	for _, v := range g.Career.Vehicles {
		if v.ID != g.Career.PlayerVehicleID && v.Line >= 0 {
			g.LineCount = max(g.LineCount, v.Line+1)
		}
	}
	if g.PlayerLine >= g.LineCount {
		g.PlayerLine = 0
		g.Route = g.Lines[0]
	}
	defaults := sim.DefaultTransitLines(g.World)
	for i := 0; i < g.LineCount && i < len(g.Lines); i++ {
		if len(g.Lines[i].StopIDs) == 0 && i < len(defaults) {
			g.Lines[i].StopIDs = append([]int(nil), defaults[i].StopIDs...)
			g.Lines[i].CurrentIndex = 0
		}
	}
	g.configureCareerPlans()
	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.Routes = g.activeLines()
		kept := g.AIBusMgr.Buses[:0]
		usedOwned := map[int]bool{g.Career.PlayerVehicleID: true}
		for _, a := range g.AIBusMgr.Buses {
			v := g.Career.Vehicle(a.Bus.OwnedVehicleID)
			if v == nil || usedOwned[v.ID] || v.Line != a.LineIndex || a.Retiring {
				if g.Operations != nil {
					if ov := g.Operations.Vehicles[a.ID]; ov != nil {
						ov.ServiceEnded = true
					}
				}
				continue
			}
			s, _ := sim.FindVehicleSpec(v.SpecID)
			if a.Bus.Model != s.Model {
				capacity := g.Settings.ModelPassengerCapacity(s.Model)
				if len(a.Bus.Passengers) > capacity {
					a.Bus.Passengers = a.Bus.Passengers[:capacity]
					a.Bus.PassengerCount = len(a.Bus.Passengers)
				}
				a.Bus.SetModelWithCapacity(s.Model, capacity)
			}
			a.Bus.PendingModel = nil
			usedOwned[v.ID] = true
			kept = append(kept, a)
		}
		g.AIBusMgr.Buses = kept
		g.AIBusMgr.BusCount = len(kept)
		var cars []*sim.TrafficCar
		if g.TrafficMgr != nil {
			cars = g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
		}
		plans := make([]traffic.LineFleetPlan, 0, g.LineCount)
		for i, r := range g.operatingLines() {
			plans = append(plans, traffic.LineFleetPlan{Route: r, Settings: g.LineFleets[i]})
		}
		g.syncPassengerCapacities()
		g.AIBusMgr.ConfigureLineFleets(plans, g.World, g.Settings.GetLaneWidth(), g.Bus, false, cars)
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.PendingModel != nil {
				capacity := g.Settings.ModelPassengerCapacity(*a.Bus.PendingModel)
				if len(a.Bus.Passengers) > capacity {
					a.Bus.Passengers = a.Bus.Passengers[:capacity]
					a.Bus.PassengerCount = len(a.Bus.Passengers)
				}
				a.Bus.SetModelWithCapacity(*a.Bus.PendingModel, capacity)
				a.Bus.PendingModel = nil
			}
		}
	}
	g.bindCareerVehicles()
	if g.Bus.Model == sim.BusArticulated {
		sim.EnsureLongBusBays(g.World, g.Route)
		if !g.sessionStarted {
			g.Bus.ResetArticulation()
		}
	}
	g.Cash = int(math.Floor(g.Career.Balance))
	if g.Operations != nil {
		g.careerRevenueMark = g.Operations.Revenue
	}
}
