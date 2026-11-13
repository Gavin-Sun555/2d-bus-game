package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) EnterCity(theme sim.MapTheme) {
	if !g.careerMapAllowed(theme) {
		g.saveMessage = "城市尚未解锁：扩充车队后再来 / Expand your fleet to unlock this city"
		return
	}
	if g.isCareer() {
		if g.sessionStarted && g.World != nil {
			g.leaveCurrentCareerCity()
		}
		wantLarge := theme != sim.MapThemeCustom && g.Settings.IsLargeRoadNetwork(theme)
		wantBRT := g.Settings.IsCenterBRT(theme)
		if saved, ok := g.careerCities[theme]; ok && saved.World != nil && saved.World.LargeGrid == wantLarge && saved.World.CenterBRT == wantBRT {
			if err := g.restoreCareerCity(saved); err == nil {
				g.State = StatePlaying
				g.sessionStarted = true
				g.IsPlanMode = false
				g.Camera.RestoreDrivingZoom()
				g.saveCurrentCareerCity()
				return
			}
		}
	}
	g.Settings.LargeRoadNetwork = g.Settings.IsLargeRoadNetwork(theme)
	g.SetMapTheme(theme)
	g.State = StatePlaying
	g.sessionStarted = true
	g.IsPlanMode = false
	g.Camera.RestoreDrivingZoom()
	if g.isCareer() {
		g.saveCurrentCareerCity()
	}
}

func (g *Game) SetMapTheme(theme sim.MapTheme) {
	if !g.careerMapAllowed(theme) {
		return
	}
	g.syncCareerCondition()
	g.resetAnnouncements()
	centerBRT := g.Settings.IsCenterBRT(theme)
	g.Settings.SetCenterBRT(theme, centerBRT)
	g.Settings.MapTheme = theme
	g.SelectedMapTheme = theme
	if g.Settings.LargeRoadNetworkChanged {
		g.Settings.LargeRoadNetworkChanged = false
		g.Settings.SetLargeRoadNetwork(theme, g.Settings.LargeRoadNetwork)
	}
	isLarge := g.Settings.IsLargeRoadNetwork(theme)
	g.Settings.LargeRoadNetwork = isLarge
	currentLaneW := g.Settings.GetLaneWidth()
	if g.isCareer() && g.World != nil {
		customMap := g.World.CustomMap
		g.World = sim.NewWorld(currentLaneW)
		g.World.CustomMap = customMap
		g.World.PassengerSpawnRate = g.Settings.PassengerSpawnRate
		g.TrafficLights = sim.NewTrafficLightSystem()
	}
	g.World.LargeGrid = (theme != sim.MapThemeCustom && isLarge)
	g.World.CenterBRT = centerBRT
	g.World.BuildMap(theme, currentLaneW)
	g.Settings.CenterBRT = g.World.CenterBRT
	g.closeStopNameEditor()
	g.closeStationSearch()
	g.applyStopNames()

	// Rebuild traffic lights and AI civilian vehicles for active map
	g.syncAISpeedRanges()
	g.TrafficLights.BuildForWorld(g.World)
	g.TrafficMgr.BuildForWorld(g.World, traffic.MaxTrafficCars, currentLaneW)
	if g.AutopilotDriver != nil {
		g.AutopilotDriver.Reset()
	}

	// Clear Dijkstra memoized path cache because roads/roundabouts differ
	sim.ClearRoutePathCache()

	// Update starting route to standard route for this map
	g.Lines = sim.DefaultTransitLines(g.World)
	g.LineCount = 1
	g.PlayerLine = 0
	g.NetworkSelected = 0
	g.initializeLineFleets()
	if g.isCareer() {
		for _, v := range g.Career.Vehicles {
			if v.ID != g.Career.PlayerVehicleID && v.Line >= 0 {
				g.LineCount = max(g.LineCount, v.Line+1)
			}
		}
		g.configureCareerPlans()
	}
	g.Route = g.Lines[0]
	g.NetworkOpen = false
	g.planRoute = nil
	g.planModeRouteModified = false
	g.Route.CurrentIndex = 0
	g.Route.InvalidateDescription()

	// Reposition bus cleanly to the first stop of the route
	firstStop := g.World.Stops[0]
	if len(g.Route.StopIDs) > 0 {
		startID := g.Route.StopIDs[0]
		if startID >= 0 && startID < len(g.World.Stops) {
			firstStop = g.World.Stops[startID]
		}
	}
	g.Bus.Pos = firstStop.FrontBerthPosition()
	g.Bus.Heading = firstStop.Heading
	g.Bus.Speed = 0
	g.Bus.SteeringAngle = 0
	g.Bus.DoorsOpen = true
	g.Bus.IsBraking = false
	g.Bus.OnShoulder = false
	g.Bus.ClearPassengers()
	g.Bus.PassengerService = sim.PassengerService{}
	g.Bus.SetModel(g.LineFleets[g.PlayerLine].PlayerModel)
	g.applyPlayerPassengerCapacity()
	if g.Bus.Model == sim.BusArticulated {
		sim.EnsureLongBusBays(g.World, g.Route)
		g.Bus.ResetArticulation()
	}

	g.queuePlayerAtStop(firstStop)
	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.Routes = g.activeLines()
		g.buildLineFleets(true)
	}

	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos

	g.World.Clock.Rate = g.Settings.ClockRate
	g.Bus.HeadlightsOn = false
	g.syncAutoHeadlights()
	g.resetOperations()
	g.careerRevenueMark = 0
	g.bindCareerVehicles()
	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()
	if g.World.CenterBRT {
		g.HUD.AddToast("【中央 BRT】八车道公交专用道已移至道路中央，沿中央站位靠站。", true)
	}
	g.HUD.AddToast(fmt.Sprintf("已进入城市地图: 【%s】！", theme.DisplayName()), true)
	if theme != sim.MapThemeCustom && g.Settings.LargeRoadNetwork {
		g.HUD.AddToast("【大路网模式 4x】街区长度延展 4 倍，尽情巡航驾驶！", true)
	} else {
		g.HUD.AddToast(fmt.Sprintf("风貌特色: %s", theme.Subtitle()), false)
	}
}

// ReloadCustomMap reads custom_map.json and refreshes the world if playing on custom map
func (g *Game) ReloadCustomMap() error {
	customMap, err := sim.LoadCustomMapFromFile("custom_map.json")
	if err != nil {
		g.HUD.AddWarningToast(fmt.Sprintf("读取 custom_map.json 失败: %v", err))
		return err
	}
	g.World.CustomMap = customMap
	if g.SelectedMapTheme == sim.MapThemeCustom {
		g.SetMapTheme(sim.MapThemeCustom)
		g.HUD.AddToast(fmt.Sprintf("已成功重载自定义地图: 【%s】！", customMap.Name), true)
	} else {
		g.HUD.AddToast(fmt.Sprintf("已成功校验自定义地图: 【%s】(可按[5]或[C]切换)", customMap.Name), true)
	}
	return nil
}

func (g *Game) centerBRTAvailable(theme sim.MapTheme) bool {
	return sim.BuiltinMapSupportsCenterBRT(theme)
}
