package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func (g *Game) EnterCity(theme sim.MapTheme) {
	g.SetMapTheme(theme)
	g.State = StatePlaying
	g.IsPlanMode = false
	g.Camera.RestoreDrivingZoom()
}

func (g *Game) SetMapTheme(theme sim.MapTheme) {
	g.resetAnnouncements()
	g.Settings.MapTheme = theme
	g.SelectedMapTheme = theme
	g.World.LargeGrid = (theme == sim.MapThemeMetropolis && g.Settings.LargeRoadNetwork)
	currentLaneW := g.Settings.GetLaneWidth()
	g.World.BuildMap(theme, currentLaneW)
	g.closeStopNameEditor()
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
	if g.Bus.Model == sim.BusArticulated {
		sim.EnsureLongBusBays(g.World, g.Route)
		g.Bus.ResetArticulation()
	}

	if g.AIBusMgr != nil {
		g.AIBusMgr.ActiveRoute = g.Route
		g.AIBusMgr.Routes = g.activeLines()
		g.buildLineFleets(true)
	}

	g.Camera.FollowBus(g.Bus)
	g.Camera.Pos = g.Bus.Pos
	g.Camera.TargetPos = g.Bus.Pos

	g.resetOperations()
	g.SyncStopsWithRoute()
	g.rollAlightingForNextStop()
	g.HUD.AddToast(fmt.Sprintf("已进入城市地图: 【%s】！", theme.DisplayName()), true)
	if theme == sim.MapThemeMetropolis && g.Settings.LargeRoadNetwork {
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
