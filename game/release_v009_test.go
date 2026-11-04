package game

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Candidate matrix supplements the six-map legacy soak with the new median
// layout, both grid sizes, all player models and a full mixed AI fleet.
func TestRelease009BRTSoak(t *testing.T) {
	if testing.Short() || os.Getenv("BUSGAME_RELEASE009_SOAK") != "1" {
		t.Skip("set BUSGAME_RELEASE009_SOAK=1 for the 0.0.9 BRT endurance matrix")
	}
	themes := []sim.MapTheme{}
	for _, th := range sim.BuiltinMapThemes() {
		if sim.BuiltinMapSupportsCenterBRT(th) {
			themes = append(themes, th)
		}
	}
	themes = append(themes, sim.MapThemeCustom)
	for _, th := range themes {
		grids := []bool{false, true}
		if th == sim.MapThemeCustom {
			grids = []bool{false}
		}
		for _, large := range grids {
			for model := sim.BusStandard; model < sim.BusModelCount; model++ {
				for _, berserk := range []bool{false, true} {
					t.Run(fmt.Sprintf("map%d/large%v/model%d/berserk%v", th, large, model, berserk), func(t *testing.T) {
						sim.SeedSimulation(900 + int64(th)*100 + int64(model)*10)
						g := NewGame(1280, 720)
						if th == sim.MapThemeCustom {
							data := sim.GetDefaultCustomMapData()
							data.Roads[0].Lanes = 8
							data.Roads[0].BusLane = true
							if err := sim.ValidateCustomMapData(data); err != nil {
								t.Fatal(err)
							}
							g.World.CustomMap = data
						}
						g.Settings.SetCenterBRT(th, true)
						g.Settings.SetLargeRoadNetwork(th, large)
						g.Settings.AutopilotEnabled = true
						g.Settings.AutopilotBerserk = berserk
						g.Settings.AutopilotBerserkMaxSpeed = 120
						g.Settings.TrafficDensity = 4
						g.EnterCity(th)
						g.SetLineFleet(0, traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 3, 3}, PlayerModel: model, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
						g.Bus.SetModel(model)
						g.ResetBusesToRouteStart()
						g.savePathOverride = filepath.Join(t.TempDir(), "brt009.json")
						if !g.World.CenterBRT || !g.Settings.IsCenterBRT(th) {
							t.Fatal("BRT option was not applied")
						}
						centralStops := 0
						for _, s := range g.World.Stops {
							if s.CenterBRT {
								centralStops++
							}
						}
						if centralStops == 0 {
							t.Fatal("no usable central stations")
						}
						distance := 0.
						maxCars := 0
						aiDistances := map[int]float64{}
						for tick := 0; tick < 7200; tick++ {
							if directory := os.Getenv("BUSGAME_RELEASE009_CAPTURE"); directory != "" && th == sim.MapThemeMetropolis && large && model == sim.BusDoubleDecker && berserk && tick == 5400 {
								oldPath := g.savePathOverride
								g.savePathOverride = filepath.Join(directory, "brt-large-doubledecker-before-contact.json")
								if e := g.SaveGame(); e != nil {
									t.Fatal(e)
								}
								g.savePathOverride = oldPath
							}
							before := g.Bus.Pos
							health := g.Bus.Durability
							aiBefore := map[int]sim.Vec2{}
							aiHealth := map[int]float64{}
							for _, a := range g.AIBusMgr.Buses {
								aiBefore[a.ID] = a.Bus.Pos
								aiHealth[a.ID] = a.Bus.Durability
							}
							if e := g.Update(); e != nil {
								t.Fatal(e)
							}
							distance += before.Distance(g.Bus.Pos)
							maxCars = max(maxCars, len(g.TrafficMgr.GetActiveCars(4)))
							if g.Bus.Durability < health {
								t.Fatalf("player collision tick%d health%.0f->%.0f pos%v", tick, health, g.Bus.Durability, g.Bus.Pos)
							}
							if !finiteVehicle(g.Bus.Pos, g.Bus.Heading, g.Bus.Speed) {
								t.Fatal("non-finite player state")
							}
							for _, a := range g.AIBusMgr.Buses {
								aiDistances[a.ID] += a.Bus.Pos.Distance(aiBefore[a.ID])
								if a.Bus.Durability < aiHealth[a.ID] {
									t.Fatalf("AI%d collision tick%d health%.0f->%.0f pos%v", a.ID, tick, aiHealth[a.ID], a.Bus.Durability, a.Bus.Pos)
								}
								if !finiteVehicle(a.Bus.Pos, a.Bus.Heading, a.Bus.Speed) {
									t.Fatalf("AI%d non-finite state", a.ID)
								}
							}
							if tick%60 == 0 {
								for _, b := range append([]*sim.Bus{g.Bus}, fleetBuses(g)...) {
									if hit := b.CheckBoundaries(g.World, 52); hit.Zone == sim.ZoneHardBarrier {
										t.Fatalf("barrier tick%d at%v %s", tick, b.Pos, hit.ObstacleName)
									}
								}
							}
							if tick == 3599 {
								pos, heading, tail := g.Bus.Pos, g.Bus.Heading, g.Bus.TailHeading()
								if e := g.SaveGame(); e != nil {
									t.Fatal(e)
								}
								if e := g.LoadGame(); e != nil {
									t.Fatal(e)
								}
								if !g.World.CenterBRT || g.Bus.Pos != pos || g.Bus.Heading != heading || g.Bus.TailHeading() != tail {
									t.Fatal("save/load altered BRT state or pose")
								}
							}
						}
						if distance < 1000 || maxCars != sim.MaxTrafficCars {
							t.Fatalf("inactive scenario distance%.0f cars%d", distance, maxCars)
						}
						for _, a := range g.AIBusMgr.Buses {
							if aiDistances[a.ID] < 100 {
								t.Errorf("AI%d insufficient progress %.0f", a.ID, aiDistances[a.ID])
							}
						}
						if math.IsNaN(distance) {
							t.Fatal("non-finite distance")
						}
						t.Logf("120s BRT grid%v model%d berserk%v: distance%.0f health%.0f stations%d cars%d AI distances%v", large, model, berserk, distance, g.Bus.Durability, centralStops, maxCars, aiDistances)
					})
				}
			}
		}
	}
}
