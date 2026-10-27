package game

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Candidate-specific live game matrix: mixed fleets, full traffic, articulated
// player, cold caches after save/load, and no durability loss across two minutes.
func TestRelease007MixedFleetSoak(t *testing.T) {
	if testing.Short() || os.Getenv("BUSGAME_RELEASE007_SOAK") != "1" {
		t.Skip("set BUSGAME_RELEASE007_SOAK=1 for the candidate endurance matrix")
	}
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		t.Run(fmt.Sprintf("map%d", theme), func(t *testing.T) {
			sim.SeedSimulation(700 + int64(theme))
			g := NewGame(1280, 720)
			g.savePathOverride = filepath.Join(t.TempDir(), "candidate-save.json")
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			g.Settings.AutopilotBerserkMaxSpeed = 120
			g.Settings.TrafficDensity = 4
			g.EnterCity(theme)
			g.SetLineFleet(0, traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 3, 3}, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 80}})
			g.Bus.SetModel(sim.BusArticulated)
			g.ResetBusesToRouteStart()
			distance := 0.
			maxCars := 0
			aiDistances := make(map[int]float64)
			ticks := 7200
			if value := os.Getenv("BUS_RING_SOAK_TICKS"); value != "" {
				var err error
				ticks, err = strconv.Atoi(value)
				if err != nil || ticks < 7200 {
					t.Fatal("BUS_RING_SOAK_TICKS must be >= 7200")
				}
			}
			for tick := 0; tick < ticks; tick++ {
				before := g.Bus.Pos
				aiPositions := make(map[int]sim.Vec2, len(g.AIBusMgr.Buses))
				for _, ab := range g.AIBusMgr.Buses {
					aiPositions[ab.ID] = ab.Bus.Pos
				}
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				step := g.Bus.Pos.Distance(before)
				distance += step
				if step > g.Bus.MaxSpeedFwd/60+20 {
					t.Fatalf("player discontinuity tick%d distance%.1f", tick, step)
				}
				if g.Bus.Durability < 100 {
					logNearbyVehicles(t, g)
					t.Logf("last HUD events: %+v", g.HUD.Toasts)
					t.Logf("physical body and boundary: %+v %+v", g.Bus.BodyParts(), g.Bus.CheckBoundaries(g.World, g.Settings.GetLaneWidth()))
					t.Fatalf("player damage tick%d pos%v health%.1f tail%v", tick, g.Bus.Pos, g.Bus.Durability, g.Bus.TrailerBody())
				}
				cars := g.TrafficMgr.GetActiveCars(4)
				maxCars = max(maxCars, len(cars))
				if len(cars) > sim.MaxTrafficCars {
					t.Fatal("traffic overflow")
				}
				for _, c := range cars {
					if !finiteVehicle(c.Pos, c.Heading, c.Speed) {
						t.Fatalf("invalid car%d tick%d", c.ID, tick)
					}
				}
				check := func(id int, b *sim.Bus) {
					if !finiteVehicle(b.Pos, b.Heading, b.Speed) || math.IsNaN(b.TailHeading()) || math.IsInf(b.TailHeading(), 0) {
						t.Fatalf("invalid bus%d tick%d", id, tick)
					}
					if b.PassengerCount < 0 || b.PassengerCount > b.Capacity {
						t.Fatalf("invalid capacity bus%d", id)
					}
					if b.Durability < 100 {
						t.Fatalf("bus%d damage tick%d pos%v health%.1f", id, tick, b.Pos, b.Durability)
					}
					if b.IsArticulated() && math.Abs(sim.NormalizeAngle(b.Heading-b.TailHeading())) > sim.MaxArticulationAngle+1e-8 {
						t.Fatalf("bus%d excessive articulation", id)
					}
				}
				check(0, g.Bus)
				for _, ab := range g.AIBusMgr.Buses {
					check(ab.ID, ab.Bus)
					aiStep := ab.Bus.Pos.Distance(aiPositions[ab.ID])
					if aiStep > ab.Bus.MaxSpeedFwd/60+20 {
						t.Fatalf("AI%d discontinuity tick%d distance%.1f", ab.ID, tick, aiStep)
					}
					aiDistances[ab.ID] += aiStep
				}
				captureTick := 6900
				if theme == sim.MapThemeCustom {
					captureTick = 1000
				}
				if value := os.Getenv("BUS_RELEASE_CAPTURE_TICK"); value != "" {
					var err error
					captureTick, err = strconv.Atoi(value)
					if err != nil {
						t.Fatal(err)
					}
				}
				if tick == captureTick && os.Getenv("BUSGAME_RELEASE007_CAPTURE") != "" {
					oldPath := g.savePathOverride
					g.savePathOverride = filepath.Join(os.Getenv("BUSGAME_RELEASE007_CAPTURE"), fmt.Sprintf("mixed-map%d-before-contact.json", theme))
					if err := g.SaveGame(); err != nil {
						t.Fatal(err)
					}
					g.savePathOverride = oldPath
				}
				if tick == 3599 {
					position, heading, tail, target := g.Bus.Pos, g.Bus.Heading, g.Bus.TailHeading(), g.Route.GetCurrentTargetStopID()
					if err := g.SaveGame(); err != nil {
						t.Fatal(err)
					}
					if err := g.LoadGame(); err != nil {
						t.Fatal(err)
					}
					if g.Bus.Pos != position || g.Bus.Heading != heading || g.Bus.TailHeading() != tail || g.Route.GetCurrentTargetStopID() != target {
						t.Fatal("save/load altered active vehicle progress")
					}
				}
			}
			minDistance := 1000.
			if theme == sim.MapThemeCustom {
				minDistance = 3000
			}
			if distance < minDistance || maxCars != sim.MaxTrafficCars {
				t.Fatalf("inactive player/traffic: distance%.1f maxCars%d", distance, maxCars)
			}
			logStoppedFleet(t, g)
			counts := [sim.BusModelCount]int{}
			for _, ab := range g.AIBusMgr.Buses {
				counts[ab.Bus.Model]++
				if aiDistances[ab.ID] < 100 {
					t.Errorf("AI%d insufficient progress %.1f", ab.ID, aiDistances[ab.ID])
				}
			}
			if counts != [sim.BusModelCount]int{2, 3, 3} {
				t.Fatalf("mixed fleet changed %v", counts)
			}
			t.Logf("%ds, save/load at60s, player distance%.0f health%.0f cars%d->%d AI distances%v", ticks/60, distance, g.Bus.Durability, maxCars, len(g.TrafficMgr.GetActiveCars(4)), aiDistances)
		})
	}
}
