package game

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Opt-in soak runs the actual game coordinator (not isolated traffic managers)
// for two simulated minutes per city/mode, including recovery and respawning.
func TestRelease005TrafficSoak(t *testing.T) {
	if testing.Short() || os.Getenv("BUSGAME_SOAK") != "1" {
		t.Skip("Run make test-soak for the release endurance matrix")
	}
	for theme := sim.MapThemeMetropolis; theme <= sim.MapThemeCustom; theme++ {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("map%d/berserk%t", theme, berserk), func(t *testing.T) {
				sim.SeedSimulation(500 + int64(theme))
				g := NewGame(1280, 720)
				g.Settings.AutopilotEnabled = true
				g.Settings.AutopilotBerserk = berserk
				g.Settings.AutopilotBerserkMaxSpeed = 120
				g.Settings.TrafficDensity = 4
				g.Settings.AIBusCount = 8
				g.EnterCity(theme)
				start := g.Bus.Pos
				distance := 0.0
				maxCars := 0
				for tick := 0; tick < 7200; tick++ {
					if tick == 1200 && theme == sim.MapThemeCustom && !berserk && os.Getenv("BUSGAME_RELEASE007_CAPTURE") != "" {
						oldPath := g.savePathOverride
						g.savePathOverride = filepath.Join(os.Getenv("BUSGAME_RELEASE007_CAPTURE"), "custom-normal-before-contact.json")
						if err := g.SaveGame(); err != nil {
							t.Fatal(err)
						}
						g.savePathOverride = oldPath
					}
					previous := g.Bus.Pos
					health := g.Bus.Durability
					if err := g.Update(); err != nil {
						t.Fatal(err)
					}
					if g.Bus.Durability < health {
						nearest := math.Inf(1)
						var near *sim.TrafficCar
						for _, car := range g.TrafficMgr.GetActiveCars(4) {
							if d := car.Pos.Distance(g.Bus.Pos); d < nearest {
								nearest = d
								near = car
							}
						}
						if len(g.HUD.Toasts) > 0 {
							t.Logf("latest event: %s", g.HUD.Toasts[len(g.HUD.Toasts)-1].Text)
						}
						if near != nil {
							t.Logf("contact t=%.2f bus=%v heading=%.2f speed=%.1f lane=%.1f state=%v nearest car=%d pos=%v heading=%.2f speed=%.1f ring=%t health=%.0f", float64(tick)/60, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.State, near.ID, near.Pos, near.Heading, near.Speed, near.InRoundabout, g.Bus.Durability)
						}
						t.Fatalf("release safety regression at %.2fs: durability %.0f -> %.0f, pos=%v", float64(tick)/60, health, g.Bus.Durability, g.Bus.Pos)
					}

					distance += g.Bus.Pos.Distance(previous)
					cars := g.TrafficMgr.GetActiveCars(4)
					maxCars = max(maxCars, len(cars))
					if len(cars) > sim.MaxTrafficCars {
						t.Fatalf("Population overflow: %d", len(cars))
					}
					for _, car := range cars {
						if !finiteVehicle(car.Pos, car.Heading, car.Speed) {
							t.Fatalf("Non-finite car at tick %d", tick)
						}
					}
					buses := append([]*sim.Bus{g.Bus}, fleetBuses(g)...)
					for _, bus := range buses {
						if !finiteVehicle(bus.Pos, bus.Heading, bus.Speed) {
							t.Fatalf("Non-finite bus at tick %d", tick)
						}
						if bus.PassengerCount < 0 || bus.PassengerCount > bus.Capacity {
							t.Fatalf("Invalid passenger count %d", bus.PassengerCount)
						}
					}
				}
				if berserk && theme == sim.MapThemeTwinCities && distance < 8000 {
					t.Fatalf("release safety regression: insufficient forward progress %.0fpx in 120s", distance)
				}
				if berserk && theme == sim.MapThemeEmeraldLake && distance < 2000 {
					t.Fatalf("emerald lake progress regression: %.0fpx", distance)
				}
				if berserk && theme == sim.MapThemeRiverValley && distance < 6000 {
					t.Fatalf("river valley progress regression: %.0fpx", distance)
				}
				if berserk && theme == sim.MapThemeCustom && distance < 3000 {
					t.Fatalf("custom skipped-bay progress regression: %.0fpx", distance)
				}
				if distance < 100 || maxCars != sim.MaxTrafficCars {
					t.Fatalf("Scene inactive: player distance=%.1f maxCars=%d start=%v", distance, maxCars, start)
				}
				t.Logf("120 simulated seconds; traveled %.0fpx; cars %d -> %d; fleet %d; durability %.0f%%", distance, maxCars, len(g.TrafficMgr.GetActiveCars(4)), len(g.AIBusMgr.Buses), g.Bus.Durability)
			})
		}
	}
}
func finiteVehicle(pos sim.Vec2, heading, speed float64) bool {
	for _, v := range []float64{pos.X, pos.Y, heading, speed} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}
func fleetBuses(g *Game) []*sim.Bus {
	buses := make([]*sim.Bus, 0, len(g.AIBusMgr.Buses))
	for _, ab := range g.AIBusMgr.Buses {
		buses = append(buses, ab.Bus)
	}
	return buses
}
