package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"
)

func TestMixedTrafficSpeedSoak(t *testing.T) {
	if os.Getenv("BUSGAME_TRAFFIC_SOAK") != "1" {
		t.Skip("opt-in live traffic soak")
	}
	for _, theme := range []sim.MapTheme{sim.MapThemeMetropolis, sim.MapThemeEmeraldLake, sim.MapThemeTwinCities} {
		t.Run(fmt.Sprint(theme), func(t *testing.T) {
			sim.SeedSimulation(91 + int64(theme))
			g := NewGame(1280, 720)
			g.Settings.AICarMaxSpeed = 100
			g.Settings.AIBusMaxSpeed = 80
			g.Settings.AIBusCount = 8
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			g.Settings.TrafficDensity = 4
			g.EnterCity(theme)
			start := g.Bus.Pos
			last := start
			distance := 0.
			stoppedTicks := 0
			legalWaitTicks, continuousLegalWait, maxLegalWait := 0, 0, 0
			diagnostic := os.Getenv("BUS_TRAFFIC_DIAG") == "1"
			steps := 7200
			if n, err := strconv.Atoi(os.Getenv("BUSGAME_SOAK_STEPS")); err == nil && n > 0 {
				steps = n
			}
			for tick := 0; tick < steps; tick++ {
				health := g.Bus.Durability
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < health {
					t.Fatalf("contact tick=%d t=%.2fs pos=%v heading=%.3f speed=%.1f state=%v health=%.0f->%.0f", tick, float64(tick)/60, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.AutopilotDriver.State, health, g.Bus.Durability)
				}
				if math.IsNaN(g.Bus.Pos.X) || math.IsNaN(g.Bus.Pos.Y) {
					t.Fatal("non-finite player position")
				}
				if g.Bus.Pos.Distance(last) > g.Bus.MaxSpeedFwd/60+20 {
					t.Fatalf("player teleported tick%d", tick)
				}
				if math.Abs(g.Bus.Speed) < 1 {
					stoppedTicks++
				}
				if g.waitingInRedQueue() || (g.Bus.DoorsOpen && g.Bus.PassengerService.Active) {
					legalWaitTicks++
					continuousLegalWait++
					maxLegalWait = max(maxLegalWait, continuousLegalWait)
				} else {
					continuousLegalWait = 0
				}
				if diagnostic && tick%300 == 0 {
					d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
					d.Path = nil
					t.Logf("tick%d pos%v speed%.1f dist%.0f stopped%.1fs diag%+v", tick, g.Bus.Pos, g.Bus.Speed, distance, float64(stoppedTicks)/60, d)
				}
				distance += g.Bus.Pos.Distance(last)
				last = g.Bus.Pos
			}
			t.Logf("map%d distance=%.0f health=%.0f cars=%d fleet=%d", theme, distance, g.Bus.Durability, len(g.TrafficMgr.Cars), len(g.AIBusMgr.Buses))
			if directory := os.Getenv("BUS_PROGRESS_CAPTURE"); directory != "" {
				g.savePathOverride = filepath.Join(directory, fmt.Sprintf("progress-map%d.json", theme))
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			legalWait := float64(legalWaitTicks) / 60
			activeFraction := float64(steps-legalWaitTicks) / float64(steps)
			floor := 10000 * activeFraction
			if theme == sim.MapThemeEmeraldLake {
				floor = 5000 * activeFraction
			}
			t.Logf("verified red-queue wait%.1fs longest%.1fs, active-time progress floor%.0f", legalWait, float64(maxLegalWait)/60, floor)
			if distance < 1000 || distance < floor || maxLegalWait > 80*60 {
				d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
				d.Path = nil
				t.Logf("progress failure: pos%v heading%.3f target%d diagnosis%+v", g.Bus.Pos, g.Bus.Heading, g.Route.GetCurrentTargetStopID(), d)
				t.Fatal("traffic made no sustained progress")
			}
		})
	}
}
