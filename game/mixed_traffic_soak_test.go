package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
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
				distance += g.Bus.Pos.Distance(last)
				last = g.Bus.Pos
			}
			t.Logf("map%d distance=%.0f health=%.0f cars=%d fleet=%d", theme, distance, g.Bus.Durability, len(g.TrafficMgr.Cars), len(g.AIBusMgr.Buses))
			if distance < 10000 {
				t.Fatal("traffic made no sustained progress")
			}
		})
	}
}
