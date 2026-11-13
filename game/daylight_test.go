package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestDaylightAutoLightsPlayerChoiceAndPersistence(t *testing.T) {
	for _, mode := range []PlayMode{ModeFast, ModeCareer} {
		t.Run(mode.String(), func(t *testing.T) {
			g := autosaveTestGame(t, mode)
			if mode == ModeCareer {
				g.Career.Balance = 3000000
				id, err := g.Career.Buy("xingtu-a100")
				if err != nil {
					t.Fatal(err)
				}
				g.Cash = int(g.Career.Balance)
				g.State = StateMainMenu
				if err = g.assignCareerVehicle(id, 0); err != nil {
					t.Fatal(err)
				}
				g.EnterCity(sim.MapThemeMetropolis)
			} else {
				g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 2})
			}
			if g.World.Clock.TimeString() != "08:00:00" || g.Bus.HeadlightsOn {
				t.Fatal("wrong new-game lighting")
			}
			g.World.Clock.ElapsedMinutes = 780
			g.syncAutoHeadlights()
			if !g.World.Clock.IsNight() || g.Bus.HeadlightsOn {
				t.Fatal("AI lighting overrode the player's off choice")
			}
			for _, c := range g.TrafficMgr.Cars {
				if !c.HeadlightsOn {
					t.Fatal("car did not switch on at night")
				}
			}
			for _, b := range g.AIBusMgr.Buses {
				if !b.Bus.HeadlightsOn {
					t.Fatal("AI bus did not switch on at night")
				}
			}
			g.ToggleHeadlights()
			if !g.Bus.HeadlightsOn {
				t.Fatal("manual lights did not switch on")
			}
			g.Settings.ClockRate = sim.ClockThirtySeconds
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
			g.World.Clock.ElapsedMinutes = 0
			g.Bus.HeadlightsOn = false
			g.Settings.ClockRate = sim.ClockRealTime
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			if g.World.Clock.TimeString() != "21:00:00" || !g.Bus.HeadlightsOn || g.Settings.ClockRate != sim.ClockThirtySeconds || g.World.Clock.Rate != sim.ClockThirtySeconds {
				t.Fatal("save lost time/player lights")
			}
			g.World.Clock.ElapsedMinutes = 1440
			g.syncAutoHeadlights()
			if !g.Bus.HeadlightsOn {
				t.Fatal("daybreak overrode the player's choice")
			}
			for _, c := range g.TrafficMgr.Cars {
				if c.HeadlightsOn {
					t.Fatal("car stayed lit by day")
				}
			}
			for _, b := range g.AIBusMgr.Buses {
				if b.Bus.HeadlightsOn {
					t.Fatal("AI bus stayed lit by day")
				}
			}
		})
	}
}
func TestDaylightTimePausesWithMenusAndPlanning(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.Settings.AutopilotEnabled = false
	start := g.World.Clock.ElapsedMinutes
	for i := 0; i < 60; i++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if math.Abs(g.World.Clock.ElapsedMinutes-start-1./60) > 1e-7 {
		t.Fatal("incorrect real-time scale")
	}
	for _, pause := range []string{"planning", "settings", "menu", "garage", "save picker"} {
		before := g.World.Clock.ElapsedMinutes
		g.State = StatePlaying
		g.IsPlanMode = false
		g.Settings.IsOpen = false
		g.savePicker.open = false
		switch pause {
		case "planning":
			g.IsPlanMode = true
		case "settings":
			g.Settings.IsOpen = true
		case "menu":
			g.State = StateMainMenu
		case "garage":
			g.SelectMode(ModeCareer)
			g.EnterCity(sim.MapThemeMetropolis)
			g.OpenCareerOffice(0)
			before = g.World.Clock.ElapsedMinutes
		case "save picker":
			g.careerOffice.open = false
			g.Mode = ModeFast
			g.openSavePicker(true)
		}
		for i := 0; i < 60; i++ {
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
		}
		if g.World.Clock.ElapsedMinutes != before {
			t.Fatal("clock progressed while paused", pause)
		}
	}
}

func TestDaylightTransitionsAutomaticallyDuringUpdate(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 2})
	g.Bus.HeadlightsOn = true
	for _, tc := range []struct {
		elapsed float64
		night   bool
	}{{599.9999, true}, {1319.9999, false}} {
		g.World.Clock.ElapsedMinutes = tc.elapsed
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.World.Clock.IsNight() != tc.night || !g.Bus.HeadlightsOn {
			t.Fatal("wrong boundary or player choice")
		}
		for _, c := range g.TrafficMgr.Cars {
			if c.HeadlightsOn != tc.night {
				t.Fatal("car did not follow time transition")
			}
		}
		for _, b := range g.AIBusMgr.Buses {
			if b.Bus.HeadlightsOn != tc.night {
				t.Fatal("bus did not follow time transition")
			}
		}
	}
}

func TestClockRateChangesAndCityReset(t *testing.T) {
	g := autosaveTestGame(t, ModeFast)
	g.Settings.AutopilotEnabled = false
	for _, rate := range []sim.ClockRate{sim.ClockRealTime, sim.ClockThirtySeconds, sim.ClockOneMinute} {
		before := g.World.Clock.ElapsedMinutes
		g.Settings.ClockRate = rate
		for i := 0; i < 60; i++ {
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
		}
		if math.Abs(g.World.Clock.ElapsedMinutes-before-rate.Multiplier()/60) > 1e-8 {
			t.Fatalf("rate %d was not applied", rate)
		}
	}
	g.EnterCity(sim.MapThemeMetropolis)
	if g.World.Clock.TimeString() != "08:00:00" || g.World.Clock.Rate != sim.ClockOneMinute {
		t.Fatal("city reset lost chosen rate")
	}
}
