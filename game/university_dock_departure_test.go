package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

func TestUniversityDockCycleDepartsPastStoppedRearBus(t *testing.T) {
	for _, level := range []int{0, 1, 3} {
		t.Run(fmt.Sprint(level), func(t *testing.T) {
			sim.SeedSimulation(17)
			g := NewGame(1280, 720)
			g.Settings.LaneWidthLevel = level
			g.Settings.AutopilotEnabled = true
			g.Settings.AutopilotBerserk = true
			g.Settings.TrafficDensity = 0
			g.EnterCity(sim.MapThemeMetropolis)
			st := g.World.Stops[7]
			lw := g.Settings.GetLaneWidth()
			u := sim.Vec2{X: math.Cos(st.Heading), Y: math.Sin(st.Heading)}
			r := sim.Vec2{X: -u.Y, Y: u.X}
			g.Route = &sim.Route{StopIDs: []int{6, 7, 9, 6}, CurrentIndex: 1}
			g.Bus = sim.NewBus(st.Pos.X, st.Pos.Y, st.Heading)
			g.Bus.BoardPassengers(5, []int{7})
			g.Bus.BoardPassengers(10, []int{9})
			st.WaitingPassengers = 1
			g.stopBellRung = true
			g.AutopilotDriver.Reset()
			g.AutopilotDriver.State = autopilot.AutopilotStateDocked
			g.AutopilotDriver.DockedStopID = 7
			g.AutopilotDriver.DwellTimer = 1
			p := st.Pos.Sub(u.Mul(150)).Sub(r.Mul(50))
			rear := sim.NewBus(p.X, p.Y, st.Heading+.5)
			g.AIBusMgr.Buses = []*traffic.AIBus{{ID: 2, Bus: rear, Driver: autopilot.NewAutopilotDriver(), Route: &sim.Route{StopIDs: []int{6, 7, 9, 6}, CurrentIndex: 1}}}
			sawOpen, closedAfterBoard := false, false
			for tick := 0; tick < 1200; tick++ {
				g.updateDriverMode(1. / 60)
				sawOpen = sawOpen || g.Bus.DoorsOpen
				closedAfterBoard = closedAfterBoard || (sawOpen && !g.Bus.DoorsOpen && g.Bus.DoorProgress < .05)
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: g.Bus.Pos, Heading: g.Bus.Heading, HalfLen: 48, HalfWid: 18}, sim.OBB{Center: rear.Pos, Heading: rear.Heading, HalfLen: 48, HalfWid: 18}); hit {
					t.Fatalf("departure contact tick%d", tick)
				}
				if g.Bus.Pos.Sub(st.Pos).Dot(u) > 220 {
					if !closedAfterBoard || g.Route.GetCurrentTargetStopID() != 9 || g.Bus.CountPassengersAlightingAt(7) != 0 || g.Operations.Vehicle(0).Alighted != 5 {
						t.Fatal("invalid stop/door/route exchange")
					}
					return
				}
			}
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, lw)
			t.Fatalf("docked bus did not depart: state%v doors%v/%.2f route%d pos%v limits%v", g.AutopilotDriver.State, g.Bus.DoorsOpen, g.Bus.DoorProgress, g.Route.GetCurrentTargetStopID(), g.Bus.Pos, d.Limits)
		})
	}
}
