package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"os"
	"testing"
)

func TestProbeTechGreen(t *testing.T) {
	if os.Getenv("BUSGAME_TECH_PROBE") != "1" {
		t.Skip("live traffic diagnostic")
	}
	for _, seed := range []int64{7, 13, 19, 31, 43, 91} {
		t.Run(fmt.Sprint(seed), func(t *testing.T) { probeTechGreen(t, seed) })
	}
}
func probeTechGreen(t *testing.T, seed int64) {
	sim.SeedSimulation(seed)
	g := NewGame(1280, 720)
	g.Settings.AIBusCount = 8
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 3
	g.EnterCity(sim.MapThemeMetropolis)
	front, back := g.AIBusMgr.Buses[5], g.AIBusMgr.Buses[0]
	g.AIBusMgr.Buses = []*traffic.AIBus{front, back}
	st := g.World.Stops[3]
	front.Bus.Pos = st.Pos
	front.Bus.Heading = st.Heading
	front.Bus.DoorsOpen = true
	front.Bus.DoorProgress = 1
	front.Route.CurrentIndex = 4
	front.Driver.Reset()
	front.Driver.State = autopilot.AutopilotStateDocked
	front.Driver.DockedStopID = 3
	front.Driver.DwellTimer = 1
	back.Bus.Pos = st.Pos.Add(sim.Vec2{Y: -240})
	back.Bus.Heading = st.Heading
	back.Route.CurrentIndex = 4
	back.Driver.Reset()
	g.Bus.Pos = st.Pos.Add(sim.Vec2{Y: -120})
	g.Bus.Heading = st.Heading
	g.Route.CurrentIndex = 4
	g.AutopilotDriver.Reset()
	var it *sim.TrafficIntersection
	for _, p := range g.TrafficLights.Intersections {
		if p.Center.Distance(sim.Vec2{X: 7800, Y: -1800}) < 1 {
			it = p
			break
		}
	}
	waits := make(map[int]int)
	for tick := 0; tick < 6000; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, ab := range g.AIBusMgr.Buses {
			b := ab.Bus
			if math.Abs(b.Pos.X-7618) < 50 && b.Pos.Y > -2450 && b.Pos.Y < -2250 && b.Speed < 1 && it.Phase == sim.PhaseNSGreen {
				waits[ab.ID]++
			} else {
				waits[ab.ID] = 0
			}
			if waits[ab.ID] == 120 {
				t.Logf("driver %+v", ab.Driver)
				for _, o := range g.AIBusMgr.GetObstacles(ab.ID) {
					t.Logf("busob %+v", o)
				}
				for _, car := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
					if car.Pos.Distance(b.Pos) < 1500 {
						t.Logf("car %+v", car)
					}
				}
				t.Fatalf("AI%d missed green tick%d phaseTimer%.2f pos%v h%.2f target%.1f route%d driver%v path%v", ab.ID, tick, it.Timer, b.Pos, b.Heading, ab.Driver.LastTargetSpeed, ab.Route.GetCurrentTargetStopID(), ab.Driver.State, ab.Driver.DrivePath())
			}
		}
	}
}
