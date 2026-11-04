package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"testing"
)

func TestRelease009CapturedContactReplay(t *testing.T) {
	p := os.Getenv("BUSGAME_RELEASE009_REPLAY")
	if p == "" {
		t.Skip("set BUSGAME_RELEASE009_REPLAY for captured candidate replay")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if e := g.LoadGame(); e != nil {
		t.Fatal(e)
	}
	g.Settings.IsOpen = false
	health := g.Bus.Durability
	for tick := 0; tick < 600; tick++ {
		if tick%10 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, 52)
			d.Path = nil
			t.Logf("tick%d pos%v h%.3f sp%.1f steer%.3f off%.1f->%.1f state%v target%.1f limits%v blockers%+v", tick, g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.Bus.SteeringAngle, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset, g.AutopilotDriver.State, d.TargetSpeed, d.Limits, d.Blockers)
		}
		if e := g.Update(); e != nil {
			t.Fatal(e)
		}
		if g.Bus.Durability < health {
			u := sim.UnitFromAngle(g.Bus.Heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
				if c.Pos.Distance(g.Bus.Pos) < 300 {
					t.Logf("near car%d p%v h%.3f sp%.1f along%.1f lat%.1f", c.ID, c.Pos, c.Heading, c.Speed, c.Pos.Sub(g.Bus.Pos).Dot(u), c.Pos.Sub(g.Bus.Pos).Dot(r))
				}
			}
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(g.Bus.Pos) < 350 {
					t.Logf("near bus%d p%v h%.3f sp%.1f obs%+v", a.ID, a.Bus.Pos, a.Bus.Heading, a.Bus.Speed, autopilot.ObserveBus(a.Bus))
				}
			}
			t.Fatalf("contact tick%d health%.0f->%.0f sp%.1f pos%v", tick, health, g.Bus.Durability, g.Bus.Speed, g.Bus.Pos)
		}
		if math.IsNaN(g.Bus.Speed) {
			t.Fatal("non-finite")
		}
	}
}
