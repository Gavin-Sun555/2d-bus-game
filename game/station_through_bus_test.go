package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

// Real CBD District 18 snapshot: Line 2 does not serve the player's target bay.
func TestPlayerDocksAtCBD18WhileOtherLinePasses(t *testing.T) {
	for _, fast := range []bool{false, true} {
		name := "standard"
		if fast {
			name = "overtaking"
		}
		t.Run(name, func(t *testing.T) { testCBD18ThroughBus(t, fast) })
	}
}
func testCBD18ThroughBus(t *testing.T, fast bool) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	stop := g.World.Stops[29]
	if stop.CleanName() != "都会新区18站" {
		t.Fatalf("unexpected repro station %s", stop.Name)
	}
	u := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	r := sim.Vec2{X: -u.Y, Y: u.X}
	from, to := -1, -1
	before, after := math.Inf(1), math.Inf(1)
	for _, s := range g.World.Stops {
		d := s.RoadCenter.Sub(stop.RoadCenter)
		if math.Abs(d.Dot(r)) > 1 || math.Cos(s.Heading-stop.Heading) < .99 {
			continue
		}
		along := d.Dot(u)
		if along < -400 && -along < before {
			from, before = s.ID, -along
		}
		if along > 400 && along < after {
			to, after = s.ID, along
		}
	}
	if from < 0 || to < 0 {
		t.Fatal("need adjacent through stops")
	}
	g.Route = &sim.Route{Name: "Line 1", StopIDs: []int{from, stop.ID, to}, CurrentIndex: 1}
	playerPos := stop.Pos.Sub(u.Mul(90)).Sub(r.Mul(20))
	g.Bus = sim.NewBus(playerPos.X, playerPos.Y, stop.Heading+.24)
	g.AutopilotDriver = autopilot.NewAutopilotDriver()
	g.AutopilotDriver.BerserkMode = true
	lane := stop.RoadCenter.Sub(u.Mul(190)).Add(r.Mul((float64(stop.RoadLanes)/2 - .5) * 52))
	follower := &traffic.AIBus{ID: 9, LineName: "Line 2", Bus: sim.NewBus(lane.X, lane.Y, stop.Heading), Route: &sim.Route{Name: "Line 2", StopIDs: []int{from, to}, CurrentIndex: 1}, Driver: autopilot.NewAutopilotDriver(), LastExchangedStopID: -1}
	follower.Driver.BerserkMode = fast
	follower.Driver.BerserkMaxSpeed = sim.SpeedKPH(50)
	follower.Driver.CruiseSpeedLimit = sim.SpeedKPH(50)
	g.AIBusMgr.Buses = []*traffic.AIBus{follower}
	g.resetOperations()
	if follower.Route.ContainsStop(stop.ID) || g.IsStopOccupiedByOtherVehicle(stop) {
		t.Fatal("through bus must not occupy the bay")
	}
	docked := false
	passed := false
	body := func(b *sim.Bus) sim.OBB {
		return sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}
	}
	for tick := 0; tick < 1800; tick++ {
		g.AIBusMgr.Update(1./60, g.Bus, g.World, g.TrafficLights, nil, 52, 1)
		if !docked {
			th, br, st, hb, doors, dock := g.AutopilotDriver.ComputeControls(g.Bus, g.Route, g.World, g.TrafficLights, g.AIBusMgr.GetObstacles(0), 52, 1./60)
			g.Bus.Update(1./60, th, br, st, hb, doors)
			docked = dock
		} else {
			g.Bus.Speed = 0
		}
		if hit, _, _ := sim.CheckOBBCollision(body(g.Bus), body(follower.Bus)); hit {
			t.Fatalf("collision tick%d player%v follower%v", tick, g.Bus.Pos, follower.Bus.Pos)
		}
		if follower.Bus.Pos.Sub(stop.Pos).Dot(u) > 200 {
			passed = true
		}
		if docked && passed {
			return
		}
	}
	t.Fatalf("docked%v passed%v player%v follower%v target%.2f", docked, passed, g.Bus.Pos, follower.Bus.Pos, g.AutopilotDriver.LastTargetSpeed)
}
