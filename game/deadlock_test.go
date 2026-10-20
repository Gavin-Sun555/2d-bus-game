package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestPlayerRecoversOnlyAfterCompleteStationaryTimeout(t *testing.T) {
	g := NewGame(1280, 720)
	g.Route.CurrentIndex = 1
	g.Bus.Pos = sim.Vec2{X: -400, Y: 26}
	g.Bus.Heading = 0
	g.Bus.Speed = 0
	g.Bus.DoorsOpen = false
	g.TrafficLights = nil
	ob := []autopilot.VehicleObstacle{{Pos: g.Bus.Pos.Add(sim.Vec2{X: 60}), Heading: math.Pi, Length: 46}}
	for i := 0; i < 110; i++ {
		g.updateAutopilotDeadlock(.5, 52, ob, nil)
	}
	if g.Route.CurrentIndex != 1 {
		t.Fatal("Player reset before 55.5 seconds")
	}
	g.updateAutopilotDeadlock(.5, 52, ob, nil)
	if g.Route.CurrentIndex != 0 || g.AutopilotDeadlockTimer != 0 {
		t.Fatal("Player did not reset after 55.5 seconds")
	}
}
