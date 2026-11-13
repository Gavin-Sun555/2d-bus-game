package game

import (
	"math"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Standard autopilot must drive the player's bus exactly as the fleet loop
// drives a route bus. Two identical games run the player's line; after the
// first departure one hands its bus, route and controller to the fleet. The
// two buses must then follow the same trajectory through cruising, signals,
// served stops and skipped stops.
func TestStandardAutopilotMatchesFleetBus(t *testing.T) {
	if testing.Short() {
		t.Skip("long parity simulation")
	}
	newGame := func() *Game {
		sim.SeedSimulation(7)
		g := NewGame(1280, 720)
		g.State = StatePlaying
		g.AIBusMgr.Buses = nil
		g.Settings.AutopilotEnabled = true
		g.Settings.AutopilotBerserk = false
		return g
	}
	player, fleet := newGame(), newGame()
	step := func(f int) {
		// Empty alternate stops so the run also covers stop skipping.
		for _, g := range []*Game{player, fleet} {
			for i, id := range g.activeLines()[0].StopIDs {
				if i%2 == 1 {
					st := g.World.Stops[id]
					st.WaitingPassengers, st.WaitingDestinations, st.WaitingSince = 0, nil, nil
				}
			}
		}
		sim.SeedSimulation(int64(f))
		_ = player.Update()
		sim.SeedSimulation(int64(f))
		_ = fleet.Update()
	}
	const warm = 60 * 40
	for f := 0; f < warm; f++ {
		step(f)
	}
	if player.Bus.Pos != fleet.Bus.Pos || player.Bus.DoorsOpen {
		t.Fatalf("games diverged before the handover: %v %v", player.Bus.Pos, fleet.Bus.Pos)
	}

	ab := &traffic.AIBus{ID: 1, LineName: fleet.Route.Name, Bus: fleet.Bus, Route: fleet.Route, Driver: fleet.AutopilotDriver, LastExchangedStopID: -1}
	fleet.AIBusMgr.Buses = []*traffic.AIBus{ab}
	park := farFromRoads(fleet.World)
	fleet.Bus = sim.NewBus(park.X, park.Y, 0)
	fleet.Route = &sim.Route{Name: "parked", StopIDs: append([]int(nil), ab.Route.StopIDs...)}
	fleet.AutopilotDriver = autopilot.NewAutopilotDriver()
	fleet.Settings.AutopilotEnabled = false

	// The parked player bus changes camera-relative car spawning far away;
	// stop before that reaches the line.
	served, skipped, moved := map[int]bool{}, false, 0.
	for f := warm; f < 60*390; f++ {
		before := player.Bus.Pos
		step(f)
		moved += player.Bus.Pos.Distance(before)
		if d := player.Bus.Pos.Distance(ab.Bus.Pos); d > .01 || math.Abs(player.Bus.Speed-ab.Bus.Speed) > .01 {
			t.Fatalf("t=%.2f player %v %.1fpx/s %s, fleet %v %.1fpx/s %s", float64(f)/60,
				player.Bus.Pos, player.Bus.Speed, player.AutopilotDriver.State, ab.Bus.Pos, ab.Bus.Speed, ab.Driver.State)
		}
		if ab.Bus.DoorsOpen {
			served[ab.Route.GetCurrentTargetStopID()] = true
		}
		for _, toast := range player.HUD.Toasts {
			skipped = skipped || strings.Contains(toast.Text, "AI Skip") || strings.Contains(toast.Text, "AI自主飞站")
		}
	}
	if len(served) < 5 || !skipped || moved < 15000 {
		t.Fatalf("run too short to compare: served %d stops, skipped %v, moved %.0fpx", len(served), skipped, moved)
	}
}

func farFromRoads(w *sim.World) sim.Vec2 {
	minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
	for _, r := range w.Roads {
		minX, maxX = math.Min(minX, math.Min(r.P1.X, r.P2.X)), math.Max(maxX, math.Max(r.P1.X, r.P2.X))
		minY, maxY = math.Min(minY, math.Min(r.P1.Y, r.P2.Y)), math.Max(maxY, math.Max(r.P1.Y, r.P2.Y))
	}
	best, clearance := sim.Vec2{}, -1.
	for x := minX; x <= maxX; x += 50 {
		for y := minY; y <= maxY; y += 50 {
			p, d := sim.Vec2{X: x, Y: y}, math.Inf(1)
			for _, r := range w.Roads {
				seg := r.P2.Sub(r.P1)
				u := math.Max(0, math.Min(1, p.Sub(r.P1).Dot(seg)/math.Max(seg.Dot(seg), 1e-9)))
				d = math.Min(d, p.Distance(r.P1.Add(seg.Mul(u))))
			}
			if d > clearance {
				best, clearance = p, d
			}
		}
	}
	return best
}
