package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

// Read only a copied save. The stationary car beside AI 2's rear used to be
// exempted as a lawful queue forever, blocking both AI 2 and the player.
func TestSavedDepartureDeadlockRecovery(t *testing.T) {
	path := os.Getenv("BUS_DEPARTURE_JAM_SAVE")
	if path == "" {
		t.Skip("BUS_DEPARTURE_JAM_SAVE not set")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	target := g.Route.GetCurrentTargetStopID()
	traveled := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	contacts := map[int]int{}
	motionTimes := map[int]float64{}
	leftLaneAt := -1.
	aiLeftLaneAt := -1.
	source := g.World.Stops[g.AutopilotDriver.Snapshot().PathFromID]
	u := sim.UnitFromAngle(source.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for _, a := range g.AIBusMgr.Buses {
		health[a.ID] = a.Bus.Durability
	}
	firstMotion, servedAt := -1., -1.
	distanceAtService := 0.
	extended := os.Getenv("BUS_DEPARTURE_JAM_EXTENDED") == "1"
	doors := false
	for tick := 0; tick < 21600; tick++ {
		positions := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, a := range g.AIBusMgr.Buses {
			positions[a.ID] = a.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		check := func(id int, b *sim.Bus) {
			step := b.Pos.Distance(positions[id])
			if step > b.MaxSpeedFwd/60+5 {
				t.Fatalf("vehicle%d jumped at tick%d", id, tick)
			}
			if b.Durability < health[id] {
				contacts[id]++
				t.Logf("vehicle%d contact at %.2fs: %.1f->%.1f", id, float64(tick)/60, health[id], b.Durability)
				health[id] = b.Durability
				if contacts[id] > 1 || b.Durability < 94 {
					t.Fatalf("vehicle%d repeated/heavy contact: contacts%d health%.1f", id, contacts[id], b.Durability)
				}
			}
			if id == 2 && aiLeftLaneAt < 0 {
				relative := b.Pos.Sub(source.RoadCenter)
				if relative.Dot(u) > 0 && relative.Dot(u) < 800 && relative.Dot(r) > b.Width/2 && relative.Dot(r) < g.Settings.GetLaneWidth()*.75 && sim.UnitFromAngle(b.Heading).Dot(u) > .94 {
					aiLeftLaneAt = float64(tick) / 60
					t.Logf("AI 2 entered forward left-turn lane %.2fs", aiLeftLaneAt)
				}
			}
			traveled[id] += step
			if _, moving := motionTimes[id]; !moving && traveled[id] > 20 {
				motionTimes[id] = float64(tick) / 60
				if id == 2 {
					blockingCarPresent := false
					for _, car := range g.TrafficMgr.Cars {
						blockingCarPresent = blockingCarPresent || car.ID == 65
					}
					if !blockingCarPresent {
						t.Fatal("AI departure relied on deleting original rear car")
					}
				}
			}
		}
		check(0, g.Bus)
		for _, a := range g.AIBusMgr.Buses {
			check(a.ID, a.Bus)
		}
		relative := g.Bus.Pos.Sub(source.RoadCenter)
		if leftLaneAt < 0 && relative.Dot(u) > 0 && relative.Dot(u) < 800 && relative.Dot(r) > g.Bus.Width/2 && relative.Dot(r) < g.Settings.GetLaneWidth()*.75 && sim.UnitFromAngle(g.Bus.Heading).Dot(u) > .94 {
			leftLaneAt = float64(tick) / 60
			t.Logf("entered forward left-turn lane %.2fs", leftLaneAt)
		}
		if firstMotion < 0 && traveled[0] > 20 {
			firstMotion = float64(tick) / 60
			t.Logf("player resumed %.2fs", firstMotion)
		}
		if g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			doors = true
		}
		if servedAt < 0 && g.Route.GetCurrentTargetStopID() != target {
			if !doors || g.lastSkippedStopID == target {
				t.Fatal("advanced without serving target")
			}
			distanceAtService = traveled[0]
			servedAt = float64(tick) / 60
			t.Logf("original target served %.2fs", servedAt)
		}
		// Qualify this saved departure through real service at its next stop
		// and a subsequent 300px departure. An opt-in longer soak retains the
		// separate post-recovery contact reproduction; do not conflate it with
		// the original stationary deadlock.
		if !extended && servedAt >= 0 && traveled[0]-distanceAtService > 300 {
			t.Logf("cleared original target after service at %.2fs", float64(tick)/60)
			break
		}
	}
	if firstMotion < 0 || firstMotion > 3 || leftLaneAt < 0 || leftLaneAt > 30 || motionTimes[2] > 1 || aiLeftLaneAt < 0 || aiLeftLaneAt > 20 || servedAt < 0 || servedAt > 65 || traveled[0] < 2000 || traveled[2] < 1000 {
		t.Fatalf("recovery incomplete: first%.2f served%.2f distance%v", firstMotion, servedAt, traveled)
	}
	t.Logf("replay: travel%v contacts%v final health%v first motion%v", traveled, contacts, health, motionTimes)
}
