package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"testing"
)

// Read-only copied saves; vary future ambient traffic without altering the
// original scene. Distinguish the original departure lock from later red lights
// and passenger-dependent station service.
func TestSavedDepartureSlotRecovery(t *testing.T) {
	path := os.Getenv("BUS_STATION_DEPARTURE_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_DEPARTURE_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprintf("seed%d", seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := NewGame(1280, 720)
			g.savePathOverride = path
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			sim.SeedSimulation(seed)
			origin := g.Bus.Pos
			health := g.Bus.Durability
			var follower *sim.Bus
			nearest := math.Inf(1)
			followerID := -1
			forward := sim.UnitFromAngle(g.Bus.Heading)
			for _, a := range g.AIBusMgr.Buses {
				d := a.Bus.Pos.Sub(origin)
				if d.Dot(forward) < -8 && d.Length() < nearest {
					nearest = d.Length()
					follower = a.Bus
					followerID = a.ID
				}
			}
			if follower == nil {
				t.Fatal("missing saved rear follower")
			}
			rearOrigin, rearHealth := follower.Pos, follower.Durability
			playerMoved, rearMoved := -1., -1.
			for tick := 0; tick < 1800; tick++ {
				before := g.Bus.Pos
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Pos.Distance(before) > g.Bus.MaxSpeedFwd/60+1 {
					t.Fatal("recovery relocated player bus")
				}
				elapsed := float64(tick+1) / 60
				if playerMoved < 0 && g.Bus.Pos.Distance(origin) > 20 {
					playerMoved = elapsed
				}
				if rearMoved < 0 && follower.Pos.Distance(rearOrigin) > 20 {
					rearMoved = elapsed
				}
				if g.Bus.Durability < health {
					t.Fatalf("player contact after %.2fs", elapsed)
				}
				if follower.Pos.Distance(rearOrigin) < 700 && follower.Durability < rearHealth {
					t.Fatalf("AI%d contact in original station zone after %.2fs", followerID, elapsed)
				}
				for _, part := range follower.BodyParts() {
					if part.HalfLen > 0 {
						if hit, _, depth := g.Bus.CollidesBody(part); hit && depth > 1e-6 {
							t.Fatalf("physical overlap %.2fs depth%g", elapsed, depth)
						}
					}
				}
			}
			if playerMoved < 0 || playerMoved > 5 || rearMoved < 0 || rearMoved > 15 || g.Bus.Pos.Distance(origin) < 200 {
				t.Fatalf("player%.2fs AI%d %.2fs player distance%g", playerMoved, followerID, rearMoved, g.Bus.Pos.Distance(origin))
			}
			t.Logf("player resumed%.2fs AI%d resumed%.2fs; no contact through30s", playerMoved, followerID, rearMoved)
		})
	}
}
