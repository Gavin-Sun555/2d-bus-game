package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

// Replays a copied slot one without changing the user's file. The far-side
// short bus should start before the articulated owner fully leaves the ring.
func TestSavedRingEntryGap(t *testing.T) {
	path := os.Getenv("BUS_RING_ENTRY_SAVE")
	if path == "" {
		t.Skip("BUS_RING_ENTRY_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		sim.SeedSimulation(seed)
		g := NewGame(1280, 720)
		g.savePathOverride = path
		if err := g.LoadGame(); err != nil {
			t.Fatal(err)
		}
		sim.SeedSimulation(seed)
		var ai *sim.Bus
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == 14 {
				ai = a.Bus
			}
		}
		if ai == nil {
			t.Fatal("missing AI14")
		}
		rb := g.World.Roundabouts[0]
		initial := ai.Pos
		playerHealth, aiHealth := g.Bus.Durability, ai.Durability
		firstMotion, entered, exited := -1., -1., -1.
		tailStillInside := false
		for tick := 0; tick < 1800; tick++ {
			previous := ai.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			elapsed := float64(tick) / 60
			if ai.Pos.Distance(previous) > ai.MaxSpeedFwd/60+1 {
				t.Fatal("admission teleported bus")
			}
			if ai.Durability < aiHealth || g.Bus.Durability < playerHealth {
				t.Fatalf("seed%d contact at %.2fs player%.1f AI14%.1f", seed, elapsed, g.Bus.Durability, ai.Durability)
			}
			for _, own := range ai.BodyParts() {
				for _, other := range g.Bus.BodyParts() {
					if own.HalfLen <= 0 || other.HalfLen <= 0 {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(own, other); hit && depth > 1e-6 {
						t.Fatalf("bodies overlap at %.2fs", elapsed)
					}
				}
			}
			if firstMotion < 0 && ai.Pos.Distance(initial) > 20 {
				firstMotion = elapsed
				for _, part := range g.Bus.BodyParts() {
					if part.HalfLen <= 0 {
						continue
					}
					u := sim.UnitFromAngle(part.Heading)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					d := rb.Center.Sub(part.Center)
					nearest := part.Center.Add(u.Mul(sim.Clamp(d.Dot(u), -part.HalfLen, part.HalfLen))).Add(r.Mul(sim.Clamp(d.Dot(r), -part.HalfWid, part.HalfWid)))
					tailStillInside = tailStillInside || nearest.Distance(rb.Center) < rb.Radius+8
				}
			}
			if entered < 0 && ai.Pos.Distance(rb.Center) < rb.Radius {
				entered = elapsed
			}
			if entered >= 0 && exited < 0 && ai.Pos.X < rb.Center.X-rb.Radius-100 {
				exited = elapsed
			}
		}
		if firstMotion < 0 || firstMotion > .8 || !tailStillInside || entered < 0 || entered > 2.5 || exited < 0 || exited > 15 || ai.Pos.Distance(initial) < 1000 {
			t.Fatalf("seed%d move%.2f entry%.2f exit%.2f ownerStillInside%v", seed, firstMotion, entered, exited, tailStillInside)
		}
		t.Logf("seed%d AI14 first20px%.2fs (owner still in ring), entry%.2fs exit%.2fs; 30s no overlap/contact", seed, firstMotion, entered, exited)
	}
}
