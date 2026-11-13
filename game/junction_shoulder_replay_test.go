package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// Opt-in replay of slot 1's high-speed junction exit and stop-demand rebuild.
func TestJunctionShoulderReplay(t *testing.T) {
	path := os.Getenv("BUS_JUNCTION_SHOULDER_REPLAY")
	if path == "" {
		t.Skip("BUS_JUNCTION_SHOULDER_REPLAY not set")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	for _, seed := range []int64{5, 17} {
		sim.SeedSimulation(seed)
		g := NewGame(1280, 720)
		g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
		if err = os.WriteFile(g.savePathOverride, raw, 0600); err != nil {
			t.Fatal(err)
		}
		if err = g.LoadGame(); err != nil {
			t.Fatal(err)
		}
		health := g.Bus.Durability
		demandChanged := false
		skip := g.AutopilotDriver.CanSkipTargetStop
		maxYaw := 0.
		g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player repositioned") }
		cleared := false
		for tick := 0; tick < 900; tick++ {
			before := g.Bus.Pos
			if err = g.Update(); err != nil {
				t.Fatal(err)
			}
			demandChanged = demandChanged || g.AutopilotDriver.CanSkipTargetStop != skip
			yaw := math.Abs(sim.NormalizeAngle(g.Bus.Heading - math.Pi/2))
			maxYaw = math.Max(maxYaw, yaw)
			if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.05 || g.Bus.Durability < health {
				t.Fatalf("seed%d player repositioned or damaged", seed)
			}
			if g.Bus.CheckBoundaries(g.World, 52).Zone != sim.ZoneRoad {
				t.Fatalf("seed%d entered shoulder at %.2fs pos%v", seed, float64(tick+1)/60, g.Bus.Pos)
			}
			if yaw > .3 {
				t.Fatalf("seed%d abrupt turn %.1f degrees at %.2fs", seed, yaw*180/math.Pi, float64(tick+1)/60)
			}
			if g.Bus.Pos.Y > 2500 {
				cleared = true
				t.Logf("seed%d cleared exit in %.2fs, maximum yaw %.1f degrees", seed, float64(tick+1)/60, maxYaw*180/math.Pi)
				break
			}
		}
		if !cleared || !demandChanged {
			t.Fatalf("seed%d did not exercise junction exit and stop-demand change", seed)
		}
	}
}
