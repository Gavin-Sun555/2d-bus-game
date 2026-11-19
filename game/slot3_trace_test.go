package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"os"
	"path/filepath"
	"strconv"
	"testing"
)

// Optional full-city trace. Keeps every original timer and saves the first
// large tracking deviation before off-route recovery can replace its path.
func TestSlot3TraceFleetTracking(t *testing.T) {
	p := os.Getenv("BUS_TRACE_SAVE")
	if p == "" {
		t.Skip("BUS_TRACE_SAVE not set")
	}
	id, err := strconv.Atoi(os.Getenv("BUS_TRACE_ID"))
	if err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, p, 1280, 720)
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	var a *traffic.AIBus
	if id == 0 {
		a = &traffic.AIBus{Bus: g.Bus, Driver: g.AutopilotDriver, Route: g.Route}
	} else {
		a = fleetBus(t, g, id)
	}
	seconds := 180
	if raw := os.Getenv("BUS_TRACE_SECONDS"); raw != "" {
		seconds, err = strconv.Atoi(raw)
		if err != nil || seconds < 1 {
			t.Fatal("invalid BUS_TRACE_SECONDS")
		}
	}
	captureTick, _ := strconv.Atoi(os.Getenv("BUS_TRACE_CAPTURE_TICK"))
	saved := false
	for tick := 0; tick < seconds*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		d := a.Driver
		if captureTick > 0 && tick+1 == captureTick {
			g.savePathOverride = filepath.Join(os.Getenv("BUS_TRACE_OUTPUT"), "before-event.json")
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
		}
		if tick%60 == 0 {
			s := d.Snapshot()
			t.Logf("t%.2f pos%v h%.3f v%.1f wheel%.2f goal%v idx%d s%.1f lat%.1f offset%.1f/%.1f turn%+v right%v approach%+v exit%+v", float64(tick)/60, a.Bus.Pos, a.Bus.Heading, a.Bus.Speed, a.Bus.SteeringAngle, d.LastPursuitPt, d.WaypointIdx, d.LastPathS, d.LastLateralErr, d.LaneOffset, d.TargetLaneOffset, s.Turn, s.RightTurnJunction, s.ApproachMerge, s.ExitCruise)
		}
		if !saved && d.LastLateralErr > 80 {
			if out := os.Getenv("BUS_TRACE_OUTPUT"); out != "" {
				g.savePathOverride = filepath.Join(out, "first-deviation.json")
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			saved = true
		}
	}
}
