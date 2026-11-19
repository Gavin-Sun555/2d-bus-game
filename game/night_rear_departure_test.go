package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"os"
	"testing"
)

func TestNightCaptureRearDeparture(t *testing.T) {
	source, out := os.Getenv("BUS_REAR_CAPTURE_SAVE"), os.Getenv("BUS_REAR_CAPTURE_OUTPUT")
	if source == "" || out == "" {
		t.Skip("set rear capture input and output")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, source, 1280, 720)
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	for tick := 0; tick < 60*300; tick++ {
		if e := g.Update(); e != nil {
			t.Fatal(e)
		}
		s := g.AutopilotDriver.Snapshot()
		if s.PathFromID == 9 && s.DockDepartureRun > 0 && s.Public.LastPathS < 5 {
			g.savePathOverride = out
			if e := g.SaveGame(); e != nil {
				t.Fatal(e)
			}
			t.Logf("captured %.2fs pos%v head%.5f tail%.5f wheel%.5f", float64(tick)/60, g.Bus.Pos, g.Bus.Heading, g.Bus.TailHeading(), g.Bus.SteeringAngle)
			return
		}
	}
	t.Fatal("rear departure not observed")
}
