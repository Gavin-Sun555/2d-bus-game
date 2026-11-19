package game

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot3-bus90-before-platform-20261117.json.gz
var slot3Bus90PlatformScene []byte

// Retain the complete city and the original recovery timers. The old exit
// policy started a high-speed divider crossing just before the BRT island.
func TestSlot3Bus90PassesPlatformWithoutLateDividerCross(t *testing.T) {
	prevLang := locale.CurrentLanguage
	t.Cleanup(func() { locale.SetLanguage(prevLang) })
	z, err := gzip.NewReader(bytes.NewReader(slot3Bus90PlatformScene))
	if err != nil {
		t.Fatal(err)
	}
	defer z.Close()
	var saved gameSave
	if err = json.NewDecoder(z).Decode(&saved); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if err = g.restoreFromSave(saved); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	a := fleetBus(t, g, 90)
	start, health := a.Bus.Pos, a.Bus.Durability
	a.Driver.BeforeRecovery = func() { t.Fatal("bus replanned or relocated") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == 90 {
			t.Fatal("bus reset", e)
		}
	}
	for tick := 0; tick < 60*60; tick++ {
		before := a.Bus.Pos
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if a.Bus.Pos.Distance(before) > math.Abs(a.Bus.Speed)/60+.1 || a.Bus.Durability < health {
			t.Fatal("bus damaged or pushed")
		}
		if a.Bus.Pos.Distance(start) > 2500 {
			t.Logf("platform cleared in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("bus remained at platform", a.Bus.Pos)
}
