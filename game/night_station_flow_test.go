package game

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"strconv"
	"testing"
)

//go:embed testdata/slot2-night-station20-20261116.json.gz
var nightStationFlow []byte

func TestNightStationArrivalAndDepartureBothClear(t *testing.T) {
	z, e := gzip.NewReader(bytes.NewReader(nightStationFlow))
	if e != nil {
		t.Fatal(e)
	}
	defer z.Close()
	var s gameSave
	if e = json.NewDecoder(z).Decode(&s); e != nil {
		t.Fatal(e)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	if e = g.restoreFromSave(s); e != nil {
		t.Fatal(e)
	}
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	a, b := fleetBus(t, g, 161), fleetBus(t, g, 164)
	sa, sb := a.Bus.Pos, b.Bus.Pos
	ha, hb := a.Bus.Durability, b.Bus.Durability
	a.Driver.BeforeRecovery = func() { t.Fatal("arrival relocated") }
	b.Driver.BeforeRecovery = func() { t.Fatal("departure relocated") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == 161 || e.ID == 164 {
			t.Fatal("station bus reset")
		}
	}
	seconds := 120
	if raw := os.Getenv("BUS_NIGHT_DEBUG_SECONDS"); raw != "" {
		seconds, _ = strconv.Atoi(raw)
	}
	boarded := false
	for tick := 0; tick < 60*seconds; tick++ {
		pa, pb := a.Bus.Pos, b.Bus.Pos
		if e = g.Update(); e != nil {
			t.Fatal(e)
		}
		if a.Bus.Pos.Distance(pa) > math.Abs(a.Bus.Speed)/60+.1 || b.Bus.Pos.Distance(pb) > math.Abs(b.Bus.Speed)/60+.1 {
			t.Fatal("bus jumped")
		}
		if a.Bus.Durability < ha || b.Bus.Durability < hb {
			t.Fatal("bus damaged")
		}
		if hit, _, _ := sim.CheckBusesCollision(a.Bus, b.Bus); hit {
			t.Fatal("station buses collided")
		}
		if os.Getenv("BUS_NIGHT_DEBUG_OUTPUT") != "" && tick%600 == 0 {
			for _, id := range []int{161, 164, 170, 178} {
				x := fleetBus(t, g, id)
				t.Logf("t%.0f bus%d pos%v h%.3f tail%.3f speed%.2f state%v target%.1f gear%v", float64(tick)/60, id, x.Bus.Pos, x.Bus.Heading, x.Bus.TailHeading(), x.Bus.Speed, x.Driver.State, x.Driver.LastTargetSpeed, x.Bus.Gear)
			}
		}
		boarded = boarded || (a.Bus.DoorsOpen && a.Driver.DockedStopID == 20)
		if boarded && a.Bus.Pos.Distance(sa) > 800 && b.Bus.Pos.Distance(sb) > 800 {
			t.Logf("both cleared in %.2fs", float64(tick)/60)
			return
		}
	}
	if path := os.Getenv("BUS_NIGHT_DEBUG_OUTPUT"); path != "" {
		g.savePathOverride = path
		if e := g.SaveGame(); e != nil {
			t.Fatal(e)
		}
	}
	t.Fatalf("station still blocked: boarded%v arrival%v departure%v", boarded, a.Bus.Pos, b.Bus.Pos)
}
