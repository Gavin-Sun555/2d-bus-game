package game

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot1-early-shoulder-20261116.json.gz
var slot1EarlyShoulderScene []byte

func TestSavedSlot1BerserkAvoidsEarlyShoulder(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot1EarlyShoulderScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s gameSave
	if err := json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	position := s.Player.Bus.Pos
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Pos != position {
		t.Fatal("load repair moved the player")
	}
	sim.SeedSimulation(5)
	verifySlot1EarlyShoulder(t, g)
}

func TestCurrentSlot1BerserkAvoidsEarlyShoulder(t *testing.T) {
	p := os.Getenv("BUS_SLOT1_SHOULDER_SAVE")
	if p == "" {
		t.Skip("set BUS_SLOT1_SHOULDER_SAVE")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, p, 1280, 720)
	sim.SeedSimulation(5)
	verifySlot1EarlyShoulder(t, g)
}

func verifySlot1EarlyShoulder(t *testing.T, g *Game) {
	t.Helper()
	if len(g.TrafficMgr.Cars) != 864 || len(g.AIBusMgr.Buses) != 72 {
		t.Fatal("fixture traffic changed")
	}
	d := g.AutopilotDriver.Snapshot()
	stop := g.World.Stops[198]
	u := sim.UnitFromAngle(stop.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	curb := 3.5 * g.Settings.GetLaneWidth()
	target := stop.FrontBerthPosition()
	entry := target.Sub(u.Mul(230)).Add(r.Mul(-47.12))
	found := false
	for _, p := range d.DrivePath {
		if p.Distance(entry) < .01 {
			found = true
		}
	}
	if !found {
		t.Fatal("missing travel-lane waypoint before station taper")
	}
	health := g.Bus.Durability
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player relocated") }
	recovered, served := false, false
	recoveryAt, serviceAt := -1, -1
	for tick := 0; tick < 60*45; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatal("player damaged")
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.1 {
			t.Fatal("player position jumped")
		}
		distance := target.Sub(g.Bus.Pos).Dot(u)
		lateral := g.Bus.Pos.Sub(stop.RoadCenter).Dot(r)
		if !recovered && distance > 450 && math.Abs(lateral-curb) < 5 {
			recovered = true
			recoveryAt = tick
		}
		if distance > 450 && recovered && lateral > curb+5 {
			t.Fatal("returned to shoulder before station approach")
		}
		if distance < 450 && !recovered {
			t.Fatal("never returned to road before arrival")
		}
		if !served && g.Bus.DoorsOpen && stop.ContainsBus(g.Bus) {
			served = true
			serviceAt = tick
		}
		if served && g.Route.GetCurrentTargetStopID() != 198 && g.Bus.Pos.Sub(target).Dot(u) > 500 {
			t.Logf("rejoined road %.2fs, boarded %.2fs, departed %.2fs; no damage or relocation", float64(recoveryAt)/60, float64(serviceAt)/60, float64(tick)/60)
			return
		}
	}
	t.Fatalf("failed to recover and serve: recovered%v served%v pos%v speed%.1f", recovered, served, g.Bus.Pos, g.Bus.Speed)
}
