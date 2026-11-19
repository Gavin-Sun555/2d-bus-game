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

//go:embed testdata/slot1-berserk-left-20261116.json.gz
var slot1BerserkLeftScene []byte

func TestSavedSlot1BerserkCommitsToFasterLeftTurnLane(t *testing.T) {
	gz, err := gzip.NewReader(bytes.NewReader(slot1BerserkLeftScene))
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
	if err := g.restoreFromSave(s); err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	verifySlot1BerserkLeft(t, g)
}
func TestCurrentSlot1BerserkCommitsToFasterLeftTurnLane(t *testing.T) {
	path := os.Getenv("BUS_SLOT1_LEFT_SAVE")
	if path == "" {
		t.Skip("set BUS_SLOT1_LEFT_SAVE")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	verifySlot1BerserkLeft(t, g)
}
func verifySlot1BerserkLeft(t *testing.T, g *Game) {
	t.Helper()
	health := g.Bus.Durability
	g.AutopilotDriver.BeforeRecovery = func() { t.Fatal("player relocated instead of choosing left lane") }
	reachedInner := false
	innerAt := -1
	for tick := 0; tick < 60*90; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health {
			t.Fatal("player damaged during committed left turn")
		}
		if g.Bus.Pos.Distance(before) > math.Abs(g.Bus.Speed)/60+.1 {
			t.Fatal("player moved discontinuously")
		}
		d := g.AutopilotDriver.Snapshot()
		if tick == 0 && !d.OrdinaryBusLeft {
			t.Fatal("berserk player retained queued curb-left choice despite empty inner lane")
		}
		if g.Bus.Pos.X < 62000 && g.Bus.Pos.Y < 21655 {
			reachedInner = true
			if innerAt < 0 {
				innerAt = tick
			}
		}
		if g.Bus.Pos.X < 62000 && !d.OrdinaryBusLeft {
			t.Fatal("left-lane choice reversed during approach")
		}
		if reachedInner && g.Bus.Pos.X < 62000 && g.Bus.Pos.Y > 21665 {
			t.Fatal("player returned toward curb after reaching its selected left lane")
		}
		if g.Bus.Pos.Y < 21000 && math.Cos(g.Bus.Heading+math.Pi/2) > .9 {
			if !reachedInner {
				t.Fatal("player did not use leftmost approach")
			}
			t.Logf("leftmost lane reached at %.2fs; completed left turn at %.2fs pos%v with no damage or relocation", float64(innerAt)/60, float64(tick)/60, g.Bus.Pos)
			return
		}
	}
	t.Fatalf("left turn stalled: pos%v speed%.2f state%+v", g.Bus.Pos, g.Bus.Speed, g.AutopilotDriver.Snapshot().Public)
}
