package game

import (
	"encoding/json"
	"math"
	"testing"
)

// Native probe runs cover GPU submission/compilation. This regression checks
// that the loading gate holds simulation and preserves fractional Mac layout.
func TestGraphicsWarmupGatesSimulationAndPreservesLayout(t *testing.T) {
	g := NewGame(1280, 720)
	g.State = StatePlaying
	startup := WithGraphicsWarmup(g)
	snapshot := func() string {
		b, err := json.Marshal(struct{ Bus, World, Route any }{g.Bus, g.World, g.Route})
		if err != nil {
			t.Fatal(err)
		}
		return string(b)
	}
	before := snapshot()
	for i := 0; i < 10; i++ {
		if err := startup.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if startup.ready || before != snapshot() {
		t.Fatal("simulation advanced before loading draw completed")
	}
	w, h := startup.LayoutF(1470, 956)
	if w != 1280 || math.Abs(h-1280*956./1470) > 1e-9 {
		t.Fatalf("fractional fullscreen layout changed: %v %v", w, h)
	}
	startup.ready = true
	g.Bus.Speed = 100
	position := g.Bus.Pos
	if err := startup.Update(); err != nil {
		t.Fatal(err)
	}
	if g.Bus.Pos.Distance(position) == 0 {
		t.Fatal("simulation did not resume after loading")
	}
}
