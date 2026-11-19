package traffic

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot3-crossing-wedge-20261116.json.gz
var crossingWedge []byte

func crossingWedgeFixture(t *testing.T) (*sim.World, *TrafficManager, *sim.TrafficLightSystem) {
	t.Helper()
	z, err := gzip.NewReader(bytes.NewReader(crossingWedge))
	if err != nil {
		t.Fatal(err)
	}
	defer z.Close()
	var s struct {
		World  *sim.World
		Cars   []*sim.TrafficCar
		Lights *sim.TrafficLightSystem
	}
	if err = json.NewDecoder(z).Decode(&s); err != nil {
		t.Fatal(err)
	}
	return s.World, &TrafficManager{Cars: s.Cars}, s.Lights
}

func TestCommittedCrossingWedgeReleasesWithoutReset(t *testing.T) {
	w, tm, lights := crossingWedgeFixture(t)
	starts := []sim.Vec2{tm.Cars[0].Pos, tm.Cars[1].Pos}
	tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { t.Fatal("reset") }
	reversed := false
	for tick := 0; tick < 30*60; tick++ {
		lights.Update(1. / 60)
		for i, c := range tm.Cars {
			before := c.Pos
			tm.StepCar(1./60, c, i, nil, lights, 2, 52, w)
			if c.Speed < 0 {
				reversed = true
			}
			if c.Pos.Distance(before) > math.Abs(c.Speed)/60+.01 {
				t.Fatal("discontinuous motion")
			}
		}
		a, b := tm.Cars[0], tm.Cars[1]
		if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: a.Length / 2, HalfWid: a.Width / 2}, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}); hit {
			t.Fatal("cars collided")
		}
		if a.Pos.Distance(starts[0]) > 650 && b.Pos.Distance(starts[1]) > 650 {
			if !reversed {
				t.Fatal("expected physical retreat")
			}
			return
		}
	}
	t.Fatal("crossing pair did not clear")
}

func TestCommittedCrossingRetreatSafety(t *testing.T) {
	for _, name := range []string{"rear-car", "moving-rear-car", "rear-bus", "ordinary-queue", "uncommitted", "not-stuck", "hard-barrier"} {
		t.Run(name, func(t *testing.T) {
			w, tm, _ := crossingWedgeFixture(t)
			c := tm.Cars[0]
			if c.ID != 117 {
				c = tm.Cars[1]
			}
			f := carFrame{car: c, world: w, laneWidth: 52, activeCount: 2}
			switch name {
			case "rear-car", "moving-rear-car":
				rear := &sim.TrafficCar{ID: 999, Pos: c.Pos.Sub(sim.UnitFromAngle(c.Heading).Mul(55)), Heading: c.Heading, Length: 50, Width: 24}
				if name == "moving-rear-car" {
					rear.Pos = rear.Pos.Sub(sim.UnitFromAngle(c.Heading).Mul(80))
					rear.Speed = 120
				}
				tm.Cars = append(tm.Cars, rear)
			case "rear-bus":
				f.buses = []*sim.Bus{sim.NewBus(c.Pos.X, c.Pos.Y-65, c.Heading)}
			case "ordinary-queue":
				for _, o := range tm.Cars {
					if o != c {
						o.JunctionTurn.Straight = true
					}
				}
			case "uncommitted":
				c.JunctionTurn = nil
			case "hard-barrier":
				w.Buildings = append(w.Buildings, sim.Building{Pos: c.Pos.Sub(sim.UnitFromAngle(c.Heading).Mul(c.Length/2 + 10)), W: 80, H: 4})
			case "not-stuck":
				c.DeadlockTimer = 1
			}
			f.activeCount = len(tm.Cars)
			if handled, v := tm.stepJunctionRetreat(f); handled || v < 0 || c.JunctionReverseYieldID != 0 {
				t.Fatal("unsafe retreat accepted")
			}
		})
	}
}

func TestCommittedCrossingRetreatSupportsZeroIDAndSave(t *testing.T) {
	w, tm, _ := crossingWedgeFixture(t)
	var c *sim.TrafficCar
	for _, other := range tm.Cars {
		if other.ID == 255 {
			other.ID = 0
		} else {
			c = other
		}
	}
	f := carFrame{car: c, world: w, laneWidth: 52, activeCount: 2}
	if handled, speed := tm.stepJunctionRetreat(f); !handled || speed >= 0 || !c.JunctionReverseActive || c.JunctionReverseYieldID != 0 {
		t.Fatal("zero ID turn not released")
	}
	raw, err := json.Marshal(c)
	if err != nil {
		t.Fatal(err)
	}
	var restored sim.TrafficCar
	if err = json.Unmarshal(raw, &restored); err != nil {
		t.Fatal(err)
	}
	if !restored.JunctionReverseActive || restored.JunctionReverseYieldID != 0 || restored.ReverseRemaining != c.ReverseRemaining {
		t.Fatal("lost reverse commitment on save")
	}
}

func TestCommittedStraightReverseBacktracksSegment(t *testing.T) {
	c := &sim.TrafficCar{Pos: sim.Vec2{X: 25}, Speed: -10, JunctionReverseActive: true, JunctionTurn: &sim.CarJunctionTurn{Straight: true, Points: []sim.Vec2{{}, {X: 20}, {X: 40}}, Segment: 1, Progress: 5}}
	moveCarThroughJunction(c, 1)
	if c.Pos.X != 15 || c.JunctionTurn.Segment != 0 || c.JunctionTurn.Progress != 15 || c.Heading != 0 {
		t.Fatalf("incorrect continuous backtracking: %+v", c)
	}
}
