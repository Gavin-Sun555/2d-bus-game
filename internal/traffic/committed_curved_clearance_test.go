package traffic

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/curved-car-bus37-2026-11-08.json
var curvedCarFixture []byte

func TestSavedCurvedCarClearsStationaryUTurningBus(t *testing.T) {
	var x struct {
		Car *sim.TrafficCar
		Bus *sim.Bus
	}
	if err := json.Unmarshal(curvedCarFixture, &x); err != nil {
		t.Fatal(err)
	}
	c, b := x.Car, x.Bus
	tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
	start := c.Pos
	if c.JunctionTurn == nil || c.JunctionTurn.Straight {
		t.Fatal("fixture lost committed curved movement")
	}
	for tick := 0; tick < 900; tick++ {
		f := carFrame{car: c, buses: []*sim.Bus{b}, laneWidth: 52, activeCount: 1}
		target := tm.busFollowingSpeed(f, 85)
		if c.Speed < target {
			c.Speed = math.Min(target, c.Speed+2)
		} else {
			c.Speed = math.Max(target, c.Speed-280./60)
		}
		moveCarThroughJunction(c, 1./60)
		if hit, _, depth := b.CollidesBody(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}); hit && depth > 0 {
			t.Fatal("car touched actual bus section")
		}
		if c.Pos.Distance(start) > 300 {
			return
		}
	}
	t.Fatalf("car still waits on straight extrapolation %v", c.Pos)
}
func TestCommittedCurvedCarStillStopsForRealCollision(t *testing.T) {
	var x struct {
		Car *sim.TrafficCar
		Bus *sim.Bus
	}
	json.Unmarshal(curvedCarFixture, &x)
	p, h := x.Car.JunctionTurn.Predict(40)
	x.Bus.Pos = p
	x.Bus.Heading = h
	x.Bus.ResetArticulation()
	if committedCarClearsStoppedTurn(x.Car, x.Bus) {
		t.Fatal("ignored actual obstruction on chosen curve")
	}
}

func TestReleasedTurningCarDoesNotResetAtSavedTimeout(t *testing.T) {
	var x struct {
		Car *sim.TrafficCar
		Bus *sim.Bus
	}
	json.Unmarshal(curvedCarFixture, &x)
	tm := &TrafficManager{Cars: []*sim.TrafficCar{x.Car}, AIBusMgr: &AIBusManager{Buses: []*AIBus{{Bus: x.Bus}}}}
	x.Car.DeadlockTimer = 2 * sim.DefaultSignalCycle
	x.Car.Speed = 2
	x.Car.IsBraking = false
	tm.BeforeRecovery = func(sim.TrafficRecoveryEvent) { t.Fatal("reset an accelerating, released car") }
	tm.ResolveDeadlocks(1./60, nil, 1, 52, nil)
	if x.Car.DeadlockTimer != 0 || len(tm.Cars) != 1 {
		t.Fatal("stale timeout survived a released movement")
	}
}
