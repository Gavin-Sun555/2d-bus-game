package autopilot

import (
	_ "embed"
	"encoding/json"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/four-lane-left-gap-bus72-2026-11-08.json
var fourLaneGapFixture []byte

// The root of the captured car147745 queue was bus72 silently pursuing the
// median reference from an offset physical lane. A saved normal-mode driver
// must request a slot, retain that request, and keep the trip intact.
func TestCapturedFourLaneBusPublishesAndRestoresLeftMerge(t *testing.T) {
	var x struct {
		World  *sim.World
		Bus    *sim.Bus
		Route  *sim.Route
		Lights *sim.TrafficLightSystem
		Driver DriverSnapshot
	}
	if err := json.Unmarshal(fourLaneGapFixture, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	if ad.BerserkMode || ad.approachMerge != nil {
		t.Fatal("fixture lost normal unannounced merge")
	}
	before, index, passengers := x.Bus.Pos, x.Route.CurrentIndex, x.Bus.PassengerCount
	ad.BeforeRecovery = func() { t.Fatal("repositioned instead of requesting slot") }
	ob := VehicleObstacle{Pos: sim.Vec2{X: 61882.044073454555, Y: 14426}, Length: 46, Width: 22}
	for step := 0; step < 2; step++ {
		ad.ComputeControls(x.Bus, x.Route, x.World, x.Lights, []VehicleObstacle{ob}, 52, 1./60)
		r := x.Bus.LaneMergeRequest
		if r == nil || !r.LeftTurn || r.Junction != (sim.Vec2{X: 62400, Y: 14400}) || r.Target.Y != 14426 {
			t.Fatalf("missing safe-slot request: %+v", r)
		}
		if x.Bus.Pos != before || x.Route.CurrentIndex != index || x.Bus.PassengerCount != passengers {
			t.Fatal("changed journey to clear jam")
		}
		snapshot := ad.Snapshot()
		raw, err := json.Marshal(snapshot)
		if err != nil {
			t.Fatal(err)
		}
		var saved DriverSnapshot
		if err = json.Unmarshal(raw, &saved); err != nil {
			t.Fatal(err)
		}
		ad, err = saved.Restore(x.World)
		if err != nil {
			t.Fatal(err)
		}
	}
}
