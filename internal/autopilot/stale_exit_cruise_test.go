package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot1-stale-exit-slow25-2026-11-08.json
var staleExitFixture []byte

func loadStaleExit(t *testing.T) (*AutopilotDriver, *sim.Bus, *sim.Route, *sim.World, *sim.TrafficLightSystem) {
	t.Helper()
	var x struct {
		World  *sim.World
		Bus    *sim.Bus
		Route  *sim.Route
		Driver DriverSnapshot
		Lights *sim.TrafficLightSystem
	}
	if err := json.Unmarshal(staleExitFixture, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	return ad, x.Bus, x.Route, x.World, x.Lights
}

func TestSavedSlot1StaleExitReleases25KPHCap(t *testing.T) {
	ad, b, route, w, tls := loadStaleExit(t)
	if ad.exitCruise == nil || ad.approachMerge == nil || ad.exitCruise.Junction.Distance(ad.approachMerge.Junction) < 40000 {
		t.Fatal("lost captured stale-junction condition")
	}
	health, passengers, index := b.Durability, b.PassengerCount, route.CurrentIndex
	ad.BeforeRecovery = func() { t.Fatal("unexpected reposition") }
	for tick := 0; tick < 180; tick++ {
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, nil, 52, 1./60)
		if tick == 0 && (ad.LastTargetSpeed < sim.SpeedKPH(80) || ad.exitCruise != nil) {
			t.Fatalf("old exit still caps clear lane: target %.2f kph", ad.LastTargetSpeed*.18)
		}
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.05 || b.Durability != health || b.PassengerCount != passengers || route.CurrentIndex != index {
			t.Fatal("journey state changed or vehicle repositioned")
		}
		for _, body := range b.BodyParts() {
			if body.HalfLen <= 0 {
				continue
			}
			for _, p := range sim.BodyCorners(body) {
				if p.Y < 0 || p.Y > 104 {
					t.Fatalf("left legal carriageway: %v", p)
				}
			}
		}
	}
	if b.Speed < sim.SpeedKPH(100) {
		t.Fatalf("failed to accelerate on clear road: %.2f kph", b.Speed*.18)
	}
	t.Logf("after 3 seconds: %.2f kph", b.Speed*.18)
}

func TestSupersededExitDoesNotDisableActualFrontObstacle(t *testing.T) {
	ad, b, route, w, tls := loadStaleExit(t)
	ob := VehicleObstacle{Pos: b.Pos.Add(sim.UnitFromAngle(b.Heading).Mul(b.Length/2 + 24 + 3)), Heading: b.Heading, Length: 48, Width: 22}
	_, br, _, _, _, _ := ad.ComputeControls(b, route, w, tls, []VehicleObstacle{ob}, 52, 1./60)
	if ad.LastTargetSpeed > b.Speed || br == 0 {
		t.Fatalf("front obstacle was ignored: target %v brake %v", ad.LastTargetSpeed, br)
	}
}

func TestCurrentJunctionExitStillRequiresPhysicalSettling(t *testing.T) {
	for _, mode := range []string{"approach", "right", "bus lane"} {
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		center := sim.Vec2{X: 500}
		ad.exitCruise = &sim.BusLaneMergeRequest{Junction: center, Target: sim.Vec2{Y: 78}, Joined: true}
		switch mode {
		case "approach":
			ad.approachMerge = &sim.BusLaneMergeRequest{Junction: center}
		case "right":
			ad.rightTurnJunction = &center
		case "bus lane":
			ad.busLaneJunction = &center
		}
		b := sim.NewBus(800, 26, 0)
		f := autopilotFrame{bus: b, laneWidth: 52, remaining: 4000}
		ad.retireSupersededExitCruise()
		if ad.exitCruise == nil || math.IsInf(ad.exitCruiseSpeed(f), 1) {
			t.Fatalf("%s lost current turn protection", mode)
		}
		later := sim.Vec2{X: 2000}
		switch mode {
		case "approach":
			ad.approachMerge.Junction = later
		case "right":
			ad.rightTurnJunction = &later
		case "bus lane":
			ad.busLaneJunction = &later
		}
		if !math.IsInf(ad.exitCruiseSpeed(f), 1) {
			t.Fatalf("%s obsolete turn still caps speed", mode)
		}
		ad.retireSupersededExitCruise()
		if ad.exitCruise != nil {
			t.Fatalf("%s retained obsolete lane intent", mode)
		}
	}
}
