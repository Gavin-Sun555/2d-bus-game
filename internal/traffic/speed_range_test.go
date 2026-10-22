package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestSpawnSpeedRangesAndStableAssignments(t *testing.T) {
	sim.SeedSimulation(87)
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	tm := &TrafficManager{CarSpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 100}}
	tm.BuildForWorld(w, 288, 52)
	seen := map[int]bool{}
	original := make(map[*sim.TrafficCar]int)
	for _, c := range tm.Cars {
		if c.MaxSpeedKPH < 30 || c.MaxSpeedKPH > 100 || c.MaxSpeedKPH%10 != 0 || c.CanOvertake != (c.MaxSpeedKPH > 30) || c.TargetSpeed != sim.SpeedKPH(c.MaxSpeedKPH) {
			t.Fatalf("bad spawn speed: %+v", c)
		}
		seen[c.MaxSpeedKPH] = true
		original[c] = c.MaxSpeedKPH
	}
	if len(seen) != 8 {
		t.Fatalf("missing speed bins: %v", seen)
	}
	tm.CarSpeedRange = sim.SpeedRange{MinKPH: 70, MaxKPH: 80}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	for tick := 0; tick < 20; tick++ {
		tm.Update(1./60, nil, tls, 4, 52, w)
	}
	for c, speed := range original {
		if c.MaxSpeedKPH != speed || c.TargetSpeed != sim.SpeedKPH(speed) {
			t.Fatal("existing speed changed after settings edit")
		}
	}
	replacement := tm.spawnAtOtherIntersection(sim.Vec2{X: 1e6}, nil, 52, w)
	if replacement == nil {
		t.Fatal("no boundary replacement")
	}
	if replacement.MaxSpeedKPH != 70 && replacement.MaxSpeedKPH != 80 {
		t.Fatal("replacement ignored new range")
	}
	for _, rangeKPH := range []sim.SpeedRange{{MinKPH: 30, MaxKPH: 50}, {MinKPH: 60, MaxKPH: 80}, {MinKPH: 40, MaxKPH: 40}} {
		fleet := NewAIBusManager()
		fleet.BusSpeedRange = rangeKPH
		fleet.BuildFleet(w, 8, 52)
		for _, ab := range fleet.Buses {
			speed := ab.MaxSpeedKPH
			if !ab.Driver.DisallowOncomingOvertake || speed < rangeKPH.MinKPH || speed > rangeKPH.MaxKPH || speed%10 != 0 || ab.CanOvertake != (speed > rangeKPH.MinKPH) || ab.Driver.BerserkMode != ab.CanOvertake || math.Abs(ab.Bus.MaxSpeedFwd-sim.SpeedKPH(speed)) > .01 || ab.Driver.CruiseSpeedLimit != sim.SpeedKPH(speed) {
				t.Fatalf("invalid bus profile: %+v", ab)
			}
			ab.ResetToStartingStation(w)
			if !ab.Driver.DisallowOncomingOvertake || ab.MaxSpeedKPH != speed || ab.Driver.CruiseSpeedLimit != sim.SpeedKPH(speed) {
				t.Fatal("reset rerolled bus speed")
			}
		}
		// Even a replacement controller must receive the fleet-only policy.
		fleet.Buses[0].Driver.DisallowOncomingOvertake = false
		fleet.Update(1./60, nil, w, tls, nil, 52)
		if !fleet.Buses[0].Driver.DisallowOncomingOvertake {
			t.Fatal("fleet update lost passing restriction")
		}

	}
}
