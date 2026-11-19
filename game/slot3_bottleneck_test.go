package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"os"
	"reflect"
	"strconv"
	"testing"
)

// Replay a captured bottleneck without refreshing any recovery timers. A reset
// of its blocking car is a failure even if it would free the observed bus.
func TestSlot3CapturedBottleneckReleases(t *testing.T) {
	p := os.Getenv("BUS_SLOT3_CAPTURE")
	if p == "" {
		t.Skip("BUS_SLOT3_CAPTURE not set")
	}
	busID, err := strconv.Atoi(os.Getenv("BUS_SLOT3_BUS"))
	if err != nil {
		t.Fatal(err)
	}
	carID := -1
	if raw := os.Getenv("BUS_SLOT3_CAR"); raw != "" {
		carID, err = strconv.Atoi(raw)
		if err != nil {
			t.Fatal(err)
		}
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, p, 1280, 720)
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	if p := os.Getenv("BUS_SLOT3_NORMALIZED"); p != "" {
		g.savePathOverride = p
		if err := g.SaveGame(); err != nil {
			t.Fatal(err)
		}
	}
	var a *traffic.AIBus
	if busID == 0 {
		a = &traffic.AIBus{Bus: g.Bus, Driver: g.AutopilotDriver, Route: g.Route}
	} else {
		a = fleetBus(t, g, busID)
	}
	start := a.Bus.Pos
	health := a.Bus.Durability
	a.Driver.BeforeRecovery = func() { t.Fatal("bus replanned or relocated") }
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == busID {
			t.Fatal("bus reset", e)
		}
	}
	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		if e.ID == carID {
			t.Fatal("blocking car reset", e)
		}
	}
	var car *sim.TrafficCar
	for _, c := range g.TrafficMgr.Cars {
		if c.ID == carID {
			car = c
		}
	}
	if carID >= 0 && car == nil {
		t.Fatal("car missing")
	}
	served := false
	clearDistance := 1000.
	if raw := os.Getenv("BUS_SLOT3_DISTANCE"); raw != "" {
		clearDistance, err = strconv.ParseFloat(raw, 64)
		if err != nil || clearDistance <= 0 {
			t.Fatal("invalid BUS_SLOT3_DISTANCE")
		}
	}
	maxTicks := 90 * 60
	if raw := os.Getenv("BUS_SLOT3_TICKS"); raw != "" {
		maxTicks, err = strconv.Atoi(raw)
		if err != nil || maxTicks <= 0 {
			t.Fatal("invalid BUS_SLOT3_TICKS")
		}
	}
	traceEvery := 0
	switch os.Getenv("BUS_SLOT3_TRACE") {
	case "1":
		traceEvery = 60
	case "short":
		traceEvery = 6
	}
	for tick := 0; tick < maxTicks; tick++ {
		before := a.Bus.Pos
		var cp sim.Vec2
		if car != nil {
			cp = car.Pos
		}
		if err = g.Update(); err != nil {
			t.Fatal(err)
		}
		if traceEvery > 0 && tick%traceEvery == 0 {
			value := reflect.ValueOf(a.Driver).Elem()
			barrier := value.FieldByName("barrierReverse")
			headon := value.FieldByName("headonRecovery")
			var obs []autopilot.VehicleObstacle
			for _, other := range g.AIBusMgr.Buses {
				if other.Bus != nil && other.Bus != a.Bus {
					obs = append(obs, autopilot.ObserveBus(other.Bus))
				}
			}
			if g.Bus != nil && g.Bus != a.Bus {
				obs = append(obs, autopilot.ObserveBus(g.Bus))
			}
			for _, c := range g.TrafficMgr.Cars {
				if c != nil {
					obs = append(obs, autopilot.VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor, YieldingTo: c.YieldingToBus})
				}
			}
			diag := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, obs, 52)
			t.Logf("t%.1f pos%v h%.3f v%.2f wheel%.3f target%.1f offset%.1f/%.1f barrier%v headon%v roadHold%v limits%v blockers%v", float64(tick)/60, a.Bus.Pos, a.Bus.Heading, a.Bus.Speed, a.Bus.SteeringAngle, a.Driver.LastTargetSpeed, a.Driver.LaneOffset, a.Driver.TargetLaneOffset, !barrier.IsNil(), !headon.IsNil(), a.Driver.Snapshot().RoadSafetyHold, diag.Limits, diag.Blockers)
		}
		if a.Bus.Pos.Distance(before) > math.Abs(a.Bus.Speed)/60+.1 || a.Bus.Durability < health {
			if p := os.Getenv("BUS_SLOT3_FAILURE_OUT"); p != "" {
				g.savePathOverride = p
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			t.Fatalf("bus%d damaged or relocated at %.3fs: %v to %v speed%.3f health%.1f->%.1f car3=%v/%.2f", busID, float64(tick)/60, before, a.Bus.Pos, a.Bus.Speed, health, a.Bus.Durability, cp, func() float64 {
				if car != nil {
					return car.Speed
				}
				return -1
			}())
		}
		if car != nil && car.Pos.Distance(cp) > math.Abs(car.Speed)/60+.1 {
			if p := os.Getenv("BUS_SLOT3_FAILURE_OUT"); p != "" {
				g.savePathOverride = p
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			t.Fatalf("car%d relocated at %.3fs: %v to%v speed%.3f heading%.3f ring%v turn%+v", carID, float64(tick)/60, cp, car.Pos, car.Speed, car.Heading, car.InRoundabout, car.JunctionTurn)
		}
		served = served || a.Bus.DoorsOpen
		if a.Bus.Pos.Distance(start) > clearDistance {
			t.Logf("bus%d cleared after %.2fs, served%v", busID, float64(tick+1)/60, served)
			return
		}
	}
	t.Fatalf("bus%d stuck at%v", busID, a.Bus.Pos)
}
