package autopilot

import (
	"bytes"
	"compress/gzip"
	"embed"
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"io"
	"math"
	"testing"
)

//go:embed testdata/stopped-slot*-2026-10-29.json.gz
var stoppedSlotScenes embed.FS

type stoppedSlotScene struct {
	Bus        *sim.Bus
	Driver     DriverSnapshot
	Route      *sim.Route
	World      *sim.World
	Lights     *sim.TrafficLightSystem
	OtherBuses []*sim.Bus
	Cars       []*sim.TrafficCar
}

func loadStoppedSlot(t *testing.T, slot int) (*AutopilotDriver, autopilotFrame, *stoppedSlotScene) {
	t.Helper()
	raw, err := stoppedSlotScenes.ReadFile(fmt.Sprintf("testdata/stopped-slot%d-2026-10-29.json.gz", slot))
	if err != nil {
		t.Fatal(err)
	}
	gz, err := gzip.NewReader(bytes.NewReader(raw))
	if err != nil {
		t.Fatal(err)
	}
	data, err := io.ReadAll(gz)
	gz.Close()
	if err != nil {
		t.Fatal(err)
	}
	var scene stoppedSlotScene
	if err := json.Unmarshal(data, &scene); err != nil {
		t.Fatal(err)
	}
	ad, err := scene.Driver.Restore(scene.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: scene.Bus, world: scene.World, tls: scene.Lights, laneWidth: 52, targetStop: scene.World.Stops[scene.Route.GetCurrentTargetStopID()], s: ad.LastPathS, remaining: ad.LastRemaining, speedAbs: math.Abs(scene.Bus.Speed)}
	for _, b := range scene.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	for _, c := range scene.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Length: c.Length, Width: c.Width, Speed: c.Speed, JunctionCorridor: c.JunctionCorridor, JunctionTurn: c.JunctionTurn})
	}
	return ad, f, &scene
}

func TestSavedShelterDeparturePresteersAndKeepsTailClear(t *testing.T) {
	ad, f, s := loadStoppedSlot(t, 1)
	steer, _, _, _ := ad.pursuitSteering(f)
	if _, clear := ad.stationDepartureEscape(f, steer); clear || !ad.departurePresteerValid {
		t.Fatal("shelter-edge departure did not request stationary steering")
	}
	start := s.Bus.Pos
	restored := false
	for tick := 0; tick < 1200; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, f.obstacles, 52, 1./60)
		s.Bus.Update(1./60, th, br, st, hb, doors)
		if boundary := s.Bus.CheckBoundaries(s.World, 52); boundary.Zone == sim.ZoneHardBarrier {
			t.Fatalf("shelter contact at tick%d: %+v", tick, boundary)
		}
		for _, other := range s.OtherBuses {
			if hit, _, depth := sim.CheckBusesCollision(s.Bus, other); hit && depth > 1e-6 {
				t.Fatalf("rear-bus contact tick%d", tick)
			}
		}
		if !restored && ad.departureEscapeActive && s.Bus.Pos.Distance(start) > 20 {
			snapshot := ad.Snapshot()
			raw, _ := json.Marshal(snapshot)
			var decoded DriverSnapshot
			json.Unmarshal(raw, &decoded)
			var err error
			ad, err = decoded.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
			if !ad.departureEscapeActive {
				t.Fatal("save lost protective departure state")
			}
			restored = true
		}
		if s.Bus.Pos.Distance(start) > 250 {
			if !restored || ad.departureEscapeActive {
				t.Fatal("departure failed to leave recovery")
			}
			t.Logf("departure cleared in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("saved departure remained blocked")
}

func TestSavedPassedJunctionReleasesOldMerge(t *testing.T) {
	ad, f, _ := loadStoppedSlot(t, 3)
	if ad.busLaneJunction == nil {
		t.Fatal("fixture lost stale junction")
	}
	old := *ad.busLaneJunction
	// Current sparse segment starts beside the old junction, but its travelled
	// portion must not count as a future return to that junction.
	if d, _ := sim.DistToSegment(old, ad.drivePath[ad.WaypointIdx], ad.drivePath[ad.WaypointIdx+1]); d > 1000 {
		t.Fatal("fixture no longer covers sparse segment")
	}
	ad.clearPassedBusLaneJunction(f)
	if ad.busLaneJunction != nil || ad.busTurnHoldDistance != 0 {
		t.Fatal("old junction remained reserved")
	}
	for _, name := range []string{"upcoming", "entered", "recent exit", "future return"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadStoppedSlot(t, 3)
			switch name {
			case "upcoming":
				p := f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(100))
				ad.busLaneJunction = &p
			case "entered":
				ad.busJunctionEntered = true
			case "recent exit":
				g := f.world.JunctionAt(*ad.busLaneJunction, 52)
				reach := math.Max(g.HalfX, g.HalfY) + f.bus.OverallLength() + 100
				f.bus.Pos = ad.busLaneJunction.Add(sim.UnitFromAngle(approachRoadHeading(f.bus.Heading)).Mul(1.5 * reach))
			case "future return":
				ad.drivePath = append(ad.drivePath, *ad.busLaneJunction)
				ad.cumLen = append(ad.cumLen, ad.cumLen[len(ad.cumLen)-1]+ad.drivePath[len(ad.drivePath)-1].Distance(ad.drivePath[len(ad.drivePath)-2]))
			}
			ad.clearPassedBusLaneJunction(f)
			if ad.busLaneJunction == nil {
				t.Fatal("live junction reservation released")
			}
		})
	}
}

func TestOrdinaryBayDepartureClearsStoppedRearMerger(t *testing.T) {
	ad, f, s := loadStoppedSlot(t, 3)
	s.Bus.Pos = sim.Vec2{X: -15828.94211098288, Y: 4488.191691169671}
	s.Bus.Heading = 1.5643736873870182
	s.Bus.Speed = 0
	s.Bus.SteeringAngle = 0
	rear := *s.Bus
	rear.Pos = sim.Vec2{X: -15786.956749976407, Y: 4389.395527470835}
	rear.Heading = 1.6464316493982982
	rear.SteeringAngle = 0
	s.Route.AdvanceToNextStop()
	ad.Reset()
	ad.State = AutopilotStateDeparting
	ad.departureFromDock = true
	obstacles := []VehicleObstacle{ObserveBus(&rear)}
	start := s.Bus.Pos
	usedRecovery := false
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, d, _ := ad.ComputeControls(s.Bus, s.Route, s.World, nil, obstacles, 52, 1./60)
		usedRecovery = usedRecovery || ad.departureEscapeActive
		s.Bus.Update(1./60, th, br, st, hb, d)
		if s.Bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("departure hit shelter tick%d", tick)
		}
		if hit, _, depth := sim.CheckBusesCollision(s.Bus, &rear); hit && depth > 1e-6 {
			t.Fatalf("departure hit rear merger tick%d", tick)
		}
		if s.Bus.Pos.Distance(start) > 300 {
			if !usedRecovery {
				t.Fatal("fixture did not exercise departure escape")
			}
			return
		}
	}
	t.Fatal("ordinary departure stayed blocked")
}
