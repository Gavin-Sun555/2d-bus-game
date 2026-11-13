package autopilot

import (
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/station-entry-braking-margin-2026-10-29.json
var entryBrakingMarginFixture []byte

//go:embed testdata/station-sharp-moving-entry-2026-10-29.json
var sharpMovingEntryFixture []byte

func TestSavedMovingArticulatedEntryBrakesBeforeShelter(t *testing.T) {
	var s rearDepartureFixture
	if err := json.Unmarshal(sharpMovingEntryFixture, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	if s.Bus.Speed < 100 {
		t.Fatal("fixture must be moving into a sharp bay entry")
	}
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, doors, docked := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, nil, 52, 1./60)
		s.Bus.Update(1./60, th, br, st, hb, doors)
		if s.Bus.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("sharp entry reached shelter at %.2fs", float64(tick+1)/60)
		}
		if docked && s.Bus.DoorsOpen {
			return
		}
	}
	t.Fatal("safe sharp entry did not complete boarding arrival")
}

func loadEntryMargin(t *testing.T) (rearDepartureFixture, *AutopilotDriver) {
	t.Helper()
	var s rearDepartureFixture
	if err := json.Unmarshal(entryBrakingMarginFixture, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	return s, ad
}

func TestSavedStationEntryBrakingMarginReachesBoardingArea(t *testing.T) {
	s, ad := loadEntryMargin(t)
	stop := s.World.Stops[s.Route.GetCurrentTargetStopID()]
	if stop.ContainsBus(s.Bus) || ad.stationDock == nil || !ad.stationDock.alignmentRecovery {
		t.Fatal("fixture must stop just outside boarding area on an alignment arc")
	}
	start := s.Bus.Pos
	var obs []VehicleObstacle
	for _, b := range s.OtherBuses {
		obs = append(obs, ObserveBus(b))
	}
	for tick := 0; tick < 1800; tick++ {
		before := s.Bus.Pos
		th, br, st, hb, doors, docked := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, obs, 52, 1./60)
		s.Bus.Update(1./60, th, br, st, hb, doors)
		if before.Distance(s.Bus.Pos) > s.Bus.MaxSpeedFwd/60+1 {
			t.Fatal("entry recovery relocated vehicle")
		}
		if s.Bus.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("entry recovery crossed hard boundary")
		}
		for _, ob := range obs {
			for _, body := range predictedObstacleBodies(ob, 0) {
				if body.HalfLen > 0 {
					if hit, _, _ := s.Bus.CollidesBody(body); hit {
						t.Fatal("entry recovery hit traffic")
					}
				}
			}
		}
		if tick == 60 {
			raw, _ := json.Marshal(ad.Snapshot())
			var saved DriverSnapshot
			if err := json.Unmarshal(raw, &saved); err != nil {
				t.Fatal(err)
			}
			var err error
			ad, err = saved.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
		}
		if docked && s.Bus.DoorsOpen {
			if !stop.ContainsBus(s.Bus) || s.Bus.Pos.Distance(start) < 20 {
				t.Fatal("opened before reaching safe boarding pose")
			}
			t.Logf("saved entry reached boarding area %.2fs, progress%.1f", float64(tick+1)/60, s.Bus.Pos.Distance(start))
			return
		}
	}
	t.Fatalf("entry remained stuck: pos%v speed%g heading%g", s.Bus.Pos, s.Bus.Speed, s.Bus.Heading)
}

func TestEntryMarginRecoveryWaitsForRealFrontObstruction(t *testing.T) {
	s, ad := loadEntryMargin(t)
	u := sim.UnitFromAngle(s.World.Stops[232].Heading)
	ob := VehicleObstacle{Pos: s.Bus.Pos.Add(u.Mul(85)), Heading: s.World.Stops[232].Heading, Length: 46, Width: 22}
	start := s.Bus.Pos
	for tick := 0; tick < 120; tick++ {
		th, br, st, hb, doors, docked := ad.ComputeControls(s.Bus, s.Route, s.World, s.Lights, []VehicleObstacle{ob}, 52, 1./60)
		s.Bus.Update(1./60, th, br, st, hb, doors)
		if doors || docked {
			t.Fatal("blocked entry opened doors")
		}
		if math.Abs(s.Bus.Pos.Sub(start).Dot(u)) > 2 {
			t.Fatal("entry recovery ignored frontal traffic")
		}
	}
}

func TestStationDockRecoveryDoesNotReplanWithinTick(t *testing.T) {
	const laneWidth = 52.
	const heading = -math.Pi / 2
	stop := sim.NewBusStop(0, "test", sim.Vec2{}, heading, 8, sim.RGBA(0, 0, 0, 255), laneWidth, 10)
	world := &sim.World{Stops: []*sim.BusStop{stop}, Roads: []sim.RoadSegment{
		{P1: sim.Vec2{Y: -1000}, P2: sim.Vec2{Y: 1000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
	}}
	bus := sim.NewBus(220.783445, 310.958255, -1.2779578)
	bus.Speed = 7.829013
	bus.SteeringAngle = -.0368402
	obstacles := []VehicleObstacle{
		{Pos: sim.Vec2{X: 217.842589, Y: -50.902967}, Heading: -1.8010245, Speed: 47.840255, Length: 96, Width: 36},
		{Pos: sim.Vec2{X: 182.001662, Y: -887.739242}, Heading: -1.5709088, Speed: 443.852778, Length: 96, Width: 36},
	}
	driver := NewAutopilotDriver()
	driver.DisallowOncomingOvertake = true
	rear := sim.Vec2{X: stop.Pos.X, Y: 71.938116}
	driver.stationDock = &stationDockPlan{stopID: stop.ID, target: rear, points: stationDockCurve(bus, rear, sim.UnitFromAngle(heading))}
	// The signal holds the bus even though the alignment curve is clear.
	signal := &sim.TrafficIntersection{Phase: sim.PhaseEWGreen, ApproachLanes: [4]int{4, 4, 8, 8}}
	bumper := bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Length / 2))
	signal.Center = sim.Vec2{X: 40, Y: bumper.Y - signal.GetStopLineDistForHeading(heading, laneWidth) - 5}
	lights := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{signal}}
	frame := autopilotFrame{bus: bus, world: world, tls: lights, targetStop: stop, fromStop: stop, obstacles: obstacles, laneWidth: laneWidth, speedAbs: bus.Speed, remaining: 238.37}
	inputs, active := driver.stationDockControls(frame)
	if !active || driver.stationDock == nil || !driver.stationDock.alignmentRecovery || inputs.brake != 1 || inputs.toggleDoors || inputs.isDocked {
		t.Fatalf("expected an alignment plan while stopped at red, got %+v", inputs)
	}
	plan := driver.stationDock
	for tick := 0; tick < 120; tick++ {
		inputs, active = driver.stationDockControls(frame)
		if !active || driver.stationDock != plan || inputs.brake != 1 || inputs.toggleDoors || inputs.isDocked {
			t.Fatal("red light discarded the checked recovery plan")
		}
	}
	signal.Phase = sim.PhaseNSGreen
	inputs, active = driver.stationDockControls(frame)
	if !active || driver.stationDock != plan || inputs.throttle <= 0 {
		t.Fatal("bus did not continue on its checked recovery plan after green")
	}
}
