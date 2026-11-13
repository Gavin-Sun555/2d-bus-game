package traffic

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestFleetEmptyStationPassKeepsCruiseAndServesRequiredStops(t *testing.T) {
	for _, scenario := range []struct {
		name               string
		waiting            []int
		alighting          int
		terminal, retiring bool
		replacing          bool
		lateWaiting        bool
		wantSkip           bool
	}{
		{name: "empty", wantSkip: true},
		// A passenger reaching the platform after the bus committed to the
		// pass waits for the next bus instead of forcing a late hard stop.
		{name: "passenger after skip decided", lateWaiting: true, wantSkip: true},
		{name: "waiting for another line", waiting: []int{0}, wantSkip: true},
		{name: "boarding", waiting: []int{2, 2}},
		{name: "alighting", alighting: 1},
		{name: "empty terminal", terminal: true, wantSkip: true},
		{name: "retiring at empty terminal", terminal: true, retiring: true},
		{name: "replacing at empty terminal", terminal: true, replacing: true},
	} {
		t.Run(scenario.name, func(t *testing.T) {
			const lw = 52.
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2200}, P2: sim.Vec2{X: 2400}, Lanes: 4}}}
			for i, x := range []float64{-1200, 0, 1200} {
				w.Stops = append(w.Stops, sim.NewBusStop(i, "stop", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw, 0))
			}
			stop := w.Stops[1]
			stop.SeedWaitingPassengers(len(scenario.waiting))
			stop.WaitingDestinations = scenario.waiting
			route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
			if scenario.terminal {
				route.StopIDs = route.StopIDs[:2]
			}
			bus := sim.NewBus(-580, 1.5*lw, 0)
			bus.Speed = 150
			bus.BoardPassengers(scenario.alighting, []int{1})
			if scenario.replacing {
				bus.RequestModel(sim.BusDoubleDecker)
			}
			ad := autopilot.NewAutopilotDriver()
			ab := &AIBus{ID: 1, Bus: bus, Route: route, Driver: ad, Retiring: scenario.retiring}
			fleet := NewAIBusManager()
			fleet.Buses = []*AIBus{ab}
			for tick := 0; tick < 900; tick++ {
				if scenario.lateWaiting && stop.WaitingPassengers == 0 && bus.Pos.X > -300 {
					stop.SeedWaitingPassengers(1)
					stop.WaitingDestinations = []int{2}
				}
				fleet.Update(1./60, nil, w, nil, nil, lw)
				if scenario.wantSkip && (bus.DoorsOpen || bus.Speed < 145) {
					t.Fatalf("empty station interrupted cruise at %v, speed %.1f", bus.Pos, bus.Speed)
				}
				if !scenario.wantSkip && bus.DoorsOpen {
					if scenario.retiring && len(fleet.Buses) != 0 {
						t.Fatal("retiring vehicle did not leave service at the terminal")
					}
					if scenario.replacing && bus.Model != sim.BusDoubleDecker {
						t.Fatal("scheduled replacement did not complete")
					}
					return
				}
				if route.CurrentIndex != 1 {
					if !scenario.wantSkip || bus.Pos.X < -3 || (scenario.terminal && route.TotalTrips != 1) {
						t.Fatalf("invalid station advancement at %v, trip %d", bus.Pos, route.TotalTrips)
					}
					return
				}
			}
			t.Fatalf("vehicle did not reach or serve the station: %v, speed %.1f", bus.Pos, bus.Speed)
		})
	}
}

func TestPassedTerminalBacksClearAndResumesService(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -1800}, P2: sim.Vec2{X: 1800}, Lanes: 4},
	}}
	for id, x := range []float64{0, 1200, -1200} {
		w.Stops = append(w.Stops, sim.NewBusStop(id, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw))
	}
	route := &sim.Route{StopIDs: []int{0, 1, 2, 0}, CurrentIndex: 3}
	bus := sim.NewBus(152.778, 98.188, .92083)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = .14605
	bus.SteeringAngle = bus.MaxSteerAngle
	ad := autopilot.NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[2], w.Stops[0], lw)
	ad.State, ad.LastTargetID, ad.LastRemaining, ad.LastLateralErr = autopilot.AutopilotStateApproaching, 0, 0, 62.8
	ab := &AIBus{Bus: bus, Route: route, Driver: ad}
	start := bus.Pos
	if !ab.recoverPassedStation(w, lw, 1./60, true) || route.TotalTrips != 1 || bus.Pos != start {
		t.Fatal("passed terminal did not advance from its actual pose")
	}
	reversed := false
	for tick := 0; tick < 1200; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1./60)
		before := bus.Pos
		bus.Update(1./60, th, br, st, hb, doors)
		reversed = reversed || bus.Speed < 0
		if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.01 || bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
			t.Fatalf("unsafe terminal recovery at tick %d, position %v", tick, bus.Pos)
		}
		if reversed && bus.Pos.X > start.X+160 {
			return
		}
	}
	t.Fatalf("missed berth blocked next leg at %v", bus.Pos)
}

//go:embed testdata/passed-station-slot3-2026-10-29.json.gz
var passedStationScene []byte

func loadPassedStation(t *testing.T) (*AIBus, *sim.World) {
	t.Helper()
	var s struct {
		Bus    *sim.Bus
		Driver autopilot.DriverSnapshot
		Route  *sim.Route
		World  *sim.World
	}
	gz, err := gzip.NewReader(bytes.NewReader(passedStationScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	if err = json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	return &AIBus{Bus: s.Bus, Route: s.Route, Driver: ad}, s.World
}

func TestSavedPassedStationContinuesFromActualPose(t *testing.T) {
	ab, w := loadPassedStation(t)
	pos, health := ab.Bus.Pos, ab.Bus.Durability
	old := ab.Route.GetCurrentTargetStopID()
	count := len(ab.Bus.Passengers)
	if !ab.recoverPassedStation(w, 52, 1./60, true) {
		t.Fatal("exhausted past-platform arrival remained stuck")
	}
	if ab.Route.GetCurrentTargetStopID() == old || ab.Bus.Pos != pos || ab.Bus.Durability != health || len(ab.Bus.Passengers) != count {
		t.Fatal("recovery changed physical bus or lost passengers")
	}
	for _, p := range ab.Bus.Passengers {
		if p.TargetStopID == old {
			t.Fatal("missed-stop passenger not reassigned")
		}
	}
	for tick := 0; tick < 600; tick++ {
		th, br, st, hb, d, _ := ab.Driver.ComputeControls(ab.Bus, ab.Route, w, nil, nil, 52, 1./60)
		ab.Bus.Update(1./60, th, br, st, hb, d)
		if ab.Bus.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("departure hit boundary at %d", tick)
		}
		if ab.Bus.Pos.Distance(pos) > 200 {
			return
		}
	}
	t.Fatal("recovery did not resume driving")
}

func TestPassedStationDoesNotSkipValidArrivals(t *testing.T) {
	for _, name := range []string{"approaching", "rear berth", "doors", "service", "turning", "other road", "future path", "docked", "departing"} {
		t.Run(name, func(t *testing.T) {
			ab, w := loadPassedStation(t)
			stop := w.Stops[ab.Route.GetCurrentTargetStopID()]
			switch name {
			case "approaching":
				ab.Bus.Pos = stop.Pos.Sub(sim.UnitFromAngle(stop.Heading).Mul(300))
			case "rear berth":
				ab.Bus.Pos = stop.RearBerthPosition()
			case "doors":
				ab.Bus.DoorsOpen = true
			case "service":
				ab.Bus.AddPassengerService(1, 1)
			case "turning":
				ab.Bus.Heading += .4
			case "other road":
				ab.Bus.Pos.X += 200
			case "future path":
				ab.Driver.LastRemaining = 500
			case "docked":
				ab.Driver.State = autopilot.AutopilotStateDocked
			case "departing":
				ab.Driver.State = autopilot.AutopilotStateDeparting
			}
			if ab.recoverPassedStation(w, 52, 1./60, true) {
				t.Fatal("valid station approach was skipped")
			}
		})
	}
}

//go:embed testdata/late-entry-slot1-2026-10-30.json.gz
var lateStationScene []byte

func loadLateStation(t *testing.T) (*AIBus, *sim.World) {
	t.Helper()
	var s struct {
		Bus    *sim.Bus
		Driver autopilot.DriverSnapshot
		Route  *sim.Route
		World  *sim.World
	}
	gz, err := gzip.NewReader(bytes.NewReader(lateStationScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	if err = json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	return &AIBus{Bus: s.Bus, Route: s.Route, Driver: ad}, s.World
}
func TestSavedLateOuterArrivalReleasesAfterRetry(t *testing.T) {
	ab, w := loadLateStation(t)
	pos, target, count := ab.Bus.Pos, ab.Route.GetCurrentTargetStopID(), len(ab.Bus.Passengers)
	if ab.Driver.LastRemaining <= 2 {
		t.Fatal("fixture lost positive projected remainder")
	}
	for tick := 0; tick < 180; tick++ {
		if ab.recoverPassedStation(w, 52, 1./60, true) {
			t.Fatalf("skipped before retry finished tick%d", tick)
		}
	}
	if !ab.recoverPassedStation(w, 52, 1./60, true) {
		t.Fatal("late outside arrival remained blocked")
	}
	if ab.Bus.Pos != pos || len(ab.Bus.Passengers) != count || ab.Route.GetCurrentTargetStopID() == target {
		t.Fatal("recovery changed bus/passengers or kept old target")
	}
	for _, p := range ab.Bus.Passengers {
		if p.TargetStopID == target {
			t.Fatal("missed-stop passenger not reassigned")
		}
	}
}
func TestLateArrivalDoesNotSkipFeasibleStationWork(t *testing.T) {
	for _, name := range []string{"moving", "doors", "service", "inside bay", "before marker", "turning", "more path", "ready to drive", "departing", "docked"} {
		t.Run(name, func(t *testing.T) {
			ab, w := loadLateStation(t)
			stop := w.Stops[ab.Route.GetCurrentTargetStopID()]
			switch name {
			case "moving":
				ab.Bus.Speed = 1
			case "doors":
				ab.Bus.DoorsOpen = true
			case "service":
				ab.Bus.AddPassengerService(1, 1)
			case "inside bay":
				ab.Bus.Pos = stop.RearBerthPosition()
				ab.Bus.Heading = stop.Heading
				ab.Bus.ResetArticulation()
			case "before marker":
				ab.Bus.Pos = stop.FrontBerthPosition().Sub(sim.UnitFromAngle(stop.Heading).Mul(20))
			case "turning":
				ab.Bus.Heading += .5
			case "more path":
				ab.Driver.LastRemaining = 500
			case "ready to drive":
				ab.Driver.LastTargetSpeed = 50
			case "departing":
				ab.Driver.State = autopilot.AutopilotStateDeparting
			case "docked":
				ab.Driver.State = autopilot.AutopilotStateDocked
			}
			for tick := 0; tick < 240; tick++ {
				if ab.recoverPassedStation(w, 52, 1./60, name != "ready to drive") {
					t.Fatal("valid work was skipped")
				}
			}
		})
	}
	ab, w := loadLateStation(t)
	for tick := 0; tick < 150; tick++ {
		ab.recoverPassedStation(w, 52, 1./60, true)
	}
	ab.Bus.Speed = 5
	ab.recoverPassedStation(w, 52, 1./60, true)
	ab.Bus.Speed = 0
	for tick := 0; tick < 150; tick++ {
		if ab.recoverPassedStation(w, 52, 1./60, true) {
			t.Fatal("old stationary wait survived renewed progress")
		}
	}
}

//go:embed testdata/yawed-past-berth-2026-10-30.json.gz
var yawedStationScene []byte

func loadYawedStation(t *testing.T) (*AIBus, *sim.World) {
	t.Helper()
	var s struct {
		Bus    *sim.Bus
		Driver autopilot.DriverSnapshot
		Route  *sim.Route
		World  *sim.World
	}
	gz, err := gzip.NewReader(bytes.NewReader(yawedStationScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	if err = json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	return &AIBus{Bus: s.Bus, Route: s.Route, Driver: ad}, s.World
}
func TestYawedPastTaperReleasesWithoutMovingBus(t *testing.T) {
	ab, w := loadYawedStation(t)
	pos, count, old := ab.Bus.Pos, len(ab.Bus.Passengers), ab.Route.GetCurrentTargetStopID()
	if !ab.recoverPassedStation(w, 52, 1./60, true) {
		t.Fatal("exhausted yawed arrival remained blocked")
	}
	if ab.Bus.Pos != pos || len(ab.Bus.Passengers) != count || ab.Route.GetCurrentTargetStopID() == old {
		t.Fatal("recovery altered physical bus or failed to advance")
	}
	if !ab.Driver.Snapshot().DepartureEscapeActive {
		t.Fatal("first-tick safe crawl not armed")
	}
	for _, p := range ab.Bus.Passengers {
		if p.TargetStopID == old {
			t.Fatal("passenger left targeting missed stop")
		}
	}
}

func TestPassedBerthInParallelTravelLaneRecovers(t *testing.T) {
	const laneWidth = 52.
	for _, side := range []float64{47, 101, 130, -130, 400} {
		stop := &sim.BusStop{ID: 1, Pos: sim.Vec2{Y: 177}, Length: 336, Width: 44, RoadLanes: 6}
		world := &sim.World{
			Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: 6}},
			Stops: []*sim.BusStop{{ID: 0}, stop, {ID: 2}},
		}
		bus := sim.NewBus(215, side, -.21)
		bus.SetModel(sim.BusArticulated)
		bus.TrailerHeading = -.06
		snapshot := autopilot.NewAutopilotDriver().Snapshot()
		snapshot.Public.State = autopilot.AutopilotStateApproaching
		snapshot.Public.LastTargetID = stop.ID
		snapshot.PathFromID, snapshot.PathToID = 0, stop.ID
		snapshot.DrivePath = []sim.Vec2{{X: -500, Y: stop.Pos.Y}, stop.FrontBerthPosition()}
		driver, err := snapshot.Restore(world)
		if err != nil {
			t.Fatal(err)
		}
		route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
		ab := &AIBus{Bus: bus, Route: route, Driver: driver}
		bus.BoardPassengers(1, []int{1})
		pos := bus.Pos
		want := side >= 0 && side <= 130
		if got := ab.recoverPassedStation(world, laneWidth, 1./60, true); got != want {
			t.Fatalf("lateral %.0f: recovered=%v, want %v", side, got, want)
		}
		if want && (bus.Pos != pos || route.GetCurrentTargetStopID() != 2 || len(bus.Passengers) != 1 || bus.Passengers[0].TargetStopID != 2) {
			t.Fatalf("lateral %.0f: recovery moved the bus or lost its passenger", side)
		}
	}
}

func TestYawedPastTaperRetainsTurnAndServiceGuards(t *testing.T) {
	for _, name := range []string{"moving", "doors", "service", "unfinished path", "tail turning", "too far", "before berth", "wrong road", "junction", "roundabout", "not held"} {
		t.Run(name, func(t *testing.T) {
			ab, w := loadYawedStation(t)
			stop := w.Stops[ab.Route.GetCurrentTargetStopID()]
			switch name {
			case "moving":
				ab.Bus.Speed = 5
			case "doors":
				ab.Bus.DoorsOpen = true
			case "service":
				ab.Bus.AddPassengerService(1, 1)
			case "unfinished path":
				ab.Driver.LastRemaining = 300
			case "tail turning":
				ab.Bus.TrailerHeading += .4
			case "too far":
				ab.Bus.Pos.Y -= 500
			case "before berth":
				ab.Bus.Pos = ab.Bus.Pos.Add(sim.UnitFromAngle(stop.Heading).Mul(-150))
			case "wrong road":
				ab.Bus.Pos.X -= 1000
			case "junction":
				w.Intersections = append(w.Intersections, ab.Bus.Pos)
			case "roundabout":
				w.Roundabouts = append(w.Roundabouts, sim.Roundabout{Center: ab.Bus.Pos, Radius: 100})
			}
			if ab.recoverPassedStation(w, 52, 1./60, name != "not held") {
				t.Fatal("live turn/service skipped")
			}
		})
	}
}

func TestWideLateArrivalWaitsThenPreservesPassengersOnDeparture(t *testing.T) {
	const lw = 52.
	for _, scenario := range []string{"stranded", "moving", "earlier approach", "valid taper", "opposite side"} {
		t.Run(scenario, func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 4000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
			for id, x := range []float64{-2000, 0, 2000} {
				w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", sim.Vec2{X: x}, 0, 8, sim.RGBA(0, 0, 0, 255), lw))
			}
			stop := w.Stops[1]
			bus := sim.NewBus(stop.FrontBerthPosition().X-3, 130, 0)
			bus.SetModel(sim.BusArticulated)
			bus.BoardPassengers(2, []int{1, 2})
			ad := autopilot.NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], stop, lw)
			ad.State = autopilot.AutopilotStateApproaching
			ad.LastTargetID, ad.LastRemaining = 1, 51
			switch scenario {
			case "moving":
				bus.Speed = 5
			case "earlier approach":
				bus.Pos.X -= 100
			case "valid taper":
				bus.Pos.Y = stop.Pos.Y - 25
			case "opposite side":
				bus.Pos.Y = -130
			}
			route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
			ab := &AIBus{Bus: bus, Route: route, Driver: ad}
			start := bus.Pos
			for tick := 0; tick < 179; tick++ {
				if ab.recoverPassedStation(w, lw, 1./60, true) {
					t.Fatal("abandoned berth before the recovery window")
				}
			}
			recovered := false
			for tick := 0; tick < 3; tick++ {
				recovered = ab.recoverPassedStation(w, lw, 1./60, true) || recovered
			}
			if recovered != (scenario == "stranded") {
				t.Fatalf("unexpected recovery %v", recovered)
			}
			if bus.Pos != start || bus.PassengerCount != 2 || len(bus.Passengers) != 2 {
				t.Fatal("recovery changed pose or lost passengers")
			}
			if recovered && (route.GetCurrentTargetStopID() != 2 || bus.Passengers[0].TargetStopID != 2) {
				t.Fatal("missed-stop passenger did not continue to next stop")
			}
		})
	}
}

func TestYawedPastTaperIncludesSixtyDegreeLimit(t *testing.T) {
	for _, alignment := range []float64{.57, .536, .49} {
		ab, w := loadYawedStation(t)
		old := ab.Route.GetCurrentTargetStopID()
		ab.Bus.Heading = w.Stops[old].Heading + math.Acos(alignment)
		pos, count := ab.Bus.Pos, ab.Bus.PassengerCount
		recovered := ab.recoverPassedStation(w, 52, 1./60, true)
		if recovered != (alignment > .5) {
			t.Fatalf("alignment %.3f: recovered %v", alignment, recovered)
		}
		if ab.Bus.Pos != pos || ab.Bus.PassengerCount != count {
			t.Fatal("missed taper recovery moved bus or lost passengers")
		}
	}
}

// Bus10's 104px endpoint error was mostly longitudinal, not a lane error.
// Its nose had passed by 103px: beyond the powered section but within the
// full articulated vehicle. Neither is a reason to exclude missed-berth recovery.
func TestArticulatedPassedBerthUsesFullLengthAndLateralProjection(t *testing.T) {
	for _, along := range []float64{80, 103.4, 180, 220} {
		ab, w := loadYawedStation(t)
		stop := w.Stops[ab.Route.GetCurrentTargetStopID()]
		u := sim.UnitFromAngle(stop.Heading)
		right := sim.Vec2{X: -u.Y, Y: u.X}
		ab.Bus.Pos = stop.FrontBerthPosition().Add(u.Mul(along)).Add(right.Mul(14))
		ab.Bus.Heading = stop.Heading + .611523008
		ab.Bus.TrailerHeading = stop.Heading + .121299632
		ab.Driver.LastLateralErr = math.Hypot(along, 14)
		ab.Driver.LastRemaining = 0
		start, count, old := ab.Bus.Pos, ab.Bus.PassengerCount, ab.Route.GetCurrentTargetStopID()
		want := along <= ab.Bus.OverallLength()
		if got := ab.recoverPassedStation(w, 52, 1./60, true); got != want {
			t.Fatalf("passed distance %.1f endpoint error %.1f: recovered %v want %v", along, ab.Driver.LastLateralErr, got, want)
		}
		if ab.Bus.Pos != start || ab.Bus.PassengerCount != count {
			t.Fatal("missed-berth recovery moved bus or lost passengers")
		}
		if want && (ab.Route.GetCurrentTargetStopID() == old || !ab.Driver.Snapshot().DepartureEscapeActive) {
			t.Fatal("missed-berth crawl was not armed")
		}
	}
}
