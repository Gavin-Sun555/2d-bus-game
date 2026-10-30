package traffic

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

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
