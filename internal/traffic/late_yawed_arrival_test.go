package traffic

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/late-yawed-roadside-20261111.json.gz
var lateYawedRoadsideScene []byte

func loadLateYawedRoadside(t *testing.T) (*AIBus, *sim.World) {
	t.Helper()
	var s struct {
		World  *sim.World
		Bus    *sim.Bus
		Driver autopilot.DriverSnapshot
		Route  *sim.Route
	}
	gz, err := gzip.NewReader(bytes.NewReader(lateYawedRoadsideScene))
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
	return &AIBus{Bus: s.Bus, Driver: ad, Route: s.Route}, s.World
}

func TestLateYawedRoadsideArrivalRetriesThenContinues(t *testing.T) {
	ab, w := loadLateYawedRoadside(t)
	old := ab.Route.GetCurrentTargetStopID()
	start, count := ab.Bus.Pos, ab.Bus.PassengerCount
	if len(ab.Bus.Passengers) == 0 {
		t.Fatal("fixture needs passenger")
	}
	ab.Bus.Passengers[0].TargetStopID = old
	for tick := 0; tick < 180; tick++ {
		if ab.recoverPassedStation(w, 52, 1./60, true) {
			t.Fatal("arrival abandoned before retry window")
		}
	}
	if !ab.recoverPassedStation(w, 52, 1./60, true) {
		t.Fatal("slightly yawed late arrival remained blocked")
	}
	if ab.Bus.Pos != start || ab.Bus.PassengerCount != count || ab.Route.GetCurrentTargetStopID() == old {
		t.Fatal("recovery moved bus, lost passengers or kept old target")
	}
	if ab.Bus.Passengers[0].TargetStopID != ab.Route.GetCurrentTargetStopID() {
		t.Fatal("missed-stop passenger was not carried to next stop")
	}
}

func TestLateYawedRoadsideArrivalRetainsGuards(t *testing.T) {
	for _, name := range []string{"moving", "doors", "service", "before berth", "unfinished path", "head turning", "tail turning", "other road", "opposite side", "not held", "center BRT"} {
		t.Run(name, func(t *testing.T) {
			ab, w := loadLateYawedRoadside(t)
			stop := w.Stops[ab.Route.GetCurrentTargetStopID()]
			switch name {
			case "moving":
				ab.Bus.Speed = 5
			case "doors":
				ab.Bus.DoorsOpen = true
			case "service":
				ab.Bus.AddPassengerService(1, 1)
			case "before berth":
				ab.Bus.Pos = ab.Bus.Pos.Sub(sim.UnitFromAngle(stop.Heading).Mul(100))
			case "unfinished path":
				ab.Driver.LastRemaining = 500
			case "head turning":
				ab.Bus.Heading = stop.Heading + .5
			case "tail turning":
				ab.Bus.TrailerHeading = stop.Heading + .5
			case "other road":
				ab.Bus.Pos.Y -= 1000
			case "opposite side":
				ab.Bus.Pos.Y = stop.RoadCenter.Y - 100
				ab.Bus.Heading = math.Pi
			case "center BRT":
				stop.CenterBRT = true
			}
			for tick := 0; tick < 240; tick++ {
				if ab.recoverPassedStation(w, 52, 1./60, name != "not held") {
					t.Fatal("live station work or turn skipped")
				}
			}
		})
	}
}
