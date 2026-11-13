package autopilot

import (
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/straddled-left-ai34.json
var straddledLeftFixture []byte

func loadStraddledLeft(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	var s struct {
		rearDepartureFixture
		OtherCars []VehicleObstacle
	}
	if err := json.Unmarshal(straddledLeftFixture, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: s.Bus, world: s.World, laneWidth: 52, s: ad.LastPathS, speedAbs: math.Abs(s.Bus.Speed), obstacles: s.OtherCars}
	for _, b := range s.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, s.Route
}

func TestSavedAI34StraddledLeftRecoversToClearLegalLane(t *testing.T) {
	ad, f, route := loadStraddledLeft(t)
	start, health, passengers, maxSpeed, index, trips := f.bus.Pos, f.bus.Durability, f.bus.PassengerCount, f.bus.MaxSpeedFwd, route.CurrentIndex, route.TotalTrips
	for tick := 0; tick < 660; tick++ {
		if ad.RecoverStrandedMedianApproach(f.bus, f.world, f.obstacles, 52, 1./60) {
			if tick < 598 {
				t.Fatal("recovered before sustained stop")
			}
			if math.Abs(f.bus.Pos.Y-start.Y) > .001 || math.Abs(f.bus.Pos.X+15782) > 1 || ad.ordinaryBusLeft {
				t.Fatalf("did not settle in the clear curb-left lane: %v", f.bus.Pos)
			}
			if !wholeBusInMergeLane(f.bus, &sim.BusLaneMergeRequest{Target: sim.Vec2{X: -15782}, Heading: math.Pi / 2}, 52) {
				t.Fatal("still straddling after recovery")
			}
			if f.bus.Durability != health || f.bus.PassengerCount != passengers || f.bus.MaxSpeedFwd != maxSpeed || route.CurrentIndex != index || route.TotalTrips != trips {
				t.Fatal("changed passengers, limits or route progress")
			}
			for _, ob := range f.obstacles {
				if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 0, 0) {
					t.Fatal("recovery landed on traffic")
				}
			}
			t.Logf("settled at %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatal("straddled approach never recovered")
}

func TestStraddledLeftRecoveryRetainsQueueAndLandingGuards(t *testing.T) {
	for _, name := range []string{"blocked landings", "centered queue", "moving", "doors"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadStraddledLeft(t)
			switch name {
			case "blocked landings":
				for _, x := range []float64{-15626, -15782} {
					f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: x, Y: f.bus.Pos.Y}, Heading: math.Pi / 2, Length: 96, Width: 36})
				}
			case "centered queue":
				f.bus.Pos.X = -15678
				f.bus.Heading = math.Pi / 2
				f.bus.ResetArticulation()
			case "moving":
				f.bus.Speed = 10
			case "doors":
				f.bus.DoorsOpen = true
			}
			start := f.bus.Pos
			for tick := 0; tick < 720; tick++ {
				if ad.RecoverStrandedMedianApproach(f.bus, f.world, f.obstacles, 52, 1./60) || f.bus.Pos != start {
					t.Fatal("unsafe or unnecessary recovery")
				}
			}
		})
	}
}
