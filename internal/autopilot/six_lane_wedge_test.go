package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/sidecar-entry-ai61-2026-11-08.json
var entryAI61Fixture []byte

//go:embed testdata/straddled-six-ai34-2026-11-08.json
var straddledSixFixture []byte

func TestSavedAI61AngledEntryClearsWithoutReset(t *testing.T) {
	replaySidecarFixture(t, entryAI61Fixture, true)
}
func loadSixLaneWedge(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if err := json.Unmarshal(straddledSixFixture, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{world: x.World, bus: x.Bus, tls: x.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: x.World.Stops[x.Route.GetCurrentTargetStopID()]}
	for _, c := range x.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
	}
	for _, b := range x.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, x.Route
}
func TestSavedSixLaneWedgeRecoversWithoutJourneyReset(t *testing.T) {
	ad, f, route := loadSixLaneWedge(t)
	start, passengers, health, index, trips, speed := f.bus.Pos, f.bus.PassengerCount, f.bus.Durability, route.CurrentIndex, route.TotalTrips, f.bus.MaxSpeedFwd
	called := false
	ad.BeforeRecovery = func() {
		called = true
		if f.bus.Pos != start {
			t.Fatal("observer received changed pose")
		}
	}
	for tick := 0; tick < 4800; tick++ {
		f.tls.Update(1. / 60)
		if !ad.RecoverStrandedMedianApproach(f.bus, f.world, f.obstacles, 52, 1./60, f.tls) {
			continue
		}
		if !called || f.bus.PassengerCount != passengers || f.bus.Durability != health || route.CurrentIndex != index || route.TotalTrips != trips || f.bus.MaxSpeedFwd != speed {
			t.Fatal("local alignment reset journey state")
		}
		if math.Abs(f.bus.Pos.X-start.X) > .01 || math.Abs(f.bus.Pos.Y-28826) > 1 {
			t.Fatalf("did not settle locally in legal left lane: %v", f.bus.Pos)
		}
		for _, ob := range f.obstacles {
			if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 0, 0) {
				t.Fatal("recovery touched traffic")
			}
		}
		t.Logf("local lane alignment at %.2fs", float64(tick+1)/60)
		return
	}
	t.Fatal("saved six-lane wedge was never handled")
}
func TestSixLaneWedgeRecoveryRetainsSafetyGuards(t *testing.T) {
	for _, name := range []string{"red", "moving", "centered", "blocked", "wrong direction"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadSixLaneWedge(t)
			switch name {
			case "red":
				for _, i := range f.tls.Intersections {
					i.Phase = sim.PhaseNSGreen
				}
			case "moving":
				f.bus.Speed = 5
			case "centered":
				f.bus.Pos.Y = 28826
				f.bus.Heading = 0
				f.bus.ResetArticulation()
			case "blocked":
				for _, y := range []float64{28826, 28930} {
					f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: f.bus.Pos.X, Y: y}, Heading: 0, Length: 192, Width: 36})
				}
			case "wrong direction":
				f.bus.Heading = -1.1
			}
			start := f.bus.Pos
			for tick := 0; tick < 1200; tick++ {
				if ad.RecoverStrandedMedianApproach(f.bus, f.world, f.obstacles, 52, 1./60, f.tls) || f.bus.Pos != start {
					t.Fatal("unsafe or unnecessary alignment")
				}
			}
		})
	}
}
func TestSixLaneCurbLeftDoesNotRequestMedianSlot(t *testing.T) {
	ad, f, _ := loadSixLaneWedge(t)
	f.bus.Pos = sim.Vec2{X: -950, Y: 28930}
	f.bus.Heading = 0
	f.bus.Speed = 0
	f.bus.SteeringAngle = 0
	f.bus.ResetArticulation()
	ad.LastPathS = 0
	ad.WaypointIdx = 0
	ad.pathSynced = false
	f.s, _ = ad.trackPath(f.bus.Pos)
	f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -920, Y: 28878}, Heading: 0, Length: 46, Width: 22}}
	ad.upgradeWideCurbPath(f.bus, f.world, 52)
	ad.PrepareLeg(f.world, f.world.Stops[ad.pathFromID], f.targetStop, 52)
	f.s, _ = ad.trackPath(f.bus.Pos)
	if ad.updateRequiredApproachMerge(f, 1./60) || ad.approachMerge != nil || f.bus.LaneMergeRequest != nil {
		t.Fatal("curb-left permission still forced a median merge")
	}

	_, _, target, _ := ad.pursuitSteering(f)
	if math.Abs(target.Y-f.bus.Pos.Y) > 2 {
		t.Fatalf("steered into blocked middle lane: %v", target)
	}
}

func TestSixLaneCenteredLeftQueueDoesNotRequestCurrentLane(t *testing.T) {
	ad, f, _ := loadSixLaneWedge(t)
	f.bus.Pos = sim.Vec2{X: -950, Y: 28826}
	f.bus.Heading = 0
	f.bus.Speed = 0
	f.bus.SteeringAngle = 0
	f.bus.ResetArticulation()
	ad.LastPathS = 0
	ad.WaypointIdx = 0
	ad.pathSynced = false
	f.s, _ = ad.trackPath(f.bus.Pos)
	f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -886, Y: 28826}, Heading: 0, Length: 96, Width: 36}}
	if ad.updateRequiredApproachMerge(f, 1./60) || f.bus.LaneMergeRequest != nil {
		t.Fatal("normal left queue reserved its current lane as a merge")
	}
}
