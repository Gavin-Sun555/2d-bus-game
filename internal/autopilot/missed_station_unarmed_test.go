package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/missed-taper-unarmed-20261111.json
var unarmedMissedTaper []byte

func loadUnarmedMissedTaper(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	var scene rearDepartureFixture
	if err := json.Unmarshal(unarmedMissedTaper, &scene); err != nil {
		t.Fatal(err)
	}
	ad, err := scene.Driver.Restore(scene.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: scene.Bus, world: scene.World, tls: scene.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: scene.World.Stops[scene.Route.GetCurrentTargetStopID()]}
	for _, b := range scene.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, scene.Route
}

func TestStoppedMissedTaperReacquiresRecovery(t *testing.T) {
	ad, f, route := loadUnarmedMissedTaper(t)
	if ad.departureEscapeActive {
		t.Fatal("fixture must reproduce missing recovery state")
	}
	ad.BeforeRecovery = func() { t.Fatal("recovery repositioned bus") }
	start, passengers := f.bus.Pos, f.bus.PassengerCount
	reversed := false
	for tick := 0; tick < 600; tick++ {
		before := f.bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
		if tick == 0 && ad.roundaboutEdgeRecovery == nil {
			t.Fatal("stalled missed taper did not reacquire checked reverse")
		}
		f.bus.Update(1./60, th, br, st, hb, doors)
		reversed = reversed || f.bus.Speed < 0
		if f.bus.Pos.Distance(before) > math.Abs(f.bus.Speed)/60+.001 || f.bus.PassengerCount != passengers {
			t.Fatal("recovery relocated bus or lost passengers")
		}
		if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("recovery crossed boundary")
		}
		for _, ob := range f.obstacles {
			for _, part := range ob.Bus.BodyParts() {
				if part.HalfLen > 0 {
					if hit, _, _ := f.bus.CollidesBody(part); hit {
						t.Fatal("recovery hit another bus")
					}
				}
			}
		}
		if tick == 30 {
			var err error
			ad, err = ad.Snapshot().Restore(f.world)
			if err != nil {
				t.Fatal(err)
			}
		}
	}
	if !reversed || f.bus.Pos.Distance(start) < 400 {
		t.Fatalf("missed taper still blocked: reversed %v travel %.1f", reversed, f.bus.Pos.Distance(start))
	}
}

func TestMissedTaperReacquisitionRetainsGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "moving", "served stop", "no missed stop", "before berth", "oncoming", "physical obstacle"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadUnarmedMissedTaper(t)
			switch name {
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 1)
			case "moving":
				f.speedAbs = 30
				f.bus.Speed = 30
			case "served stop":
				ad.skippedDeparture.served = true
			case "no missed stop":
				ad.skippedDeparture = nil
			case "before berth":
				f.bus.Pos.X = f.world.Stops[23].FrontBerthPosition().X - 10
			case "oncoming":
				f.bus.Heading = math.Pi
			case "physical obstacle":
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos, Heading: f.bus.Heading, Length: 40, Width: 36})
			}
			ad.stationDepartureEscape(f, -1)
			if ad.roundaboutEdgeRecovery != nil {
				t.Fatal("unsafe or unrelated reverse planned")
			}
		})
	}
}
