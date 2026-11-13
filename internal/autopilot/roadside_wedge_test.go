package autopilot

import (
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/roadside-departure-wedge.json
var roadsideWedgeFixture []byte

func loadRoadsideWedge(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	var s struct {
		rearDepartureFixture
		OtherCars []VehicleObstacle
	}
	if err := json.Unmarshal(roadsideWedgeFixture, &s); err != nil {
		t.Fatal(err)
	}
	ad, err := s.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: s.Bus, world: s.World, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: s.World.Stops[ad.LastTargetID], obstacles: s.OtherCars}
	for _, b := range s.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, s.Route
}

func TestSavedStationGeometryStallRecoversLocally(t *testing.T) {
	ad, f, route := loadRoadsideWedge(t)
	f.world.BeginBoundaryQueries(52)
	defer f.world.EndBoundaryQueries()
	start, health, passengers := f.bus.Pos, f.bus.Durability, f.bus.PassengerCount
	maxSpeed, model, index, trips := f.bus.MaxSpeedFwd, f.bus.Model, route.CurrentIndex, route.TotalTrips
	for tick := 0; tick < 2400; tick++ {
		before := f.bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, nil, f.obstacles, 52, 1./60)
		recovered := ad.RecoverStrandedStationDeparture(f.bus, f.world, nil, f.obstacles, 52, 1./60, th == 0 && br > 0)
		if recovered {
			if math.Abs(f.bus.Pos.Y-before.Y) > 12.001 || math.Abs(f.bus.Pos.X-before.X) > 65 {
				t.Fatal("recovery moved off the local station approach")
			}
		} else {
			f.bus.Update(1./60, th, br, st, hb, doors)
		}
		if (!recovered && f.bus.Pos.Distance(before) > math.Abs(f.bus.Speed)/60+.05) || f.bus.Durability < health || f.bus.PassengerCount != passengers {
			t.Fatalf("relocated, damaged or lost passengers tick%d", tick)
		}
		if f.bus.MaxSpeedFwd != maxSpeed || f.bus.Model != model || route.CurrentIndex != index || route.TotalTrips != trips {
			t.Fatal("recovery changed vehicle limits or route progress")
		}
		if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("hit shelter at tick%d", tick)
		}
		for _, ob := range f.obstacles {
			for _, part := range predictedObstacleBodies(ob, 0) {
				if part.HalfLen > 0 {
					if hit, _, depth := f.bus.CollidesBody(part); hit && depth > 1e-6 {
						t.Fatalf("hit traffic at tick%d depth%.5f", tick, depth)
					}
				}
			}
		}
		// Restoring in the middle of presteering/crawling retains the chosen turn.
		if tick == 60 {
			raw, err := json.Marshal(ad.Snapshot())
			if err != nil {
				t.Fatal(err)
			}
			var s DriverSnapshot
			if err = json.Unmarshal(raw, &s); err != nil {
				t.Fatal(err)
			}
			ad, err = s.Restore(f.world)
			if err != nil {
				t.Fatal(err)
			}
		}
		if f.bus.Pos.Distance(start) > 400 {
			t.Logf("cleared departure in %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatalf("departure remained wedged at %v", f.bus.Pos)
}

func TestStationGeometryRecoveryStillStopsForFrontTraffic(t *testing.T) {
	ad, f, _ := loadRoadsideWedge(t)
	forward := sim.UnitFromAngle(f.bus.Heading)
	f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos.Add(forward.Mul(f.bus.Length/2 + 29)), Heading: f.bus.Heading, Length: 48, Width: 40})
	ad.stationStallWait = 12
	ad.stationStallStart = f.bus.Pos
	before := f.bus.Pos
	if ad.RecoverStrandedStationDeparture(f.bus, f.world, nil, f.obstacles, 52, 1./60, true) || f.bus.Pos != before {
		t.Fatal("accepted recovery into blocked space")
	}
}

func TestStationGeometryRecoveryDoesNotResetOrdinaryWaits(t *testing.T) {
	for _, scenario := range []string{"doors open", "passenger service", "cruising queue", "clear geometry"} {
		t.Run(scenario, func(t *testing.T) {
			ad, f, _ := loadRoadsideWedge(t)
			switch scenario {
			case "doors open":
				f.bus.DoorsOpen = true
			case "passenger service":
				f.bus.AddPassengerService(1, 1)
			case "cruising queue":
				ad.State = AutopilotStateCruising
			case "clear geometry":
				f.bus.Pos.X = -62582
				f.bus.Heading = math.Pi / 2
				f.bus.ResetArticulation()
				ad.drivePath = []sim.Vec2{f.bus.Pos, f.bus.Pos.Add(sim.Vec2{Y: 1000})}
				ad.cumLen = []float64{0, 1000}
				ad.WaypointIdx = 0
				ad.LastPathS = 0
				f.obstacles = nil
			}
			start := f.bus.Pos
			ad.stationStallWait = 12
			ad.stationStallStart = start
			if ad.RecoverStrandedStationDeparture(f.bus, f.world, nil, f.obstacles, 52, 1./60, true) || f.bus.Pos != start {
				t.Fatal("ordinary wait triggered recovery")
			}
		})
	}
}
