package autopilot

import (
	"embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/arrival-side-lock-*.json
var arrivalSideFixtures embed.FS

func loadArrivalSideLock(t *testing.T, model string) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	data, e := arrivalSideFixtures.ReadFile("testdata/arrival-side-lock-" + model + ".json")
	if e != nil {
		t.Fatal(e)
	}
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if e = json.Unmarshal(data, &x); e != nil {
		t.Fatal(e)
	}
	ad, e := x.Driver.Restore(x.World)
	if e != nil {
		t.Fatal(e)
	}
	f := autopilotFrame{bus: x.Bus, world: x.World, tls: x.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: x.World.Stops[x.Route.GetCurrentTargetStopID()]}
	for _, b := range x.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	for _, c := range x.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
	}
	return ad, f, x.Route
}
func TestSavedArrivalSideLockMovesWithoutContact(t *testing.T) {
	for _, model := range []string{"articulated", "standard"} {
		t.Run(model, func(t *testing.T) {
			ad, f, route := loadArrivalSideLock(t, model)
			origin := f.bus.Pos
			desired, _, _, _ := ad.pursuitSteering(f)
			if _, ok := ad.stationArrivalEscape(f, desired); !ok {
				t.Fatal("no safe forward arrival adjustment found")
			}
			for tick := 0; tick < 600; tick++ {
				th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
				before := f.bus.Pos
				f.bus.Update(1./60, th, br, st, hb, doors)
				if before.Distance(f.bus.Pos) > f.bus.MaxSpeedFwd/60+1 {
					t.Fatal("arrival relocated bus")
				}
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatal("arrival crossed hard boundary")
				}
				if math.Abs(sim.NormalizeAngle(f.bus.Heading-f.bus.TailHeading())) > sim.MaxArticulationAngle {
					t.Fatal("excess articulation")
				}
				for _, ob := range f.obstacles {
					for _, body := range predictedObstacleBodies(ob, 0) {
						if body.HalfLen > 0 {
							if hit, _, depth := f.bus.CollidesBody(body); hit && depth > 1e-6 {
								t.Fatalf("arrival contact at %.2fs depth%g", float64(tick)/60, depth)
							}
						}
					}
				}
			}
			if f.bus.Pos.Distance(origin) < 20 {
				t.Fatal("safe arrival remained pinned")
			}
		})
	}
}
func TestArrivalEscapeRetainsDoorAndObstacleGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "speed", "admission", "blocked"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadArrivalSideLock(t, "articulated")
			switch name {
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 0)
			case "speed":
				f.speedAbs = 40
			case "admission":
				f.bus.RoundaboutAdmissionHold = &sim.RoundaboutAdmissionHold{}
			case "blocked":
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: f.bus.Pos, Heading: f.bus.Heading, Length: 150, Width: 150})
			}
			desired, _, _, _ := ad.pursuitSteering(f)
			if _, ok := ad.stationArrivalEscape(f, desired); ok {
				t.Fatal("arrival ignored safety guard")
			}
		})
	}
}

func TestLeadingArrivalBoardsWithoutWaitingForRearReservation(t *testing.T) {
	ad, f, route := loadArrivalSideLock(t, "front")
	if !f.targetStop.ContainsBus(f.bus) || len(availableStationBerths(f.bus, f.targetStop, f.obstacles)) != 0 {
		t.Fatal("fixture no longer reproduces rear reservation of occupied front pose")
	}
	if !stationFrontArrivalClear(f) {
		t.Fatal("valid leading boarding pose was reserved by a rear follower")
	}
	th, br, _, _, doors, docked := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
	if th != 0 || br == 0 || !doors || !docked {
		t.Fatal("leading bus did not open doors in its existing valid front pose")
	}
	for _, ob := range f.obstacles {
		if stationClosedRearNeighbor(f, ob) && f.targetStop.OccupiesBerth(ob.Bus.BodyEnvelope()) {
			ob.Bus.DoorsOpen = true
			if stationFrontArrivalClear(f) {
				t.Fatal("an actually open-door occupied berth was ignored")
			}
			return
		}
	}
	t.Fatal("missing rear platform follower")
}

func TestArrivalAtFrontEdgeFinishesInsidePlatform(t *testing.T) {
	ad, f, route := loadArrivalSideLock(t, "edge")
	for tick := 0; tick < 900; tick++ {
		th, br, st, hb, doors, docked := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
		f.bus.Update(1./60, th, br, st, hb, doors)
		for _, ob := range f.obstacles {
			for _, body := range predictedObstacleBodies(ob, 0) {
				if body.HalfLen > 0 {
					if hit, _, depth := f.bus.CollidesBody(body); hit && depth > 1e-6 {
						t.Fatal("front-edge arrival touched a neighbor")
					}
				}
			}
		}
		if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("front-edge arrival crossed a boundary")
		}
		if docked && f.bus.DoorsOpen {
			if !f.targetStop.ContainsBus(f.bus) {
				t.Fatal("opened doors outside the real platform")
			}
			return
		}
	}
	t.Fatalf("arrival remained outside: pos%v heading%v contains%v targetSpeed%v plan%+v",f.bus.Pos,f.bus.Heading,f.targetStop.ContainsBus(f.bus),ad.LastTargetSpeed,ad.stationDock)
}
