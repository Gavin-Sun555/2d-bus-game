package autopilot

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

// Captured controller/vehicle/light state from the native qualification soak.
// Decorative world data and passenger counts are omitted; driving geometry,
// cached paths and the two conflicting pairs are retained.
//
//go:embed testdata/bus-interlocks.json
var busInterlockFixture []byte

type interlockScene struct {
	World  *sim.World
	Lights *sim.TrafficLightSystem
	Fleet  []struct {
		ID     int
		Bus    *sim.Bus
		Route  *sim.Route
		Driver DriverSnapshot
	}
}

func loadInterlockScene(t *testing.T) interlockScene {
	t.Helper()
	var s interlockScene
	if err := json.Unmarshal(busInterlockFixture, &s); err != nil {
		t.Fatal(err)
	}
	return s
}

func TestCapturedBusPairsReleaseWithoutCars(t *testing.T) {
	for _, pair := range [][2]int{{18, 23}, {19, 20}} {
		for _, reverse := range []bool{false, true} {
			for _, reload := range []bool{false, true} {
				t.Run(fmt.Sprintf("%d-%d/reverse%v/reload%v", pair[0], pair[1], reverse, reload), func(t *testing.T) {
					s := loadInterlockScene(t)
					var buses [2]*sim.Bus
					var routes [2]*sim.Route
					var drivers [2]*AutopilotDriver
					for _, a := range s.Fleet {
						for i, id := range pair {
							if a.ID == id {
								buses[i] = a.Bus
								routes[i] = a.Route
								var err error
								drivers[i], err = a.Driver.Restore(s.World)
								if err != nil {
									t.Fatal(err)
								}
							}
						}
					}
					start := [2]sim.Vec2{buses[0].Pos, buses[1].Pos}
					distance := [2]float64{}
					for tick := 0; tick < 7200; tick++ {
						s.Lights.Update(1. / 60)
						for n := 0; n < 2; n++ {
							i := n
							if reverse {
								i = 1 - n
							}
							b := buses[i]
							before := b.Pos
							th, br, st, hb, doors, _ := drivers[i].ComputeControls(b, routes[i], s.World, s.Lights, []VehicleObstacle{ObserveBus(buses[1-i])}, 52, 1./60)
							b.Update(1./60, th, br, st, hb, doors)
							distance[i] += before.Distance(b.Pos)
							if hit, _, depth := sim.CheckBusesCollision(buses[0], buses[1]); hit {
								t.Fatalf("contact tick%d depth%.3f", tick, depth)
							}
							if boundary := b.CheckBoundaries(s.World, 52); boundary.Zone == sim.ZoneHardBarrier {
								t.Fatalf("barrier tick%d bus%d: %+v", tick, pair[i], boundary)
							}
						}
						if reload && tick == 300 {
							for i, ad := range drivers {
								data, err := json.Marshal(ad.Snapshot())
								if err != nil {
									t.Fatal(err)
								}
								var snap DriverSnapshot
								if err = json.Unmarshal(data, &snap); err != nil {
									t.Fatal(err)
								}
								drivers[i], err = snap.Restore(s.World)
								if err != nil {
									t.Fatal(err)
								}
							}
						}
					}
					for i, b := range buses {
						if distance[i] < 1000 || b.Pos.Distance(start[i]) < 500 {
							t.Fatalf("bus%d still held: distance%.1f displacement%.1f", pair[i], distance[i], b.Pos.Distance(start[i]))
						}
						t.Logf("bus%d %.1fpx", pair[i], distance[i])
					}
				})
			}
		}
	}
}

func TestMidLaneRecoveryUsesPhysicalNeighborSections(t *testing.T) {
	s := loadInterlockScene(t)
	var bus, neighbor *sim.Bus
	var saved DriverSnapshot
	for _, a := range s.Fleet {
		if a.ID == 19 {
			bus, saved = a.Bus, a.Driver
		}
		if a.ID == 20 {
			neighbor = a.Bus
		}
	}
	ad, err := saved.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: bus, world: s.World, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(neighbor)}}
	if !ad.midLaneSweepSafe(f, -1) {
		t.Fatal("separated physical sections were treated as an envelope collision")
	}
	f.obstacles[0].Bus = nil
	if ad.midLaneSweepSafe(f, -1) {
		t.Fatal("fixture no longer exercises the inflated-envelope false block")
	}
	blocked := *neighbor
	blocked.Pos = bus.Pos
	blocked.Heading = bus.Heading
	blocked.TrailerHeading = bus.TrailerHeading
	f.obstacles = []VehicleObstacle{ObserveBus(&blocked)}
	if ad.midLaneSweepSafe(f, -1) {
		t.Fatal("real overlapping bus was allowed")
	}
}

func TestStationYieldRecoveryRequiresLiveStoppedBusRequest(t *testing.T) {
	for _, tc := range []string{"valid", "no request", "expired", "joined", "moving", "other target", "car", "doors open", "unsafe sweep"} {
		t.Run(tc, func(t *testing.T) {
			s := loadInterlockScene(t)
			var bus, neighbor *sim.Bus
			var saved DriverSnapshot
			for _, a := range s.Fleet {
				if a.ID == 18 {
					bus, saved = a.Bus, a.Driver
				}
				if a.ID == 23 {
					neighbor = a.Bus
				}
			}
			ad, err := saved.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
			neighbor.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{X: -78}, Heading: neighbor.Heading, Remaining: .3}
			switch tc {
			case "no request":
				neighbor.LaneMergeRequest = nil
			case "expired":
				neighbor.LaneMergeRequest.Remaining = 0
			case "joined":
				neighbor.LaneMergeRequest.Joined = true
			case "moving":
				neighbor.Speed = 40
			case "other target":
				neighbor.LaneMergeRequest.Target.X = -26
			case "doors open":
				bus.DoorsOpen = true
			}
			ob := ObserveBus(neighbor)
			if tc == "car" {
				ob.Bus = nil
			}
			f := autopilotFrame{bus: bus, world: s.World, laneWidth: 52, obstacles: []VehicleObstacle{ob}, s: ad.LastPathS}
			ad.prepareStationYieldRecovery(f)
			if tc == "valid" || tc == "unsafe sweep" {
				if ad.midLaneRecovery == nil {
					t.Fatal("valid waiting dependency not recognized")
				}
				if tc == "unsafe sweep" {
					neighbor.Pos = bus.Pos
					ob = ObserveBus(neighbor)
					f.obstacles = []VehicleObstacle{ob}
				}
				ad.midLaneSafeSteering(f, 1)
				if tc == "unsafe sweep" {
					if ad.clearsStationYieldNeighbor(f, ob) {
						t.Fatal("unsafe sweep granted departure")
					}
					return
				}
				if !ad.clearsStationYieldNeighbor(f, ob) {
					t.Fatal("safe bay-side recenter did not release waiting margin")
				}
				neighbor.Speed = 40
				ob = ObserveBus(neighbor)
				if ad.clearsStationYieldNeighbor(f, ob) {
					t.Fatal("moving road bus lost priority")
				}
			} else if ad.midLaneRecovery != nil {
				t.Fatal("recovery started without a live safe dispatch dependency")
			}
		})
	}
}
