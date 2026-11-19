package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/ai54-station-fit-20261116.json.gz
var ai54StationFitScene []byte

func TestSavedAI54LateArrivalUsesForwardRunway(t *testing.T) {
	verifyAI54BayEntry(t, false, false)
}
func TestAI54ApproachClearsBothSections(t *testing.T) {
	verifyAI54BayEntry(t, true, false)
}
func TestAI54RearBerthClearsBothSections(t *testing.T) { verifyAI54BayEntry(t, true, true) }
func verifyAI54BayEntry(t *testing.T, earlier, rear bool) {
	gz, err := gzip.NewReader(bytes.NewReader(ai54StationFitScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var scene rearDepartureFixture
	if err := json.NewDecoder(gz).Decode(&scene); err != nil {
		t.Fatal(err)
	}
	ad, err := scene.Driver.Restore(scene.World)
	if err != nil {
		t.Fatal(err)
	}
	bus, stop := scene.Bus, scene.World.Stops[192]
	// Re-evaluate the saved arrival pose just before opening the doors. An
	// existing passenger exchange must never be interrupted or moved on load.
	ad.State, ad.DockedStopID, ad.stationDock = AutopilotStateCruising, -1, nil
	ad.DwellTimer, ad.DwellFinished = 0, false
	bus.DoorsOpen, bus.DoorProgress, bus.PassengerService.Active = false, 0, false
	if earlier {
		u := sim.UnitFromAngle(stop.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		bus.Pos = stop.FrontBerthPosition().Sub(u.Mul(650)).Sub(r.Mul(47.12))
		bus.Heading = stop.Heading
		bus.ResetArticulation()
		bus.Speed = 130
		ad.pathSynced = false
		ad.drivePath = nil
		ad.stationDock = nil
		scene.OtherBuses = nil
	}
	if rear {
		p := stop.FrontBerthPosition()
		lead := sim.NewBus(p.X, p.Y, stop.Heading)
		lead.DoorsOpen = true
		scene.OtherBuses = []*sim.Bus{lead}
	}
	var obs []VehicleObstacle
	for _, b := range scene.OtherBuses {
		obs = append(obs, ObserveBus(b))
	}
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, doors, dock := ad.ComputeControls(bus, scene.Route, scene.World, nil, obs, 52, 1./60)
		before := bus.Pos
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.05 {
			t.Fatal("position jumped")
		}
		if bus.CheckBoundaries(scene.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("hit station boundary")
		}
		for _, ob := range obs {
			if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
				t.Fatal("hit other bus")
			}
		}
		if dock {
			maxLat := 0.
			u := sim.UnitFromAngle(stop.Heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			for _, part := range bus.BodyParts() {
				for _, p := range sim.BodyCorners(part) {
					maxLat = math.Max(maxLat, math.Abs(p.Sub(stop.Pos).Dot(r)))
				}
			}
			t.Logf("docked at %.2fs pos%v head%.4f tail%.4f lateral extent %.3f / %.3f", float64(tick)/60, bus.Pos, bus.Heading, bus.TailHeading(), maxLat, stop.Width/2)
			limit := stop.Width/2 + 1
			if !earlier {
				limit = 35
				if stop.FrontBerthPosition().Sub(bus.Pos).Dot(u) > 10 {
					t.Fatal("did not use available forward runway")
				}
			}
			if maxLat > limit {
				t.Fatalf("boarding lateral extent %.2f exceeds %.2f", maxLat, limit)
			}
			return
		}
	}
	t.Fatalf("did not dock: pos%v speed%.1f plan%+v", bus.Pos, bus.Speed, ad.stationDock)
}

func TestArticulatedBayEntryFitsBothSectionsInEveryDirection(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, rear := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/rear%v", h, rear), func(t *testing.T) {
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				for i, x := range []float64{-1500, 0, 1500} {
					w.Stops = append(w.Stops, sim.NewBusStop(i, "station", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), 52, 0))
				}
				stop := w.Stops[1]
				pos := stop.FrontBerthPosition().Sub(u.Mul(650)).Sub(r.Mul(47.12))
				bus := sim.NewBus(pos.X, pos.Y, h)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 130
				ad := NewAutopilotDriver()
				ad.FleetBus = true
				ad.DisallowOncomingOvertake = true
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				var obs []VehicleObstacle
				if rear {
					p := stop.FrontBerthPosition()
					lead := sim.NewBus(p.X, p.Y, h)
					lead.DoorsOpen = true
					obs = []VehicleObstacle{ObserveBus(lead)}
				}
				for tick := 0; tick < 1800; tick++ {
					if tick == 120 {
						var err error
						ad, err = ad.Snapshot().Restore(w)
						if err != nil {
							t.Fatal(err)
						}
						ad.FleetBus = true
					}
					th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obs, 52, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
						t.Fatal("hit station boundary")
					}
					for _, ob := range obs {
						if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
							t.Fatal("hit lead bus")
						}
					}
					if dock {
						if extent := stationBayLateralExtent(bus, stop); extent > stop.Width/2+1 {
							t.Fatalf("body outside bay: %.3f / %.3f", extent, stop.Width/2)
						}
						return
					}
				}
				t.Fatalf("failed to dock pos%v speed%.1f plan%+v", bus.Pos, bus.Speed, ad.stationDock)
			})
		}
	}
}

func TestArticulatedRunInUpgradePreservesApproachEnvelope(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		stop := sim.NewBusStop(0, "station", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), 52, 0)
		end := stop.FrontBerthPosition()
		start := end.Sub(u.Mul(500)).Sub(r.Mul(47.12))
		path := []sim.Vec2{start.Sub(u.Mul(1000)), start, end.Sub(u.Mul(160)), end}
		fixed, changed := upgradeArticulatedStationRunIn(path, stop)
		if !changed || fixed[0] != path[0] || fixed[1] != start || fixed[3] != end || math.Abs(end.Sub(fixed[2]).Dot(u)-280) > .001 {
			t.Fatal("upgrade changed approach envelope or target")
		}
		if math.Abs(end.Sub(path[2]).Dot(u)-160) > .001 {
			t.Fatal("upgrade mutated input")
		}
		if _, changed := upgradeArticulatedStationRunIn(fixed, stop); changed {
			t.Fatal("upgrade is not idempotent")
		}
	}
}
