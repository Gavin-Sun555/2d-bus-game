package autopilot

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"slices"
	"testing"
)

//go:embed testdata/station_shelter_exit.json
var shelterExitFixture []byte

func shelterExitCase(t *testing.T) (*sim.World, *sim.Bus, *AutopilotDriver, *sim.Route) {
	t.Helper()
	var saved struct {
		Bus    sim.Bus
		Driver DriverSnapshot
	}
	if err := json.Unmarshal(shelterExitFixture, &saved); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad, err := saved.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	return w, &saved.Bus, ad, &sim.Route{StopIDs: []int{6, 7}, CurrentIndex: 1}
}

func TestServedArticulatedExitClearsShelterWithoutContact(t *testing.T) {
	w, b, ad, r := shelterExitCase(t)
	start := b.Pos
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, r, w, nil, nil, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if result := b.CheckBoundaries(w, 52); result.Zone == sim.ZoneHardBarrier {
			t.Fatalf("contact at tick %d: %+v", tick, result)
		}
		if b.Pos.Distance(start) > 600 {
			return
		}
	}
	t.Fatalf("station exit remained stopped: distance %.2f target %.3f", b.Pos.Distance(start), ad.LastTargetSpeed)
}

func TestServedBayDepartureDoesNotReconnectRoadLane(t *testing.T) {
	w, b, ad, _ := shelterExitCase(t)
	stop := w.Stops[6]
	u := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	b.Pos = stop.RoadCenter.Add(right.Mul(3.5 * 52))
	ad.skippedDeparture.position = b.Pos
	before := append([]sim.Vec2(nil), ad.drivePath...)
	ad.reconnectServedBayDeparture(b, stop, 52)
	if !slices.Equal(before, ad.drivePath) {
		t.Fatal("roadside service was redirected into a bay departure")
	}
}

func TestServedBayDepartureShelterClearanceAcrossHeadings(t *testing.T) {
	original, captured, _, _ := shelterExitCase(t)
	source := original.Stops[6]
	sourceForward := sim.UnitFromAngle(source.Heading)
	sourceRight := sim.Vec2{X: -sourceForward.Y, Y: sourceForward.X}
	relative := captured.Pos.Sub(source.Pos)
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			w, u, r := serviceTestWorld(heading, 52)
			stop := w.Stops[1]
			b := *captured
			b.Pos = stop.Pos.Add(u.Mul(relative.Dot(sourceForward))).Add(r.Mul(relative.Dot(sourceRight)))
			b.Heading = sim.NormalizeAngle(heading + captured.Heading - source.Heading)
			b.TrailerHeading = sim.NormalizeAngle(heading + captured.TailHeading() - source.Heading)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.beginRoadDeparture(&b, stop, 52)
			ad.skippedDeparture.served = true
			route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 2}
			start := b.Pos
			for tick := 0; tick < 1800; tick++ {
				th, br, st, hb, doors, _ := ad.ComputeControls(&b, route, w, nil, nil, 52, 1./60)
				b.Update(1./60, th, br, st, hb, doors)
				if boundary := b.CheckBoundaries(w, 52); boundary.Zone == sim.ZoneHardBarrier {
					t.Fatalf("boundary tick%d %+v", tick, boundary)
				}
				if b.Pos.Distance(start) > 600 {
					return
				}
			}
			t.Fatal("failed to clear served bay")
		})
	}
}

func TestServedBayDepartureRetainsPhysicalFrontTraffic(t *testing.T) {
	w, b, ad, r := shelterExitCase(t)
	start := b.Pos
	forward := sim.UnitFromAngle(w.Stops[6].Heading)
	obstacle := VehicleObstacle{Pos: b.Pos.Add(forward.Mul(80)), Heading: w.Stops[6].Heading, Length: 46, Width: 22}
	for tick := 0; tick < 120; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, r, w, nil, []VehicleObstacle{obstacle}, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if hit, _, _ := b.CollidesBody(sim.OBB{Center: obstacle.Pos, Heading: obstacle.Heading, HalfLen: 23, HalfWid: 11}); hit {
			t.Fatal("departure crossed front traffic")
		}
	}
	if b.Pos.Distance(start) > 2 {
		t.Fatal("departure bypassed stopped front vehicle")
	}
}

//go:embed testdata/station_waiting_realign.json
var waitingRealignFixture []byte

func TestVacatedStationQueueExtendsUnfinishedAlignment(t *testing.T) {
	var saved struct {
		Bus    sim.Bus
		Driver DriverSnapshot
	}
	if err := json.Unmarshal(waitingRealignFixture, &saved); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad, err := saved.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	b := &saved.Bus
	route := &sim.Route{StopIDs: []int{5, 6, 7}, CurrentIndex: 1}
	start := b.Pos
	opened := false
	for tick := 0; tick < 3600; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, nil, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		opened = opened || b.DoorsOpen
		if boundary := b.CheckBoundaries(w, 52); boundary.Zone == sim.ZoneHardBarrier {
			t.Fatalf("contact tick%d %+v", tick, boundary)
		}
		if opened && route.GetCurrentTargetStopID() != 6 && b.Pos.Distance(start) > 600 {
			return
		}
	}
	t.Fatalf("vacated waiting berth did not resume: opened %v distance%.1f target%d plan%+v", opened, b.Pos.Distance(start), route.GetCurrentTargetStopID(), ad.stationDock)
}

func TestStationQueueRealignmentWaitsForOccupiedForwardBerth(t *testing.T) {
	var saved struct {
		Bus    sim.Bus
		Driver DriverSnapshot
	}
	if err := json.Unmarshal(waitingRealignFixture, &saved); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad, err := saved.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	stop := w.Stops[6]
	p := stop.FrontBerthPosition()
	front := sim.NewBus(p.X, p.Y, stop.Heading)
	originalTarget := ad.stationDock.target
	ad.ComputeControls(&saved.Bus, &sim.Route{StopIDs: []int{5, 6, 7}, CurrentIndex: 1}, w, nil, []VehicleObstacle{ObserveBus(front)}, 52, 1./60)
	if ad.stationDock == nil || !ad.stationDock.waiting || ad.stationDock.target != originalTarget {
		t.Fatal("occupied forward berth was treated as free")
	}
}
