package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

func TestShortArticulatedDepartureDoesNotFoldBack(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	path := buildAutopilotDrivePath(w, w.Stops[8], w.Stops[20], 52, false, false, sim.RoadPathOptions{LongVehicle: true})
	if departureFold(path, w.Stops[8].Heading) {
		t.Fatalf("departure folds back: %v", path[:3])
	}
	old := append([]sim.Vec2(nil), path...)
	old[2] = sim.Vec2{X: -3718, Y: 877.9041647804892}
	old[1] = old[0].Add(sim.UnitFromAngle(w.Stops[8].Heading).Mul(120))
	snap := DriverSnapshot{Public: *NewAutopilotDriver(), ArticulatedPath: true, DrivePath: old, PathRevision: 2, PathFromID: 8, PathToID: 20, PathLane: 52, PathSynced: true, SkippedStop: -1}
	snap.BusLaneJunction = &sim.Vec2{X: -3900}
	snap.Public.BerserkMaxSpeed = 333
	snap.Public.CruiseSpeedLimit = 333
	snap.Public.BerserkMode = true
	snap.Public.DisallowOncomingOvertake = true
	snap.Public.LastTargetID = 20
	snap.Public.LastPathS = 120
	snap.Public.WaypointIdx = 1
	ad, err := snap.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	if departureFold(ad.drivePath, w.Stops[8].Heading) || ad.pathRevision != currentDrivePathRevision {
		t.Fatal("saved folded departure was not upgraded")
	}
	// An already-entered maneuver must retain its cached path on load.
	snap.BusJunctionEntered = true
	entered, err := snap.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	if !departureFold(entered.drivePath, w.Stops[8].Heading) {
		t.Fatal("in-progress junction path was replaced")
	}
}

func TestIncomingRoadOffsetDoesNotShiftLongitudinalPosition(t *testing.T) {
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad := NewAutopilotDriver()
	ad.drivePath = []sim.Vec2{{X: -3718, Y: 900}, {X: -3796, Y: 735}, {X: -3874, Y: 625}, {X: -3874}}
	ad.LaneOffset = 78
	ad.busLaneJunction = &sim.Vec2{X: -3900}
	p := sim.Vec2{X: -3728, Y: 700}
	base := ad.basePathPosition(p, w, -1.5707963267948966, 52)
	if base.Y != p.Y || base.X >= p.X {
		t.Fatalf("incoming-road offset changed forward coordinate: %v -> %v", p, base)
	}
}
