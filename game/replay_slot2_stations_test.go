package game

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func findSlot2Block197File() string {
	return firstFixture(filepath.Join("..", "dist", "slot2-197block-2026-11-02-1150", "slot2.json"))
}

// TestSavedSlot2Block197StationDeadlockDissolves verifies that the deadlock at Stop 6
// (opposite Stop 196 / Block 197 Northbound) in slot 2 dissolves cleanly:
// AIBus 6 (in the Stop 6 bay) escapes/departs onto Road 11,
// AIBus 5 (waiting in Lane 1) pulls into the bay and docks,
// and the player bus accelerates and cruises smoothly down Road 11.
func TestSavedSlot2Block197StationDeadlockDissolves(t *testing.T) {
	savePath := findSlot2Block197File()
	if savePath == "" {
		t.Skip("save file not found for slot2 block 197")
	}
	g := loadSaveForTest(t, savePath, 1280, 720)

	b5 := g.AIBusMgr.Buses[5]
	b6 := g.AIBusMgr.Buses[6]
	if b5 == nil || b6 == nil {
		t.Fatalf("could not find AI buses 5 and 6")
	}

	initB5Y := b5.Bus.Pos.Y
	initB6Y := b6.Bus.Pos.Y

	for step := 1; step <= 1200; step++ { // 20 seconds at 60Hz
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at step %d: %v", step, err)
		}
	}

	// B6 should have departed from bay (Y ~ 859.5) and moved down Road 11 past Y=1200
	if b6.Bus.Pos.Y <= initB6Y+200 {
		t.Fatalf("B6 remained stuck at Stop 6: initial Y=%.1f, current Y=%.1f", initB6Y, b6.Bus.Pos.Y)
	}

	// B5 should have entered the Stop 6 bay (X moves from ~7719 to < 7690, Y moves from ~758 to > 840)
	if b5.Bus.Pos.Y <= initB5Y+50 || b5.Bus.Pos.X > 7700 {
		t.Fatalf("B5 failed to enter Stop 6 bay: pos=(%.1f, %.1f)", b5.Bus.Pos.X, b5.Bus.Pos.Y)
	}

	// Player bus should have cleared through the upstream signal and traveled down Road 11 past Y=400
	if g.Bus.Pos.Y < 400 {
		t.Fatalf("Player bus failed to advance: pos=(%.1f, %.1f)", g.Bus.Pos.X, g.Bus.Pos.Y)
	}
}

func TestSavedSlot2ShoulderDepartureMergesPromptlyOntoRoad(t *testing.T) {
	savePath := firstFixture(filepath.Join("..", "dist", "slot2-shoulder-2026-11-04", "slot2.json"))
	if savePath == "" {
		t.Skip("slot2-shoulder-2026-11-04 fixture not found")
	}

	// 1. Loaded Save Slot 2 must immediately finish merging onto the road without shoulder drag.
	g := loadSaveForTest(t, savePath, 1280, 720)
	laneW := g.Settings.GetLaneWidth()
	for tick := 0; tick < 180; tick++ {
		g.Update()
		if g.Bus.OnShoulder {
			t.Fatalf("loaded slot2 bus hit shoulder at tick %d: pos=%v", tick, g.Bus.Pos)
		}
		if res := g.Bus.CheckBoundaries(g.World, laneW); res.Zone == sim.ZoneHardBarrier {
			t.Fatalf("loaded slot2 bus hit barrier at tick %d: %+v", tick, res)
		}
	}
	if g.Bus.Pos.Y > 14485.0 {
		t.Fatalf("loaded slot2 bus did not merge onto roadway: pos=%v", g.Bus.Pos)
	}

	// 2. Fresh departure from Stop 9 in both Berserk and Normal mode with curb-lane traffic ahead
	// must merge onto the road within the station exit taper and never drive along the shoulder.
	for _, berserk := range []bool{true, false} {
		g2 := NewGame(1280, 720)
		g2.savePathOverride = savePath
		if err := g2.LoadGame(); err != nil {
			t.Fatalf("LoadGame failed: %v", err)
		}
		b := g2.Bus
		ad := g2.AutopilotDriver
		prevIdx := (g2.Route.CurrentIndex - 1 + len(g2.Route.StopIDs)) % len(g2.Route.StopIDs)
		prevPrevIdx := (prevIdx - 1 + len(g2.Route.StopIDs)) % len(g2.Route.StopIDs)
		stop9 := g2.World.Stops[g2.Route.StopIDs[prevIdx]]
		berth := stop9.FrontBerthPosition()

		b.Pos = berth
		b.Heading = stop9.Heading
		b.Speed = 0
		b.SteeringAngle = 0
		g2.Route.CurrentIndex = prevIdx
		g2.Settings.AutopilotBerserk = berserk
		ad.Reset()
		ad.BerserkMode = berserk
		ad.BerserkMaxSpeed = 667
		ad.PrepareLeg(g2.World, g2.World.Stops[g2.Route.StopIDs[prevPrevIdx]], stop9, laneW)
		ad.State = 2 // AutopilotStateDocked
		ad.DockedStopID = stop9.ID
		ad.DwellFinished = true

		if len(g2.TrafficMgr.Cars) > 220 {
			g2.TrafficMgr.Cars[220].Pos.X = berth.X + 120
			g2.TrafficMgr.Cars[220].Pos.Y = 14478.0
			g2.TrafficMgr.Cars[220].Heading = 0
			g2.TrafficMgr.Cars[220].Speed = 222.2
		}

		mergedCurb := false
		mergedLane := false
		for tick := 0; tick < 360; tick++ {
			g2.Update()
			if b.OnShoulder {
				t.Fatalf("berserk=%v: bus drove on shoulder at tick %d: pos=%v", berserk, tick, b.Pos)
			}
			if res := b.CheckBoundaries(g2.World, laneW); res.Zone == sim.ZoneHardBarrier {
				t.Fatalf("berserk=%v: bus hit barrier at tick %d: %+v", berserk, tick, res)
			}
			if !mergedCurb && b.Pos.Y < 14504.0 {
				mergedCurb = true
				if b.Pos.X-berth.X > 180.0 {
					t.Fatalf("berserk=%v: bus took too long to cross curb onto road: dx=%.1f pos=%v", berserk, b.Pos.X-berth.X, b.Pos)
				}
			}
			if !mergedLane && b.Pos.Y <= 14485.0 {
				mergedLane = true
				if b.Pos.X-berth.X > 280.0 {
					t.Fatalf("berserk=%v: bus took too long to merge into outer lane: dx=%.1f pos=%v", berserk, b.Pos.X-berth.X, b.Pos)
				}
			}
		}
		if !mergedCurb || !mergedLane {
			t.Fatalf("berserk=%v: bus failed to merge onto road within 6s: pos=%v", berserk, b.Pos)
		}
	}
}

func findSlot2Station1WedgeFile() string {
	return firstFixture(
		filepath.Join("..", "dist", "slot2-jam-2026-11-02-1430", "slot2.json"),
		filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-02.json"),
	)
}

// TestSavedSlot2Station1WedgeDissolves verifies that the corridor bottleneck in save slot 2
// (adjacent-lane wedging between Bus 2 and Bus 41 approaching Stop 1, and Bus 43 departing Stop 1)
// dissolves cleanly without manual intervention:
// 1. Bus 43 clears its departure crawl and accelerates along its route.
// 2. Bus 2 (leader) smoothly proceeds into Stop 1, exchanges passengers, and continues to Stop 7.
// 3. Bus 41 safely follows Bus 2 into the Stop 1 bay.
// 4. The following queue (Bus 1, Bus 8) moves forward smoothly.
func TestSavedSlot2Station1WedgeDissolves(t *testing.T) {
	savePath := findSlot2Station1WedgeFile()
	if savePath == "" {
		t.Skip("save file not found for slot2 station 1 wedge")
	}

	g := loadSaveForTest(t, savePath, 1440, 900)

	initialPositions := make(map[int][2]float64)
	for _, ab := range g.AIBusMgr.Buses {
		initialPositions[ab.ID] = [2]float64{ab.Bus.Pos.X, ab.Bus.Pos.Y}
	}

	// The fallback is the user's live slot, which may have been overwritten
	// with another scenario. Only run this historical replay with its buses.
	for _, id := range []int{43, 2, 41, 1, 8} {
		if _, ok := initialPositions[id]; !ok {
			t.Skipf("slot 2 is not the historical station-1 wedge fixture (missing bus #%d)", id)
		}
	}

	// Run for 1800 ticks (30 seconds)
	for tick := 1; tick <= 1800; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed at tick %d: %v", tick, err)
		}
	}

	type busCheck struct {
		id      int
		minDist float64
		desc    string
	}

	checks := []busCheck{
		{id: 43, minDist: 1000.0, desc: "Bus 43 (departing Stop 1)"},
		{id: 2, minDist: 250.0, desc: "Bus 2 (approaching & servicing Stop 1)"},
		{id: 41, minDist: 150.0, desc: "Bus 41 (following into Stop 1 bay)"},
		{id: 1, minDist: 150.0, desc: "Bus 1 (following queue forward)"},
		{id: 8, minDist: 100.0, desc: "Bus 8 (following queue forward)"},
	}

	for _, chk := range checks {
		found := false
		for _, ab := range g.AIBusMgr.Buses {
			if ab.ID == chk.id {
				found = true
				initPos := initialPositions[chk.id]
				moved := math.Hypot(ab.Bus.Pos.X-initPos[0], ab.Bus.Pos.Y-initPos[1])
				if moved < chk.minDist {
					t.Errorf("%s moved only %.1f px, expected at least %.1f px", chk.desc, moved, chk.minDist)
				}
				break
			}
		}
		if !found {
			t.Errorf("bus #%d not found in manager", chk.id)
		}
	}
}

func findSlot2Station21File() string {
	_, source, _, _ := runtime.Caller(0)
	return firstFixture(
		filepath.Join(filepath.Dir(source), "..", "dist", "slot2-station21-jam-2026-11-03-0654", "slot2.json"),
		filepath.Join("..", "dist", "slot2-station21-jam-2026-11-03-0654", "slot2.json"),
	)
}

// TestSavedSlot2Station21DockingDissolves verifies that an AI bus arriving inside
// a station bay (e.g. Stop 32 / Coastal District 21 in slot 2) docks properly instead
// of stalling forever at speed=0 due to over-strict lateral tolerances.
func TestSavedSlot2Station21DockingDissolves(t *testing.T) {
	savePath := findSlot2Station21File()
	if savePath == "" {
		t.Skip("slot2 snapshot not found; skipping")
	}

	g := loadSaveForTest(t, savePath, 1280, 720)

	b20 := g.AIBusMgr.Buses[19]
	if b20 == nil || b20.ID != 20 {
		t.Fatalf("expected bus 20 at index 19")
	}

	// Within 60 frames (1 second), b20 must successfully enter docked state and open doors.
	dockedWithin1s := false
	for step := 1; step <= 60; step++ {
		if err := g.Update(); err != nil {
			t.Fatalf("Update failed: %v", err)
		}
		if b20.Bus.DoorsOpen && b20.Bus.PassengerService.Active {
			dockedWithin1s = true
			break
		}
	}

	if !dockedWithin1s {
		t.Fatalf("b20 failed to dock and open doors at Stop 32 within 1s: doorsOpen=%v, servActive=%v, state=%s",
			b20.Bus.DoorsOpen, b20.Bus.PassengerService.Active, b20.Driver.State.String())
	}
}

// TestSavedSlot2PlayerBerserkDoesNotStallSlanted verifies that when the player bus in Berserk mode
// approaches Stop 32 (Coastal District 21) on multi-lane road in the passing lane while the bay is occupied,
// it does not freeze at 0 km/h diagonally slanted across the travel lanes.
func TestSavedSlot2PlayerBerserkDoesNotStallSlanted(t *testing.T) {
	savePath := findSlot2Station21File()
	if savePath == "" {
		t.Skip("slot2 station21 snapshot not found")
	}
	g := loadSaveForTest(t, savePath, 1280, 720)

	// 1. Verify saved position can resume and cruise to Stop 4
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	for step := 1; step <= 60; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if g.Route.StopIDs[g.Route.CurrentIndex] != 4 {
		t.Fatalf("Expected route to advance to Stop 4, got Stop %d", g.Route.StopIDs[g.Route.CurrentIndex])
	}
	if g.Bus.Speed < 30.0 {
		t.Fatalf("Expected bus to accelerate after resuming, got speed %.1f", g.Bus.Speed)
	}

	// 2. Test approaching Stop 32 in passing lane with occupied bay
	g.Route.CurrentIndex = 3 // Stop 32
	g.Bus.Pos = sim.Vec2{X: -5100, Y: -7174}
	g.Bus.Heading = 0.0
	g.Bus.Speed = 150.0
	g.Bus.SteeringAngle = 0.0
	g.AutopilotDriver.BerserkMode = true
	g.AutopilotDriver.LaneOffset = -52
	g.AutopilotDriver.TargetLaneOffset = -52
	g.AutopilotDriver.OvertakeState = autopilot.OvertakePassing

	stalled := false
	for step := 1; step <= 250; step++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if step == 1 {
			targetStop := g.World.Stops[g.Route.GetCurrentTargetStopID()]
			t.Logf("Stop %d: pos=%v frontBerth=%v rearBerth=%v len=%.1f",
				targetStop.ID, targetStop.Pos, targetStop.FrontBerthPosition(), targetStop.RearBerthPosition(), targetStop.Length)
			for i, ob := range g.obstacles {
				if ob.Pos.Distance(targetStop.Pos) < 200 {
					t.Logf("  near ob %d: pos=%v spd=%.1f", i, ob.Pos, ob.Speed)
				}
			}
		}
		// Bus must never stall diagonally slanted in the travel lanes with 0 km/h speed
		isSlanted := math.Abs(g.Bus.Heading) > 0.08
		if g.Bus.Speed < 1.0 && isSlanted && g.AutopilotDriver.State != autopilot.AutopilotStateDocked {
			stalled = true
			t.Errorf("Step %d: Player bus stalled diagonally slanted on road: Pos=(%.1f, %.1f), Heading=%.4f (%.1f deg), Spd=%.1f",
				step, g.Bus.Pos.X, g.Bus.Pos.Y, g.Bus.Heading, g.Bus.Heading*180/math.Pi, g.Bus.Speed)
			break
		}
	}
	if stalled {
		t.Fatal("Player bus stalled slanted in travel lanes")
	}
}
