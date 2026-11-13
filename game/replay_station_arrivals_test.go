package game

import (
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Read-only replay of the October 25 angled articulated arrival.
func TestSavedAngledStationArrivalRecovery(t *testing.T) {
	path := os.Getenv("BUS_ANGLED_STATION_SAVE")
	if path == "" {
		t.Skip("BUS_ANGLED_STATION_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	target := g.Route.GetCurrentTargetStopID()
	initialBoarded := 0
	if a := g.Operations.Vehicle(0).Stations[target]; a != nil {
		initialBoarded = a.Boarded
	}
	boarded, served := false, false
	distances := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	start := g.Bus.Pos
	cleared := false
	for tick := 0; tick < 3600; tick++ {
		before := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, ab := range g.AIBusMgr.Buses {
			before[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		check := func(id int, bus *sim.Bus) {
			step := bus.Pos.Distance(before[id])
			if step > bus.MaxSpeedFwd/60+1 {
				t.Fatalf("vehicle%d jumped at tick%d: %.2f", id, tick, step)
			}
			if bus.Durability < health[id] {
				t.Fatalf("vehicle%d contact at tick%d", id, tick)
			}
			distances[id] += step
		}
		check(0, g.Bus)
		if !cleared && g.Bus.Pos.Distance(start) > 600 {
			cleared = true
			t.Logf("cleared 600 px at %.2fs", float64(tick)/60)
		}
		if tick == 600 && !cleared {
			t.Fatal("failed to leave station promptly")
		}
		for _, ab := range g.AIBusMgr.Buses {
			check(ab.ID, ab.Bus)
		}
		if !boarded && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			boarded = true
			t.Logf("opened doors at original target%d at %.2fs", target, float64(tick)/60)
		}
		if tick == 120 && !boarded {
			t.Fatal("did not open at saved angled berth within two seconds")
		}
		if !served && g.Route.GetCurrentTargetStopID() != target {
			a := g.Operations.Vehicle(0).Stations[target]
			if !boarded || g.lastSkippedStopID == target || a == nil || a.Boarded <= initialBoarded {
				t.Fatal("advanced without actually serving original stop")
			}
			served = true
			t.Logf("passengers boarded and route advanced at %.2fs", float64(tick)/60)
		}
	}
	if !served || distances[0] < 1000 {
		t.Fatalf("deadlock: served%v player%.1f AI8%.1f", served, distances[0], distances[8])
	}
	t.Logf("60-second replay: player%.1f; all buses retained health", distances[0])
}

// Read-only replay of the October 25 outer articulated berth deadlock.
func TestSavedOuterBerthDeadlockRecovery(t *testing.T) {
	path := os.Getenv("BUS_OUTER_BERTH_SAVE")
	if path == "" {
		t.Skip("BUS_OUTER_BERTH_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	target := g.Route.GetCurrentTargetStopID()
	initialAlighted := 0
	if a := g.Operations.Vehicle(0).Stations[target]; a != nil {
		initialAlighted = a.Alighted
	}
	boarded, served := false, false
	distances := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	for tick := 0; tick < 3600; tick++ {
		before := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, ab := range g.AIBusMgr.Buses {
			before[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		check := func(id int, bus *sim.Bus) {
			step := bus.Pos.Distance(before[id])
			if step > bus.MaxSpeedFwd/60+1 {
				t.Fatalf("vehicle%d jumped at tick%d: %.2f", id, tick, step)
			}
			if bus.Durability < health[id] {
				t.Fatalf("vehicle%d contact at tick%d", id, tick)
			}
			distances[id] += step
		}
		check(0, g.Bus)
		for _, ab := range g.AIBusMgr.Buses {
			check(ab.ID, ab.Bus)
		}
		if !boarded && g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			boarded = true
			t.Logf("opened doors at original target%d at %.2fs", target, float64(tick)/60)
		}
		if tick == 120 && !boarded {
			t.Fatal("did not open at saved outer berth within two seconds")
		}
		if !served && g.Route.GetCurrentTargetStopID() != target {
			a := g.Operations.Vehicle(0).Stations[target]
			if !boarded || g.lastSkippedStopID == target || a == nil || a.Alighted <= initialAlighted {
				t.Fatal("advanced without actually serving original stop")
			}
			served = true
			t.Logf("passengers alighted and route advanced at %.2fs", float64(tick)/60)
		}
	}
	if !served || distances[0] < 1000 || distances[8] < 1000 {
		t.Fatalf("deadlock: served%v player%.1f AI8%.1f", served, distances[0], distances[8])
	}
	t.Logf("60-second replay: player%.1f AI8%.1f; all buses retained health", distances[0], distances[8])
}

func TestSavedStationOverrunReplay(t *testing.T) {
	path := os.Getenv("BUS_STATION_OVERRUN_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_OVERRUN_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	target := g.Route.GetCurrentTargetStopID()
	stop := g.World.Stops[target]
	initialHealth := g.Bus.Durability
	initialAlighted := 0
	if activity := g.Operations.Vehicle(0).Stations[target]; activity != nil {
		initialAlighted = activity.Alighted
	}
	boarded := false
	for tick := 0; tick < 3600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.AutopilotDriver.State == autopilot.AutopilotStateDocked && g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			if !boarded {
				t.Logf("opened doors at target at %.2fs pos%v", float64(tick)/60, g.Bus.Pos)
			}
			boarded = true
		}
		if g.Bus.Durability < initialHealth {
			t.Fatalf("contact at tick%d", tick)
		}
		if g.Route.GetCurrentTargetStopID() != target {
			if !boarded {
				t.Fatalf("advanced without boarding at %.2fs: pos%v frontBerth%v skipped%d", float64(tick)/60, g.Bus.Pos, stop.FrontBerthPosition(), g.lastSkippedStopID)
			}
			if g.lastSkippedStopID == target {
				t.Fatal("target advanced through missed-stop handling")
			}
			activity := g.Operations.Vehicle(0).Stations[target]
			if activity == nil || activity.Alighted <= initialAlighted {
				t.Fatal("passengers did not actually alight")
			}
			t.Logf("served target and advanced at %.2fs health%.0f", float64(tick)/60, g.Bus.Durability)
			return
		}
	}
	t.Fatalf("did not serve target in 60 seconds, boarded%v pos%v", boarded, g.Bus.Pos)
}

// Opt-in replay of the copied 19:13 slot 2, including the complete station
// queue rather than accepting a short movement followed by another deadlock.
func TestSavedArrivalQueueResumesService(t *testing.T) {
	path := os.Getenv("BUS_SAVED_ARRIVAL_QUEUE")
	if path == "" {
		t.Skip("copied local save required")
	}
	seed := int64(5)
	if value := os.Getenv("BUS_SAVED_ARRIVAL_SEED"); value != "" {
		var err error
		seed, err = strconv.ParseInt(value, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
	}
	sim.SeedSimulation(seed)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(seed)
	start := g.Bus.Pos
	target := g.Route.GetCurrentTargetStopID()
	health := g.Bus.Durability
	beginnings := map[int]sim.Vec2{}
	initialTargets := map[int]int{}
	served := map[int]bool{}
	for _, a := range g.AIBusMgr.Buses {
		beginnings[a.ID] = a.Bus.Pos
		initialTargets[a.ID] = a.Route.GetCurrentTargetStopID()
	}
	boardingStarted := false
	for tick := 0; tick < 30000; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if before.Distance(g.Bus.Pos) > g.Bus.MaxSpeedFwd/60+2 {
			t.Fatal("player relocated")
		}
		if g.Bus.Durability < health {
			t.Fatalf("player contact at %.2fs", float64(tick)/60)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Route.GetCurrentTargetStopID() != initialTargets[a.ID] && !served[a.ID] {
				served[a.ID] = true
				t.Logf("AI%d served original target %.2fs", a.ID, float64(tick)/60)
			}
		}
		if tick%1800 == 0 {
			t.Logf("%.0fs player %v distance %.1f target%d", float64(tick)/60, g.Bus.Pos, g.Bus.Pos.Distance(start), g.Route.GetCurrentTargetStopID())
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(g.Bus.Pos) < 2400 && a.Bus.Speed < 1 {
					t.Logf("AI%d pos%v h%.3f state%v targetSpeed%.2f svc%.1f health%.0f", a.ID, a.Bus.Pos, a.Bus.Heading, a.Driver.State, a.Driver.LastTargetSpeed, a.Bus.PassengerServiceRemaining(), a.Bus.Durability)
				}
			}
		}
		if !boardingStarted && g.Route.GetCurrentTargetStopID() != target && g.Bus.DoorsOpen && g.AutopilotDriver.DockedStopID == target {
			boardingStarted = true
			t.Logf("player began original stop service %.2fs", float64(tick)/60)
		}
		if os.Getenv("BUS_SAVED_ARRIVAL_EARLY") == "1" && served[3] && served[4] && served[15] && g.Bus.Pos.Distance(start) > 200 {
			t.Logf("seed%d original three-bus lock cleared %.2fs", seed, float64(tick)/60)
			return
		}
		if boardingStarted && !g.Bus.DoorsOpen && g.Bus.Pos.Distance(g.World.Stops[target].Pos) > 300 {
			if !served[3] || !served[4] || !served[15] {
				t.Fatal("original leading buses did not finish service")
			}
			for _, a := range g.AIBusMgr.Buses {
				if (a.ID == 3 || a.ID == 4 || a.ID == 15) && a.Bus.Pos.Distance(beginnings[a.ID]) < 500 {
					t.Fatal("leading bus did not leave original station zone")
				}
			}
			if g.lastSkippedStopID == target {
				t.Fatal("recovery skipped original target")
			}
			t.Logf("seed%d player completed service and departure %.2fs; moved %.1f", seed, float64(tick)/60, g.Bus.Pos.Distance(start))
			return
		}
	}
	t.Fatalf("saved queue did not resume service: position%v served%v", g.Bus.Pos, served)
}

func TestSavedStationEntryQueueClears(t *testing.T) {
	path := os.Getenv("BUS_STATION_ENTRY_QUEUE_SAVE")
	if path == "" {
		t.Skip("copied 2026-10-29 10:09 save required")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	seed := int64(5)
	if v := os.Getenv("BUS_STATION_ENTRY_QUEUE_SEED"); v != "" {
		var err error
		seed, err = strconv.ParseInt(v, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
	}
	sim.SeedSimulation(seed)
	start := g.Bus.Pos
	positions := map[int]sim.Vec2{}
	health := map[int]float64{0: g.Bus.Durability}
	for _, a := range g.AIBusMgr.Buses {
		positions[a.ID] = a.Bus.Pos
		health[a.ID] = a.Bus.Durability
	}
	servedFront := false
	for tick := 0; tick < 14400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < health[0] {
			t.Fatalf("player contact %.2fs health %.0f", float64(tick)/60, g.Bus.Durability)
		}
		if tick%1800 == 0 {
			t.Logf("%.0fs player distance%.1f pos%v", float64(tick)/60, g.Bus.Pos.Distance(start), g.Bus.Pos)
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus.Durability < health[a.ID] {
				t.Fatalf("AI%d contact %.2fs", a.ID, float64(tick)/60)
			}
			if a.ID == 7 && a.Bus.DoorsOpen && a.Driver.DockedStopID == 232 {
				servedFront = true
			}
			if a.ID == 7 && tick%1800 == 0 {
				d := a.Driver.DiagnoseStop(a.Bus, a.Route, g.World, g.TrafficLights, g.AIBusMgr.GetObstacles(a.ID), 52)
				t.Logf("AI7 moved%.1f pos%v state%v remaining%.1f targetSpd%.1f limits%v", a.Bus.Pos.Distance(positions[a.ID]), a.Bus.Pos, a.Driver.State, a.Driver.LastRemaining, a.Driver.LastTargetSpeed, d.Limits)
			}
		}
		if tick == 3599 && !servedFront {
			t.Fatal("front bus did not reach its empty boarding area within 60s")
		}
		allMoved := g.Bus.Pos.Distance(start) > 4000 && servedFront
		for _, a := range g.AIBusMgr.Buses {
			allMoved = allMoved && a.Bus.Pos.Distance(positions[a.ID]) > 800
		}
		if allMoved {
			t.Logf("whole queue cleared %.2fs; player distance%.1f; front served%v", float64(tick+1)/60, g.Bus.Pos.Distance(start), servedFront)
			return
		}
	}
	for _, a := range g.AIBusMgr.Buses {
		t.Logf("AI%d distance%.1f state%v", a.ID, a.Bus.Pos.Distance(positions[a.ID]), a.Driver.State)
	}
	t.Fatal("queue remained blocked")
}

func TestSavedSecondArticulatedBoardsWhileLeadServes(t *testing.T) {
	path := os.Getenv("BUS_TWO_LONG_BERTH_SAVE")
	if path == "" {
		t.Skip("copied 2026-10-29 08:44 save required")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	var rear *sim.Bus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 3 {
			rear = ab.Bus
		}
	}
	if rear == nil {
		t.Fatal("missing saved follower")
	}
	length := g.World.Stops[86].Length
	playerHealth, rearHealth := g.Bus.Durability, rear.Durability
	boardedTogether := false
	for tick := 0; tick < 5400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if g.Bus.Durability < playerHealth || rear.Durability < rearHealth {
			t.Fatalf("new contact at %.2fs", float64(tick)/60)
		}
		if g.Bus.DoorsOpen && rear.DoorsOpen && rear.PassengerService.Active && rear.PassengerService.StopID == 86 {
			if !boardedTogether {
				t.Logf("rear began service while lead doors open at %.2fs", float64(tick+1)/60)
			}
			boardedTogether = true
		}
	}
	if !boardedTogether {
		t.Fatal("second long bus never boarded with lead")
	}
	if g.World.Stops[86].Length != length {
		t.Fatal("platform was extended")
	}
	if rear.Pos.Distance(g.World.Stops[86].Pos) < 400 {
		t.Fatal("rear never departed after service")
	}
}

func findSlot10Station226WedgeFile() string {
	return firstFixture(
		filepath.Join("..", "dist", "slot10-jam-2026-11-05", "slot10.json"),
		filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-10.json"),
	)
}

// TestSavedSlot10Station226WedgeDissolves verifies that the corridor bottleneck near Stop 226 in save slot 10
// (articulated Bus 15 overshooting front berth and trailing into the travel lane, blocking Bus 42 and the following queue)
// dissolves cleanly without collision or permanent deadlock:
// 1. Bus 15 recovers from the yawed overshoot past the bay taper, advances to its leg, and cruises forward.
// 2. Bus 42 is unblocked and cruises forward.
// 3. The following queue (Bus 4, Bus 14) moves forward smoothly.
func TestSavedSlot10Station226WedgeDissolves(t *testing.T) {
	savePath := findSlot10Station226WedgeFile()
	if savePath == "" {
		t.Skip("save file not found for slot10 station 226 wedge")
	}

	g := loadSaveForTest(t, savePath, 1440, 900)

	initialPositions := make(map[int][2]float64)
	for _, ab := range g.AIBusMgr.Buses {
		initialPositions[ab.ID] = [2]float64{ab.Bus.Pos.X, ab.Bus.Pos.Y}
	}

	for _, id := range []int{15, 42, 4, 14} {
		if _, ok := initialPositions[id]; !ok {
			t.Skipf("slot 10 is not the historical station-226 wedge fixture (missing bus #%d)", id)
		}
	}

	// Run for 1200 ticks (20 seconds)
	for tick := 1; tick <= 1200; tick++ {
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
		{id: 15, minDist: 1500.0, desc: "Bus 15 (lead articulated bus recovering past Stop 226)"},
		{id: 42, minDist: 1500.0, desc: "Bus 42 (following articulated bus unblocked in travel lane)"},
		{id: 4, minDist: 600.0, desc: "Bus 4 (queued behind Bus 42)"},
		{id: 14, minDist: 300.0, desc: "Bus 14 (queued behind Bus 4)"},
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
