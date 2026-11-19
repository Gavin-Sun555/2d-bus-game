package game

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Replay the 2026-11-12 18:46 slot 2 bus 99 capture at the CBD central
// roundabout. Two articulated Line buses waited side by side at the west
// entrance: bus 99 in the curb lane held the long-vehicle reservation, bus 98
// in the inner lane was held for it. Bus 99 still yielded to bus 98 because
// the entry separation test swept 98's straight whole-body envelope along the
// curve, so neither moved and all four approaches queued. Both must enter.
func TestSavedSlot2ParallelArticulatedRoundaboutEntryClears(t *testing.T) {
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184608.434635000-000008-bus99.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 bus 99 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	rb := g.World.Roundabouts[0]
	for _, r := range g.World.Roundabouts {
		if r.Center.Length() < rb.Center.Length() {
			rb = r
		}
	}
	buses := map[int]*traffic.AIBus{}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 98 || ab.ID == 99 {
			buses[ab.ID] = ab
		}
	}
	if len(buses) != 2 {
		t.Fatalf("capture is missing the waiting buses: found %d", len(buses))
	}
	health := map[int]float64{}
	entered := map[int]bool{}
	for id, ab := range buses {
		health[id] = ab.Bus.Durability
	}
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, ab := range buses {
			if ab.Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
			if ab.Bus.Pos.Distance(rb.Center) < rb.Radius {
				entered[id] = true
			}
		}
		if entered[98] && entered[99] {
			return
		}
	}
	for id, ab := range buses {
		if !entered[id] {
			t.Errorf("bus %d never entered the roundabout: pos%v speed%.1f state %v", id, ab.Bus.Pos, ab.Bus.Speed, ab.Driver.State)
		}
	}
}

// Replay the 2026-11-12 18:43 slot 2 capture taken 3 s after loading. Bus 114's
// route turns left at junction (11700, 0) straight out of the curb lane, which
// is also its approach-merge corridor. The merge handed back to the route only
// while the route was within 4 px of that corridor; the articulated bus
// straightened only after the route had begun curving into the turn, so the
// merge kept it on the straight curb lane through the junction, its lane
// offset ran away and it stalled off-route before stop 16. It must turn.
func TestSavedSlot2CurbLaneLeftTurnHandsOffToRoute(t *testing.T) {
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184339.418177000-000001-car141.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 early capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	var bus *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 114 {
			bus = ab
		}
	}
	if bus == nil {
		t.Fatal("capture is missing bus 114")
	}
	health := bus.Bus.Durability
	for tick := 0; tick < 60*20; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if math.Abs(bus.Driver.LaneOffset) > 120 {
			t.Fatalf("lane offset ran away at tick %d: %.1f pos%v", tick, bus.Driver.LaneOffset, bus.Bus.Pos)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus 114 damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X > 11950 && bus.Bus.Pos.Y < 300 {
			return
		}
		if bus.Bus.Pos.Y > 400 {
			t.Fatalf("bus 114 went straight through the junction: pos%v", bus.Bus.Pos)
		}
	}
	t.Fatalf("bus 114 did not complete its left turn: pos%v speed%.1f", bus.Bus.Pos, bus.Bus.Speed)
}

// Replay the 2026-11-12 18:45 slot 2 bus 119 capture. The articulated bus had
// served stop 22 and stopped at the end of its bay with its nose turned 0.2 rad
// toward the road. Its commanded steering swept the trailer 0.1 px into the
// shelter, so it held forever, although a gentler steering was clear. The
// checked departure crawl must take it out without damage.
func TestSavedSlot2ArticulatedBayExitGrazingShelterCrawlsOut(t *testing.T) {
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2", "congestion-20261112-184532.481490000-000007-bus119.json"))
	if p == "" {
		t.Skip("2026-11-12 slot 2 bus 119 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	var bus *traffic.AIBus
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 119 {
			bus = ab
		}
	}
	if bus == nil {
		t.Fatal("capture is missing bus 119")
	}
	start, health := bus.Bus.Pos, bus.Bus.Durability
	for tick := 0; tick < 60*15; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus 119 damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X-start.X > 400 {
			return
		}
	}
	t.Fatalf("bus 119 still held at the bay exit: pos%v speed%.1f", bus.Bus.Pos, bus.Bus.Speed)
}

func slot2EveningCapture(t *testing.T) *Game {
	t.Helper()
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2-evening", "congestion-20261112-192348.245185000-000018-car592.json"))
	if p == "" {
		t.Skip("2026-11-12 19:23 slot 2 congestion capture not found")
	}
	return loadSaveForTest(t, p, 1280, 720)
}

func fleetBus(t *testing.T, g *Game, id int) *traffic.AIBus {
	t.Helper()
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == id {
			return ab
		}
	}
	t.Fatalf("capture is missing bus %d", id)
	return nil
}

// In the 19:23 slot 2 capture two articulated buses had turned right out of
// junction (-15600, -7200) toward terminus 13. Leader 93 overshot its exit
// toward the curb and its junction sweep hit follower 94, stopped against its
// trailer mid-turn; 94 waited for 93. The pinned-leader retreat was limited to
// straight roads, so neither moved. The follower must back off and free it.
func TestSavedSlot2JunctionPinnedLeaderFollowerBacksOff(t *testing.T) {
	g := slot2EveningCapture(t)
	leader, follower := fleetBus(t, g, 93), fleetBus(t, g, 94)
	start := leader.Bus.Pos
	health := [2]float64{leader.Bus.Durability, follower.Bus.Durability}
	for tick := 0; tick < 60*20; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if leader.Bus.Durability < health[0] || follower.Bus.Durability < health[1] {
			t.Fatalf("bus damaged at tick %d", tick)
		}
		if leader.Bus.Pos.Distance(start) > 150 {
			return
		}
	}
	t.Fatalf("bus 93 still pinned: pos%v; follower pos%v", leader.Bus.Pos, follower.Bus.Pos)
}

// In the same capture car 1059 had been pushed off the CBD roundabout by the
// west approach queue. Ring motion follows the actual position, so it kept
// "circulating" through that queue 470-660 px from the center and still
// counted as inside: the long-bus reservation waited for it to drain, holding
// reservation owner bus 82 at the yield line and every entrance with it.
// The displaced car must be replaced, and the owner must enter.
func TestSavedSlot2RoundaboutDrainIgnoresDisplacedCar(t *testing.T) {
	g := slot2EveningCapture(t)
	rb := g.World.Roundabouts[0]
	owner := fleetBus(t, g, 82)
	entered := false
	for tick := 0; tick < 60*30; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if owner.Bus.Pos.Distance(rb.Center) < rb.Radius {
			entered = true
		}
		if tick == 60*2 {
			for _, c := range g.TrafficMgr.Cars {
				if c.InRoundabout && c.RoundaboutCurve == nil && c.Pos.Distance(c.RoundaboutCenter) > rb.Radius+c.Length+24 {
					t.Fatalf("car %d still circulating %.0f px from the center", c.ID, c.Pos.Distance(c.RoundaboutCenter))
				}
			}
		}
		if entered && tick > 60*2 {
			return
		}
	}
	t.Fatalf("reservation owner bus 82 never entered: pos%v", owner.Bus.Pos)
}

func slot2Save(t *testing.T) *Game {
	t.Helper()
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2", "save-02.json"))
	if p == "" {
		t.Skip("2026-11-12 18:46 slot 2 save not found")
	}
	return loadSaveForTest(t, p, 1280, 720)
}

// replayMoves runs g until every listed bus has moved at least distance from
// its start, failing on damage or after limit seconds.
func replayMoves(t *testing.T, g *Game, ids []int, distance, limit float64) {
	t.Helper()
	buses := map[int]*traffic.AIBus{}
	start := map[int]sim.Vec2{}
	health := map[int]float64{}
	for _, id := range ids {
		ab := fleetBus(t, g, id)
		buses[id], start[id], health[id] = ab, ab.Bus.Pos, ab.Bus.Durability
	}
	for tick := 0; tick < int(limit*60); tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		done := true
		for id, ab := range buses {
			if ab.Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
			done = done && ab.Bus.Pos.Distance(start[id]) >= distance
		}
		if done {
			return
		}
	}
	for id, ab := range buses {
		if ab.Bus.Pos.Distance(start[id]) < distance {
			t.Errorf("bus %d still held near %v (moved %.0f)", id, ab.Bus.Pos, ab.Bus.Pos.Distance(start[id]))
		}
	}
}

// In the 18:46 slot 2 save nine Line buses had left stop 45 on the 8-lane
// boulevard at x=-3900 and all needed the curb lane for the next right turn.
// Bus 123, ahead, and bus 126, 230 px behind it straddling two lanes, each
// reserved the lanes the other had to cross, although 126 was stopped; the
// curb-lane leader 122 yielded to 123 and the rest queued behind them. The bus
// ahead must merge first, and the cluster must drive off.
func TestSavedSlot2MergeLeaderGoesBeforeStoppedMergingFollower(t *testing.T) {
	replayMoves(t, slot2Save(t), []int{122, 125, 127, 128, 130}, 300, 120)
}

// In the same save bus 114 had driven straight through its left turn at
// junction (11700, 0) under an older build and stopped across two lanes
// before stop 16, far from its route; buses 109-116 queued behind it. Once a
// freed cluster-A bus also missed its turn the same way. A fleet bus that has
// left its route must re-plan its leg from where it is.
func TestSavedSlot2OffRouteBusReplansLeg(t *testing.T) {
	replayMoves(t, slot2Save(t), []int{114, 109, 112, 113, 116}, 300, 90)
}

// Captured 30 s into a replay of the 18:46 slot 2 save: westbound bus 142
// stopped at x=-1822, steering back out of the next lane, with car 1618
// stopped beside the rear of its trailer. The car waits for the trailer in
// its lane, and every steering the bus has sweeps the trailer into the car.
// The car's deadlock check measured only to the bus's powered section, 147 px
// ahead, so its timer never ran and bus 143 queued behind for good. A car
// beside a trailer must count as jammed and be cleared, and the bus go on.
func TestSavedSlot2CarBesideTrailerIsCleared(t *testing.T) {
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-12-slot2", "wedge-bus142-car1618.json"))
	if p == "" {
		t.Skip("slot 2 bus 142 trailer wedge capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	nearest := func(at sim.Vec2) *traffic.AIBus {
		var best *traffic.AIBus
		for _, ab := range g.AIBusMgr.Buses {
			if best == nil || ab.Bus.Pos.Distance(at) < best.Bus.Pos.Distance(at) {
				best = ab
			}
		}
		if best == nil || best.Bus.Pos.Distance(at) > 5 {
			t.Fatalf("capture has no bus at %v", at)
		}
		return best
	}
	bus := nearest(sim.Vec2{X: -1822, Y: -3643})
	health := bus.Bus.Durability
	for tick := 0; tick < 60*150; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Durability < health {
			t.Fatalf("bus damaged at tick %d", tick)
		}
		if bus.Bus.Pos.X < -2500 {
			return
		}
	}
	t.Fatalf("bus still wedged at %v", bus.Bus.Pos)
}

func slot2MorningCapture(t *testing.T, name string) *Game {
	t.Helper()
	p := slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-13-slot2", name))
	if p == "" {
		t.Skip("2026-11-13 slot 2 congestion capture not found")
	}
	return loadSaveForTest(t, p, 1280, 720)
}

// replayEntersRoundabout fails unless the bus enters the CBD roundabout in time.
func replayEntersRoundabout(t *testing.T, g *Game, id int, limit float64) {
	t.Helper()
	rb := g.World.Roundabouts[0]
	bus := fleetBus(t, g, id)
	for tick := 0; tick < int(limit*60); tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Pos.Distance(rb.Center) < rb.Radius {
			return
		}
	}
	t.Fatalf("bus %d never entered the roundabout: pos%v", id, bus.Bus.Pos)
}

// 07:36 capture, shortly before the CBD roundabout locked up. Articulated
// bus 178, in the south approach's curb lane with a stale "passing" state,
// is about to be sent into the inner lane by the rule that keeps a passing
// bus there near a roundabout, onto cars queued ahead of it. It then owns the
// long-vehicle reservation, so those cars wait for it at the yield line while
// it waits for them, and every entrance is held. It must still get in.
func TestSavedSlot2ReservationOwnerCuttingIntoQueue(t *testing.T) {
	replayEntersRoundabout(t, slot2MorningCapture(t, "congestion-20261113-073654.682166000-000021-car2344.json"), 178, 60)
}

// 07:39 capture of the resulting deadlock: reservation owner 178 is angled
// into the inner lane behind car 1023, which waits at the yield line for
// that same reservation while every entrance is held. The car ahead in the
// lane the stopped owner is turning into must be let in first.
func TestSavedSlot2ReservationOwnerTurningIntoQueuedLane(t *testing.T) {
	replayEntersRoundabout(t, slot2MorningCapture(t, "congestion-20261113-073952.384808000-000036-bus178.json"), 178, 40)
}

// slot2Fixture locates a slot 2 replay capture. These replays run tens of
// simulated seconds each, so -short skips them.
func slot2Fixture(t *testing.T, path string) string {
	t.Helper()
	if testing.Short() {
		t.Skip("slot 2 replay skipped in -short mode")
	}
	return firstFixture(path)
}

// Long buses reserve the whole CBD roundabout. The reservation used to close
// every entrance from the moment a bus was chosen, often 500 px out and
// queued, so over five minutes the ring was held about five times longer than
// a bus spent in it and cars barely got a turn. Entrances now close only when
// the owner reaches its yield line. From the 07:36 capture both cars and
// buses must keep entering.
func TestSavedSlot2RoundaboutReservationLetsCarsFlow(t *testing.T) {
	g := slot2MorningCapture(t, "congestion-20261113-073654.682166000-000021-car2344.json")
	rb := g.World.Roundabouts[0]
	carIn, busIn := map[int]bool{}, map[int]bool{}
	cars, buses := 0, 0
	health := map[int]float64{}
	for _, ab := range g.AIBusMgr.Buses {
		health[ab.ID] = ab.Bus.Durability
	}
	for tick := 0; tick < 60*240; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, c := range g.TrafficMgr.Cars {
			in := c.InRoundabout && c.RoundaboutCenter == rb.Center
			if in && !carIn[c.ID] {
				cars++
			}
			carIn[c.ID] = in
		}
		for _, ab := range g.AIBusMgr.Buses {
			in := ab.Bus.Pos.Distance(rb.Center) < rb.Radius
			if in && !busIn[ab.ID] {
				buses++
			}
			busIn[ab.ID] = in
			if in && ab.Bus.Durability < health[ab.ID] {
				t.Fatalf("bus %d damaged in the roundabout at tick %d", ab.ID, tick)
			}
		}
	}
	// Measured over these 240 s: 78-106 cars and 13-15 buses; holding the
	// entrances from selection let only about 54 cars and 14 buses in.
	if cars < 70 || buses < 10 {
		t.Fatalf("roundabout throughput too low in 240 s: %d cars, %d buses", cars, buses)
	}
}

// Slot 1, 2026-11-13 09:55: Line 2 bus 55 had driven far past its target stop
// 230 under an older build, down the western edge road and beyond its end at
// the map's south-west corner. With no road under it no leg can be re-planned,
// and alone in empty space it never counted as jammed, so it waited forever.
// It must be returned to its line and resume service.
func TestSavedSlot1BusOffRoadNetworkReturnsToLine(t *testing.T) {
	g := loadSaveForTest(t, slot2Fixture(t, filepath.Join("..", "dist", "debug-2026-11-13-slot1-bus55", "congestion-20261113-095556.306240000-000001-bus55.json")), 1280, 720)
	bus := fleetBus(t, g, 55)
	for tick := 0; tick < 60*60; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if bus.Bus.Pos.Y < 28800 && bus.Route.CurrentIndex >= 2 {
			return
		}
	}
	t.Fatalf("bus 55 did not resume service: pos%v state %v stop index %d", bus.Bus.Pos, bus.Driver.State, bus.Route.CurrentIndex)
}

// Slot 1 runs its fleet at a single speed (120–120 km/h), so no route bus
// may overtake and each queues behind the buses already at a stop. The
// player's 120 km/h standard autopilot must do the same instead of boarding
// from the adjacent lane beside them as Berserk does.
func TestSavedSlot1StandardAutopilotQueuesLikeFleetAtStops(t *testing.T) {
	path := slot2Fixture(t, "../dist/save1-auto-2026-11-13-1040.json")
	sim.SeedSimulation(3)
	g := loadSaveForTest(t, path, 1280, 720)
	g.State = StatePlaying
	g.Settings.AutopilotEnabled, g.Settings.AutopilotBerserk = true, false
	lw := g.Settings.GetLaneWidth()
	served := map[int]bool{}
	for f := 0; f < 60*120 && len(served) < 3; f++ {
		_ = g.Update()
		if g.AutopilotDriver.BerserkMode {
			t.Fatal("standard autopilot overtakes although no fleet bus on a single-speed line may")
		}
		if !g.Bus.DoorsOpen {
			continue
		}
		for _, id := range []int{188, 186, 184} {
			st := g.World.Stops[id]
			if st.ContainsBus(g.Bus) {
				served[id] = true
			} else if st.ContainsBusInOuterLane(g.Bus, lw) {
				t.Fatalf("t=%.1f boarded at stop %d from the adjacent lane at %v", float64(f)/60, id, g.Bus.Pos)
			}
		}
	}
	if !served[188] || !served[186] || !served[184] {
		s := g.AutopilotDriver.Snapshot()
		t.Logf("player pos%v h%.3f state%v from%d to%d origin%+v target%v speed%.1f remaining%.1f", g.Bus.Pos, g.Bus.Heading, g.AutopilotDriver.State, s.PathFromID, s.PathToID, s.OffRouteOrigin, s.Public.LastPursuitPt, s.Public.LastTargetSpeed, s.Public.LastRemaining)
		t.Fatalf("served %v, want stops 188, 186 and 184 from their bays", served)
	}
}

// Save 2 (2026-11-16): the player's long bus finished its lane change into
// the left-turn lane 2px over the centerline. The lane guard stopped it and
// then crawled it at 20px/s along the line for 500px. It must steer back at
// normal speed instead.
func TestSavedSlot2LeftLaneChangeOverCenterlineKeepsDriving(t *testing.T) {
	path := slot2Fixture(t, "../dist/slot2-left-lane-stop-2026-11-16.json")
	sim.SeedSimulation(3)
	g := loadSaveForTest(t, path, 1440, 900)
	g.State = StatePlaying
	start := g.Bus.Pos
	lw := g.Settings.GetLaneWidth()
	center, _, _, _ := sim.CurrentRoadInfo(g.World, g.Bus.Pos, math.Pi, lw)
	if start.Y-center <= -g.Bus.Width/2 || start.Y-center > -g.Bus.Width/2+4 {
		t.Fatalf("fixture no longer straddles the centerline: lateral %.1f", start.Y-center)
	}
	back := false
	for f := 0; f < 120; f++ {
		_ = g.Update()
		// Before the junction, the body returns fully to its own side.
		if g.Bus.Pos.X > -15000 && g.Bus.Pos.Y-center < -g.Bus.Width/2 {
			back = true
		}
	}
	if moved := start.X - g.Bus.Pos.X; moved < 80 {
		t.Fatalf("bus moved only %.0fpx in 2s at lateral %.1f (speed %.1f)", moved, g.Bus.Pos.Y-center, g.Bus.Speed)
	}
	if !back {
		t.Fatal("bus stayed over the centerline up to the junction")
	}
}

// Save 2 (2026-11-16 09:14): long buses 147 and 153 stood touching beside
// stop 31, 153 angled with its nose against 147's side. Every move or
// boarding check of either rejected the contact, so both waited until the
// fleet reset. The more angled bus now backs out and the other waits for it.
func TestSavedSlot2SideBySideBusWedgeAtStop31Clears(t *testing.T) {
	path := slot2Fixture(t, "../dist/debug-2026-11-16-slot2/congestion-20261116-091403.066467000-000005-bus153.json")
	sim.SeedSimulation(9)
	g := loadSaveForTest(t, path, 1440, 900)
	g.State = StatePlaying
	a, b := fleetBus(t, g, 147), fleetBus(t, g, 153)
	touching := func() bool {
		for _, part := range a.Bus.BodyParts() {
			if part.HalfLen > 0 {
				part.HalfLen++
				part.HalfWid++
				if hit, _, _ := b.Bus.CollidesBody(part); hit {
					return true
				}
			}
		}
		return false
	}
	if !touching() {
		t.Fatal("fixture no longer has the side-by-side contact")
	}
	served := false
	for f := 0; f < 60*90 && !served; f++ {
		_ = g.Update()
		served = (a.Bus.DoorsOpen && a.Driver.DockedStopID == 31) || (b.Bus.DoorsOpen && b.Driver.DockedStopID == 31)
	}
	if !served {
		for _, x := range []int{147, 153} {
			a := fleetBus(t, g, x)
			s := a.Driver.Snapshot()
			t.Logf("bus%d from%d to%d state%v origin%+v target%v speed%.1f remaining%.1f", x, s.PathFromID, s.PathToID, s.Public.State, s.OffRouteOrigin, s.Public.LastPursuitPt, s.Public.LastTargetSpeed, s.Public.LastRemaining)
		}
		t.Fatalf("neither bus served stop 31 in 90s; touching %v, 147 at %v, 153 at %v", touching(), a.Bus.Pos, b.Bus.Pos)
	}
}
