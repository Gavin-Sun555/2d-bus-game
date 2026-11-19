package game

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Opt-in replay from an isolated copy of the October 29 evening save.
func TestSavedArticulatedStationQueueRecovery(t *testing.T) {
	path := os.Getenv("BUS_ARTICULATED_JAM_SAVE")
	if path == "" {
		t.Skip("BUS_ARTICULATED_JAM_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprint(seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := loadSaveForTest(t, path, 1280, 720)
			type progress struct {
				last              sim.Vec2
				distance          float64
				afterAdvance      float64
				initialTarget     int
				advanced, served  bool
				health            float64
				longestWait, wait int
			}
			ps := map[int]*progress{}
			for _, a := range g.AIBusMgr.Buses {
				ps[a.ID] = &progress{last: a.Bus.Pos, initialTarget: a.Route.GetCurrentTargetStopID(), health: a.Bus.Durability}
			}
			playerHealth := g.Bus.Durability
			for tick := 0; tick < 18000; tick++ {
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < playerHealth {
					t.Fatalf("player contact tick%d", tick)
				}
				for _, a := range g.AIBusMgr.Buses {
					p := ps[a.ID]
					if p == nil {
						t.Fatal("unexpected replacement during replay")
					}
					step := a.Bus.Pos.Distance(p.last)
					if step > a.Bus.MaxSpeedFwd/60+2 {
						t.Fatalf("AI%d relocated tick%d by%g", a.ID, tick, step)
					}
					p.distance += step
					if p.advanced {
						p.afterAdvance += step
					}
					p.last = a.Bus.Pos
					if a.Bus.Durability < p.health {
						t.Fatalf("AI%d contact tick%d", a.ID, tick)
					}
					if a.Route.GetCurrentTargetStopID() != p.initialTarget {
						p.advanced = true
					}
					if a.Driver.State == autopilot.AutopilotStateDocked && a.Driver.DockedStopID == 184 && a.Bus.DoorsOpen {
						p.served = true
					}
					if a.ID <= 5 {
						if math.Abs(a.Bus.Speed) < .5 && a.Driver.State != autopilot.AutopilotStateDocked {
							p.wait++
							p.longestWait = max(p.longestWait, p.wait)
						} else {
							p.wait = 0
						}
						if a.Bus.CheckBoundaries(g.World, 52).Zone == sim.ZoneHardBarrier {
							t.Fatalf("AI%d boundary tick%d", a.ID, tick)
						}
					}
				}
				if tick == 10799 {
					for _, a := range g.AIBusMgr.Buses {
						if a.ID == 3 {
							t.Logf("AI3 at180s state%v doors%v dwell%.2f service%.2f target%d", a.Driver.State, a.Bus.DoorsOpen, a.Driver.DwellTimer, a.Bus.PassengerServiceRemaining(), a.Route.GetCurrentTargetStopID())
						}
					}
				}
				if tick >= 10799 {
					clear := true
					for id := 1; id <= 5; id++ {
						if !ps[id].advanced || ps[id].distance < 1000 || ps[id].afterAdvance < 300 {
							clear = false
						}
					}
					if clear {
						break
					}
				}
			}
			for id := 1; id <= 5; id++ {
				p := ps[id]
				t.Logf("AI%d distance%.0f target advanced%v served184%v longest non-service wait%.2fs", id, p.distance, p.advanced, p.served, float64(p.longestWait)/60)
				if !p.advanced || p.distance < 1000 || p.afterAdvance < 300 || p.longestWait > 7200 {
					t.Errorf("AI%d original queue did not clear", id)
				}
			}
			for _, id := range []int{2, 3, 4} {
				if !ps[id].served {
					t.Errorf("AI%d skipped a feasible station", id)
				}
			}
		})
	}
}

func TestSavedSlot1MassJamResolves(t *testing.T) {
	savePath := firstFixture(filepath.Join("..", "dist", "slot1-jam-2026-11-03", "slot1.json"))
	if savePath == "" {
		t.Skip("slot1 fixture not found")
	}

	g := loadSaveForTest(t, savePath, 1280, 720)

	// Capture initial positions of the queue head
	b47 := g.AIBusMgr.Buses[46] // ID 47
	b41 := g.AIBusMgr.Buses[40] // ID 41
	b46 := g.AIBusMgr.Buses[45] // ID 46
	b45 := g.AIBusMgr.Buses[44] // ID 45
	b42 := g.AIBusMgr.Buses[41] // ID 42

	init47X := b47.Bus.Pos.X
	init41X := b41.Bus.Pos.X
	init45Y := b45.Bus.Pos.Y
	init42Y := b42.Bus.Pos.Y

	// Count how many buses in the vertical avenue are stopped initially
	initialStoppedBuses := 0
	for _, b := range g.AIBusMgr.Buses {
		if math.Abs(b.Bus.Speed) < 2.0 && b.Bus.Pos.Y < -300 && b.Bus.Pos.X < -62000 {
			initialStoppedBuses++
		}
	}
	t.Logf("Initial stopped buses in vertical avenue: %d", initialStoppedBuses)

	stoppedInAvenue := func() int {
		n := 0
		for _, b := range g.AIBusMgr.Buses {
			if math.Abs(b.Bus.Speed) < 2.0 && b.Bus.Pos.Y < -300 && b.Bus.Pos.X < -62000 {
				n++
			}
		}
		return n
	}
	// Simulate 45 seconds. The jam must drain through the signal cycles; a
	// later red phase may legitimately re-form a queue, so track the least
	// congested moment rather than the count at the final tick.
	fewestStopped := initialStoppedBuses
	for tick := 1; tick <= 60*45; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d: %v", tick, err)
		}
		fewestStopped = min(fewestStopped, stoppedInAvenue())
	}

	t.Logf("After 45s: Bus47 X=%.1f (was %.1f), Bus41 X=%.1f (was %.1f), Bus46 Pos=(%.1f, %.1f), Bus45 Pos=(%.1f, %.1f), Bus42 Pos=(%.1f, %.1f)",
		b47.Bus.Pos.X, init47X, b41.Bus.Pos.X, init41X, b46.Bus.Pos.X, b46.Bus.Pos.Y, b45.Bus.Pos.X, b45.Bus.Pos.Y, b42.Bus.Pos.X, b42.Bus.Pos.Y)

	// Verify Bus 47 and Bus 41 moved significantly eastward
	if b47.Bus.Pos.X <= init47X+1000.0 {
		t.Fatalf("Bus 47 did not advance: X=%.1f", b47.Bus.Pos.X)
	}
	if b41.Bus.Pos.X <= init41X+1000.0 {
		t.Fatalf("Bus 41 did not advance: X=%.1f", b41.Bus.Pos.X)
	}

	// Verify Bus 46 entered the junction and turned east
	if b46.Bus.Pos.X <= -61000.0 {
		t.Fatalf("Bus 46 did not exit junction eastbound: Pos=%v", b46.Bus.Pos)
	}

	// Verify Bus 45 advanced through or into junction
	if b45.Bus.Pos.Y <= init45Y+300.0 {
		t.Fatalf("Bus 45 did not advance: Y=%.1f", b45.Bus.Pos.Y)
	}

	// Verify Bus 42 advanced
	if b42.Bus.Pos.Y <= init42Y+200.0 {
		t.Fatalf("Bus 42 did not advance: Y=%.1f", b42.Bus.Pos.Y)
	}

	// Verify the stopped queue on the vertical road drained significantly
	t.Logf("Fewest stopped buses in vertical avenue: %d (started with %d, %d at the end)", fewestStopped, initialStoppedBuses, stoppedInAvenue())
	if fewestStopped >= initialStoppedBuses/2 {
		t.Fatalf("Queue did not drain adequately: fewest %d vs initial %d", fewestStopped, initialStoppedBuses)
	}
}

// Replay Save Slot 1 from 2026-10-30 where articulated bus AI #1 was jammed
// entering Stop 228 (街区229·南行站) with 13 queued buses behind it.
func TestSavedSlot1Oct30JamRecovery(t *testing.T) {
	// Replays must not inherit random traffic from earlier tests.
	sim.SeedSimulation(5)
	savePath := firstFixture(filepath.Join("..", "dist", "slot1-jam-2026-10-30-1045", "slot1.json"))
	if savePath == "" {
		wd, _ := os.Getwd()
		t.Skipf("save file not found, wd=%s", wd)
	}

	absPath, err := filepath.Abs(savePath)
	if err != nil {
		t.Fatal(err)
	}

	g := loadSaveForTest(t, absPath, 1280, 720)

	type busTracker struct {
		lastPos           sim.Vec2
		distTraveled      float64
		initTarget        int
		reachedBoarding   bool
		departedStop      bool
		initialDurability float64
	}

	trackers := make(map[int]*busTracker)
	for _, a := range g.AIBusMgr.Buses {
		trackers[a.ID] = &busTracker{
			lastPos:           a.Bus.Pos,
			initTarget:        a.Route.GetCurrentTargetStopID(),
			initialDurability: a.Bus.Durability,
		}
	}

	playerInitHealth := g.Bus.Durability

	// Run 4200 ticks (70 seconds)
	for tick := 0; tick < 4200; tick++ {
		if err := g.Update(); err != nil {
			t.Fatalf("tick %d Update failed: %v", tick, err)
		}

		if g.Bus.Durability < playerInitHealth {
			t.Fatalf("player bus durability decreased at tick %d: %f < %f", tick, g.Bus.Durability, playerInitHealth)
		}

		for _, a := range g.AIBusMgr.Buses {
			tr := trackers[a.ID]
			if tr == nil {
				continue
			}

			step := a.Bus.Pos.Distance(tr.lastPos)
			if step > a.Bus.MaxSpeedFwd/60+10 {
				t.Fatalf("AI #%d moved too far in single tick (%f units) at tick %d", a.ID, step, tick)
			}
			tr.distTraveled += step
			tr.lastPos = a.Bus.Pos

			if a.Bus.Durability < tr.initialDurability-0.01 {
				t.Fatalf("AI #%d sustained damage at tick %d", a.ID, tick)
			}

			if a.Bus.DoorsOpen {
				tr.reachedBoarding = true
			}
			if a.Route.GetCurrentTargetStopID() != tr.initTarget {
				tr.departedStop = true
			}
		}
	}

	// AI #1 was the blocker at Stop 228. It must have recovered and cruised ahead.
	tr1 := trackers[1]
	if tr1 == nil || tr1.distTraveled < 500 {
		t.Fatalf("AI #1 did not recover and move forward: dist=%v", tr1)
	}
	if !tr1.departedStop {
		t.Fatalf("AI #1 did not advance target stop from Stop 228")
	}

	// AI #2 was queued behind Stop 228. It must have served Stop 228 and departed.
	tr2 := trackers[2]
	if tr2 == nil || !tr2.reachedBoarding {
		t.Fatalf("AI #2 did not reach boarding state at Stop 228")
	}
	if !tr2.departedStop {
		t.Fatalf("AI #2 did not depart Stop 228 after boarding")
	}
	if tr2.distTraveled < 300 {
		t.Fatalf("AI #2 did not cruise after departure: dist=%f", tr2.distTraveled)
	}
}

// TestSaveSlot1TerminalModelSwitchAndDeparture verifies that when Save Slot 1 is loaded
// (where the player's bus reached starting terminus Stop 226 with a pending model change to
// an articulated bus, but had previously been held in TerminalReplacementHold because the route
// index had already advanced upon door opening), the terminal replacement cleanly succeeds,
// the model switches to the articulated bus, the hold releases, passengers board, and the bus
// departs toward the next stop.
func TestSaveSlot1TerminalModelSwitchAndDeparture(t *testing.T) {
	candidates := []string{
		filepath.Join("..", "dist", "slot1-jam-2026-10-30-1224", "slot1.json"),
	}
	var savePath string
	for _, c := range candidates {
		g := NewGame(1280, 720)
		g.savePathOverride = c
		if err := g.LoadGame(); err == nil {
			savePath = c
			break
		}
	}
	if savePath == "" {
		t.Skip("slot1 terminal model save file not found")
	}
	g := loadSaveForTest(t, savePath, 1280, 720)

	p := g.Bus
	ad := g.AutopilotDriver

	t.Logf("INITIAL: Model=%v PendingModel=%v TerminalHold=%v Pos=%v Spd=%.2f DoorsOpen=%v PassCount=%d",
		p.Model, p.PendingModel, p.TerminalReplacementHold, p.Pos, p.Speed, p.DoorsOpen, p.PassengerCount)

	modelSwitched := false
	departed := false

	for tick := 1; tick <= 600; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}

		if p.Model == sim.BusArticulated {
			if !modelSwitched {
				modelSwitched = true
				t.Logf("Tick %d: MODEL SWITCHED TO ARTICULATED! Pos=%v TerminalHold=%v DoorsOpen=%v PassCount=%d",
					tick, p.Pos, p.TerminalReplacementHold, p.DoorsOpen, p.PassengerCount)
			}
		}

		if modelSwitched && p.Speed > 10.0 {
			if !departed {
				departed = true
				t.Logf("Tick %d: BUS DEPARTED! Spd=%.2f Pos=%v State=%v TargetStop=%v",
					tick, p.Speed, p.Pos, ad.State, g.Route.GetCurrentTargetStopID())
			}
		}

		if tick%60 == 0 {
			t.Logf("Tick %d: Spd=%.2f Pos=%v State=%v TerminalHold=%v DoorsOpen=%v DoorProg=%.2f TargetStop=%v",
				tick, p.Speed, p.Pos, ad.State, p.TerminalReplacementHold, p.DoorsOpen, p.DoorProgress, g.Route.GetCurrentTargetStopID())
		}
	}

	if !modelSwitched {
		t.Fatalf("bus never switched to articulated model, current model: %v pending: %v", p.Model, p.PendingModel)
	}
	if !departed {
		t.Fatalf("bus failed to depart from station after switching, state: %v spd: %.2f hold: %v doors: %v",
			ad.State, p.Speed, p.TerminalReplacementHold, p.DoorsOpen)
	}
}

// Replay a 2026-11-12 slot 1 congestion capture: articulated bus 67 nosed
// into the same right turn beside articulated bus 90, a lane to its left.
// Each blocked the other and the whole eastbound queue waited. The follower
// must back off so the leader can turn, without any contact.
func TestSavedSlot1ParallelRightTurnWedgeClears(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "slot1-bus67-68-2026-11-12", "congestion-20261112-074543.404422000-000001-bus67.json"))
	if p == "" {
		t.Skip("slot1 bus67 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	sim.SeedSimulation(1)
	buses := map[int]*traffic.AIBus{}
	for _, ab := range g.AIBusMgr.Buses {
		buses[ab.ID] = ab
	}
	ids := []int{67, 90, 88}
	start, health := map[int]sim.Vec2{}, map[int]float64{}
	for _, id := range ids {
		if buses[id] == nil {
			t.Fatalf("capture is missing bus %d", id)
		}
		start[id], health[id] = buses[id].Bus.Pos, buses[id].Bus.Durability
	}
	for tick := 0; tick < 60*30; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, id := range ids {
			if buses[id].Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
		}
	}
	for _, id := range ids {
		if moved := buses[id].Bus.Pos.Distance(start[id]); moved < 400 {
			t.Errorf("bus %d moved only %.0f px: still wedged at %v", id, moved, buses[id].Bus.Pos)
		}
	}
}
