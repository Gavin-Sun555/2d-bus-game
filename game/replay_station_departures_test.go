package game

import (
	"fmt"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Read only a copied save. The stationary car beside AI 2's rear used to be
// exempted as a lawful queue forever, blocking both AI 2 and the player.
func TestSavedDepartureDeadlockRecovery(t *testing.T) {
	path := os.Getenv("BUS_DEPARTURE_JAM_SAVE")
	if path == "" {
		t.Skip("BUS_DEPARTURE_JAM_SAVE not set")
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, path, 1280, 720)
	sim.SeedSimulation(5)
	target := g.Route.GetCurrentTargetStopID()
	traveled := map[int]float64{}
	health := map[int]float64{0: g.Bus.Durability}
	contacts := map[int]int{}
	motionTimes := map[int]float64{}
	leftLaneAt := -1.
	aiLeftLaneAt := -1.
	source := g.World.Stops[g.AutopilotDriver.Snapshot().PathFromID]
	u := sim.UnitFromAngle(source.Heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	for _, a := range g.AIBusMgr.Buses {
		health[a.ID] = a.Bus.Durability
	}
	firstMotion, servedAt := -1., -1.
	distanceAtService := 0.
	extended := os.Getenv("BUS_DEPARTURE_JAM_EXTENDED") == "1"
	doors := false
	for tick := 0; tick < 21600; tick++ {
		positions := map[int]sim.Vec2{0: g.Bus.Pos}
		for _, a := range g.AIBusMgr.Buses {
			positions[a.ID] = a.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		check := func(id int, b *sim.Bus) {
			step := b.Pos.Distance(positions[id])
			if step > b.MaxSpeedFwd/60+5 {
				t.Fatalf("vehicle%d jumped at tick%d", id, tick)
			}
			if b.Durability < health[id] {
				contacts[id]++
				t.Logf("vehicle%d contact at %.2fs: %.1f->%.1f", id, float64(tick)/60, health[id], b.Durability)
				health[id] = b.Durability
				if contacts[id] > 1 || b.Durability < 94 {
					t.Fatalf("vehicle%d repeated/heavy contact: contacts%d health%.1f", id, contacts[id], b.Durability)
				}
			}
			if id == 2 && aiLeftLaneAt < 0 {
				relative := b.Pos.Sub(source.RoadCenter)
				if relative.Dot(u) > 0 && relative.Dot(u) < 800 && relative.Dot(r) > b.Width/2 && relative.Dot(r) < g.Settings.GetLaneWidth()*.75 && sim.UnitFromAngle(b.Heading).Dot(u) > .94 {
					aiLeftLaneAt = float64(tick) / 60
					t.Logf("AI 2 entered forward left-turn lane %.2fs", aiLeftLaneAt)
				}
			}
			traveled[id] += step
			if _, moving := motionTimes[id]; !moving && traveled[id] > 20 {
				motionTimes[id] = float64(tick) / 60
				if id == 2 {
					blockingCarPresent := false
					for _, car := range g.TrafficMgr.Cars {
						blockingCarPresent = blockingCarPresent || car.ID == 65
					}
					if !blockingCarPresent {
						t.Fatal("AI departure relied on deleting original rear car")
					}
				}
			}
		}
		check(0, g.Bus)
		for _, a := range g.AIBusMgr.Buses {
			check(a.ID, a.Bus)
		}
		relative := g.Bus.Pos.Sub(source.RoadCenter)
		if leftLaneAt < 0 && relative.Dot(u) > 0 && relative.Dot(u) < 800 && relative.Dot(r) > g.Bus.Width/2 && relative.Dot(r) < g.Settings.GetLaneWidth()*.75 && sim.UnitFromAngle(g.Bus.Heading).Dot(u) > .94 {
			leftLaneAt = float64(tick) / 60
			t.Logf("entered forward left-turn lane %.2fs", leftLaneAt)
		}
		if firstMotion < 0 && traveled[0] > 20 {
			firstMotion = float64(tick) / 60
			t.Logf("player resumed %.2fs", firstMotion)
		}
		if g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
			doors = true
		}
		if servedAt < 0 && g.Route.GetCurrentTargetStopID() != target {
			if !doors || g.lastSkippedStopID == target {
				t.Fatal("advanced without serving target")
			}
			distanceAtService = traveled[0]
			servedAt = float64(tick) / 60
			t.Logf("original target served %.2fs", servedAt)
		}
		// Qualify this saved departure through real service at its next stop
		// and a subsequent 300px departure. An opt-in longer soak retains the
		// separate post-recovery contact reproduction; do not conflate it with
		// the original stationary deadlock.
		if !extended && servedAt >= 0 && traveled[0]-distanceAtService > 300 {
			t.Logf("cleared original target after service at %.2fs", float64(tick)/60)
			break
		}
	}
	if firstMotion < 0 || firstMotion > 3 || leftLaneAt < 0 || leftLaneAt > 30 || motionTimes[2] > 1 || aiLeftLaneAt < 0 || aiLeftLaneAt > 20 || servedAt < 0 || servedAt > 65 || traveled[0] < 2000 || traveled[2] < 1000 {
		t.Fatalf("recovery incomplete: first%.2f served%.2f distance%v", firstMotion, servedAt, traveled)
	}
	t.Logf("replay: travel%v contacts%v final health%v first motion%v", traveled, contacts, health, motionTimes)
}

// Read-only copied saves; vary future ambient traffic without altering the
// original scene. Distinguish the original departure lock from later red lights
// and passenger-dependent station service.
func TestSavedDepartureSlotRecovery(t *testing.T) {
	path := os.Getenv("BUS_STATION_DEPARTURE_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_DEPARTURE_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprintf("seed%d", seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := loadSaveForTest(t, path, 1280, 720)
			sim.SeedSimulation(seed)
			origin := g.Bus.Pos
			health := g.Bus.Durability
			var follower *sim.Bus
			nearest := math.Inf(1)
			followerID := -1
			forward := sim.UnitFromAngle(g.Bus.Heading)
			for _, a := range g.AIBusMgr.Buses {
				d := a.Bus.Pos.Sub(origin)
				if d.Dot(forward) < -8 && d.Length() < nearest {
					nearest = d.Length()
					follower = a.Bus
					followerID = a.ID
				}
			}
			if follower == nil {
				t.Fatal("missing saved rear follower")
			}
			rearOrigin, rearHealth := follower.Pos, follower.Durability
			playerMoved, rearMoved := -1., -1.
			for tick := 0; tick < 1800; tick++ {
				before := g.Bus.Pos
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Pos.Distance(before) > g.Bus.MaxSpeedFwd/60+1 {
					t.Fatal("recovery relocated player bus")
				}
				elapsed := float64(tick+1) / 60
				if playerMoved < 0 && g.Bus.Pos.Distance(origin) > 20 {
					playerMoved = elapsed
				}
				if rearMoved < 0 && follower.Pos.Distance(rearOrigin) > 20 {
					rearMoved = elapsed
				}
				if g.Bus.Durability < health {
					t.Fatalf("player contact after %.2fs", elapsed)
				}
				if follower.Pos.Distance(rearOrigin) < 700 && follower.Durability < rearHealth {
					t.Fatalf("AI%d contact in original station zone after %.2fs", followerID, elapsed)
				}
				for _, part := range follower.BodyParts() {
					if part.HalfLen > 0 {
						if hit, _, depth := g.Bus.CollidesBody(part); hit && depth > 1e-6 {
							t.Fatalf("physical overlap %.2fs depth%g", elapsed, depth)
						}
					}
				}
			}
			if playerMoved < 0 || playerMoved > 5 || rearMoved < 0 || rearMoved > 15 || g.Bus.Pos.Distance(origin) < 200 {
				t.Fatalf("player%.2fs AI%d %.2fs player distance%g", playerMoved, followerID, rearMoved, g.Bus.Pos.Distance(origin))
			}
			t.Logf("player resumed%.2fs AI%d resumed%.2fs; no contact through30s", playerMoved, followerID, rearMoved)
		})
	}
}

// Optional end-to-end replay of the read-only October 28 slot-one copy.
func TestSavedParallelBusDeadlock(t *testing.T) {
	path := os.Getenv("BUS_PARALLEL_SAVE")
	if path == "" {
		t.Skip("BUS_PARALLEL_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		sim.SeedSimulation(seed)
		g := loadSaveForTest(t, path, 1280, 720)
		sim.SeedSimulation(seed)
		var lead *sim.Bus
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == 15 {
				lead = a.Bus
			}
		}
		if lead == nil {
			t.Fatal("missing saved leading AI 15")
		}
		initialPlayer, initialLead := g.Bus.Pos, lead.Pos
		healthPlayer, healthLead := g.Bus.Durability, lead.Durability
		target := g.Route.GetCurrentTargetStopID()
		firstLead, firstPlayer, served := -1., -1., -1.
		doors := false
		for tick := 0; tick < 3600; tick++ {
			beforePlayer, beforeLead := g.Bus.Pos, lead.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			elapsed := float64(tick) / 60
			if g.Bus.Pos.Distance(beforePlayer) > g.Bus.MaxSpeedFwd/60+1 || lead.Pos.Distance(beforeLead) > lead.MaxSpeedFwd/60+1 {
				t.Fatal("recovery teleported a bus")
			}
			if g.Bus.Durability < healthPlayer || lead.Durability < healthLead {
				t.Fatalf("seed%d recovery caused contact at %.2fs: player %.1f/%.1f lead %.1f/%.1f", seed, elapsed, g.Bus.Durability, healthPlayer, lead.Durability, healthLead)
			}
			for _, own := range g.Bus.BodyParts() {
				for _, other := range lead.BodyParts() {
					if own.HalfLen <= 0 || other.HalfLen <= 0 {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(own, other); hit && depth > 1e-6 {
						t.Fatalf("seed%d physical bodies overlap at %.2fs: depth %g", seed, elapsed, depth)
					}
				}
			}
			if firstLead < 0 && lead.Pos.Distance(initialLead) > 20 {
				firstLead = elapsed
			}
			if firstPlayer < 0 && g.Bus.Pos.Distance(initialPlayer) > 20 {
				firstPlayer = elapsed
			}
			if g.Bus.DoorsOpen && g.AutopilotDriver.DockedStopID == target {
				doors = true
			}
			if served < 0 && g.Route.GetCurrentTargetStopID() != target {
				if !doors || g.lastSkippedStopID == target {
					t.Fatal("recovery skipped original station")
				}
				served = elapsed
			}
		}
		if firstLead < 0 || firstLead > 3 || firstPlayer < 0 || firstPlayer > 8 || served < 0 || served > 20 || g.Bus.Pos.Distance(initialPlayer) < 2000 || lead.Pos.Distance(initialLead) < 500 {
			t.Fatalf("seed%d lead%.2f player%.2f served%.2f final%v", seed, firstLead, firstPlayer, served, g.Bus.Pos)
		}
		t.Logf("seed%d AI15 resumed%.2fs player%.2fs served%.2fs; no contact through60s", seed, firstLead, firstPlayer, served)
	}
}

func TestSavedSidecarSceneResumesStationService(t *testing.T) {
	path := os.Getenv("BUS_SIDECAR_SAVE")
	if path == "" {
		t.Skip("copied local save required")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprintf("seed%d", seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := loadSaveForTest(t, path, 1280, 720)
			sim.SeedSimulation(seed)
			start, target, health := g.Bus.Pos, g.Route.GetCurrentTargetStopID(), g.Bus.Durability
			var follower *sim.Bus
			var followRoute *sim.Route
			for _, a := range g.AIBusMgr.Buses {
				if a.ID == 4 {
					follower, followRoute = a.Bus, a.Route
				}
			}
			if follower == nil {
				t.Fatal("missing saved follower")
			}
			followStart, followTarget, followHealth := follower.Pos, followRoute.GetCurrentTargetStopID(), follower.Durability
			playerMoved, followerMoved := -1., -1.
			boarded, left := false, false
			carExists := func(id int) bool {
				for _, c := range g.TrafficMgr.Cars {
					if c.ID == id {
						return true
					}
				}
				return false
			}
			for tick := 0; tick < 9000; tick++ {
				before := g.Bus.Pos
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				elapsed := float64(tick+1) / 60
				if g.Bus.Pos.Distance(before) > g.Bus.MaxSpeedFwd/60+2 {
					t.Fatal("player relocated")
				}
				if g.Bus.Durability < health {
					t.Fatalf("recovery contact %.2fs", elapsed)
				}
				if follower.Pos.Distance(followStart) < 900 && follower.Durability < followHealth {
					t.Fatalf("follower contact %.2fs", elapsed)
				}
				if playerMoved < 0 && g.Bus.Pos.Distance(start) > 20 {
					playerMoved = elapsed
					if !carExists(41913) {
						t.Fatal("player recovery relied on deleting original sidecar")
					}
				}
				if followerMoved < 0 && follower.Pos.Distance(followStart) > 20 {
					followerMoved = elapsed
					if !carExists(41727) {
						t.Fatal("follower recovery relied on deleting original sidecar")
					}
				}
				if g.AutopilotDriver.DockedStopID == target && g.Bus.DoorsOpen {
					boarded = true
				}
				if boarded && !g.Bus.DoorsOpen && g.Route.GetCurrentTargetStopID() != target && g.Bus.Pos.Distance(g.World.Stops[target].Pos) > 200 {
					left = true
				}
				if left && followRoute.GetCurrentTargetStopID() != followTarget {
					if playerMoved < 0 || playerMoved > 5 || followerMoved < 0 || followerMoved > 8 {
						t.Fatalf("slow/unresolved original lock player%.2f follower%.2f", playerMoved, followerMoved)
					}
					if g.lastSkippedStopID == target {
						t.Fatal("target skipped instead of served")
					}
					t.Logf("player moved %.2fs AI4 moved %.2fs; player completed original service/departure, AI4 began service by %.2fs; health %.0f", playerMoved, followerMoved, elapsed, g.Bus.Durability)
					return
				}
			}
			t.Fatalf("service did not resume: moved%.2f/%.2f boarded%v left%v player%v AI4%v", playerMoved, followerMoved, boarded, left, g.Bus.Pos, follower.Pos)
		})
	}
}

// Replay only the copied save. Follow every originally blocked bus through real
// boarding and departure, including the later player bus and red-light queues.
func TestSavedBlock279Yield(t *testing.T) {
	path := os.Getenv("BUS_BLOCK279_SAVE")
	if path == "" {
		t.Skip("BUS_BLOCK279_SAVE not set")
	}
	seeds := []int64{5, 17, 29}
	if v := os.Getenv("BUS_BLOCK279_SEED"); v != "" {
		seed, err := strconv.ParseInt(v, 10, 64)
		if err != nil {
			t.Fatal(err)
		}
		seeds = []int64{seed}
	}
	for _, seed := range seeds {
		sim.SeedSimulation(seed)
		g := loadSaveForTest(t, path, 1280, 720)
		sim.SeedSimulation(seed)
		ids := map[int]bool{1: true, 2: true, 6: true, 7: true}
		distance := map[int]float64{}
		doors := map[int]bool{}
		served := map[int]float64{}
		serviceDistance := map[int]float64{}
		health := map[int]float64{}
		for _, a := range g.AIBusMgr.Buses {
			if ids[a.ID] {
				health[a.ID] = a.Bus.Durability
				if a.Route.GetCurrentTargetStopID() != 278 {
					t.Fatal("wrong saved target")
				}
			}
		}
		playerHealth := g.Bus.Durability
		for tick := 0; tick < 10800; tick++ {
			positions := map[int]sim.Vec2{}
			for _, a := range g.AIBusMgr.Buses {
				if ids[a.ID] {
					positions[a.ID] = a.Bus.Pos
				}
			}
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			if tick == 1200 {
				if _, ok := served[1]; !ok {
					t.Fatal("lead AI1 did not serve within20s")
				}
				if _, ok := served[2]; !ok {
					t.Fatal("lead AI2 did not serve within20s")
				}
			}
			if g.Bus.Durability < playerHealth {
				if g.Bus.Pos.Distance(g.World.Stops[278].Pos) < 700 {
					t.Fatalf("player contact in original station zone seed%d at %.2fs pos%v", seed, float64(tick)/60, g.Bus.Pos)
				}
				t.Logf("post-recovery contact outside original station: seed%d %.2fs pos%v health%.1f", seed, float64(tick)/60, g.Bus.Pos, g.Bus.Durability)
				playerHealth = g.Bus.Durability
			}
			for _, a := range g.AIBusMgr.Buses {
				if !ids[a.ID] {
					continue
				}
				step := a.Bus.Pos.Distance(positions[a.ID])
				distance[a.ID] += step
				if step > a.Bus.MaxSpeedFwd/60+1 {
					t.Fatal("queue recovery teleported bus")
				}
				if a.Bus.Durability < health[a.ID] {
					if a.Bus.Pos.Distance(g.World.Stops[278].Pos) < 700 {
						t.Fatalf("AI%d contact in original station zone seed%d at%.2fs", a.ID, seed, float64(tick)/60)
					}
					t.Logf("post-recovery AI%d contact outside original station %.2fs pos%v", a.ID, float64(tick)/60, a.Bus.Pos)
					health[a.ID] = a.Bus.Durability
				}
				if a.Bus.DoorsOpen && a.Driver.DockedStopID == 278 {
					doors[a.ID] = true
				}
				if _, done := served[a.ID]; !done && a.Route.GetCurrentTargetStopID() != 278 {
					if !doors[a.ID] {
						t.Fatalf("AI%d skipped boarding", a.ID)
					}
					served[a.ID] = float64(tick) / 60
					serviceDistance[a.ID] = distance[a.ID]
				}
			}
		}
		for id := range ids {
			if _, ok := served[id]; !ok || distance[id] < 500 || distance[id]-serviceDistance[id] < 300 {
				t.Fatalf("seed%d AI%d stuck: served%v travel%.1f", seed, id, served, distance[id])
			}
		}
		t.Logf("seed%d served times%v travel%v, all original buses served and departed300px; original station zone undamaged through180s", seed, served, distance)
	}
}

// Run against a copy of a local save; never writes to it.
func TestSavedStationQueueReplay(t *testing.T) {
	path := os.Getenv("BUS_STATION_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_REPLAY_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	start := g.Bus.Pos
	target := g.Route.GetCurrentTargetStopID()
	stop := g.World.Stops[target]
	u := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	initialAIHealth := map[int]float64{}
	for _, a := range g.AIBusMgr.Buses {
		initialAIHealth[a.ID] = a.Bus.Durability
	}
	followerID := -1
	for _, a := range g.AIBusMgr.Buses {
		if a.Route.GetCurrentTargetStopID() == target && a.Bus.Pos.Distance(stop.Pos) < 500 {
			followerID = a.ID
			break
		}
	}
	entryTime, boardingTime := -1., -1.
	frontTime := 0.
	peak := 0.
	for tick := 0; tick < 2400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		peak = math.Max(peak, g.Bus.Speed)
		if frontTime == 0 && g.Bus.Pos.Sub(start).Dot(u) > 620 {
			frontTime = float64(tick) / 60
		}
		if tick%60 == 0 && tick < 660 {
			snap := g.AutopilotDriver.Snapshot()
			t.Logf("tick%d along%.1f lat%.1f speed%.1f target%.1f steer%.3f bend%.3f smooth%v run%.1f state%v shoulder%v", tick, g.Bus.Pos.Sub(start).Dot(u), g.Bus.Pos.Sub(start).Dot(sim.Vec2{X: -u.Y, Y: u.X}), g.Bus.Speed, snap.Public.LastTargetSpeed, g.Bus.SteeringAngle, sim.NormalizeAngle(g.Bus.Heading-g.Bus.TailHeading()), snap.SmoothDockDeparture, snap.DockDepartureRun, snap.Public.State, g.Bus.OnShoulder)
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(stop.Pos) < 500 {
					t.Logf("follower%d along%.1f lat%.1f speed%.1f state%v", a.ID, a.Bus.Pos.Sub(stop.Pos).Dot(u), a.Bus.Pos.Sub(stop.Pos).Dot(sim.Vec2{X: -u.Y, Y: u.X}), a.Bus.Speed, a.Driver.State)
				}
			}
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == followerID {
				if entryTime < 0 && math.Abs(a.Bus.Pos.Sub(stop.Pos).Dot(sim.Vec2{X: -u.Y, Y: u.X})) < 10 {
					entryTime = float64(tick) / 60
				}
				if boardingTime < 0 && a.Driver.DockedStopID == target && a.Bus.DoorsOpen {
					boardingTime = float64(tick) / 60
				}
			}
			if a.Bus.Durability < initialAIHealth[a.ID] {
				t.Fatalf("AI%d health lost", a.ID)
			}
		}
		if g.Bus.Durability < 100 {
			t.Fatalf("player health%.1f", g.Bus.Durability)
		}
	}
	t.Logf("620px including doors %.2fs peak%.1f follower%d entered%.2fs boarded%.2fs", frontTime, peak, followerID, entryTime, boardingTime)
	if followerID < 0 || entryTime < 0 || entryTime > 3 || boardingTime < 0 || boardingTime > 8 {
		t.Fatal("rear bus did not enter and board promptly")
	}
	if frontTime == 0 || frontTime > 8 {
		t.Fatalf("slow departure %.2fs", frontTime)
	}
}
