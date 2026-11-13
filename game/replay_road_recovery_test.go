package game

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Optional local reproduction: copies/reads a save without changing its source.
func TestSavedJamReplay(t *testing.T) {
	path := os.Getenv("BUS_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_REPLAY_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	initialPos, initialHealth := g.Bus.Pos, g.Bus.Durability
	initialTarget := g.Route.GetCurrentTargetStopID()
	type aiStart struct {
		pos      sim.Vec2
		target   int
		distance float64
		last     sim.Vec2
	}
	aiStarts := map[int]*aiStart{}
	for _, ab := range g.AIBusMgr.Buses {
		aiStarts[ab.ID] = &aiStart{pos: ab.Bus.Pos, target: ab.Route.GetCurrentTargetStopID(), last: ab.Bus.Pos}
	}
	ticks := 3600
	if value := os.Getenv("BUS_REPLAY_TICKS"); value != "" {
		n, err := strconv.Atoi(value)
		if err != nil || n < 1 {
			t.Fatal("invalid BUS_REPLAY_TICKS")
		}
		ticks = n
	}
	for tick := 0; tick <= ticks; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for _, ab := range g.AIBusMgr.Buses {
			if st, ok := aiStarts[ab.ID]; ok {
				st.distance += ab.Bus.Pos.Distance(st.last)
				st.last = ab.Bus.Pos
			}
		}
		if tick%300 == 0 {
			d := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			d.Path = nil
			snap := g.AutopilotDriver.Snapshot()
			record := struct {
				Tick      int
				Bus       any
				Diagnosis autopilot.StopDiagnosis
				Snapshot  any
			}{tick, struct {
				Position               any
				Heading, Speed, Health float64
			}{g.Bus.Pos, g.Bus.Heading, g.Bus.Speed, g.Bus.Durability}, d, struct {
				Revision      int
				DepartureRun  float64
				Ordinary      bool
				Mid, Approach any
			}{snap.PathRevision, snap.DockDepartureRun, snap.OrdinaryBusLeft, snap.MidLaneRecovery, snap.ApproachMerge}}
			b, _ := json.Marshal(record)
			t.Log(string(b))
			for _, ab := range g.AIBusMgr.Buses {
				if ab.Bus.Pos.Distance(g.Bus.Pos) > 650 || ab.Bus.Speed > 5 {
					continue
				}
				obs := make([]autopilot.VehicleObstacle, 0, len(g.obstacles)+1)
				for _, ob := range g.obstacles {
					if ob.Bus != ab.Bus {
						obs = append(obs, ob)
					}
				}
				obs = append(obs, autopilot.ObserveBus(g.Bus))
				d := ab.Driver.DiagnoseStop(ab.Bus, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
				d.Path = nil
				b, _ := json.Marshal(struct {
					Tick, ID  int
					Pos       any
					Heading   float64
					Diagnosis autopilot.StopDiagnosis
					Mid       any
				}{tick, ab.ID, ab.Bus.Pos, ab.Bus.Heading, d, ab.Driver.Snapshot().MidLaneRecovery})
				t.Log("fleet " + string(b))
			}
		}
	}
	for _, ab := range g.AIBusMgr.Buses {
		st := aiStarts[ab.ID]
		t.Logf("AI%d: initialPos=%.1f,%.1f initialTarget=%d finalPos=%.1f,%.1f finalTarget=%d dist=%.1f speed=%.1f state=%v",
			ab.ID, st.pos.X, st.pos.Y, st.target, ab.Bus.Pos.X, ab.Bus.Pos.Y, ab.Route.GetCurrentTargetStopID(), st.distance, ab.Bus.Speed, ab.Driver.State)
	}
	if g.Bus.Pos.Distance(initialPos) < 2000 || g.Route.GetCurrentTargetStopID() == initialTarget {
		t.Fatalf("saved jam did not resume service: start%v pos%v target%d", initialPos, g.Bus.Pos, g.Route.GetCurrentTargetStopID())
	}
	if g.Bus.Durability < initialHealth {
		t.Fatalf("saved recovery lost durability: %.1f -> %.1f", initialHealth, g.Bus.Durability)
	}
}

// Replays a copied slot one without changing the user's file. The far-side
// short bus should start before the articulated owner fully leaves the ring.
func TestSavedRingEntryGap(t *testing.T) {
	path := os.Getenv("BUS_RING_ENTRY_SAVE")
	if path == "" {
		t.Skip("BUS_RING_ENTRY_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		sim.SeedSimulation(seed)
		g := loadSaveForTest(t, path, 1280, 720)
		sim.SeedSimulation(seed)
		var ai *sim.Bus
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == 14 {
				ai = a.Bus
			}
		}
		if ai == nil {
			t.Fatal("missing AI14")
		}
		rb := g.World.Roundabouts[0]
		initial := ai.Pos
		playerHealth, aiHealth := g.Bus.Durability, ai.Durability
		firstMotion, entered, exited := -1., -1., -1.
		tailStillInside := false
		for tick := 0; tick < 1800; tick++ {
			previous := ai.Pos
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			elapsed := float64(tick) / 60
			if ai.Pos.Distance(previous) > ai.MaxSpeedFwd/60+1 {
				t.Fatal("admission teleported bus")
			}
			if ai.Durability < aiHealth || g.Bus.Durability < playerHealth {
				t.Fatalf("seed%d contact at %.2fs player%.1f AI14%.1f", seed, elapsed, g.Bus.Durability, ai.Durability)
			}
			for _, own := range ai.BodyParts() {
				for _, other := range g.Bus.BodyParts() {
					if own.HalfLen <= 0 || other.HalfLen <= 0 {
						continue
					}
					if hit, _, depth := sim.CheckOBBCollision(own, other); hit && depth > 1e-6 {
						t.Fatalf("bodies overlap at %.2fs", elapsed)
					}
				}
			}
			if firstMotion < 0 && ai.Pos.Distance(initial) > 20 {
				firstMotion = elapsed
				for _, part := range g.Bus.BodyParts() {
					if part.HalfLen <= 0 {
						continue
					}
					u := sim.UnitFromAngle(part.Heading)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					d := rb.Center.Sub(part.Center)
					nearest := part.Center.Add(u.Mul(sim.Clamp(d.Dot(u), -part.HalfLen, part.HalfLen))).Add(r.Mul(sim.Clamp(d.Dot(r), -part.HalfWid, part.HalfWid)))
					tailStillInside = tailStillInside || nearest.Distance(rb.Center) < rb.Radius+8
				}
			}
			if entered < 0 && ai.Pos.Distance(rb.Center) < rb.Radius {
				entered = elapsed
			}
			if entered >= 0 && exited < 0 && ai.Pos.X < rb.Center.X-rb.Radius-100 {
				exited = elapsed
			}
		}
		if firstMotion < 0 || firstMotion > .8 || !tailStillInside || entered < 0 || entered > 2.5 || exited < 0 || exited > 15 || ai.Pos.Distance(initial) < 1000 {
			t.Fatalf("seed%d move%.2f entry%.2f exit%.2f ownerStillInside%v", seed, firstMotion, entered, exited, tailStillInside)
		}
		t.Logf("seed%d AI14 first20px%.2fs (owner still in ring), entry%.2fs exit%.2fs; 30s no overlap/contact", seed, firstMotion, entered, exited)
	}
}

// Opt-in full traffic replay. Point at read-only copies, never the live saves.
func TestSavedSlotPairRecovery(t *testing.T) {
	dir := os.Getenv("BUS_SLOT_PAIR_DIR")
	if dir == "" {
		t.Skip("BUS_SLOT_PAIR_DIR not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		for _, slot := range []int{1, 3} {
			t.Run(fmt.Sprintf("slot%d/seed%d", slot, seed), func(t *testing.T) {
				sim.SeedSimulation(seed)
				g := NewGame(1280, 720)
				g.savePathOverride = filepath.Join(dir, fmt.Sprintf("slot%d-readonly.json", slot))
				if err := g.LoadGame(); err != nil {
					t.Fatal(err)
				}
				pos, health, target := g.Bus.Pos, g.Bus.Durability, g.Route.GetCurrentTargetStopID()
				firstMotion, advanced := -1, -1
				maxWait, wait := 0, 0
				for tick := 0; tick < 9000; tick++ {
					before := g.Bus.Pos
					if err := g.Update(); err != nil {
						t.Fatal(err)
					}
					if before.Distance(g.Bus.Pos) > 20 {
						t.Fatalf("unexpected position jump tick%d", tick)
					}
					if g.Bus.Durability < health {
						t.Fatalf("collision during recovery tick%d health%.1f -> %.1f", tick, health, g.Bus.Durability)
					}
					if firstMotion < 0 && pos.Distance(g.Bus.Pos) > 20 {
						firstMotion = tick
					}
					if g.Bus.Speed < 3 {
						wait++
						maxWait = max(maxWait, wait)
					} else {
						wait = 0
					}
					if g.Route.GetCurrentTargetStopID() != target && advanced < 0 {
						advanced = tick
					}
					// Continue at least 120s to catch the next station's queue/departure.
					if tick >= 7200 && advanced >= 0 && pos.Distance(g.Bus.Pos) > 2000 {
						break
					}
				}
				if firstMotion < 0 || firstMotion > 600 || advanced < 0 || pos.Distance(g.Bus.Pos) < 2000 {
					t.Fatalf("incomplete recovery: motion%d advanced%d distance%.1f", firstMotion, advanced, pos.Distance(g.Bus.Pos))
				}
				if maxWait > 3600 {
					t.Fatalf("new prolonged wait: %.2fs", float64(maxWait)/60)
				}
				t.Logf("motion %.2fs; next stop %.2fs; distance %.0fpx; longest stop %.2fs; health %.0f", float64(firstMotion)/60, float64(advanced)/60, pos.Distance(g.Bus.Pos), float64(maxWait)/60, g.Bus.Durability)
			})
		}
	}
}

func TestArticulatedRoundaboutQueueReplay(t *testing.T) {
	path := os.Getenv("BUS_RING_REPLAY")
	if path == "" && os.Getenv("BUS_RING_FRESH") != "1" {
		t.Skip("BUS_RING_REPLAY not set")
	}
	sim.SeedSimulation(705)
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if os.Getenv("BUS_RING_FRESH") == "1" {
		g.Settings.AutopilotEnabled = true
		g.Settings.AutopilotBerserk = true
		g.Settings.AutopilotBerserkMaxSpeed = 120
		g.Settings.TrafficDensity = 4
		g.EnterCity(sim.MapThemeCustom)
		g.SetLineFleet(0, traffic.LineFleetSettings{MixedModels: true, ModelCounts: [sim.BusModelCount]int{2, 3, 3}, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 80}})
		g.Bus.SetModel(sim.BusArticulated)
		g.ResetBusesToRouteStart()
	} else if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	ticks := 3600
	if s := os.Getenv("BUS_RING_TICKS"); s != "" {
		n, e := strconv.Atoi(s)
		if e != nil || n < 1 {
			t.Fatal("invalid ticks")
		}
		ticks = n
	}
	distance := 0.
	for tick := 0; tick < ticks; tick++ {
		before := g.Bus.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		distance += g.Bus.Pos.Distance(before)
		if g.Bus.Durability < 100 {
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(g.Bus.Pos) < 160 {
					t.Logf("contact near car%d pos%v h%.3f speed%.2f curve%+v", c.ID, c.Pos, c.Heading, c.Speed, c.RoundaboutCurve)
				}
			}
			t.Fatalf("contact tick%d pos%v health%.1f", tick, g.Bus.Pos, g.Bus.Durability)
		}
		if tick%300 == 0 {
			diag := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
			diag.Path = nil
			t.Logf("tick%d pos%v speed%.1f distance%.0f steer%.3f pursuit%v diag%+v", tick, g.Bus.Pos, g.Bus.Speed, distance, g.Bus.SteeringAngle, g.AutopilotDriver.LastPursuitPt, diag)
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(g.Bus.Pos) < 240 {
					t.Logf("car%d pos%v heading%.3f speed%.1f ring%v r%.1f angle%.3f exit%.3f traveled%.3f curve%v", c.ID, c.Pos, c.Heading, c.Speed, c.InRoundabout, c.RoundaboutRadius, c.RoundaboutAngle, c.RoundaboutExitAngle, c.RoundaboutTraveled, c.RoundaboutCurve != nil)
				}
			}
		}
		if tick == ticks-1 && os.Getenv("BUS_RING_CAPTURE") != "" {
			type capture struct {
				Bus        *sim.Bus
				Cars       []*sim.TrafficCar
				Roundabout sim.Roundabout
			}
			f := capture{Bus: g.Bus, Roundabout: g.World.Roundabouts[0]}
			for _, c := range g.TrafficMgr.GetActiveCars(4) {
				if c.Pos.Distance(f.Roundabout.Center) < 600 {
					f.Cars = append(f.Cars, c)
				}
			}
			if saved := os.Getenv("BUS_RING_CAPTURE_SAVE"); saved != "" {
				g.savePathOverride = saved
				if err := g.SaveGame(); err != nil {
					t.Fatal(err)
				}
			}
			b, e := json.MarshalIndent(f, "", "  ")
			if e != nil {
				t.Fatal(e)
			}
			if e = os.WriteFile(os.Getenv("BUS_RING_CAPTURE"), b, 0644); e != nil {
				t.Fatal(e)
			}
		}
	}
	if distance < 3000 {
		t.Fatalf("ring queue stalled: distance%.1f pos%v", distance, g.Bus.Pos)
	}
	t.Logf("distance%.1f health%.0f", distance, g.Bus.Durability)
}

// Read a copied local save, never write it. This includes the occupied platform
// and the two alongside buses that held the berserk articulated player bus.
func TestSavedBerserkTurnRecovery(t *testing.T) {
	path := os.Getenv("BUS_TURN_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_TURN_REPLAY_SAVE not set")
	}
	g := loadSaveForTest(t, path, 1280, 720)
	if !g.Settings.AutopilotEnabled || !g.Settings.AutopilotBerserk || !g.Bus.IsArticulated() {
		t.Fatal("expected saved berserk articulated scene")
	}
	initialTarget, initialHealth := g.Route.GetCurrentTargetStopID(), g.Bus.Durability
	aiHealth := map[int]float64{}
	for _, ab := range g.AIBusMgr.Buses {
		aiHealth[ab.ID] = ab.Bus.Durability
	}
	distance, resumedAt := 0., -1.
	servedTarget := false
	for tick := 0; tick < 7200; tick++ {
		before := g.Bus.Pos
		aiPositions := map[int]sim.Vec2{}
		for _, ab := range g.AIBusMgr.Buses {
			aiPositions[ab.ID] = ab.Bus.Pos
		}
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		step := g.Bus.Pos.Distance(before)
		if step > g.Bus.MaxSpeedFwd/60+1 {
			t.Fatalf("player discontinuity tick%d: %.3f", tick, step)
		}
		distance += step
		if g.Bus.Durability < initialHealth {
			t.Fatalf("player contact tick%d", tick)
		}
		if math.Abs(sim.NormalizeAngle(g.Bus.Heading-g.Bus.TailHeading())) > sim.MaxArticulationAngle+1e-9 {
			t.Fatalf("jackknife tick%d", tick)
		}
		for _, ab := range g.AIBusMgr.Buses {
			if ab.Bus.Pos.Distance(aiPositions[ab.ID]) > ab.Bus.MaxSpeedFwd/60+1 {
				t.Fatalf("AI%d discontinuity tick%d", ab.ID, tick)
			}
			if ab.Bus.Durability < aiHealth[ab.ID] {
				t.Fatalf("AI%d contact tick%d", ab.ID, tick)
			}
		}
		if g.AutopilotDriver.DockedStopID == initialTarget && g.Bus.DoorsOpen {
			servedTarget = true
		}
		if resumedAt < 0 && g.Route.GetCurrentTargetStopID() != initialTarget {
			if !servedTarget || g.lastSkippedStopID == initialTarget {
				t.Fatal("target advanced without actual boarding service")
			}
			resumedAt = float64(tick) / 60
		}
		if tick == 1799 && resumedAt < 0 {
			t.Fatal("did not resume service within 30 seconds")
		}
	}
	if distance < 2000 {
		t.Fatalf("insufficient travel %.1f", distance)
	}
	t.Logf("resumed service %.2fs, traveled %.0fpx in 120 simulated seconds, health %.0f", resumedAt, distance, g.Bus.Durability)
}
