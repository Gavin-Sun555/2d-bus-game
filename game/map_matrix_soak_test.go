package game

import (
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestMapMatrixCapturedCarReleases(t *testing.T) {
	capture := os.Getenv("BUS_MATRIX_CAR_CAPTURE")
	if capture == "" {
		t.Skip("set BUS_MATRIX_CAR_CAPTURE and BUS_MATRIX_CAR")
	}
	id, err := strconv.Atoi(os.Getenv("BUS_MATRIX_CAR"))
	if err != nil {
		t.Fatal(err)
	}
	sim.SeedSimulation(5)
	g := loadSaveForTest(t, capture, 1280, 720)
	sim.SeedSimulation(5)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false
	g.TrafficMgr.BeforeRecovery = func(event sim.TrafficRecoveryEvent) {
		if event.ID == id {
			t.Fatalf("car needed recovery: %+v", event)
		}
	}
	var watched []*sim.TrafficCar
	for _, raw := range strings.Fields(os.Getenv("BUS_MATRIX_CAR_WATCH")) {
		watchID, err := strconv.Atoi(raw)
		if err != nil {
			t.Fatal(err)
		}
		for _, car := range g.TrafficMgr.Cars {
			if car.ID == watchID {
				watched = append(watched, car)
				break
			}
		}
	}
	for _, car := range g.TrafficMgr.Cars {
		if car.ID != id {
			continue
		}
		start := car.Pos
		for tick := 0; tick < 120*60; tick++ {
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			if tick%300 == 0 {
				for _, other := range watched {
					t.Logf("t%.1f car%d %v speed %.1f yielding %v turn %v", float64(tick)/60, other.ID, other.Pos, other.Speed, other.YieldingToBus != nil, other.JunctionTurn != nil)
				}
			}
			if car.Pos.Distance(start) > 500 {
				t.Logf("car%d cleared after %.2fs", id, float64(tick+1)/60)
				return
			}
		}
		t.Fatalf("car%d remains at %v after starting at %v", id, car.Pos, start)
	}
	t.Fatalf("car%d not found in capture", id)
}

// TestMapMatrixTrafficSoak builds a fresh city scenario for one (map, gridScale,
// centerBRT) combination with maximum civilian car traffic (864 cars), all 6
// transit lines active at maximum fleet size (12 buses/line = 72 AI buses), and
// seeded-random bus models across standard, double-decker, and articulated.
func TestMapMatrixTrafficSoak(t *testing.T) {
	mapRaw, out := os.Getenv("BUS_MATRIX_MAP"), os.Getenv("BUS_MATRIX_OUTPUT")
	if mapRaw == "" || out == "" {
		t.Skip("set BUS_MATRIX_MAP and BUS_MATRIX_OUTPUT")
	}
	prevLang := locale.CurrentLanguage
	t.Cleanup(func() { locale.SetLanguage(prevLang) })

	mapNum, err := strconv.Atoi(mapRaw)
	themes := sim.BuiltinMapThemes()
	if err != nil || mapNum < 1 || mapNum > len(themes) {
		t.Fatalf("BUS_MATRIX_MAP must be 1..%d, got %q", len(themes), mapRaw)
	}
	theme := themes[mapNum-1]
	large := os.Getenv("BUS_MATRIX_LARGE") == "1"
	brt := os.Getenv("BUS_MATRIX_BRT") == "1"

	seconds := 1200
	if s := os.Getenv("BUS_MATRIX_SECONDS"); s != "" {
		seconds, err = strconv.Atoi(s)
		if err != nil || seconds < 1 {
			t.Fatalf("invalid BUS_MATRIX_SECONDS %q", s)
		}
	}
	maxHardIncidents := 0
	if s := os.Getenv("BUS_MATRIX_MAX_HARD_INCIDENTS"); s != "" {
		maxHardIncidents, err = strconv.Atoi(s)
		if err != nil || maxHardIncidents < 0 {
			t.Fatalf("invalid BUS_MATRIX_MAX_HARD_INCIDENTS %q", s)
		}
	}
	maxQueueSaves := 24
	if s := os.Getenv("BUS_MATRIX_MAX_QUEUE_CAPTURES"); s != "" {
		maxQueueSaves, err = strconv.Atoi(s)
		if err != nil || maxQueueSaves < 0 {
			t.Fatalf("invalid BUS_MATRIX_MAX_QUEUE_CAPTURES %q", s)
		}
	}
	checkpoint := 0
	if s := os.Getenv("BUS_MATRIX_CHECKPOINT_TICK"); s != "" {
		checkpoint, err = strconv.Atoi(s)
		if err != nil || checkpoint < 1 || checkpoint > seconds*60 {
			t.Fatalf("invalid BUS_MATRIX_CHECKPOINT_TICK %q", s)
		}
	}

	variantIdx := 0
	if large {
		variantIdx += 1
	}
	if brt {
		variantIdx += 2
	}
	seed := int64(20261117 + mapNum*100 + variantIdx*17)
	if s := os.Getenv("BUS_MATRIX_SEED"); s != "" {
		seed, err = strconv.ParseInt(s, 10, 64)
		if err != nil {
			t.Fatalf("invalid BUS_MATRIX_SEED %q", s)
		}
	}

	if err := os.MkdirAll(out, 0700); err != nil {
		t.Fatal(err)
	}

	sim.SeedSimulation(seed)
	mapRng := rand.New(rand.NewSource(int64(20261117 + mapNum*997)))
	perm := mapRng.Perm(4)
	modelRng := rand.New(rand.NewSource(seed*7919 + 31))

	g := NewGame(1280, 720)
	g.Settings.SetLargeRoadNetwork(theme, large)
	g.Settings.SetCenterBRT(theme, brt)
	g.Settings.TrafficDensity = sim.TrafficDensityLevels
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.AutopilotBerserkMaxSpeed = 80
	g.EnterCity(theme)
	g.State, g.Settings.IsOpen, g.IsPlanMode = StatePlaying, false, false

	if g.World.Theme != theme || g.World.LargeGrid != large || g.World.CenterBRT != brt {
		t.Fatalf("world mismatch: got theme=%v large=%v brt=%v, want theme=%v large=%v brt=%v",
			g.World.Theme, g.World.LargeGrid, g.World.CenterBRT, theme, large, brt)
	}

	g.SetTransitLineCount(6)
	lineMixes := make([][sim.BusModelCount]int, 6)
	playerModel := sim.BusModel(modelRng.Intn(int(sim.BusModelCount)))
	if perm[variantIdx] < int(sim.BusModelCount) {
		playerModel = sim.BusModel(perm[variantIdx])
	}
	for line := 0; line < 6; line++ {
		var counts [sim.BusModelCount]int
		for slot := 0; slot < sim.MaxBusesPerLine; slot++ {
			m := sim.BusModel(modelRng.Intn(int(sim.BusModelCount)))
			counts[m]++
		}
		lineMixes[line] = counts
		pm := sim.BusModel(modelRng.Intn(int(sim.BusModelCount)))
		if line == g.PlayerLine {
			pm = playerModel
		}
		g.SetLineFleet(line, traffic.LineFleetSettings{
			MixedModels: true,
			ModelCounts: counts,
			PlayerModel: pm,
			SpeedRange:  sim.SpeedRange{MinKPH: 30, MaxKPH: 80},
		})
	}
	if got := len(g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)); got != sim.MaxTrafficCars {
		t.Fatalf("maximum car density yielded %d active cars", got)
	}
	var fleetPerLine [6]int
	for _, bus := range g.AIBusMgr.Buses {
		if bus.LineIndex < 0 || bus.LineIndex >= len(fleetPerLine) {
			t.Fatalf("bus %d assigned to unexpected line %d", bus.ID, bus.LineIndex)
		}
		fleetPerLine[bus.LineIndex]++
	}
	for line, count := range fleetPerLine {
		if count != sim.MaxBusesPerLine {
			t.Fatalf("line %d has %d buses, want %d", line+1, count, sim.MaxBusesPerLine)
		}
	}
	sim.SeedSimulation(seed)

	initialPath := filepath.Join(out, "initial-state.json")
	g.savePathOverride = initialPath
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	initialRaw, err := os.ReadFile(initialPath)
	if err != nil {
		t.Fatal(err)
	}

	f, err := os.Create(filepath.Join(out, "events.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	enc := json.NewEncoder(f)

	tick := 0
	hardIncidents := 0
	queueCaptures := 0
	queueSaves := 0
	type queueArea struct {
		x, y int
		kind string
	}
	savedQueueAreas := make(map[queueArea]int)
	captureFile := func(reason, kind string, id int, pos sim.Vec2, hard bool) string {
		save := hard
		if hard {
			hardIncidents++
		} else {
			queueCaptures++
			// Preserve examples from new bottlenecks after a single long
			// queue has already produced many follower events.
			area := queueArea{int(math.Floor(pos.X / 600)), int(math.Floor(pos.Y / 600)), kind}
			save = maxQueueSaves == 0 || (queueSaves < maxQueueSaves && savedQueueAreas[area] < 2)
			if save {
				queueSaves++
				savedQueueAreas[area]++
			}
		}
		name := fmt.Sprintf("%06d-%s-%s%d.json", tick, reason, kind, id)
		if save {
			g.savePathOverride = filepath.Join(out, name)
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
		} else {
			name = "omitted-by-capture-limit"
		}
		if err := enc.Encode(map[string]any{
			"tick": tick, "seconds": float64(tick) / 60,
			"hard": hard, "reason": reason, "kind": kind, "id": id,
			"position": pos, "capture": name,
		}); err != nil {
			t.Fatal(err)
		}
		t.Logf("t=%.1f [hard=%v] %s %s%d pos%v", float64(tick)/60, hard, reason, kind, id, pos)
		return name
	}

	g.TrafficMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		captureFile(e.Reason, e.Kind, e.ID, e.Pos, true)
	}
	g.AIBusMgr.BeforeRecovery = func(e sim.TrafficRecoveryEvent) {
		captureFile(e.Reason, e.Kind, e.ID, e.Pos, true)
	}
	g.AutopilotDriver.BeforeRecovery = func() {
		captureFile("local-recovery", "player", 0, g.Bus.Pos, true)
	}
	for _, a := range g.AIBusMgr.Buses {
		a := a
		a.Driver.BeforeRecovery = func() {
			captureFile("local-recovery", "bus", a.ID, a.Bus.Pos, true)
		}
	}

	type stallEpisode struct {
		Kind        string   `json:"kind"`
		ID          int      `json:"id"`
		Reason      string   `json:"reason"`
		CaptureTick int      `json:"capture_tick"`
		CaptureFile string   `json:"capture_file"`
		Anchor      sim.Vec2 `json:"anchor"`
		ClearedTick int      `json:"cleared_tick,omitempty"`
		ClearedSecs float64  `json:"cleared_after_seconds,omitempty"`
	}
	var episodes []*stallEpisode

	type progress struct {
		anchor                sim.Vec2
		still                 int
		maxStill              int
		distance              float64
		last                  sim.Vec2
		health                float64
		inBarrier             bool
		areaAnchor            sim.Vec2
		confined, maxConfined int
		openEpisodes          []*stallEpisode
	}
	watches := map[debugVehicleKey]*progress{}

	diagnoseBus := func(id int, bus *sim.Bus, driver *autopilot.AutopilotDriver, route *sim.Route) {
		var obs []autopilot.VehicleObstacle
		if id == 0 {
			obs = g.obstacles
		} else {
			obs = append(g.AIBusMgr.GetObstacles(id), autopilot.ObserveBus(g.Bus))
			for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
				if c.Pos.Distance(bus.Pos) < 1600 {
					obs = append(obs, autopilot.VehicleObstacle{
						Pos: c.Pos, Heading: c.Heading, Speed: c.Speed,
						Length: c.Length, Width: c.Width,
						JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor,
					})
				}
			}
		}
		d := driver.DiagnoseStop(bus, route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())
		d.Path = nil
		_ = enc.Encode(map[string]any{
			"tick": tick, "id": id, "position": bus.Pos, "heading": bus.Heading,
			"speed": bus.Speed, "diagnosis": d, "driver": driver.Snapshot(),
		})
		if id == 0 {
			t.Logf("t=%.1f player pos%v speed%.1f state%s limits%v", float64(tick)/60, bus.Pos, bus.Speed, d.State, d.Limits)
		}
	}

	observe := func(k debugVehicleKey, pos sim.Vec2, health float64, service bool, recordNewStalls bool) {
		w := watches[k]
		if w == nil {
			w = &progress{anchor: pos, areaAnchor: pos, last: pos, health: health}
			watches[k] = w
		}
		step := pos.Distance(w.last)
		if recordNewStalls {
			w.distance += step
		}
		w.last = pos
		if math.IsNaN(pos.X) || math.IsNaN(pos.Y) || math.IsInf(pos.X, 0) || math.IsInf(pos.Y, 0) {
			t.Fatalf("invalid position %+v at tick %d", k, tick)
		}
		if health < w.health {
			captureFile("damage", k.Kind, k.ID, pos, true)
		}
		w.health = health

		if len(w.openEpisodes) > 0 {
			remaining := w.openEpisodes[:0]
			for _, ep := range w.openEpisodes {
				if pos.Distance(ep.Anchor) > 250 {
					ep.ClearedTick = tick
					ep.ClearedSecs = float64(tick-ep.CaptureTick) / 60
					_ = enc.Encode(map[string]any{
						"tick": tick, "event": "episode-cleared",
						"kind": ep.Kind, "id": ep.ID, "reason": ep.Reason,
						"cleared_after_seconds": ep.ClearedSecs,
					})
				} else {
					remaining = append(remaining, ep)
				}
			}
			w.openEpisodes = remaining
		}
		if !recordNewStalls {
			return
		}

		if service || pos.Distance(w.anchor) > 10 {
			w.anchor, w.still = pos, 0
		} else {
			w.still++
		}
		w.maxStill = max(w.maxStill, w.still)

		if service || pos.Distance(w.areaAnchor) > 80 {
			w.areaAnchor, w.confined = pos, 0
		} else {
			w.confined++
		}
		w.maxConfined = max(w.maxConfined, w.confined)

		if w.confined == 120*60 || w.confined == 180*60 {
			reason := fmt.Sprintf("confined-%ds", w.confined/60)
			name := captureFile(reason, k.Kind, k.ID, pos, false)
			ep := &stallEpisode{Kind: k.Kind, ID: k.ID, Reason: reason, CaptureTick: tick, CaptureFile: name, Anchor: pos}
			episodes = append(episodes, ep)
			w.openEpisodes = append(w.openEpisodes, ep)
		}
		if w.still == 60*45 {
			_ = enc.Encode(map[string]any{"tick": tick, "observation": "stationary-45s", "kind": k.Kind, "id": k.ID, "position": pos})
		}
		deadline := max(90, int(math.Ceil(g.TrafficLights.DeadlockTimeoutAt(pos)))) * 60
		if w.still == deadline || w.still == deadline+60*60 {
			reason := fmt.Sprintf("stationary-%ds", w.still/60)
			name := captureFile(reason, k.Kind, k.ID, pos, false)
			ep := &stallEpisode{Kind: k.Kind, ID: k.ID, Reason: reason, CaptureTick: tick, CaptureFile: name, Anchor: pos}
			episodes = append(episodes, ep)
			w.openEpisodes = append(w.openEpisodes, ep)
		}
	}

	checkBarrier := func(k debugVehicleKey, b *sim.Bus, lw float64) {
		w := watches[k]
		if w == nil {
			return
		}
		hit := b.CheckBoundaries(g.World, lw)
		if hit.Zone == sim.ZoneHardBarrier {
			if !w.inBarrier {
				w.inBarrier = true
				captureFile("hard-barrier-"+hit.ObstacleName, k.Kind, k.ID, b.Pos, true)
			}
		} else {
			w.inBarrier = false
		}
	}

	activeFleetCounts := [sim.BusModelCount]int{}
	for _, a := range g.AIBusMgr.Buses {
		activeFleetCounts[a.Bus.Model]++
	}
	t.Logf("map=%d (%s) large=%v brt=%v seed=%d seconds=%d cars=%d ai_buses=%d player_model=%v fleet_models=%v",
		mapNum, theme.DisplayName(), large, brt, seed, seconds,
		len(g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)), len(g.AIBusMgr.Buses),
		g.Bus.Model, activeFleetCounts)

	abortedEarly := false
	targetTicks := seconds * 60
	completedTicks := 0
	for tick = 1; tick <= targetTicks; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		completedTicks = tick
		if tick == checkpoint {
			g.savePathOverride = filepath.Join(out, "checkpoint.json")
			if err := g.SaveGame(); err != nil {
				t.Fatal(err)
			}
		}
		observe(debugVehicleKey{"player", 0}, g.Bus.Pos, g.Bus.Durability, g.Bus.DoorsOpen || g.Bus.PassengerServiceRemaining() > 0, true)
		for _, a := range g.AIBusMgr.Buses {
			observe(debugVehicleKey{"bus", a.ID}, a.Bus.Pos, a.Bus.Durability, a.Bus.DoorsOpen || a.Bus.PassengerServiceRemaining() > 0 || a.Bus.TerminalReplacementHold, true)
		}
		for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			observe(debugVehicleKey{"car", c.ID}, c.Pos, 100, false, true)
		}
		if tick%60 == 0 {
			lw := g.Settings.GetLaneWidth()
			checkBarrier(debugVehicleKey{"player", 0}, g.Bus, lw)
			for _, a := range g.AIBusMgr.Buses {
				checkBarrier(debugVehicleKey{"bus", a.ID}, a.Bus, lw)
			}
		}
		if tick == 1 || tick%1800 == 0 {
			diagnoseBus(0, g.Bus, g.AutopilotDriver, g.Route)
			for _, a := range g.AIBusMgr.Buses {
				if math.Abs(a.Bus.Speed) >= 3 || a.Bus.DoorsOpen {
					continue
				}
				diagnoseBus(a.ID, a.Bus, a.Driver, a.Route)
			}
		}
		if maxHardIncidents > 0 && hardIncidents >= maxHardIncidents {
			abortedEarly = true
			t.Logf("aborting early at t=%.1fs after %d hard incidents", float64(tick)/60, hardIncidents)
			break
		}
	}

	g.savePathOverride = filepath.Join(out, "end-state.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}

	// Drain up to 90s solely to verify whether any queue episode captured near
	// the end of the 20-minute window (or a 45s+ watch still building toward 90s)
	// clears naturally without hard incidents.
	hasLateEpisode := false
	lateWatches := map[debugVehicleKey]bool{}
	for _, ep := range episodes {
		if ep.ClearedTick == 0 && completedTicks-ep.CaptureTick < 90*60 {
			hasLateEpisode = true
		}
	}
	for k, w := range watches {
		if w.still >= 45*60 || w.confined >= 60*60 {
			hasLateEpisode = true
			lateWatches[k] = true
		}
	}
	if !abortedEarly && hardIncidents == 0 && hasLateEpisode {
		for drain := 1; drain <= 90*60; drain++ {
			tick = completedTicks + drain
			if err := g.Update(); err != nil {
				t.Fatal(err)
			}
			observe(debugVehicleKey{"player", 0}, g.Bus.Pos, g.Bus.Durability, g.Bus.DoorsOpen || g.Bus.PassengerServiceRemaining() > 0, lateWatches[debugVehicleKey{"player", 0}])
			for _, a := range g.AIBusMgr.Buses {
				observe(debugVehicleKey{"bus", a.ID}, a.Bus.Pos, a.Bus.Durability, a.Bus.DoorsOpen || a.Bus.PassengerServiceRemaining() > 0 || a.Bus.TerminalReplacementHold, lateWatches[debugVehicleKey{"bus", a.ID}])
			}
			for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
				observe(debugVehicleKey{"car", c.ID}, c.Pos, 100, false, lateWatches[debugVehicleKey{"car", c.ID}])
			}
			stillOpen := false
			for _, ep := range episodes {
				if ep.ClearedTick == 0 {
					stillOpen = true
					break
				}
			}
			if !stillOpen {
				for k := range lateWatches {
					if w := watches[k]; w != nil && (w.still >= 45*60 || w.confined >= 60*60) {
						stillOpen = true
						break
					}
				}
			}
			if !stillOpen || hardIncidents > 0 {
				break
			}
		}
	}

	unresolved := 0
	for _, ep := range episodes {
		if ep.ClearedTick == 0 {
			unresolved++
		}
	}
	maxWait := map[string]float64{}
	maxConfined := map[string]float64{}
	distance := map[string]float64{}
	for k, w := range watches {
		maxWait[k.Kind] = max(maxWait[k.Kind], float64(w.maxStill)/60)
		maxConfined[k.Kind] = max(maxConfined[k.Kind], float64(w.maxConfined)/60)
		if k.Kind != "car" {
			distance[fmt.Sprintf("%s%d", k.Kind, k.ID)] = w.distance
		}
	}

	report := map[string]any{
		"map_number":             mapNum,
		"map_name":               theme.DisplayName(),
		"large_road_network":     large,
		"center_brt":             brt,
		"seed":                   seed,
		"initial_state_sha256":   fmt.Sprintf("%x", sha256.Sum256(initialRaw)),
		"simulation_seconds":     float64(completedTicks) / 60,
		"target_seconds":         seconds,
		"ticks":                  completedTicks,
		"aborted_early":          abortedEarly,
		"cars":                   len(g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)),
		"ai_buses":               len(g.AIBusMgr.Buses),
		"player_model":           int(g.Bus.Model),
		"line_model_mixes":       lineMixes,
		"active_fleet_models":    activeFleetCounts,
		"hard_incidents":         hardIncidents,
		"queue_captures":         queueCaptures,
		"unresolved_episodes":    unresolved,
		"episodes":               episodes,
		"max_stationary_seconds": maxWait,
		"max_confined_seconds":   maxConfined,
		"bus_distance_pixels":    distance,
	}
	raw, err := json.MarshalIndent(report, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(out, "report.json"), raw, 0600); err != nil {
		t.Fatal(err)
	}
	t.Logf("completed %.1fs: hard_incidents=%d queue_captures=%d unresolved_episodes=%d",
		float64(completedTicks)/60, hardIncidents, queueCaptures, unresolved)
	if hardIncidents > 0 || unresolved > 0 || abortedEarly {
		t.Fatalf("hard_incidents=%d unresolved_episodes=%d aborted_early=%v (see %s)",
			hardIncidents, unresolved, abortedEarly, out)
	}
}
