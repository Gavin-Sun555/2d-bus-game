package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestDistantMergeYieldDoesNotSuppressJunctionFallback(t *testing.T) {
	bus := sim.NewBus(0, 55, 0)
	bus.SetModel(sim.BusArticulated)
	lead := VehicleObstacle{
		Pos: sim.Vec2{X: -245, Y: 26}, Heading: 0, Length: 46, Width: 22, YieldingTo: bus,
	}
	f := autopilotFrame{bus: bus, laneWidth: 52, obstacles: []VehicleObstacle{lead}}
	ad := NewAutopilotDriver()
	ad.approachMerge = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 26}, LeftTurn: true}
	ad.busTurnHoldDistance = 10
	forward, right := sim.Vec2{X: 1}, sim.Vec2{Y: 1}
	if ad.mergeYieldOrClearingActive(f, 0, forward, right, 55, 26) {
		t.Fatal("distant follower counted as opening the merge slot")
	}
	f.obstacles[0].Pos.X = -185
	if !ad.mergeYieldOrClearingActive(f, 0, forward, right, 55, 26) {
		t.Fatal("nearby yielding follower should count as opening a usable slot")
	}
}

func setupMetropolisNorthRingJunction(t *testing.T, lw float64) (*sim.World, *sim.TrafficLightSystem, *sim.TrafficIntersection, float64) {
	t.Helper()
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	roadY := -1200.0 * sim.MapGridScale
	var inter *sim.TrafficIntersection
	for _, cand := range tls.Intersections {
		if cand.Center.Distance(sim.Vec2{X: 0, Y: roadY}) < 10 {
			inter = cand
			break
		}
	}
	if inter == nil {
		t.Fatalf("missing 6-lane intersection at (0, %.1f)", roadY)
	}
	return w, tls, inter, roadY
}

func makeStop(id int, pos sim.Vec2, heading, lw float64) *sim.BusStop {
	return sim.NewBusStop(id, "S", pos, heading, 0, sim.RGBA(0, 0, 0, 0), lw, 0)
}

func stepAutopilotTestBus(
	ad *AutopilotDriver,
	bus *sim.Bus,
	route *sim.Route,
	w *sim.World,
	tls *sim.TrafficLightSystem,
	obs []VehicleObstacle,
	lw float64,
	seconds float64,
) {
	const dt = 1.0 / 60.0
	steps := int(math.Round(seconds / dt))
	for i := 0; i < steps; i++ {
		if tls != nil {
			tls.Update(dt)
		}
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, dt)
		bus.Update(dt, th, br, st, hb, doors)
	}
}

func TestFallbackLeftTurnFromMiddleLaneWhenLeftLaneBlocked(t *testing.T) {
	lw := 52.0
	w, tls, inter, roadY := setupMetropolisNorthRingJunction(t, lw)

	sFrom := makeStop(0, sim.Vec2{X: -1800, Y: roadY + 2.5*lw}, 0, lw)
	nsOff := sim.RoadLaneOffset(0, false, lw, w, sim.Vec2{X: 0, Y: roadY - 1200})
	sTo := makeStop(1, sim.Vec2{X: nsOff, Y: roadY - 1500}, -math.Pi/2, lw)
	w.Stops = []*sim.BusStop{sFrom, sTo}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.DisallowOncomingOvertake = true
	ad.State = AutopilotStateCruising
	ad.PrepareLeg(w, sFrom, sTo, lw)

	line := inter.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-95, roadY+1.5*lw, 0)
	bus.Gear = sim.GearDrive

	// Block the dedicated left-turn lane (0.5*lw) alongside the bus so it cannot merge left.
	obs := []VehicleObstacle{
		{Pos: sim.Vec2{X: -line - 90, Y: roadY + 0.5*lw}, Heading: 0, Speed: 0, Length: 193, Width: 39},
		{Pos: sim.Vec2{X: -line - 295, Y: roadY + 0.5*lw}, Heading: 0, Speed: 0, Length: 96, Width: 36},
	}

	// Phase 1: Through Green but Left Red -> fallback engages, bus must wait behind stop line.
	inter.Phase = sim.PhaseEWGreen
	inter.Timer = 0
	for i := 0; i < 180; i++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > inter.Center.X-line+4 {
			t.Fatalf("left turn from middle lane entered intersection during EW through green (x=%.1f, line=%.1f)", bus.Pos.X, inter.Center.X-line)
		}
	}
	if ad.fallbackJunction == nil {
		t.Fatal("expected current-lane junction fallback to activate when left-turn lane merge is blocked")
	}

	// Snapshot/restore round-trip preserves fallback junction state across save/load.
	restored, err := ad.Snapshot().Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	if restored.fallbackJunction == nil {
		t.Fatal("expected Snapshot/Restore to preserve fallbackJunction")
	}
	ad = restored

	// Phase 2: Left Green -> restored driver proceeds cleanly from middle lane without turn_hold deadlock.
	inter.Phase = sim.PhaseEWLeftGreen
	inter.Timer = 0
	for i := 0; i < int(inter.LeftGreenDuration*60); i++ {
		inter.Timer = float64(i) / 60
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if bus.Pos.X+bus.Length/2 <= inter.Center.X-line+10 {
		t.Fatalf("expected middle-lane left turn to proceed on left green; stuck at x=%.1f (hold=%.1f)", bus.Pos.X, ad.busTurnHoldDistance)
	}
	inter.Phase = sim.PhaseEWLeftGreen
	stepAutopilotTestBus(ad, bus, route, w, nil, obs, lw, 2.5)
	if bus.Pos.Y >= inter.Center.Y-100 {
		t.Fatalf("expected fallback left turn to clear northward onto exit road; pos=(%.1f, %.1f)", bus.Pos.X, bus.Pos.Y)
	}
}

func TestFallbackStraightFromLeftTurnLaneObeysThroughSignal(t *testing.T) {
	lw := 52.0
	w, tls, inter, roadY := setupMetropolisNorthRingJunction(t, lw)

	sFrom := makeStop(1, sim.Vec2{X: -1800, Y: roadY + 2.5*lw}, 0, lw)
	sTo := makeStop(2, sim.Vec2{X: 1800, Y: roadY + 2.5*lw}, 0, lw)
	w.Stops = []*sim.BusStop{sFrom, sTo}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.DisallowOncomingOvertake = true
	ad.State = AutopilotStateCruising
	ad.PrepareLeg(w, sFrom, sTo, lw)

	line := inter.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-95, roadY+0.5*lw, 0)
	bus.Gear = sim.GearDrive

	// Block the middle through lane so the bus cannot exit the dedicated left-turn lane before the junction.
	obs := []VehicleObstacle{
		{Pos: sim.Vec2{X: -line - 85, Y: roadY + 1.5*lw}, Heading: 0, Speed: 0, Length: 193, Width: 39},
		{Pos: sim.Vec2{X: -line - 285, Y: roadY + 1.5*lw}, Heading: 0, Speed: 0, Length: 96, Width: 36},
	}

	// Left Green alone does NOT allow going straight from the left-turn lane on a known straight route.
	inter.Phase = sim.PhaseEWLeftGreen
	inter.Timer = 0
	for i := 0; i < 180; i++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > inter.Center.X-line+4 {
			t.Fatalf("straight traversal from left lane entered intersection on left green (x=%.1f)", bus.Pos.X)
		}
	}
	if ad.fallbackJunction == nil {
		t.Fatal("expected current-lane fallback to splice straight path through intersection")
	}

	// On Through Green, bus goes straight through the box and rejoins downstream without crossing the blocker.
	inter.Phase = sim.PhaseEWGreen
	inter.Timer = 0
	for i := 0; i < 360; i++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if bus.Pos.X <= inter.Center.X+line+50 {
		t.Fatalf("expected straight fallback to clear intersection on through green; pos=(%.1f, %.1f)", bus.Pos.X, bus.Pos.Y)
	}
}

func TestFallbackRightTurnFromMiddleLaneWaitsOnRedAndProceedsOnGreen(t *testing.T) {
	lw := 52.0
	w, tls, inter, roadY := setupMetropolisNorthRingJunction(t, lw)

	sFrom := makeStop(1, sim.Vec2{X: -1800, Y: roadY + 2.5*lw}, 0, lw)
	nsOff := sim.RoadLaneOffset(0, false, lw, w, sim.Vec2{X: 0, Y: roadY + 1200})
	sTo := makeStop(2, sim.Vec2{X: -nsOff, Y: roadY + 1500}, math.Pi/2, lw)
	w.Stops = []*sim.BusStop{sFrom, sTo}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.DisallowOncomingOvertake = true
	ad.State = AutopilotStateCruising
	ad.PrepareLeg(w, sFrom, sTo, lw)

	line := inter.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-95, roadY+1.5*lw, 0)
	bus.Gear = sim.GearDrive

	// Block the curb lane (2.5*lw) alongside the bus so it must turn right from middle lane (1.5*lw).
	obs := []VehicleObstacle{
		{Pos: sim.Vec2{X: -line - 85, Y: roadY + 2.5*lw}, Heading: 0, Speed: 0, Length: 193, Width: 39},
		{Pos: sim.Vec2{X: -line - 285, Y: roadY + 2.5*lw}, Heading: 0, Speed: 0, Length: 96, Width: 36},
	}

	// Through Red: non-curb fallback right turn must stop at red light instead of turning on red.
	inter.Phase = sim.PhaseNSGreen
	inter.Timer = 0
	for i := 0; i < 180; i++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > inter.Center.X-line+4 {
			t.Fatalf("non-curb fallback right turn ran red light (x=%.1f, line=%.1f)", bus.Pos.X, inter.Center.X-line)
		}
	}
	if ad.fallbackJunction == nil {
		t.Fatal("expected fallback junction to activate for blocked right-turn merge")
	}

	// Through Green: fallback right turn executes cleanly southward without swinging through curb blocker.
	inter.Phase = sim.PhaseEWGreen
	inter.Timer = 0
	for i := 0; i < 420; i++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if bus.Pos.Y <= inter.Center.Y+120 {
		t.Fatalf("expected middle-lane fallback right turn to proceed southward on through green; pos=(%.1f, %.1f)", bus.Pos.X, bus.Pos.Y)
	}
}

func TestSavedMidTaperStraddlingBusSplicesImmediately(t *testing.T) {
	lw := 52.0
	w, tls, inter, roadY := setupMetropolisNorthRingJunction(t, lw)

	sFrom := makeStop(0, sim.Vec2{X: -1800, Y: roadY + 2.5*lw}, 0, lw)
	nsOff := sim.RoadLaneOffset(0, false, lw, w, sim.Vec2{X: 0, Y: roadY - 1200})
	sTo := makeStop(1, sim.Vec2{X: nsOff, Y: roadY - 1500}, -math.Pi/2, lw)
	w.Stops = []*sim.BusStop{sFrom, sTo}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.DisallowOncomingOvertake = true
	ad.State = AutopilotStateCruising
	ad.PrepareLeg(w, sFrom, sTo, lw)

	// Replicate B#149/B#176 saved state: bus sits just before stop line straddling Lane 1 (1.2*lw),
	// approachMerge is nil, while another bus occupies Lane 0 (0.5*lw) alongside it.
	line := inter.GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-6.5, roadY+1.2*lw, -0.065)
	bus.Gear = sim.GearDrive
	obs := []VehicleObstacle{
		{Pos: sim.Vec2{X: -line - 108, Y: roadY + 0.5*lw}, Heading: -0.004, Speed: 0, Length: 193, Width: 39},
	}

	inter.Phase = sim.PhaseEWLeftGreen
	inter.Timer = 0
	for i := 0; i < int(inter.LeftGreenDuration*60); i++ {
		inter.Timer = float64(i) / 60
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if i == 0 && ad.fallbackJunction == nil {
			t.Fatal("expected immediate current-lane fallback when straddling just before stop line with target lane occupied")
		}
	}
	if bus.Pos.X+bus.Length/2 <= inter.Center.X-line+20 {
		t.Fatalf("expected straddling bus to advance on left green instead of wedging on adjacent bus; pos=(%.1f, %.1f)", bus.Pos.X, bus.Pos.Y)
	}
}
