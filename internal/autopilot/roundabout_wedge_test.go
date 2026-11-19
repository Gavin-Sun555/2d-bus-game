package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRoundaboutApproachResetsBlockedInnerLaneTarget(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	fromStop := sim.NewBusStop(0, "west", sim.Vec2{X: rb.Center.X - 900, Y: rb.Center.Y + 1.5*lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	toStop := sim.NewBusStop(1, "east", sim.Vec2{X: rb.Center.X + 900, Y: rb.Center.Y + 1.5*lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{fromStop, toStop}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	// Bus physically in outer West approach lane (Y=+78 relative to center),
	// with stale overtake intent trying to merge into occupied inner lane (-52).
	bus := sim.NewBus(rb.Center.X-(rb.Radius+145), rb.Center.Y+1.5*lw, 0)
	bus.SetModel(sim.BusArticulated)
	sim.EnsureLongBusBays(w, route)
	bus.Speed = 0

	innerBlocker := sim.NewBus(rb.Center.X-(rb.Radius+170), rb.Center.Y+0.5*lw, 0)
	innerBlocker.SetModel(sim.BusArticulated)
	obstacles := []VehicleObstacle{{
		Bus:     innerBlocker,
		Pos:     innerBlocker.Pos,
		Heading: innerBlocker.Heading,
		Speed:   0,
		Length:  innerBlocker.Length,
		Width:   innerBlocker.Width,
	}}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, fromStop, toStop, lw)
	ad.OvertakeState = OvertakePassing
	ad.LaneOffset = -lw
	ad.TargetLaneOffset = -lw

	ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60.0)

	if ad.TargetLaneOffset != 0 {
		t.Fatalf("expected blocked inner TargetLaneOffset reset to 0, got %.2f", ad.TargetLaneOffset)
	}
	if ad.LaneOffset != 0 || ad.OvertakeState != OvertakeNone {
		t.Fatalf("expected snapped recovery to LaneOffset=0 OvertakeNone, got offset=%.2f state=%d", ad.LaneOffset, ad.OvertakeState)
	}
}

func TestRoundaboutEntryWedgeRetreatTriggersReverseForUnheldStoppedBus(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	rb := w.Roundabouts[0]
	fromStop := sim.NewBusStop(0, "west", sim.Vec2{X: rb.Center.X - 900, Y: rb.Center.Y + 1.5*lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	toStop := sim.NewBusStop(1, "east", sim.Vec2{X: rb.Center.X + 900, Y: rb.Center.Y + 1.5*lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{fromStop, toStop}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	// Ego bus in outer lane, slightly yawed inward near approach yield area,
	// with an inner-lane bus abreast/trailing that blocks its roundabout entry sweep.
	bus := sim.NewBus(rb.Center.X-(rb.Radius+145), rb.Center.Y+1.5*lw, -0.22)
	bus.SetModel(sim.BusArticulated)
	sim.EnsureLongBusBays(w, route)
	bus.Speed = 0

	wedgeBus := sim.NewBus(rb.Center.X-(rb.Radius+170), rb.Center.Y+0.5*lw, 0)
	wedgeBus.SetModel(sim.BusArticulated)
	obstacles := []VehicleObstacle{{
		Bus:     wedgeBus,
		Pos:     wedgeBus.Pos,
		Heading: wedgeBus.Heading,
		Speed:   0,
		Length:  wedgeBus.Length,
		Width:   wedgeBus.Width,
	}}

	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.PrepareLeg(w, fromStop, toStop, lw)

	reversing := false
	var lastTh, lastBr float64
	for tick := 0; tick < 240; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60.0)
		lastTh, lastBr = th, br
		if ad.headonRecovery != nil {
			reversing = true
			if bus.Gear != sim.GearReverse {
				t.Fatalf("expected reverse gear during roundabout wedge retreat, got gear=%v", bus.Gear)
			}
			break
		}
		bus.Update(1.0/60.0, th, br, st, hb, doors)
		if math.Abs(bus.Speed) > 1 {
			t.Fatalf("expected wedged bus to stay stopped before reverse trigger at tick %d, speed=%.2f th=%.2f br=%.2f", tick, bus.Speed, th, br)
		}
	}
	if !reversing {
		t.Fatalf("expected unheld wedged bus at roundabout entrance to trigger reverse recovery (deadlockTimer=%.2f, recovery=%v, th=%.2f, br=%.2f)", ad.headonDeadlockTimer, ad.headonRecovery != nil, lastTh, lastBr)
	}
}

func TestNarrowRoundaboutHoldsSecondArticulatedBusOutsideRing(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	w.Roundabouts[0].Radius = 240
	w.Roundabouts[0].InnerRadius = 120
	rb := w.Roundabouts[0]
	if rb.SupportsParallelArticulated() {
		t.Fatal("expected R=240 IR=120 roundabout to report narrow ring")
	}
	fromStop := sim.NewBusStop(0, "west", sim.Vec2{X: -900, Y: 1.5 * lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	toStop := sim.NewBusStop(1, "east", sim.Vec2{X: 900, Y: 1.5 * lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{fromStop, toStop}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	bus := sim.NewBus(rb.Center.X-(rb.Radius+110), rb.Center.Y+1.5*lw, 0)
	bus.SetModel(sim.BusArticulated)
	sim.EnsureLongBusBays(w, route)
	bus.Speed = 80

	circulating := sim.NewBus(rb.Center.X+20, rb.Center.Y+170, -0.1)
	circulating.SetModel(sim.BusArticulated)
	obstacles := []VehicleObstacle{{
		Bus:     circulating,
		Pos:     circulating.Pos,
		Heading: circulating.Heading,
		Speed:   30,
		Length:  circulating.Length,
		Width:   circulating.Width,
	}}

	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, fromStop, toStop, lw)
	for tick := 0; tick < 180; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obstacles, lw, 1.0/60.0)
		bus.Update(1.0/60.0, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > rb.Center.X-rb.Radius-5 {
			t.Fatalf("second articulated bus entered narrow occupied ring at tick %d: pos=%v", tick, bus.Pos)
		}
	}
	if bus.Speed > 5 {
		t.Fatalf("expected second articulated bus to yield at rest outside narrow ring, got speed=%.2f", bus.Speed)
	}

	// Once the circulating bus clears the ring, the held bus must proceed cleanly.
	entered := false
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, nil, lw, 1.0/60.0)
		bus.Update(1.0/60.0, th, br, st, hb, doors)
		if bus.Pos.Distance(rb.Center) < rb.Radius {
			entered = true
			break
		}
	}
	if !entered {
		t.Fatalf("expected held articulated bus to enter after narrow ring cleared: pos=%v", bus.Pos)
	}
}

func TestRoundaboutSweepUnwindSteeringClearsNeighborGraze(t *testing.T) {
	const lw = 52.0
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	w.Roundabouts[0].Radius = 240
	w.Roundabouts[0].InnerRadius = 120
	rb := w.Roundabouts[0]
	fromStop := sim.NewBusStop(0, "south", sim.Vec2{X: 1.5 * lw, Y: 900}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
	toStop := sim.NewBusStop(1, "east", sim.Vec2{X: 900, Y: 1.5 * lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{fromStop, toStop}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	exiting := sim.NewBus(110.9, 179.6, -0.747)
	exiting.SetModel(sim.BusArticulated)
	exiting.SteeringAngle = 0.54
	sim.EnsureLongBusBays(w, route)

	neighbor := sim.NewBus(24.4, 171.2, -0.121)
	neighbor.SetModel(sim.BusArticulated)
	neighbor.SteeringAngle = -0.408
	obstacles := []VehicleObstacle{{
		Bus:     neighbor,
		Pos:     neighbor.Pos,
		Heading: neighbor.Heading,
		Speed:   0,
		Length:  neighbor.OverallLength(),
		Width:   neighbor.Width,
	}}

	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.PrepareLeg(w, fromStop, toStop, lw)

	start := exiting.Pos
	for tick := 0; tick < 300; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(exiting, route, w, nil, obstacles, lw, 1.0/60.0)
		exiting.Update(1.0/60.0, th, br, st, hb, doors)
		for _, ownPart := range exiting.BodyParts() {
			if ownPart.HalfLen <= 0 {
				continue
			}
			for _, otherPart := range neighbor.BodyParts() {
				if otherPart.HalfLen <= 0 {
					continue
				}
				if hit, _, depth := sim.CheckOBBCollision(ownPart, otherPart); hit && depth > 0.2 {
					t.Fatalf("exiting articulated bus collided with inner neighbor at tick %d: depth=%.2f pos=%v", tick, depth, exiting.Pos)
				}
			}
		}
		if exiting.Pos.Distance(rb.Center) > rb.Radius+40 {
			return
		}
	}
	t.Fatalf("expected wedged exiting articulated bus to unwind past roundabout: start=%v end=%v", start, exiting.Pos)
}
