package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestJunctionExitFitsEntireArticulatedBus(t *testing.T) {
	geometry := sim.NewJunctionGeometry(110, 110, 52)
	corridor := &sim.JunctionCorridor{Bodies: []sim.OBB{
		{Center: sim.Vec2{X: -180, Y: 26}},
		{Center: sim.Vec2{X: 260, Y: 26}},
	}}
	lead := VehicleObstacle{Pos: sim.Vec2{X: geometry.HalfX + 160, Y: 26}, Length: 46, Width: 22}
	bus := sim.NewBus(0, 0, 0)
	if exitQueueBlocksObstacle(corridor, geometry, bus, lead) {
		t.Fatal("downstream space is already sufficient for a compact bus")
	}
	bus.SetModel(sim.BusArticulated)
	if !exitQueueBlocksObstacle(corridor, geometry, bus, lead) {
		t.Fatal("admitting the articulated bus would leave its trailer in the junction")
	}
	lead.Pos.X += 125
	if exitQueueBlocksObstacle(corridor, geometry, bus, lead) {
		t.Fatal("exit queue should release the bus once both sections fit")
	}
}

func TestBusWaitsBeforeEntryUntilExitQueueClears(t *testing.T) {
	for _, movement := range []string{"straight", "left", "right"} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("%s/h%.2f", movement, h), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				out := u
				heading := h
				if movement == "left" {
					out = r.Mul(-1)
					heading = h - math.Pi/2
				}
				if movement == "right" {
					out = r
					heading = h + math.Pi/2
				}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-5000), P2: u.Mul(5000), Lanes: 6}, {P1: r.Mul(-5000), P2: r.Mul(5000), Lanes: 4}}}
				w.RebuildIntersections()
				a := sim.NewBusStop(0, "from", u.Mul(-4000), h, 6, sim.RGBA(0, 0, 0, 255), lw)
				lanes := 4
				if movement == "straight" {
					lanes = 6
				}
				z := sim.NewBusStop(1, "to", out.Mul(4000), heading, lanes, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = []*sim.BusStop{a, z}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				it := tls.Intersections[0]
				it.Phase = sim.PhaseEWGreen
				if movement == "left" {
					it.Phase = sim.PhaseEWLeftGreen
				}
				if math.Abs(u.Y) > .5 {
					if movement == "left" {
						it.Phase = sim.PhaseNSLeftGreen
					} else {
						it.Phase = sim.PhaseNSGreen
					}
				}
				line := it.GetStopLineDistForHeading(h, lw)
				side := 2.5 * lw
				if movement == "left" {
					side = .5 * lw
				}
				p := u.Mul(-line - 150).Add(r.Mul(side))
				bus := sim.NewBus(p.X, p.Y, h)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.PrepareLeg(w, a, z, lw)
				ad.trackPath(p)
				ad.publishJunctionCorridor(autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS}, 0)
				if bus.JunctionCorridor == nil {
					t.Fatal("missing route corridor")
				}
				bodies := bus.JunctionCorridor.Bodies
				last := bodies[len(bodies)-1]
				g := w.JunctionAt(sim.Vec2{}, lw)
				half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				q := last.Center.Add(out.Mul(half + 70 - last.Center.Dot(out)))
				obstacles := []VehicleObstacle{{Pos: q, Heading: heading, Length: 48, Width: 22}}
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				for tick := 0; tick < 900; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.Pos.Dot(u)+48*math.Abs(math.Cos(bus.Heading-h))+18*math.Abs(math.Sin(bus.Heading-h)) > -line+.5 {
						t.Fatal("bus entered blocked junction")
					}
				}
				for tick := 0; tick < 2400; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if bus.Pos.Dot(out) > half+250 {
						return
					}
				}
				t.Fatalf("bus did not resume after exit cleared: %v", bus.Pos)
			})
		}
	}
}

func TestArticulatedNeighborEnvelopeDoesNotBlockClearExitLane(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -4000, Y: 0}, P2: sim.Vec2{X: 4000, Y: 0}, Lanes: 8},
			{P1: sim.Vec2{X: 0, Y: -4000}, P2: sim.Vec2{X: 0, Y: 4000}, Lanes: 8},
		},
	}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -3.5 * lw, Y: -3000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -3.5 * lw, Y: 3000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	it := tls.Intersections[0]
	it.Phase = sim.PhaseNSGreen

	line := it.GetStopLineDistForHeading(math.Pi/2, lw)
	bus := sim.NewBus(-3.5*lw, -line-150, math.Pi/2)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	// Angled articulated neighbor wholly inside Lane 2 (-2.5*lw = -130):
	// its merged BodyEnvelope() widens beyond HalfWid=18 across -2.0*lw,
	// whereas both physical sections clear Lane 3 (-3.5*lw = -182).
	g := w.JunctionAt(sim.Vec2{}, lw)
	neighbor := sim.NewBus(-2.5*lw+6, g.HalfY+80, math.Pi/2-0.30)
	neighbor.TrailerHeading = math.Pi/2 + 0.15
	neighbor.TrailerInitialized = neighbor.IsArticulated()
	obstacles := []VehicleObstacle{ObserveBus(neighbor)}

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for tick := 0; tick < 600; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		if bus.Pos.Y > g.HalfY+150 {
			return
		}
	}
	t.Fatalf("bus in clear Lane 3 stalled at y=%.1f due to angled neighbor in Lane 2", bus.Pos.Y)
}
