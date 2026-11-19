package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestMedianStraddlingSavedLaneChangeRecovers(t *testing.T) {
	const lw = 52.
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			u := sim.UnitFromAngle(heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 6}}}
			w.Stops = []*sim.BusStop{
				sim.NewBusStop(0, "from", u.Mul(-2000).Add(r.Mul(26)), heading, 6, sim.RGBA(0, 0, 0, 255), lw),
				sim.NewBusStop(1, "to", u.Mul(2000).Add(r.Mul(26)), heading, 6, sim.RGBA(0, 0, 0, 255), lw),
			}
			// Slot 2 on November 16: the cab center is 0.013 pixels beyond
			// the median, while the trailer still points across the lane change.
			b := sim.NewBus(0, 0, heading-.0126403704)
			b.SetModel(sim.BusArticulated)
			b.Pos = r.Mul(-.01329553325)
			b.TrailerHeading, b.TrailerInitialized = heading-.2553842417, true
			b.SteeringAngle = .05685160006
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			initial := roadMedianClearance(b, b.Pos, b.Heading, heading, 0, math.Abs(u.X) > .5)
			for tick := 0; tick < 60*20; tick++ {
				before := b.Pos
				th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
				b.Update(1./60, th, br, st, hb, doors)
				if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.001 {
					t.Fatal("recovery relocated the bus")
				}
				if clearance := roadMedianClearance(b, b.Pos, b.Heading, heading, 0, math.Abs(u.X) > .5); clearance < initial-.05 {
					t.Fatalf("median intrusion worsened: %.3f < %.3f", clearance, initial)
				}
			}
			if ad.roadSafetyHold || b.Pos.Dot(u) < 150 {
				t.Fatalf("straddling bus remained stuck: pos%v hold%v", b.Pos, ad.roadSafetyHold)
			}
		})
	}
}

func TestSavedSlot2WestboundMedianOverhangRecovers(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -20000, Y: -7200}, P2: sim.Vec2{X: -10000, Y: -7200}, Lanes: 4}}}
	w.Stops = []*sim.BusStop{
		sim.NewBusStop(0, "from", sim.Vec2{X: -12000, Y: -7226}, math.Pi, 4, sim.RGBA(0, 0, 0, 255), lw),
		sim.NewBusStop(1, "to", sim.Vec2{X: -18000, Y: -7226}, math.Pi, 4, sim.RGBA(0, 0, 0, 255), lw),
	}
	b := sim.NewBus(-14920.069589, -7210.712931, -2.97963358)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = -3.13600576, true
	b.SteeringAngle = -.1317158
	ad := NewAutopilotDriver()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	initial := roadMedianClearance(b, b.Pos, b.Heading, math.Pi, -7200, true)
	for tick := 0; tick < 60*15; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if clearance := roadMedianClearance(b, b.Pos, b.Heading, math.Pi, -7200, true); clearance < initial-.05 {
			t.Fatalf("median intrusion worsened at tick %d: %.3f < %.3f", tick, clearance, initial)
		}
	}
	if ad.roadSafetyHold || b.Pos.X > -15020 {
		t.Fatalf("saved player pose remained stuck: pos%v hold%v", b.Pos, ad.roadSafetyHold)
	}
}

func TestNearlyParallelMedianEncroachingArticulatedBusRecovers(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -7800, Y: 0}, P2: sim.Vec2{X: -7800, Y: 10000}, Lanes: 8},
		},
	}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -7826, Y: 1000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -7826, Y: 8000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	bus := sim.NewBus(-7818.2519, 6474.8009, 1.5585)
	bus.TrailerHeading = 1.1545
	bus.TrailerInitialized = bus.IsArticulated()
	bus.SteeringAngle = 0.020

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.FleetBus = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	startY := bus.Pos.Y
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		clearance := roadMedianClearance(bus, bus.Pos, bus.Heading, math.Pi/2, -7800, false)
		if clearance < -1.2 {
			t.Fatalf("tick %d worsened median clearance to %.3f", tick, clearance)
		}
	}
	if ad.roadSafetyHold {
		t.Fatalf("bus remained frozen in roadSafetyHold at pos=%+v h=%.4f", bus.Pos, bus.Heading)
	}
	if bus.Pos.Y < startY+80 {
		t.Fatalf("bus failed to advance after recovery: y=%.2f (start %.2f)", bus.Pos.Y, startY)
	}
}

func TestMedianRecoveryIgnoresAlreadyTouchingRearFollower(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -7800, Y: 0}, P2: sim.Vec2{X: -7800, Y: 10000}, Lanes: 8},
		},
	}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -7826, Y: 1000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -7826, Y: 8000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	bus := sim.NewBus(-7817.7, 6505.4, 1.574)
	bus.TrailerHeading = 1.571
	bus.TrailerInitialized = bus.IsArticulated()

	follower := sim.NewBus(-7826.0, 6357.0, math.Pi/2)
	follower.TrailerHeading = math.Pi / 2
	follower.TrailerInitialized = follower.IsArticulated()
	obs := []VehicleObstacle{ObserveBus(follower)}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.FleetBus = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	startY := bus.Pos.Y
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if ad.roadSafetyHold || bus.Pos.Y < startY+60 {
		t.Fatalf("rear follower trapped median recovery: hold=%v pos=%+v", ad.roadSafetyHold, bus.Pos)
	}
}

func TestShallowOverhangMedianEncroachingArticulatedBusRecovers(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -7800, Y: 0}, P2: sim.Vec2{X: -7800, Y: 10000}, Lanes: 8},
		},
	}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -7826, Y: 1000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -7826, Y: 8000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	bus := sim.NewBus(-7816.2, 6451.6, 1.560)
	bus.TrailerHeading = 1.560
	bus.TrailerInitialized = bus.IsArticulated()

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.FleetBus = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	startY := bus.Pos.Y
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if ad.roadSafetyHold || bus.Pos.Y < startY+60 {
		cl := roadMedianClearance(bus, bus.Pos, bus.Heading, math.Pi/2, -7800, false)
		t.Fatalf("shallow median encroachment froze: hold=%v pos=%+v h=%.4f th=%.4f steer=%.3f maxSteer=%.3f wb=%.1f cl=%.3f",
			ad.roadSafetyHold, bus.Pos, bus.Heading, bus.TrailerHeading, bus.SteeringAngle, bus.MaxSteerAngle, bus.WheelBase, cl)
	}
}

func TestTrailerGrazeBesideArticulatedBusAllowsMedianRecovery(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -3000, Y: 0}, P2: sim.Vec2{X: 3000, Y: 0}, Lanes: 4},
		},
	}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -2800, Y: 26}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -1200, Y: 26}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}

	bus := sim.NewBus(-2165.905, 13.074, -0.02874)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = -0.46935
	bus.TrailerInitialized = true
	bus.SteeringAngle = 0.04656

	sideCar := VehicleObstacle{
		Pos:     sim.Vec2{X: -2253.3, Y: 78.0},
		Heading: 0,
		Speed:   0,
		Length:  46,
		Width:   22,
	}
	obs := []VehicleObstacle{sideCar}

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.FleetBus = true
	ad.PrepareLeg(w, from, to, lw)
	ad.trackPath(bus.Pos)

	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	startX := bus.Pos.X
	for tick := 0; tick < 360; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
	}
	if ad.roadSafetyHold {
		t.Fatalf("bus remained frozen in roadSafetyHold with sidecar behind cab: pos=%+v h=%.4f", bus.Pos, bus.Heading)
	}
	if bus.Pos.X < startX+60 {
		t.Fatalf("bus failed to advance after recovery: x=%.2f (start %.2f)", bus.Pos.X, startX)
	}
}
