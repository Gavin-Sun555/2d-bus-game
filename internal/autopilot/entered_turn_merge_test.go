package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestEnteredCurbLeftTurnReleasesStaleApproachMerge(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
		{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 4},
	}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{Y: -2000}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{X: 2000}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)}
	bus := sim.NewBus(-123.36522303098, -177.34277639698908, 1.0346903544411532)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading = 1.2887612738967313
	bus.SteeringAngle = -.1422415428194011
	bus.JunctionCorridor = &sim.JunctionCorridor{Center: sim.Vec2{}, EntryHeading: math.Pi / 2, Entered: true, Remaining: .3}
	leader := sim.NewBus(-182, 52.82205022499, math.Pi/2)
	leader.SetModel(sim.BusArticulated)
	ad := NewAutopilotDriver()
	ad.articulatedPath = true
	ad.DisallowOncomingOvertake = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	center := sim.Vec2{}
	ad.busLaneJunction = &center
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	lights := sim.NewTrafficLightSystem()
	lights.BuildForWorld(w)
	for _, inter := range lights.Intersections {
		inter.Phase = sim.PhaseEWGreen
	}
	for tick := 0; tick < 1200; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, lights, []VehicleObstacle{ObserveBus(leader)}, lw, 1./60)
		if tick == 0 && (th <= 0 || br > 0) {
			t.Fatalf("entered turn retained approach stop: entered%v target%.3f", ad.busJunctionEntered, ad.LastTargetSpeed)
		}
		bus.Update(1./60, th, br, st, hb, doors)
		if hit, _, _ := sim.CheckBusesCollision(bus, leader); hit {
			t.Fatal("turn contacted the adjacent straight bus")
		}
		if bus.CheckBoundaries(w, lw).Zone == sim.ZoneHardBarrier {
			t.Fatal("turn crossed road boundary")
		}
		if bus.Pos.X > 700 && math.Abs(sim.NormalizeAngle(bus.Heading)) < .1 {
			return
		}
	}
	t.Fatalf("turn did not clear pos%v speed%.1f", bus.Pos, bus.Speed)
}

func TestStationMergeReservationRemainsUntilFrontBerth(t *testing.T) {
	const lw = 52.
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
	stop := sim.NewBusStop(0, "stop", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{stop}
	bus := sim.NewBus(stop.FrontBerthPosition().X-3, 130, 0)
	bus.SetModel(sim.BusArticulated)
	ad := NewAutopilotDriver()
	ad.requestStationMerge(autopilotFrame{bus: bus, world: w, targetStop: stop, laneWidth: lw})
	if bus.StationMergeRequest == nil || bus.LaneMergeRequest == nil || bus.LaneMergeRequest.Target != stop.Pos {
		t.Fatal("late bay entry lost its merge reservation after passing the station center")
	}
	bus.DoorsOpen = true
	bus.LaneMergeRequest, bus.StationMergeRequest = nil, nil
	ad.requestStationMerge(autopilotFrame{bus: bus, world: w, targetStop: stop, laneWidth: lw})
	if bus.LaneMergeRequest != nil || bus.StationMergeRequest != nil {
		t.Fatal("boarding bus retained an arrival merge reservation")
	}
}
