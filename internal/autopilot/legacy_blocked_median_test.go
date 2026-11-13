package autopilot

import (
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/legacy-blocked-median.json
var legacyBlockedMedianFixture []byte

func TestStrandedFleetMedianApproachWaitsForClearLane(t *testing.T) {
	const lw = 52.
	junction := sim.Vec2{X: 1600}
	world := &sim.World{
		Roads:         []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Type: sim.RoadBusBoulevard8Lane, Lanes: 8}},
		Intersections: []sim.Vec2{junction},
	}
	bus := sim.NewBus(0, -130, math.Pi/2)
	bus.SetModel(sim.BusArticulated)
	bus.BoardPassengers(1, []int{2})
	origin := bus.Pos
	ad := NewAutopilotDriver()
	ad.ordinaryBusLeft = true
	ad.busLaneJunction = &junction
	ad.drivePath = []sim.Vec2{{X: -300, Y: 26}, {X: 1300, Y: 26}}
	ad.cumLen = []float64{0, 1600}
	ad.LastPathS = 300
	for tick := 0; tick < 10; tick++ {
		if ad.RecoverStrandedMedianApproach(bus, world, nil, lw, .9) {
			t.Fatal("recovered before verifying a sustained stop")
		}
	}
	var occupied []VehicleObstacle
	for lane := 0; lane < 4; lane++ {
		occupied = append(occupied, VehicleObstacle{Pos: sim.Vec2{Y: (float64(lane) + .5) * lw}, Length: 48, Width: 24})
	}
	if ad.RecoverStrandedMedianApproach(bus, world, occupied, lw, 1) || bus.Pos != origin {
		t.Fatal("recovered into an occupied lane")
	}
	if !ad.RecoverStrandedMedianApproach(bus, world, occupied[1:], lw, 1) {
		t.Fatal("clear legal lane did not release stranded bus")
	}
	if bus.Pos != (sim.Vec2{Y: 26}) || bus.Heading != 0 || bus.TailHeading() != 0 || bus.PassengerCount != 1 {
		t.Fatalf("incorrect recovered state: position %v, heading %.2f, passengers %d", bus.Pos, bus.Heading, bus.PassengerCount)
	}
}

// Reduced from the old save that remained stationary through multiple green
// phases: a +68px frozen path offset, two nearby cars and the following fleet.
func TestLegacyBlockedMedianResumesCurbLeftWithoutContact(t *testing.T) {
	var f struct {
		Player struct {
			Pos                                       sim.Vec2
			Heading, Speed, SteeringAngle, Durability float64
		}
		Driver       DriverSnapshot
		Obstacles    []VehicleObstacle
		Intersection sim.TrafficIntersection
	}
	if err := json.Unmarshal(legacyBlockedMedianFixture, &f); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	tls := &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{&f.Intersection}}
	bus := sim.NewBus(f.Player.Pos.X, f.Player.Pos.Y, f.Player.Heading)
	bus.Speed = f.Player.Speed
	bus.SteeringAngle = f.Player.SteeringAngle
	bus.Durability = f.Player.Durability
	ad, err := f.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	route := &sim.Route{StopIDs: []int{279, 99}, CurrentIndex: 1}
	initial := bus.Pos
	cleared := false
	for tick := 0; tick < 3600; tick++ {
		tls.Update(1. / 60)
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, f.Obstacles, 52, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		body := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
		for _, ob := range f.Obstacles {
			if hit, _, _ := sim.CheckOBBCollision(body, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}); hit {
				t.Fatalf("recovery hit %v at tick%d", ob.Pos, tick)
			}
		}
		if bus.Pos.X < tls.Intersections[0].Center.X-w.JunctionAt(tls.Intersections[0].Center, 52).HalfX-bus.Length && math.Cos(bus.Heading-math.Pi) > .95 {
			cleared = true
			break
		}
	}
	if !cleared {
		t.Fatalf("saved partial merge stayed blocked: start%v pos%v diagnosis%+v", initial, bus.Pos, ad.DiagnoseStop(bus, route, w, tls, f.Obstacles, 52))
	}
	if ad.pathRevision != currentDrivePathRevision || ad.ordinaryBusLeft {
		t.Fatal("did not reconnect to the current curb-left route")
	}
}
