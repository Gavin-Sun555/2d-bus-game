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
