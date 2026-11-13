package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/parallel-road-arrival-bus27-2026-11-08.json
var bus27ArrivalFixture []byte

func loadBus27Arrival(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if err := json.Unmarshal(bus27ArrivalFixture, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: x.Bus, world: x.World, tls: x.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: x.World.Stops[x.Route.GetCurrentTargetStopID()]}
	for _, c := range x.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
	}
	for _, b := range x.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, x.Route
}
func TestSavedBus27ParallelDepartureOffsetReachesPlatform(t *testing.T) {
	ad, f, route := loadBus27Arrival(t)
	passengers, index, health := f.bus.PassengerCount, route.CurrentIndex, f.bus.Durability
	ad.BeforeRecovery = func() { t.Fatal("repositioned bus instead of driving to stop") }
	for tick := 0; tick < 2400; tick++ {
		before := f.bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
		f.bus.Update(1./60, th, br, st, hb, doors)
		if f.bus.Pos.Distance(before) > math.Abs(f.bus.Speed)/60+.05 || f.bus.PassengerCount != passengers || route.CurrentIndex != index || f.bus.Durability < health {
			t.Fatal("vehicle or journey was reset")
		}
		if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatalf("touched hard boundary tick%d", tick)
		}
		for _, ob := range f.obstacles {
			if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 0, 0) {
				t.Fatal("touched traffic")
			}
		}
		if ad.State == AutopilotStateDocked && f.bus.DoorsOpen {
			if ad.DockedStopID != 32 {
				t.Fatal("docked at wrong stop")
			}
			t.Logf("bus27 docked normally in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("bus27 still held %v", f.bus.Pos)
}
func TestSkippedDepartureHoldRequiresOriginalRoadArm(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		s := sim.NewBusStop(0, "source", sim.Vec2{}, h, 6, sim.RGBA(0, 0, 0, 255), 52)
		w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-2000), P2: u.Mul(2000), Lanes: 6}, {P1: u.Mul(-2000).Add(r.Mul(900)), P2: u.Mul(2000).Add(r.Mul(900)), Lanes: 6}}}
		p := u.Mul(-50).Add(r.Mul(130))
		b := sim.NewBus(p.X, p.Y, h)
		ad := NewAutopilotDriver()
		ad.pathFromID = 0
		ad.drivePath = []sim.Vec2{p, p.Add(u.Mul(500))}
		ad.skippedDeparture = &skippedStopDeparture{stop: s, position: p.Add(r.Mul(20))}
		if !ad.holdSkippedDepartureLane(b, w, 52, 1./60, 0) || math.Abs(ad.TargetLaneOffset-20) > 1e-6 {
			t.Fatal("lost real same-road departure hold")
		}
		b.Pos = b.Pos.Add(r.Mul(900))
		ad.TargetLaneOffset = 0
		if ad.holdSkippedDepartureLane(b, w, 52, 1./60, 0) || ad.TargetLaneOffset != 0 {
			t.Fatal("old departure hold contaminated parallel road")
		}
	}
}
func TestBus27GeometryCrawlKeepsActualFrontObstacle(t *testing.T) {
	ad, f, route := loadBus27Arrival(t)
	p := f.bus.Pos.Add(sim.UnitFromAngle(f.bus.Heading).Mul(f.bus.Length/2 + 23 + 4))
	ob := VehicleObstacle{Pos: p, Heading: f.bus.Heading, Length: 46, Width: 22}
	f.obstacles = append(f.obstacles, ob)
	start := f.bus.Pos
	for tick := 0; tick < 120; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
		f.bus.Update(1./60, th, br, st, hb, doors)
		if sweepBodiesCollision(f.bus.BodyParts(), ob, 0, 0, 0) {
			t.Fatal("crawl ignored physical front obstacle")
		}
	}
	if f.bus.Pos.Distance(start) > 1 {
		t.Fatal("moved into blocked front gap")
	}
}
