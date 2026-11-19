package autopilot

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func reverseYieldFixture() (*AutopilotDriver, autopilotFrame) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}}}
	b := sim.NewBus(0, 78, 0)
	b.SetModel(sim.BusArticulated)
	b.ReverseYieldRequest = .6
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	return ad, autopilotFrame{bus: b, world: w, laneWidth: 52}
}

func TestReverseYieldRelaysWithoutMovingBlockedBus(t *testing.T) {
	ad, f := reverseYieldFixture()
	car := &sim.TrafficCar{Pos: sim.Vec2{X: -178, Y: 78}, Heading: 0, Length: 46, Width: 22}
	f.obstacles = []VehicleObstacle{{Car: car, Pos: car.Pos, Heading: 0, Length: 46, Width: 22}}
	start := f.bus.Pos
	if distance := ad.requestedReverseYieldDistance(f); distance != 0 || car.YieldReverseRequest <= 0 {
		t.Fatalf("blocked bus did not relay request: distance%.1f request%.1f", distance, car.YieldReverseRequest)
	}
	if f.bus.Pos != start || car.Pos.X != -178 || car.ReverseRemaining != 0 {
		t.Fatal("relay moved a vehicle before its own controller checked the reverse")
	}

	ad, f = reverseYieldFixture()
	rear := sim.NewBus(-193, 78, 0)
	rear.SetModel(sim.BusArticulated)
	f.obstacles = []VehicleObstacle{ObserveBus(rear)}
	if distance := ad.requestedReverseYieldDistance(f); distance != 0 || rear.ReverseYieldRequest <= 0 {
		t.Fatal("boxed bus did not relay to its rear fleet bus")
	}
	receiver := f
	receiver.bus = rear
	receiver.obstacles = []VehicleObstacle{ObserveBus(f.bus)}
	if distance := ad.requestedReverseYieldDistance(receiver); distance <= 0 {
		t.Fatal("rear bus rejected a clear full-body retreat")
	}
}

func TestReverseYieldRetainsServiceAndPlayerGuards(t *testing.T) {
	for _, scenario := range []string{"player", "doors", "service", "moving", "angled"} {
		t.Run(scenario, func(t *testing.T) {
			ad, f := reverseYieldFixture()
			switch scenario {
			case "player":
				ad.FleetBus = false
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 1)
			case "moving":
				f.speedAbs = 2
			case "angled":
				f.bus.Heading = .4
			}
			if distance := ad.requestedReverseYieldDistance(f); distance > 0 {
				t.Fatalf("unsafe yield for %s", scenario)
			}
		})
	}
}
