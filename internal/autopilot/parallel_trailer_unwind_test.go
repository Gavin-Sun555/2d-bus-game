package autopilot

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func parallelTrailerFixture() (*AutopilotDriver, autopilotFrame, float64) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000, Y: -7200}, P2: sim.Vec2{X: 3000, Y: -7200}, Lanes: 6}}}
	leader := sim.NewBus(325.08113219416606, -7265.221714727524, -3.1370393284206726)
	leader.SetModel(sim.BusArticulated)
	leader.TrailerHeading, leader.TrailerInitialized, leader.SteeringAngle = 3.1397138951779735, true, .44060169107606867
	rear := sim.NewBus(373.5804326317482, -7226.056154378575, 3.1400493649643613)
	rear.SetModel(sim.BusArticulated)
	rear.TrailerHeading, rear.TrailerInitialized = 3.1348818826747835, true
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	return ad, autopilotFrame{bus: leader, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ObserveBus(rear)}}, .8159290575482753
}
func TestParallelTrailerCentersWheelsBeforeStraightCrawl(t *testing.T) {
	ad, f, desired := parallelTrailerFixture()
	if !ad.parallelTrailerStraightClear(f, desired) {
		t.Fatal("safe straight release rejected")
	}
	start := f.bus.Pos
	for tick := 0; tick < 60*20; tick++ {
		st, target := desired, 16.
		if ad.parallelTrailerStraightClear(f, desired) {
			st = 0
			if f.bus.SteeringAngle > .02 {
				target = 0
			}
		} else if ad.articulatedSweptSpeed(f, desired, true) < 16 || ad.controlSweptSpeed(f, desired) < 16 {
			target = 0
		}
		th, br := ad.longitudinalControls(f.bus.Speed, target)
		f.bus.Update(1./60, th, br, st, false, false)
		f.speedAbs = f.bus.Speed
		if hit, _, _ := sim.CheckBusesCollision(f.bus, f.obstacles[0].Bus); hit {
			t.Fatal("straight crawl hit follower")
		}
	}
	if f.bus.Pos.Distance(start) < 10 {
		t.Fatal("straight crawl made no progress")
	}
}

func TestParallelTrailerStraightCrawlGuards(t *testing.T) {
	for _, name := range []string{"player", "doors", "service", "angled cab", "angled tail", "speed", "junction", "straight path", "front occupied"} {
		t.Run(name, func(t *testing.T) {
			ad, f, desired := parallelTrailerFixture()
			switch name {
			case "player":
				ad.FleetBus = false
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 1)
			case "angled cab":
				f.bus.Heading += .1
			case "angled tail":
				f.bus.TrailerHeading += .1
			case "speed":
				f.speedAbs = 20
			case "junction":
				f.world.Intersections = []sim.Vec2{f.bus.Pos}
			case "straight path":
				desired = 0
			case "front occupied":
				front := sim.NewBus(220, -7265, -3.14)
				f.obstacles = append(f.obstacles, ObserveBus(front))
			}
			if ad.parallelTrailerStraightClear(f, desired) {
				t.Fatal("unsafe straight crawl accepted")
			}
		})
	}
}
