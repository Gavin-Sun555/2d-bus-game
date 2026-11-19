package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSlot2OuterCarBesideInnerBus(t *testing.T) {
	// The car was separated outward at the CBD entrance. Its old automatic
	// inward recenter crossed bus 163's cab while both vehicles waited.
	rb := sim.Roundabout{Radius: 240, InnerRadius: 120}
	c := &sim.TrafficCar{ID: 11547, Pos: sim.Vec2{X: 70.85355678, Y: -248.4429706}, Heading: -2.764871485, Length: 46, Width: 22, TargetSpeed: 277, InRoundabout: true, RoundaboutRadius: 258.3488652, RoundaboutAngle: -1.292981057, RoundaboutTargetR: 214, RoundaboutOuterRadius: 240, RoundaboutExitAngle: -2.7685119, RoundaboutExitHeading: math.Pi, RoundaboutTraveled: .9379575, RoundaboutExitOffset: 78, RoundaboutExitLane: 1}
	b := sim.NewBus(90.13650561, -195.37472353, -2.55345677)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized = -2.33466567, true
	w := &sim.World{Roundabouts: []sim.Roundabout{rb}}
	tm := NewTrafficManager(52)
	tm.Cars = []*sim.TrafficCar{c}
	f := carFrame{car: c, world: w, buses: []*sim.Bus{b}, activeCount: 1, index: 0, laneWidth: 52}
	if roundaboutBusPathClear(c, b) {
		t.Fatal("fixture must reproduce blocked inward return")
	}
	if !tm.holdDisplacedRoundaboutLane(f) || !roundaboutBusPathClear(c, b) {
		t.Fatal("could not keep a clear outer trajectory")
	}
	c.RoundaboutTargetR = 214
	start := c.Pos
	for tick := 0; tick < 300; tick++ {
		before := c.Pos
		tm.StepCar(1./60, c, 0, b, nil, 1, 52, w)
		if hit, _, _ := b.CollidesBody(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 23, HalfWid: 11}); hit {
			t.Fatal("outer trajectory hit articulated bus")
		}
		if c.RoundaboutTargetR != 214 {
			t.Fatal("temporary hold lost the original lane target")
		}
		if c.Pos.Distance(before) > math.Abs(c.Speed)/60*1.2+.01 {
			t.Fatal("outer recovery snapped position")
		}
	}
	if c.Pos.Distance(start) < 120 {
		t.Fatal("car remained deadlocked beside bus")
	}
}

func TestRedQueueDoesNotHideYawedBusWedge(t *testing.T) {
	car := &sim.TrafficCar{Pos: sim.Vec2{X: 681.5719758, Y: -7226}, Heading: math.Pi, Length: 46, Width: 22, RoadCenter: -7200, IsEW: true, LaneIndex: 0}
	bus := sim.NewBus(615.6200714, -7241.5990146, 2.762851643)
	bus.SetModel(sim.BusArticulated)
	bus.TrailerHeading, bus.TrailerInitialized = 3.01911356, true
	it := &sim.TrafficIntersection{Center: sim.Vec2{Y: -7200}, IsMajor: true, ApproachLanes: [4]int{6, 6, 6, 6}, Phase: sim.PhaseNSGreen}
	tm := NewTrafficManager(52)
	f := carFrame{car: car, laneWidth: 52, buses: []*sim.Bus{bus}, tls: &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{it}}}
	if !tm.carHasRedSignalAhead(f, 1200) {
		t.Fatal("fixture must have a red signal ahead")
	}
	if !tm.carBlockedByStoppedBus(f, true) {
		t.Fatal("distant red signal concealed a diagonal bus wedge")
	}
	bus.Heading = math.Pi
	bus.ResetArticulation()
	if tm.carBlockedByStoppedBus(f, true) {
		t.Fatal("ordinary red queue was treated as a wedge")
	}
}

func TestSlightlyBentTrailerDoesNotHideCarReverse(t *testing.T) {
	for _, player := range []bool{false, true} {
		t.Run(fmt.Sprintf("player%v", player), func(t *testing.T) {
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 8000, Y: 7200}, P2: sim.Vec2{X: 14000, Y: 7200}, Lanes: 6}}}
			b := sim.NewBus(11061.736407022661, 7231.037235931465, -.17417262933294492)
			b.SetModel(sim.BusArticulated)
			b.TrailerHeading = -.291
			b.TrailerInitialized = true
			b.SteeringAngle = .178
			c := &sim.TrafficCar{ID: 7707, Pos: sim.Vec2{X: 10936.474733668612, Y: 7226}, Heading: 0, Length: 46, Width: 22, TargetSpeed: 166, IsBraking: true, RoadCenter: 7200, IsEW: true, LaneIndex: 0}
			tm := NewTrafficManager(52)
			tm.Cars = []*sim.TrafficCar{c}
			var p *sim.Bus
			if player {
				p = b
			} else {
				tm.AIBusMgr = &AIBusManager{Buses: []*AIBus{{ID: 172, Bus: b}}}
			}
			tm.BeforeRecovery = func(e sim.TrafficRecoveryEvent) { t.Fatal("reset instead of reversing", e) }
			start, minX := c.Pos.X, c.Pos.X
			for tick := 0; tick < 60*9; tick++ {
				tm.resolveDeadlocks(1./60, p, 1, 52, w, nil, false)
				tm.StepCar(1./60, c, 0, p, nil, 1, 52, w)
				minX = math.Min(minX, c.Pos.X)
				if hit, _, _ := b.CollidesBody(sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: 23, HalfWid: 11}); hit {
					t.Fatal("car hit the bent trailer")
				}
			}
			if start-minX < 20 {
				t.Fatalf("trailer still disguised as ordinary queue: retreat%.2f timer%.2f", start-minX, c.DeadlockTimer)
			}
		})
	}
}
