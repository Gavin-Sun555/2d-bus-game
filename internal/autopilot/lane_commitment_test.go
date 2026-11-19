package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLeftTurnKeepsNearbyLegalLaneFromLegacySnapshot(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(heading), func(t *testing.T) {
			const lw = 52.
			w, in, _, _ := wideLeftExitWorld(lw, heading, true)
			right := sim.Vec2{X: -in.Y, Y: in.X}
			bus := sim.NewBus(0, 0, heading)
			bus.Pos = in.Mul(-1100).Add(right.Mul(.5 * lw))
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			ad.holdLaneOffset(-3 * lw)
			// Older saves already have the junction latch, but no committed lane.
			junction := sim.Vec2{}
			ad.busLaneJunction = &junction
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			frame := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, remaining: 5000, targetStop: w.Stops[1]}
			frame.s, _ = ad.trackPath(bus.Pos)
			if !ad.updateBusLane(frame, 1./60) || !ad.ordinaryBusLeft || !ad.busLaneTurnChosen {
				t.Fatal("did not retain nearby median-left lane")
			}
			if math.Abs(ad.TargetLaneOffset) > 1 {
				t.Fatalf("unnecessary return across lanes: %.1f", ad.TargetLaneOffset)
			}
			snapshot := ad.Snapshot()
			restored, err := snapshot.Restore(w)
			if err != nil {
				t.Fatal(err)
			}
			if !restored.busLaneTurnChosen {
				t.Fatal("lost committed lane after restore")
			}
			// A later median queue cannot reverse an already selected turn corridor.
			frame.s, _ = ad.trackPath(bus.Pos)
			frame.obstacles = []VehicleObstacle{{Pos: bus.Pos.Add(in.Mul(200)), Heading: heading, Length: 48, Width: 22}}
			for i := 0; i < 120; i++ {
				ad.updateBusLane(frame, 1./60)
			}
			if !ad.ordinaryBusLeft || math.Abs(ad.TargetLaneOffset) > 1 {
				t.Fatal("queue changed committed left-turn lane")
			}
		})
	}
}

func TestBusLaneChangeKeepsTargetThroughQueueFluctuations(t *testing.T) {
	const lw = 52.
	w := busLaneTestWorld(lw)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.CanSkipTargetStop = true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	bus := sim.NewBus(-1800, 3.5*lw, 0)
	frame := autopilotFrame{bus: bus, world: w, laneWidth: lw, remaining: 5000, targetStop: w.Stops[1], speedAbs: 100}
	frame.s, _ = ad.trackPath(bus.Pos)
	frame.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1300, Y: 3.5 * lw}, Heading: 0, Length: 48, Width: 22}}
	ad.updateBusLane(frame, 1./60)
	if ad.TargetLaneOffset != -lw {
		t.Fatalf("did not bypass stopped queue: %.1f", ad.TargetLaneOffset)
	}
	// While crossing, the stopped queue disappears and another lane looks better.
	frame.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1300, Y: 2.5 * lw}, Heading: 0, Length: 48, Width: 22}}
	for i := 0; i < 120; i++ {
		ad.updateBusLane(frame, 1./60)
		if ad.TargetLaneOffset != -lw {
			t.Fatalf("reversed target during change at tick%d", i)
		}
	}
	// Once settled, equally clear lanes do not cause a gratuitous curb return.
	bus.Pos.Y = 2.5 * lw
	ad.holdLaneOffset(-lw)
	frame.obstacles = nil
	for i := 0; i < 600; i++ {
		ad.updateBusLane(frame, 1./60)
		if ad.TargetLaneOffset != -lw {
			t.Fatalf("abandoned equally clear lane at tick%d", i)
		}
	}
	// A genuinely slower lead can justify a new decision after settling.
	frame.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1300, Y: 2.5 * lw}, Heading: 0, Speed: 90, Length: 48, Width: 22}}
	for i := 0; i < 600; i++ {
		ad.updateBusLane(frame, 1./60)
	}
	if ad.TargetLaneOffset != 0 {
		t.Fatal("did not choose faster clear curb lane after settling")
	}
}

func TestBusLaneChoiceRestartsReevaluationDelay(t *testing.T) {
	const lw = 52.
	w := busLaneTestWorld(lw)
	ad := NewAutopilotDriver()
	ad.BerserkMode, ad.CanSkipTargetStop = true, true
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	bus := sim.NewBus(-1800, 3.5*lw, 0)
	f := autopilotFrame{bus: bus, world: w, laneWidth: lw, remaining: 5000, targetStop: w.Stops[1], speedAbs: 100}
	f.s, _ = ad.trackPath(bus.Pos)
	f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1300, Y: 3.5 * lw}, Length: 46, Width: 22}}
	ad.OvertakeTimer = 10
	ad.updateBusLane(f, 1./60)
	if ad.TargetLaneOffset != -lw || ad.OvertakeTimer > .1 {
		t.Fatal("new lane choice retained the previous evaluation timer")
	}
	bus.Pos.Y = 2.5 * lw
	ad.holdLaneOffset(-lw)
	f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1300, Y: 2.5 * lw}, Length: 46, Width: 22}}
	for i := 0; i < 120; i++ {
		ad.updateBusLane(f, 1./60)
		if ad.TargetLaneOffset != -lw {
			t.Fatal("rescored immediately after completing the lane change")
		}
	}
	for i := 0; i < 300; i++ {
		ad.updateBusLane(f, 1./60)
	}
	if ad.TargetLaneOffset != 0 {
		t.Fatal("did not reconsider a faster lane after the evaluation delay")
	}
}
