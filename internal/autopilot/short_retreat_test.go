package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestProvedShortRetreatUsesRemainingDistance(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 6}}}
	b := sim.NewBus(0, 78, 0)
	b.SetModel(sim.BusArticulated)
	rear, _ := stationBodyRange(b, sim.Vec2{X: 1})
	ob := VehicleObstacle{Pos: sim.Vec2{X: rear - 14 - 23, Y: 78}, Heading: 0, Length: 46, Width: 22}
	ad := NewAutopilotDriver()
	f := autopilotFrame{bus: b, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ob}}
	if _, ok := ad.simulateReversePose(f, 8, 0); !ok {
		t.Fatal("fixture must have a fully proved short reverse")
	}
	ad.headonRecovery = &HeadonDeadlockRecovery{StartPos: b.Pos, TargetDistance: 8, ParallelRetreat: true, YieldHoldRemaining: 4}
	for tick := 0; tick < 60*6; tick++ {
		f.speedAbs = math.Abs(b.Speed)
		th, br, st, _ := ad.headonRecoveryControls(f, 1./60)
		b.Update(1./60, th, br, st, false, false)
		if sweepBodiesCollision(b.BodyParts(), ob, 0, 0, 0) {
			t.Fatal("reverse hit rear car")
		}
		if ad.headonRecovery.ReversingDone {
			if -b.Pos.X < 6 {
				t.Fatalf("short reverse aborted at %.2fpx", -b.Pos.X)
			}
			return
		}
	}
	t.Fatal("short reverse did not finish")
}
