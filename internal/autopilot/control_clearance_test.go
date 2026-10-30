package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// The saved westbound pair has 1.6px of true clearance at the yawed rear
// vehicle's nose, but the 1+1px prediction padding overlaps. The foremost bus
// can clear the pair while the bus needing to merge remains stopped.
func paddingPair(heading float64, leadModel, rearModel sim.BusModel) (autopilotFrame, VehicleObstacle) {
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	lead := sim.NewBus(0, 0, heading)
	lead.SetModel(leadModel)
	p := u.Mul(-31.840506128391).Add(r.Mul(-42.005062327884))
	rear := sim.NewBus(p.X, p.Y, heading+.094468018526151)
	rear.SetModel(rearModel)
	rear.TrailerHeading = heading + .054695104970652
	rear.TrailerInitialized = true
	ob := ObserveBus(rear)
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 4}}}
	return autopilotFrame{bus: lead, world: w, laneWidth: 52, obstacles: []VehicleObstacle{ob}}, ob
}
func TestControlPaddingLetsPhysicalLeaderClear(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lm := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			for _, rm := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
				t.Run(fmt.Sprintf("h%.2f/lead%d/rear%d", h, lm, rm), func(t *testing.T) {
					f, ob := paddingPair(h, lm, rm)
					ad := NewAutopilotDriver()
					if got := ad.controlSweptSpeed(f, 0); got != 35 {
						t.Fatalf("safe leader stopped: %g", got)
					}
					// Already rolling at the release cap must continue clearing, without
					// alternating between stopping and restarting every frame.
					f.bus.Speed = 35
					f.speedAbs = 35
					if got := ad.controlSweptSpeed(f, 0); got != 35 {
						t.Fatalf("continuing clearance stopped: %g", got)
					}
					// The rear vehicle does not acquire the same exemption.
					rearFrame := f
					rearFrame.bus = ob.Bus
					rearFrame.speedAbs = 0
					if controlPaddingRelease(rearFrame, ObserveBus(f.bus), 0, 20, .32) {
						t.Fatal("both buses granted priority")
					}
				})
			}
		}
	}
}
func TestControlPaddingRetainsCollisionAndOtherTrafficChecks(t *testing.T) {
	t.Run("actual side contact", func(t *testing.T) {
		f, ob := paddingPair(0, sim.BusStandard, sim.BusArticulated)
		ob.Bus.Pos.Y += 2
		f.obstacles = []VehicleObstacle{ObserveBus(ob.Bus)}
		if got := NewAutopilotDriver().controlSweptSpeed(f, 0); got != 0 {
			t.Fatalf("real contact waived: %g", got)
		}
	})
	t.Run("front obstruction", func(t *testing.T) {
		f, _ := paddingPair(0, sim.BusStandard, sim.BusArticulated)
		front := sim.NewBus(98, 0, 0)
		f.obstacles = append(f.obstacles, ObserveBus(front))
		if got := NewAutopilotDriver().controlSweptSpeed(f, 0); got != 0 {
			t.Fatalf("front blocker waived: %g", got)
		}
	})
	for _, name := range []string{"car", "moving neighbor", "opposing", "high speed", "turning"} {
		t.Run(name, func(t *testing.T) {
			f, ob := paddingPair(0, sim.BusStandard, sim.BusArticulated)
			steer := 0.
			switch name {
			case "car":
				ob.Bus = nil
			case "moving neighbor":
				ob.Speed = 5
			case "opposing":
				ob.Bus.Heading = math.Pi
			case "high speed":
				f.speedAbs = 100
			case "turning":
				steer = .1
			}
			if controlPaddingRelease(f, ob, steer, 20, .32) {
				t.Fatal("unsafe eligibility granted")
			}
		})
	}
}
