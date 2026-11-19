package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCompactBusKeepsItsBoardingWindowWhileWaiting(t *testing.T) {
	const lw = 52.
	w, forward, right := serviceTestWorld(0, lw)
	stop := w.Stops[1]
	p := stop.FrontBerthPosition().Sub(forward.Mul(41))
	p = p.Add(right.Mul(89.5 - p.Sub(stop.RoadCenter).Dot(right)))
	bus := sim.NewBus(p.X, p.Y, -.211)
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	f := autopilotFrame{bus: bus, world: w, targetStop: stop, laneWidth: lw}
	if !ad.waitingAtUsableOuterBoardingPose(f, stationDockInputs{brake: 1}) {
		t.Fatal("compact bus should wait for boarding permission at its usable pose")
	}
	ad.stationDockUnreachable = true
	ad.updateOuterLaneDock(f, outerDockUnreachableWait)
	if !ad.outerDockAllowed {
		t.Fatal("waiting at the proven outer berth should grant boarding permission")
	}
}

// The Berserk test above boards beside the platform when its front berth is
// taken. A route bus (and standard autopilot) in the same pose waits for the
// free rear berth and boards inside the bay instead.
func TestFleetBusBoardsInBayWhenRearBerthIsFree(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("h%.2f/w%.0f", h, lw), func(t *testing.T) {
				w, u, r := serviceTestWorld(h, lw)
				stop := w.Stops[1]
				lane := stop.RoadCenter.Add(r.Mul(3.5 * lw))
				p := lane.Sub(u.Mul(240))
				bus := sim.NewBus(p.X, p.Y, h)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 60
				ahead := sim.NewBus(lane.X+u.X*180, lane.Y+u.Y*180, h)
				bay := sim.NewBus(stop.FrontBerthPosition().X, stop.FrontBerthPosition().Y, h)
				obs := []VehicleObstacle{ObserveBus(ahead), ObserveBus(bay)}
				ad := NewAutopilotDriver()
				ad.BerserkMode, ad.FleetBus, ad.DisallowOncomingOvertake = true, true, true
				route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
				for tick := 0; tick < 2400; tick++ {
					th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					for _, ob := range []*sim.Bus{ahead, bay} {
						if hit, _, _ := sim.CheckBusesCollision(bus, ob); hit {
							t.Fatalf("contact tick%d", tick)
						}
					}
					if dock && bus.DoorsOpen {
						if !stop.ContainsBus(bus) {
							t.Fatalf("tick%d boarded outside the bay at %v (outer lane %v)", tick, bus.Pos, stop.ContainsBusInOuterLane(bus, lw))
						}
						return
					}
				}
				t.Fatalf("never boarded; pos %v state %s", bus.Pos, ad.State)
			})
		}
	}
}
