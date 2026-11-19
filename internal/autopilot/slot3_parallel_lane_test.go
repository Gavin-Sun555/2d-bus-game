package autopilot

import (
	_ "embed"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot3-parallel-selected-lanes-20261117.json.gz
var slot3ParallelSelectedLanes []byte

func TestSlot3ParallelBusesFinishSelectedLane(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3ParallelSelectedLanes)
	s.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.World, 52)
	drivers := make([]*AutopilotDriver, len(s.Fleet))
	starts := make([]sim.Vec2, len(s.Fleet))
	for i, a := range s.Fleet {
		var err error
		drivers[i], err = a.State.Driver.Restore(s.World)
		if err != nil {
			t.Fatal(err)
		}
		drivers[i].BeforeRecovery = func() { t.Fatal("bus replanned") }
		starts[i] = a.State.Bus.Pos
	}
	for tick := 0; tick < 60*60; tick++ {
		s.Lights.Update(1. / 60)
		cleared := 0
		for i, a := range s.Fleet {
			obs := []VehicleObstacle{ObserveBus(s.Fleet[1-i].State.Bus)}
			b := a.State.Bus
			th, br, st, hb, doors, _ := drivers[i].ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, obs, 52, 1./60)
			b.Update(1./60, th, br, st, hb, doors)
			if b.HitsHardBarrier(s.World, 52) {
				t.Fatal("rejoin crossed barrier")
			}
			if hit, _, _ := sim.CheckBusesCollision(b, s.Fleet[1-i].State.Bus); hit {
				t.Fatal("parallel buses touched")
			}
			if b.Pos.Distance(starts[i]) > 500 {
				cleared++
			}
		}
		if cleared == len(s.Fleet) {
			return
		}
	}
	t.Fatal("parallel lane rejoin did not clear")
}
