package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStoppedRearYieldCanReleaseLeadingRequest(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		lead := sim.NewBus(0, 0, h)
		lead.SetModel(sim.BusArticulated)
		rearRequest := sim.NewBus(0, 0, h)
		rearRequest.Pos = u.Mul(-10).Sub(r.Mul(52))
		rearRequest.SetModel(sim.BusArticulated)
		lead.LaneMergeRequest = &sim.BusLaneMergeRequest{Heading: h, Target: r.Mul(52), Remaining: .3}
		rearRequest.LaneMergeRequest = &sim.BusLaneMergeRequest{Heading: h, Target: r.Mul(52), Remaining: .3}
		ob := VehicleObstacle{Pos: u.Mul(-300).Add(r.Mul(52)), Heading: h, Length: 192, Width: 36, YieldingTo: rearRequest}
		if !yieldedVehicleClearsRear(lead, ob, u.X, u.Y) {
			t.Fatal("stopped rear grant for same lane failed to release leading bus")
		}
		rearRequest.Pos = u.Mul(10)
		if yieldedVehicleClearsRear(lead, ob, u.X, u.Y) {
			t.Fatal("rear request stole a leading bus's slot")
		}
		rearRequest.Pos = u.Mul(-10)
		rearRequest.LaneMergeRequest.Target = r.Mul(104)
		if yieldedVehicleClearsRear(lead, ob, u.X, u.Y) {
			t.Fatal("another lane's grant reused")
		}
		rearRequest.LaneMergeRequest.Target = r.Mul(52)
		ob.Pos = u.Mul(-150)
		if yieldedVehicleClearsRear(lead, ob, u.X, u.Y) {
			t.Fatal("overlapping rear body ignored")
		}
	}
}
