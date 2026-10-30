package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusPriorityUsesRouteLeftTurnOnBusRoad(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, in, _, _ := wideLeftExitWorld(52, h, true)
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)
		b := sim.NewBus(0, 0, h)
		r := sim.Vec2{X: -in.Y, Y: in.X}
		b.Pos = in.Mul(-700).Add(r.Mul(182))
		ad := NewAutopilotDriver()
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
		req := ad.SignalPriorityRequest(b, tls, 52)
		want := sim.PhaseEWLeftGreen
		if math.Abs(in.Y) > .5 {
			want = sim.PhaseNSLeftGreen
		}
		if req == nil || req.Phase != want {
			t.Fatalf("heading%g wrong turn priority: %+v", h, req)
		}
		b.DoorsOpen = true
		if ad.SignalPriorityRequest(b, tls, 52) != nil {
			t.Fatal("boarding bus reserved green")
		}
		b.DoorsOpen = false
		for _, i := range tls.Intersections {
			i.ApproachBusLane = [4]bool{}
		}
		if ad.SignalPriorityRequest(b, tls, 52) != nil {
			t.Fatal("non bus road reserved green")
		}
	}
}
