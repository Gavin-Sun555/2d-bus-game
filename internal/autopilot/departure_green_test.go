package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestTechCampusDockDepartureFirstGreen(t *testing.T) {
	const lw = 52.
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	var it *sim.TrafficIntersection
	for _, p := range tls.Intersections {
		if p.Center.Distance(sim.Vec2{X: 7800, Y: -1800}) < 1 {
			it = p
			break
		}
	}
	stop := w.Stops[3]
	b := sim.NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	b.DoorsOpen = true
	b.DoorProgress = 1
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.DisallowOncomingOvertake = true
	ad.State = AutopilotStateDocked
	ad.DockedStopID = 3
	ad.DwellTimer = 1
	route := sim.GetStandardRouteForMap(w.Theme, w)
	route.CurrentIndex = 4
	release := -1
	for tick := 0; tick < 3000; tick++ {
		it.Phase = sim.PhaseEWGreen
		line := it.GetStopLineDistForHeading(math.Pi/2, lw)
		gap := it.Center.Y - b.Pos.Y - line - b.Length/2
		if tick > 180 && gap < 30 && b.Speed < 1 && release < 0 {
			release = tick
		}
		if release >= 0 {
			it.Phase = sim.PhaseNSGreen
		}
		th, br, st, hb, d, _ := ad.ComputeControls(b, route, w, tls, nil, lw, 1./60)
		b.Update(1./60, th, br, st, hb, d)
		if b.Pos.Y > it.Center.Y+100 {
			return
		}
		if release >= 0 && tick-release > 450 {
			t.Fatalf("missed first green after docking: pos%v speed%.2f target%.1f state%v offset%.1f", b.Pos, b.Speed, ad.LastTargetSpeed, ad.State, ad.LaneOffset)
		}
	}
	t.Fatalf("departure did not reach junction: pos%v target%.1f state%v", b.Pos, ad.LastTargetSpeed, ad.State)
}
