package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestArticulatedBusLaneUTurnClearsShortStationApproach(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		w := sim.NewWorld(52, sim.MapThemeMetropolis)
		// Remove the surrounding loop so this exercises a necessary U-turn.
		center := w.Stops[8].RoadCenter.X
		destinationRoad := w.Stops[20].RoadCenter.Y
		roads := w.Roads[:0]
		for _, road := range w.Roads {
			if (road.P1.X == center && road.P2.X == center) ||
				(road.P1.Y == road.P2.Y && (road.P1.Y == 0 || road.P1.Y == destinationRoad)) {
				roads = append(roads, road)
			}
		}
		w.Roads = roads
		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)
		for _, it := range tls.Intersections {
			it.Phase = sim.PhaseNSLeftGreen
		}
		p := stopRoadPoint(w.Stops[8], 52, w)
		b := sim.NewBus(p.X, p.Y, w.Stops[8].Heading)
		b.SetModel(sim.BusArticulated)
		ad := NewAutopilotDriver()
		ad.BerserkMode = berserk
		ad.BerserkMaxSpeed = 333
		ad.CruiseSpeedLimit = 333
		r := &sim.Route{StopIDs: []int{8, 20}, CurrentIndex: 1}
		reversed := false
		for tick := 0; tick < 1800; tick++ {
			th, br, st, hb, doors, _ := ad.ComputeControls(b, r, w, tls, nil, 52, 1./60)
			b.Update(1./60, th, br, st, hb, doors)
			if b.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
				t.Fatalf("berserk%v boundary tick%d pos%v", berserk, tick, b.Pos)
			}
			if math.Cos(b.Heading-w.Stops[8].Heading) < -.99 && b.Pos.Y > 700 {
				reversed = true
				break
			}
		}
		if !reversed {
			t.Fatalf("berserk%v U-turn failed: pos%v target%.1f offset%.1f", berserk, b.Pos, ad.LastTargetSpeed, ad.LaneOffset)
		}
	}
}
