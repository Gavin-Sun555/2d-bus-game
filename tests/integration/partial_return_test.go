package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

// Reproduce the partly returned player at the north end of Civic Avenue,
// with a route AI bus following closely in the single lawful travel lane.
func TestPartialTwoLaneReturnAheadOfFollowingBusCompletes(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		t.Run(fmt.Sprintf("width%.0f", lw), func(t *testing.T) {
			w := legacyMetropolisWorld(lw)
			from, to := w.Stops[2], w.Stops[4]
			route := &sim.Route{StopIDs: []int{2, 4}, CurrentIndex: 1}
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			center := sim.Vec2{X: 1300 * sim.MapGridScale, Y: -1200 * sim.MapGridScale}
			var it *sim.TrafficIntersection
			for _, candidate := range tls.Intersections {
				if candidate.Center.Distance(center) < 1 {
					it = candidate
				}
			}
			if it == nil {
				t.Fatal("missing junction")
			}
			it.Phase = sim.PhaseEWGreen
			h := -math.Pi / 2
			u := sim.Vec2{Y: -1}
			r := sim.Vec2{X: 1}
			line := it.GetStopLineDistForHeading(h, lw)
			player := sim.NewBus(center.X+.15*lw, center.Y+line+48+450, h)
			rear := sim.NewBus(center.X+.5*lw, player.Pos.Y+player.Length+26, h)
			ad := autopilot.NewAutopilotDriver()
			ad.BerserkMode = true
			ad.BerserkMaxSpeed = 667
			ad.PrepareLeg(w, from, to, lw)
			ad.LaneOffset = -.35 * lw
			ad.TargetLaneOffset = 0
			ad.OvertakeState = autopilot.OvertakeReturning
			rearAD := autopilot.NewAutopilotDriver()
			rearRoute := &sim.Route{StopIDs: []int{2, 4}, CurrentIndex: 1}
			fleet := &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 5, Bus: rear, Driver: rearAD, Route: rearRoute, LastExchangedStopID: 2}}}
			aligned, passed, yielded := false, false, false
			for tick := 0; tick < 3600; tick++ {
				if tick == 720 {
					it.Phase = sim.PhaseNSGreen
				}
				fleet.Update(1./60, player, w, tls, nil, lw)
				yielded = yielded || rear.YieldingToBus == player
				obs := []autopilot.VehicleObstacle{{Pos: rear.Pos, YieldingTo: rear.YieldingToBus, Heading: rear.Heading, Speed: rear.Speed, Length: rear.Length, Width: rear.Width, JunctionCorridor: rear.JunctionCorridor}}
				before := player.Pos
				th, br, st, hb, d, _ := ad.ComputeControls(player, route, w, tls, obs, lw, 1./60)
				player.Update(1./60, th, br, st, hb, d)
				if player.Pos.Distance(before) > math.Abs(player.Speed)/60+.01 {
					t.Fatal("return teleported")
				}
				if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: player.Pos, Heading: player.Heading, HalfLen: player.Length / 2, HalfWid: player.Width / 2}, sim.OBB{Center: rear.Pos, Heading: rear.Heading, HalfLen: rear.Length / 2, HalfWid: rear.Width / 2}); hit {
					t.Fatalf("bus contact tick%d", tick)
				}
				delta := player.Pos.Sub(center)
				yaw := math.Abs(sim.NormalizeAngle(player.Heading - h))
				if delta.Dot(u) < -line && math.Abs(delta.Dot(r)-lw/2) < 3 && yaw < .08 {
					aligned = true
				}
				if delta.Dot(u) > -line-player.Length/2 && !aligned {
					t.Fatalf("entered before completing return tick%d pos%v yaw%.2f", tick, player.Pos, yaw)
				}
				if delta.Dot(r) > 500 && rear.YieldingToBus != nil {
					t.Fatal("yield did not release after return")
				}
				if delta.Dot(r) > 500 {
					passed = true
					break
				}
			}
			if !aligned || !passed || !yielded {
				t.Fatalf("partial return stalled: aligned%v passed%v pos%v rear%v speed%.2f offset%.2f/%.2f target%.2f", aligned, passed, player.Pos, rear.Pos, player.Speed, ad.LaneOffset, ad.TargetLaneOffset, ad.LastTargetSpeed)
			}
		})
	}
}

// Frozen pre-expansion road fixture for narrow-road regression scenarios.
func legacyMetropolisWorld(lw float64) *sim.World {
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	w.Roads = w.Roads[:10]
	w.Stops = w.Stops[:12]
	for i := range w.Roads {
		r := &w.Roads[i]
		if r.P1.X == -5200*sim.MapGridScale {
			r.P1.X = -2600 * sim.MapGridScale
		}
		if r.P2.X == 5200*sim.MapGridScale {
			r.P2.X = 2600 * sim.MapGridScale
		}
		if r.P1.Y == -2400*sim.MapGridScale {
			r.P1.Y = -1200 * sim.MapGridScale
		}
		if r.P2.Y == 2400*sim.MapGridScale {
			r.P2.Y = 1200 * sim.MapGridScale
		}
		if i == 1 || i == 3 || i == 6 || i == 8 {
			r.Lanes = 2
			r.Type = sim.RoadStreet2Lane
		}
	}
	w.MatchStopRoadLanes(lw)
	w.RebuildIntersections()
	sim.ClearRoutePathCache()
	return w
}
