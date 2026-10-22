package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestTechCampusBusLaneGreenRestart(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, berserk := range []bool{false, true} {
			for _, y := range []float64{-1800, 0, 1800} {
				t.Run(fmt.Sprintf("w%.0f/berserk%v/y%.0f", lw, berserk, y), func(t *testing.T) {
					w := sim.NewWorld(lw, sim.MapThemeMetropolis)
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					c := sim.Vec2{X: 7800, Y: y}
					var it *sim.TrafficIntersection
					for _, p := range tls.Intersections {
						if p.Center.Distance(c) < 1 {
							it = p
							break
						}
					}
					it.Phase = sim.PhaseNSGreen
					b := sim.NewBus(c.X-3.5*lw, c.Y-it.GetStopLineDistForHeading(math.Pi/2, lw)-62, math.Pi/2)
					r := &sim.Route{StopIDs: []int{3, 11}, CurrentIndex: 1}
					ad := NewAutopilotDriver()
					ad.BerserkMode = berserk
					ad.DisallowOncomingOvertake = true
					for tick := 0; tick < 10; tick++ {
						th, br, st, hb, d, _ := ad.ComputeControls(b, r, w, tls, nil, lw, 1./60)
						b.Update(1./60, th, br, st, hb, d)
					}
					if ad.LastTargetSpeed < 1 {
						f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: b.Speed, remaining: ad.LastRemaining, targetStop: w.Stops[11]}
						t.Fatalf("green did not release: pos%v target%.2f signals%.2f admission%.2f hold%.2f road%v", b.Pos, ad.LastTargetSpeed, ad.trafficLightSpeed(f, 150), ad.junctionAdmissionSpeed(f), ad.busTurnHoldDistance, ad.roadSafetyHold)
					}
				})
			}
		}
	}
}

func TestTechCampusBusLaneQueueRestartsTogether(t *testing.T) {
	const lw = 52.
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	c := sim.Vec2{X: 7800, Y: -1800}
	var it *sim.TrafficIntersection
	for _, p := range tls.Intersections {
		if p.Center.Distance(c) < 1 {
			it = p
			break
		}
	}
	var buses []*sim.Bus
	var drivers []*AutopilotDriver
	for i := 0; i < 3; i++ {
		b := sim.NewBus(c.X-3.5*lw, c.Y-it.GetStopLineDistForHeading(math.Pi/2, lw)-62-float64(i)*125, math.Pi/2)
		buses = append(buses, b)
		ad := NewAutopilotDriver()
		ad.BerserkMode = i != 0
		ad.DisallowOncomingOvertake = true
		drivers = append(drivers, ad)
	}
	r := &sim.Route{StopIDs: []int{3, 11}, CurrentIndex: 1}
	for tick := 0; tick < 900; tick++ {
		it.Phase = sim.PhaseNSLeftGreen
		if tick >= 180 {
			it.Phase = sim.PhaseNSGreen
		}
		for i, b := range buses {
			var obs []VehicleObstacle
			for j, other := range buses {
				if i != j {
					obs = append(obs, VehicleObstacle{Pos: other.Pos, Heading: other.Heading, Speed: other.Speed, Length: other.Length, Width: other.Width, JunctionCorridor: other.JunctionCorridor})
				}
			}
			th, br, st, hb, d, _ := drivers[i].ComputeControls(b, r, w, tls, obs, lw, 1./60)
			b.Update(1./60, th, br, st, hb, d)
		}
	}
	for i, b := range buses {
		if b.Pos.Y < c.Y+100 {
			t.Fatalf("bus%d stayed before green junction: %v target%.1f", i, b.Pos, drivers[i].LastTargetSpeed)
		}
	}
}
