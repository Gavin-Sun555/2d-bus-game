package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestArticulatedStationFollowerUsesBayOrParallelBerth(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		t.Run(fmt.Sprintf("berserk%v", berserk), func(t *testing.T) {
			lw := 52.0
			w := sim.NewWorld(lw, sim.MapThemeMetropolis)
			w.LargeGrid = true
			w.BuildMap(sim.MapThemeMetropolis, lw)
			h := math.Pi
			stop := sim.NewBusStop(188, "Green Bamboo Book Centre", sim.Vec2{X: -23400, Y: -21600}, h, 8, sim.RGBA(56, 189, 248, 255), lw)
			stop.Pos = sim.Vec2{X: -23400, Y: -21829.12}
			stop.ShelterPos = sim.Vec2{X: -23400, Y: -21865.12}
			for i, s := range w.Stops {
				if s.ID == 188 {
					stop = s
					break
				}
				if i == len(w.Stops)-1 {
					w.Stops = append(w.Stops, stop)
				}
			}

			// Lead bus docked at front berth
			lead := sim.NewBus(stop.FrontBerthPosition().X, stop.FrontBerthPosition().Y, h)
			lead.SetModel(sim.BusArticulated)
			lead.DoorsOpen = true
			lead.DoorProgress = 1.0
			lead.Speed = 0

			// Follower bus approaching from road lane
			follower := sim.NewBus(-23000, -21782, h)
			follower.SetModel(sim.BusArticulated)
			follower.Speed = 50.0

			ad := NewAutopilotDriver()
			ad.BerserkMode = berserk
			ad.articulatedPath = true

			obs := []VehicleObstacle{ObserveBus(lead)}
			route := &sim.Route{StopIDs: []int{0, 188}, CurrentIndex: 1}
			ad.PrepareLeg(w, w.Stops[0], stop, lw)
			docked := false
			for tick := 0; tick < 1200; tick++ {
				th, br, st, hb, doors, dock := ad.ComputeControls(follower, route, w, nil, obs, lw, 1./60)
				follower.Update(1./60, th, br, st, hb, doors)
				if hit, _, _ := sim.CheckBusesCollision(follower, lead); hit {
					t.Fatalf("collision with lead bus at tick %d: follower=%v lead=%v", tick, follower.Pos, lead.Pos)
				}
				if b := follower.CheckBoundaries(w, lw); b.Zone == sim.ZoneHardBarrier {
					t.Fatalf("barrier collision at tick %d: %s at %v", tick, b.ObstacleName, follower.Pos)
				}
				if dock && follower.DoorsOpen {
					docked = true
					t.Logf("Successfully docked at tick %d! Final Pos=%v, speed=%.2f, doorsOpen=%v", tick, follower.Pos, follower.Speed, follower.DoorsOpen)
					break
				}
			}
			if !docked {
				t.Fatalf("failed to dock behind lead bus: final pos=%v speed=%.2f doorsOpen=%v", follower.Pos, follower.Speed, follower.DoorsOpen)
			}
			if berserk {
				if ad.stationDock == nil || ad.stationDock.rank != 1 || !stop.ContainsBusInOuterLane(follower, lw) {
					t.Fatalf("berserk follower did not use the clear parallel berth: %v", follower.Pos)
				}
			} else {
				if !stop.ContainsBus(follower) || ad.stationDock == nil || ad.stationDock.rank != 0 {
					t.Fatalf("ordinary follower did not reach the rear bay: %v", follower.Pos)
				}
			}
		})
	}
}
