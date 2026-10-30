package game

import (
	"fmt"
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Opt-in replay from an isolated copy of the October 29 evening save.
func TestSavedArticulatedStationQueueRecovery(t *testing.T) {
	path := os.Getenv("BUS_ARTICULATED_JAM_SAVE")
	if path == "" {
		t.Skip("BUS_ARTICULATED_JAM_SAVE not set")
	}
	for _, seed := range []int64{5, 17, 29} {
		t.Run(fmt.Sprint(seed), func(t *testing.T) {
			sim.SeedSimulation(seed)
			g := NewGame(1280, 720)
			g.savePathOverride = path
			if err := g.LoadGame(); err != nil {
				t.Fatal(err)
			}
			type progress struct {
				last              sim.Vec2
				distance          float64
				afterAdvance      float64
				initialTarget     int
				advanced, served  bool
				health            float64
				longestWait, wait int
			}
			ps := map[int]*progress{}
			for _, a := range g.AIBusMgr.Buses {
				ps[a.ID] = &progress{last: a.Bus.Pos, initialTarget: a.Route.GetCurrentTargetStopID(), health: a.Bus.Durability}
			}
			playerHealth := g.Bus.Durability
			for tick := 0; tick < 18000; tick++ {
				if err := g.Update(); err != nil {
					t.Fatal(err)
				}
				if g.Bus.Durability < playerHealth {
					t.Fatalf("player contact tick%d", tick)
				}
				for _, a := range g.AIBusMgr.Buses {
					p := ps[a.ID]
					if p == nil {
						t.Fatal("unexpected replacement during replay")
					}
					step := a.Bus.Pos.Distance(p.last)
					if step > a.Bus.MaxSpeedFwd/60+2 {
						t.Fatalf("AI%d relocated tick%d by%g", a.ID, tick, step)
					}
					p.distance += step
					if p.advanced {
						p.afterAdvance += step
					}
					p.last = a.Bus.Pos
					if a.Bus.Durability < p.health {
						t.Fatalf("AI%d contact tick%d", a.ID, tick)
					}
					if a.Route.GetCurrentTargetStopID() != p.initialTarget {
						p.advanced = true
					}
					if a.Driver.State == autopilot.AutopilotStateDocked && a.Driver.DockedStopID == 184 && a.Bus.DoorsOpen {
						p.served = true
					}
					if a.ID <= 5 {
						if math.Abs(a.Bus.Speed) < .5 && a.Driver.State != autopilot.AutopilotStateDocked {
							p.wait++
							p.longestWait = max(p.longestWait, p.wait)
						} else {
							p.wait = 0
						}
						if a.Bus.CheckBoundaries(g.World, 52).Zone == sim.ZoneHardBarrier {
							t.Fatalf("AI%d boundary tick%d", a.ID, tick)
						}
					}
				}
				if tick == 10799 {
					for _, a := range g.AIBusMgr.Buses {
						if a.ID == 3 {
							t.Logf("AI3 at180s state%v doors%v dwell%.2f service%.2f target%d", a.Driver.State, a.Bus.DoorsOpen, a.Driver.DwellTimer, a.Bus.PassengerServiceRemaining(), a.Route.GetCurrentTargetStopID())
						}
					}
				}
				if tick >= 10799 {
					clear := true
					for id := 1; id <= 5; id++ {
						if !ps[id].advanced || ps[id].distance < 1000 || ps[id].afterAdvance < 300 {
							clear = false
						}
					}
					if clear {
						break
					}
				}
			}
			for id := 1; id <= 5; id++ {
				p := ps[id]
				t.Logf("AI%d distance%.0f target advanced%v served184%v longest non-service wait%.2fs", id, p.distance, p.advanced, p.served, float64(p.longestWait)/60)
				if !p.advanced || p.distance < 1000 || p.afterAdvance < 300 || p.longestWait > 7200 {
					t.Errorf("AI%d original queue did not clear", id)
				}
			}
			for _, id := range []int{2, 3, 4} {
				if !ps[id].served {
					t.Errorf("AI%d skipped a feasible station", id)
				}
			}
		})
	}
}
