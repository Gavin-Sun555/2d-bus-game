package game

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestEligibleSkipKeepsCruiseAcrossGameRouteAdvance(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, occupied := range []bool{false, true} {
			for _, inner := range []bool{false, true} {
				t.Run(fmt.Sprintf("berserk%v/occupied%v/inner%v", berserk, occupied, inner), func(t *testing.T) {
					g := NewGame(1280, 720)
					lw := g.Settings.GetLaneWidth()
					g.Settings.AutopilotEnabled = true
					g.Settings.AutopilotBerserk = berserk
					g.World = &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2500}, Lanes: 4}}}
					for i, x := range []float64{-1200, 0, 1400} {
						g.World.Stops = append(g.World.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw, 0))
					}
					g.World.Stops[2].WaitingPassengers = 10
					g.Route = &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
					g.TrafficMgr.Cars = nil
					g.AIBusMgr.Buses = nil
					g.TrafficLights = &sim.TrafficLightSystem{}
					if occupied {
						g.World.Stops[1].WaitingPassengers = 10
						st := g.World.Stops[1]
						g.AIBusMgr.Buses = []*traffic.AIBus{{Bus: sim.NewBus(st.Pos.X, st.Pos.Y, 0)}}
					}
					lateral := 1.5 * lw
					if inner {
						lateral = .5 * lw
					}
					g.Bus = sim.NewBus(-800, lateral, 0)
					cruise := 150.0
					if berserk {
						cruise = g.Settings.GetBerserkCruiseSpeed()
					}
					g.Bus.Speed = cruise
					g.AutopilotDriver = autopilot.NewAutopilotDriver()
					g.AutopilotDriver.LaneOffset = lateral - 1.5*lw
					g.AutopilotDriver.TargetLaneOffset = g.AutopilotDriver.LaneOffset
					g.stopBellRung = false
					g.skipCooldown = 0
					for frame := 0; frame < 600; frame++ {
						th, br, st, hb, doors := g.autopilotControls(1.0/60, lw)
						if br > .001 || hb || doors {
							t.Fatalf("braked to skip frame%d brake%.2f target%.1f", frame, br, g.AutopilotDriver.LastTargetSpeed)
						}
						g.Bus.Update(1.0/60, th, br, st, hb, doors)
						g.updateStopApproach()
						if math.Abs(g.Bus.Pos.Y-lateral) > 2 || g.Bus.Speed < cruise-5 {
							t.Fatalf("skip disturbed cruise frame%d pos%v speed%.1f", frame, g.Bus.Pos, g.Bus.Speed)
						}
						if g.Bus.Pos.X > 80 {
							if g.Route.CurrentIndex != 2 {
								t.Fatal("route did not advance")
							}
							return
						}
					}
					t.Fatal("did not pass station")
				})
			}
		}
	}
}
