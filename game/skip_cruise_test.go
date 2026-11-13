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
			if occupied {
				continue // Like fleet buses, autopilot serves waiting riders beside an occupied bay.
			}
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
					cruise := g.Settings.GetBusMaxSpeed()
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
						g.advanceStandardAutopilotPastStop(1.0/60, lw)
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

func TestBerserkAutopilotKeepsOccupiedStopWithBoardingDemand(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.AutopilotBerserk = true
	stop := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	stop.SeedWaitingPassengers(1)
	if g.canAutopilotSkipStop(stop, true) {
		t.Fatal("occupied berth discarded a requested boarding")
	}
}

// Standard autopilot drives as a route fleet bus at the player's top speed;
// Berserk keeps uncapped straightaways and oncoming-lane passing.
func TestStandardAutopilotUsesFleetDriverSetup(t *testing.T) {
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	for _, kmh := range []int{80, 120, 160, 180} {
		g.Settings.SetMaxSpeed(kmh)
		g.Settings.AutopilotBerserk = false
		g.syncPlayerSpeedLimit()
		g.configurePlayerAutopilot()
		ad := g.AutopilotDriver
		want := math.Min(sim.SpeedKPH(kmh), g.Bus.MaxSpeedFwd)
		if g.Bus.MaxSpeedFwd != sim.SpeedKPH(kmh) && g.Settings.GetPlayerModel() == sim.BusStandard {
			t.Fatalf("[%d] player bus limit %.1f", kmh, g.Bus.MaxSpeedFwd)
		}
		if !ad.BerserkMode || ad.CruiseSpeedLimit != want || ad.BerserkMaxSpeed != want || !ad.DisallowOncomingOvertake || !ad.FleetBus {
			t.Fatalf("[%d] standard setup %+v", kmh, ad)
		}
		g.Settings.AutopilotBerserk = true
		g.configurePlayerAutopilot()
		if !ad.BerserkMode || ad.CruiseSpeedLimit != 0 || ad.DisallowOncomingOvertake || ad.FleetBus || ad.BerserkMaxSpeed != sim.SpeedKPH(kmh) {
			t.Fatalf("[%d] berserk setup %+v", kmh, ad)
		}
	}
}

// A rider reaching the platform after standard autopilot decided to skip
// waits for the next bus, as with fleet buses.
func TestStandardAutopilotKeepsSkipForLateRider(t *testing.T) {
	g := NewGame(1280, 720)
	lw := g.Settings.GetLaneWidth()
	g.Settings.AutopilotEnabled = true
	g.Settings.SetMaxSpeed(120)
	g.World = &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2500}, Lanes: 4}}}
	for i, x := range []float64{-1200, 0, 1400} {
		g.World.Stops = append(g.World.Stops, sim.NewBusStop(i, "test", sim.Vec2{X: x}, 0, 4, sim.RGBA(0, 0, 0, 255), lw, 0))
	}
	g.World.Stops[2].WaitingPassengers = 10
	g.Route = &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	g.TrafficMgr.Cars = nil
	g.AIBusMgr.Buses = nil
	g.TrafficLights = &sim.TrafficLightSystem{}
	g.Bus = sim.NewBus(-800, 1.5*lw, 0)
	g.syncPlayerSpeedLimit()
	g.Bus.Speed = g.Bus.MaxSpeedFwd
	g.AutopilotDriver = autopilot.NewAutopilotDriver()
	g.stopBellRung = false
	decided, seeded := false, false
	for frame := 0; frame < 600; frame++ {
		th, br, st, hb, doors := g.autopilotControls(1.0/60, lw)
		if g.Route.CurrentIndex == 1 && g.AutopilotDriver.CanSkipTargetStop {
			decided = true
		}
		if decided && !seeded && g.Bus.Pos.X > -650 {
			g.World.Stops[1].SeedWaitingPassengers(3)
			seeded = true
		}
		if br > .001 || hb || doors {
			t.Fatalf("late rider cancelled the skip: frame%d decided%v brake%.2f", frame, decided, br)
		}
		g.Bus.Update(1.0/60, th, br, st, hb, doors)
		g.advanceStandardAutopilotPastStop(1.0/60, lw)
		g.updateStopApproach()
		if g.Bus.Pos.X > 80 {
			if !seeded || g.Route.CurrentIndex != 2 {
				t.Fatalf("seeded%v route index %d", seeded, g.Route.CurrentIndex)
			}
			return
		}
	}
	t.Fatal("did not pass station")
}
