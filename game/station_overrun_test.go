package game

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStationPassageUsesForwardBerth(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, length := range []float64{144, 336, 500} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
				t.Run(fmt.Sprintf("heading%.2f/length%.0f/model%d", h, length, model), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
					r := sim.Vec2{X: -u.Y, Y: u.X}
					g := NewGame(1280, 720)
					stop := sim.NewBusStop(0, "target", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), 52)
					stop.Length = length
					stop.WaitingPassengers = 4
					next := sim.NewBusStop(1, "next", u.Mul(1500), h, 8, sim.RGBA(0, 0, 0, 255), 52)
					g.World = &sim.World{Stops: []*sim.BusStop{stop, next}}
					g.Route = &sim.Route{StopIDs: []int{0, 1}}
					g.AIBusMgr.Buses = nil
					g.AutopilotDriver = autopilot.NewAutopilotDriver()
					g.Settings.AutopilotEnabled = true
					g.Settings.AutopilotBerserk = true
					g.Settings.TrafficFinesEnabled = true
					g.AutopilotDriver.LaneOffset = -52
					g.Bus.SetModel(model)
					g.Bus.Heading = h
					g.Bus.ResetArticulation()
					g.Bus.Speed = 0
					g.stopBellRung = true
					g.passengersAlighting = 1
					g.Bus.Passengers = []sim.Passenger{{TargetStopID: 0}}
					g.Bus.PassengerCount = 1
					g.Cash = 100
					// Include the rear berth, forward marker, and a nose that protrudes
					// beyond it while the front-section reference can still dock.
					for _, offset := range []float64{-120, -20, 0, 10} {
						g.Bus.Pos = stop.FrontBerthPosition().Add(u.Mul(offset)).Sub(r.Mul(70))
						g.updateStopApproach()
						if g.Route.GetCurrentTargetStopID() != 0 || g.Cash != 100 || g.Bus.Passengers[0].TargetStopID != 0 {
							t.Fatalf("premature missed stop offset%.0f cash%d target%d", offset, g.Cash, g.Route.GetCurrentTargetStopID())
						}
					}
					// A real passage still fines and advances exactly once in AI mode.
					g.Bus.Pos = stop.FrontBerthPosition().Add(u.Mul(75)).Sub(r.Mul(70))
					g.Settings.AutopilotEnabled = false
					g.updateStopApproach()
					if g.Cash != 100 || g.Route.GetCurrentTargetStopID() != 0 {
						t.Fatal("manual passage auto-fined")
					}
					g.Settings.AutopilotEnabled = true
					g.updateStopApproach()
					if g.Cash != 50 || g.Route.GetCurrentTargetStopID() != 1 || g.Bus.Passengers[0].TargetStopID != 1 {
						t.Fatalf("genuine passage not handled cash%d target%d", g.Cash, g.Route.GetCurrentTargetStopID())
					}
					g.updateStopApproach()
					if g.Cash != 50 {
						t.Fatal("duplicate fine")
					}
				})
			}
		}
	}
}
