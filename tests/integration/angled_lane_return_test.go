package integration_test

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"testing"
)

// A bus stopped obliquely in the median lane must be able to unwind and
// continue when its forward road is empty, with queues behind in both lanes.
func TestAngledBusReturnDoesNotLockStraightRoadQueue(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/berserk%v", h, berserk), func(t *testing.T) {
				const lw = 52.
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 4}}}
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-2000), h, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(2000), h, 4, sim.RGBA(0, 0, 0, 255), lw)}
				lead := sim.NewBus(r.X*26, r.Y*26, h+.23)
				driver := autopilot.NewAutopilotDriver()
				driver.BerserkMode = berserk
				driver.CruiseSpeedLimit = sim.SpeedKPH(50)
				driver.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				driver.LaneOffset = -lw
				driver.TargetLaneOffset = 0
				driver.OvertakeState = autopilot.OvertakeReturning
				rear := sim.NewBus(0, 0, h)
				rear.Pos = r.Mul(78).Sub(u.Mul(210))
				rearAD := autopilot.NewAutopilotDriver()
				rearAD.BerserkMode = true
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				rearRoute := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{
					{ID: 1, Pos: r.Mul(78).Sub(u.Mul(100)), Heading: h, Length: 48, Width: 22, TargetSpeed: 100, LaneIndex: 1, IsEW: math.Abs(u.X) > .5},
					{ID: 2, Pos: r.Mul(26).Sub(u.Mul(105)), Heading: h, Length: 48, Width: 22, TargetSpeed: 100, LaneIndex: 0, IsEW: math.Abs(u.X) > .5},
					{ID: 3, Pos: r.Mul(26).Sub(u.Mul(170)), Heading: h, Length: 48, Width: 22, TargetSpeed: 100, LaneIndex: 0, IsEW: math.Abs(u.X) > .5},
				}}
				fleet := &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 6, Bus: lead, Driver: driver, Route: route, LastExchangedStopID: 0}}}
				tm.AIBusMgr = fleet
				for tick := 0; tick < 1800; tick++ {
					for i, c := range tm.Cars {
						tm.StepCar(1./60, c, i, rear, nil, len(tm.Cars), lw, w)
					}
					fleet.Update(1./60, rear, w, nil, tm, lw)
					obs := []autopilot.VehicleObstacle{{Pos: lead.Pos, Heading: lead.Heading, Speed: lead.Speed, Length: lead.Length, Width: lead.Width}}
					for _, c := range tm.Cars {
						obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
					}
					th, br, st, hb, d, _ := rearAD.ComputeControls(rear, rearRoute, w, nil, obs, lw, 1./60)
					rear.Update(1./60, th, br, st, hb, d)
					for _, c := range tm.Cars {
						if hit, _, _ := sim.CheckOBBCollision(sim.OBB{Center: lead.Pos, Heading: lead.Heading, HalfLen: lead.Length / 2, HalfWid: lead.Width / 2}, sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}); hit {
							t.Fatalf("lead contacted car tick%d", tick)
						}
					}
					if lead.Pos.Dot(u) > 800 && rear.Pos.Dot(u) > 200 {
						return
					}
				}
				t.Fatalf("angled queue stayed locked: lead%v yaw%.2f speed%.1f target%.1f rear%v", lead.Pos, sim.NormalizeAngle(lead.Heading-h), lead.Speed, driver.LastTargetSpeed, rear.Pos)
			})
		}
	}
}
