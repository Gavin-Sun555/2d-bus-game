package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestAllBusModelsFollowArticulatedTailWithHeadway(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
			for _, berserk := range []bool{false, true} {
				for _, scenario := range []struct {
					speed float64
					brake bool
				}{{0, false}, {100, false}, {100, true}} {
					leadSpeed := scenario.speed
					t.Run(fmt.Sprintf("h%.2f/model%d/berserk%v/lead%.0f/brake%v", h, model, berserk, leadSpeed, scenario.brake), func(t *testing.T) {
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						leader := sim.NewBus(0, 0, h)
						leader.SetModel(sim.BusArticulated)
						leader.Speed = leadSpeed
						p := u.Mul(-412)
						bus := sim.NewBus(p.X, p.Y, h)
						bus.SetModel(model)
						bus.Speed = 180
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						minGap := math.Inf(1)
						for tick := 0; tick < 1800; tick++ {
							if scenario.brake && tick == 900 {
								leader.Speed = 0
							}
							leader.Pos = leader.Pos.Add(u.Mul(leader.Speed / 60))
							f := autopilotFrame{bus: bus, speedAbs: math.Abs(bus.Speed), obstacles: []VehicleObstacle{ObserveBus(leader)}, laneWidth: 52}
							target := ad.obstacleSpeed(f, 180)
							th, br := ad.longitudinalControls(bus.Speed, target)
							bus.Update(1./60, th, br, 0, false, false)
							gap := leader.Pos.Sub(bus.Pos).Dot(u) - leader.Length*1.5 - bus.Length/2
							minGap = math.Min(minGap, gap)
							if hit, _, _ := sim.CheckBusesCollision(bus, leader); hit {
								t.Fatalf("contact tick%d gap%.2f", tick, gap)
							}
						}
						if minGap < 25 {
							t.Fatalf("tail headway too short: %.2f", minGap)
						}
						if leader.Speed == 0 && (bus.Speed > .1 || minGap > 60) {
							t.Fatalf("did not settle into queue: gap%.2f speed%.2f", minGap, bus.Speed)
						}
						if leader.Speed > 0 && (math.Abs(bus.Speed-leader.Speed) > 8 || minGap < 50) {
							t.Fatalf("moving headway unstable: gap%.2f speed%.2f", minGap, bus.Speed)
						}
						t.Logf("minimum bumper gap %.2fpx", minGap)
					})
				}
			}
		}
	}
}

func TestOrdinaryBusArticulatedHeadwayKeepsAdjacentLaneOpen(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
		r := sim.Vec2{X: -u.Y, Y: u.X}
		p := u.Mul(190).Add(r.Mul(52))
		lead := sim.NewBus(p.X, p.Y, h)
		lead.SetModel(sim.BusArticulated)
		bus := sim.NewBus(0, 0, h)
		ad := NewAutopilotDriver()
		if limit := ad.followingSpeed(autopilotFrame{bus: bus, speedAbs: 80, obstacles: []VehicleObstacle{ObserveBus(lead)}}); !math.IsInf(limit, 1) {
			t.Fatalf("adjacent lane blocked: %.2f", limit)
		}
	}
}

func TestOrdinaryBusHeadwayRetainsBentTrailer(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, bend := range []float64{-.3, .3} {
			u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
			leader := sim.NewBus(0, 0, h)
			leader.SetModel(sim.BusArticulated)
			leader.TrailerHeading = h + bend
			rear := math.Inf(1)
			for _, part := range leader.BodyParts() {
				for _, corner := range sim.BodyCorners(part) {
					rear = math.Min(rear, corner.Dot(u))
				}
			}
			p := u.Mul(rear - 48 - 30)
			follower := sim.NewBus(p.X, p.Y, h)
			ad := NewAutopilotDriver()
			if limit := ad.followingSpeed(autopilotFrame{bus: follower, obstacles: []VehicleObstacle{ObserveBus(leader)}}); limit != 0 {
				t.Fatalf("bent trailer headway missed: heading%.2f bend%.2f limit%.2f", h, bend, limit)
			}
		}
	}
}
