package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBusLaneStraightUsesThroughSignal(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lw := range []float64{42, 52, 78} {
			t.Run(fmt.Sprintf("heading%.2f/width%.0f", heading, lw), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
				n := sim.Vec2{X: -u.Y, Y: u.X}
				w := busLaneTestWorld(lw)
				if math.Abs(u.Y) > .5 {
					w.Roads[0].P1 = sim.Vec2{Y: -3000}
					w.Roads[0].P2 = sim.Vec2{Y: 3000}
					w.Roads[1].P1 = sim.Vec2{X: -3000}
					w.Roads[1].P2 = sim.Vec2{X: 3000}
					w.RebuildIntersections()
				}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				inter := tls.Intersections[0]
				inter.Phase = sim.PhaseEWLeftGreen
				if math.Abs(u.Y) > .5 {
					inter.Phase = sim.PhaseNSLeftGreen
				}
				line := inter.GetStopLineDistForHeading(heading, lw)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				// Include dense straight waypoints near the center, as generated route sampling does.
				for _, x := range []float64{-1000, -line, 0, 40, 100, 300, 1000} {
					ad.drivePath = append(ad.drivePath, u.Mul(x).Add(n.Mul(3.5*lw)))
				}
				ad.cumLen = make([]float64, len(ad.drivePath))
				for i := 1; i < len(ad.drivePath); i++ {
					ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
				}
				b := sim.NewBus(0, 0, heading)
				b.Pos = u.Mul(-line - b.Length/2 - 10).Add(n.Mul(3.5 * lw))
				f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw}
				if got := ad.trafficLightSpeed(f, 445); got != 0 {
					t.Fatalf("left green / through red must stop straight bus; target %.1f", got)
				}
				inter.Phase = sim.PhaseEWGreen
				if math.Abs(u.Y) > .5 {
					inter.Phase = sim.PhaseNSGreen
				}
				if got := ad.trafficLightSpeed(f, 445); got != 445 {
					t.Fatalf("through green must release straight bus; target %.1f", got)
				}
			})
		}
	}
}

func TestBerserkFollowingBusCannotUsePassingFloor(t *testing.T) {
	for _, yaw := range []float64{-.2, .2} {
		t.Run(fmt.Sprintf("yaw%.1f", yaw), func(t *testing.T) {
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.BerserkMaxSpeed = 667
			ad.OvertakeState = OvertakeChangingLane
			ad.LaneOffset = -15
			ad.drivePath = []sim.Vec2{{X: -1000, Y: 78}, {X: 2000, Y: 78}}
			ad.cumLen = []float64{0, 3000}
			bus := sim.NewBus(0, 60, 0)
			bus.Speed = 80
			obs := VehicleObstacle{Pos: sim.Vec2{X: 110, Y: 98}, Length: 96, Width: 36, Speed: 0, Heading: yaw}
			f := autopilotFrame{bus: bus, world: busLaneTestWorld(52), laneWidth: 52, speedAbs: 80, obstacles: []VehicleObstacle{obs}}
			if got := ad.obstacleSpeed(f, 667); got > 10 {
				t.Fatalf("close stopped bus must override passing floor: target %.1f", got)
			}
		})
	}
}

func TestBusHeadwayIgnoresSeparateCorridors(t *testing.T) {
	for _, scenario := range []string{"adjacent", "behind", "oncoming", "ordinary mode"} {
		t.Run(scenario, func(t *testing.T) {
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			bus := sim.NewBus(0, 0, 0)
			bus.Speed = 80
			obs := VehicleObstacle{Pos: sim.Vec2{X: 110}, Heading: 0, Speed: 0, Length: 96, Width: 36}
			switch scenario {
			case "adjacent":
				obs.Pos.Y = 52
			case "behind":
				obs.Pos.X = -110
			case "oncoming":
				obs.Heading = math.Pi
			case "ordinary mode":
				ad.BerserkMode = false
			}
			f := autopilotFrame{bus: bus, world: busLaneTestWorld(52), laneWidth: 52, speedAbs: 80, obstacles: []VehicleObstacle{obs}}
			if got := ad.followingSpeed(f); !math.IsInf(got, 1) {
				t.Fatalf("unrelated leader limited speed to%.1f", got)
			}
		})
	}
}
