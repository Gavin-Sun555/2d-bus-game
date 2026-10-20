package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkLeftTurnPreparesLaneAndKeepsTurnDirection(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, passing := range []bool{false, true} {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f/passing%v", lw, lanes, h, passing), func(t *testing.T) {
						w, _ := turnTestWorld(lanes, lanes, h, lw)
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						for _, inter := range tls.Intersections {
							inter.Phase = sim.PhaseEWLeftGreen
							if math.Abs(math.Sin(h)) > .5 {
								inter.Phase = sim.PhaseNSLeftGreen
							}
						}
						forward := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						lateral := (float64(lanes)/2 - .5) * lw
						if passing {
							lateral = lw * .5
						}
						pos := forward.Mul(-850).Add(right.Mul(lateral))
						bus := sim.NewBus(pos.X, pos.Y, h)
						bus.Speed = 150
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						if passing {
							ad.LaneOffset = -float64(lanes/2-1) * lw
							ad.TargetLaneOffset = ad.LaneOffset
							ad.OvertakeState = OvertakePassing
						}
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						crossedLine := false
						line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
						for step := 0; step < 2400; step++ {
							th, br, st, hb, doors, dock := ad.ComputeControls(bus, route, w, tls, nil, lw, 1.0/60)
							bus.Update(1.0/60, th, br, st, hb, doors)
							front := bus.Pos.Add(sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}.Mul(bus.Length / 2))
							if !crossedLine && front.Dot(forward) >= -line {
								crossedLine = true
								if math.Abs(front.Dot(right)) > lw*.9 {
									t.Fatalf("did not enter left lane before stop line: front%v", front)
								}
							}
							relative := sim.NormalizeAngle(bus.Heading - h)
							if relative > .35 || relative < -2 {
								t.Fatalf("wrong turn step%d pos%v yaw%.2f offset%.1f/%.1f target%v", step, bus.Pos, relative, ad.LaneOffset, ad.TargetLaneOffset, ad.LastPursuitPt)
							}
							if dock {
								return
							}
						}
						t.Fatalf("did not finish left turn pos%v", bus.Pos)
					})
				}
			}
		}
	}
}

func TestLeftTurnResidualOffsetDoesNotApplyStraightRoadClamp(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f", lw, lanes, h), func(t *testing.T) {
					w, _ := turnTestWorld(lanes, lanes, h, lw)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.drivePath = sim.LeftTurnPath(sim.Vec2{}, h, lw, w.JunctionAt(sim.Vec2{}, lw).TurnRadius)
					ad.cumLen = make([]float64, len(ad.drivePath))
					for i := 1; i < len(ad.drivePath); i++ {
						ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
					}
					const index = 10
					tangent := ad.drivePath[index+1].Sub(ad.drivePath[index]).Normalize()
					right := sim.Vec2{X: -tangent.Y, Y: tangent.X}
					ad.LaneOffset = -lw * .25
					ad.TargetLaneOffset = 0
					pos := ad.drivePath[index].Add(right.Mul(ad.LaneOffset))
					bus := sim.NewBus(pos.X, pos.Y, math.Atan2(tangent.Y, tangent.X))
					bus.Speed = 80
					ad.WaypointIdx = index
					if base := ad.basePathPosition(pos, w, bus.Heading, lw); base.Distance(ad.drivePath[index]) > 1e-6 {
						t.Fatalf("junction localization lost lane offset: %v", base)
					}
					steer, _, target, _ := ad.pursuitSteering(autopilotFrame{bus: bus, world: w, laneWidth: lw, s: ad.cumLen[index], speedAbs: 80, remaining: 500, distToStop: 1000})
					if steer > 0 {
						t.Fatalf("left arc steered right %.2f toward%v from%v", steer, target, bus.Pos)
					}
				})
			}
		}
	}
}

func TestLeftTurnRecognitionDuringLaneChange(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{4, 6} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, yaw := range []float64{-.25, 0, .25} {
					t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f/yaw%.2f", lw, lanes, h, yaw), func(t *testing.T) {
						w, stops := turnTestWorld(lanes, lanes, h, lw)
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.PrepareLeg(w, stops[0], stops[1], lw)
						forward := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
						pos := forward.Mul(-line - 300).Add(right.Mul((float64(lanes)/2 - .5) * lw))
						bus := sim.NewBus(pos.X, pos.Y, h+yaw)
						s, _ := ad.trackPath(pos)
						approach := ad.approachingIntersection(autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: lw, s: s}, true, lanes == 6)
						if !approach.isApproachingIntersection || !approach.isUpcomingLeft || approach.isUpcomingRight {
							t.Fatalf("left turn not recognized: %+v", approach)
						}
					})
				}
			}
		}
	}
}
