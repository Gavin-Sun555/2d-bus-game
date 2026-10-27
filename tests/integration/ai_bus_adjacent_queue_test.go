package integration_test

import (
	"fmt"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// TestAIBusDoesNotBrakeForAdjacentLaneStoppedQueue reproduces the exact scenario
// from Metropolis CBD northern avenue (Stop 4 Olympic Stadium -> Stop 3 Tech Campus)
// where an AI bus cruising in Lane 1 must NOT freeze or brake to 0 km/h when
// Lane 2 (curb lane) is queued with stopped civilian traffic cars.
func TestAIBusDoesNotBrakeForAdjacentLaneStoppedQueue(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("width%.0f/berserk%t", lw, berserk), func(t *testing.T) {
				w := sim.NewWorld(lw, sim.MapThemeMetropolis)
				route := sim.GetStandardRouteForMap(sim.MapThemeMetropolis, w)

				// Leg from Stop 4 (Olympic Stadium, X=1950, Y=-1200) to Stop 3 (Tech Campus, X=2600, Y=-900)
				// In Metropolis standard route: [0, 1, 2, 4, 3, 11, ...] -> index 4 is target Stop 3
				route.CurrentIndex = 4

				stop4 := w.Stops[4]
				roadY := stop4.RoadCenter.Y

				// Northern 6-lane avenue is at Y = roadY. Heading East = 0.
				// Lane 0 = roadY + 0.5*lw (inner lane)
				// Lane 1 = roadY + 1.5*lw (middle lane - AI bus lane)
				// Lane 2 = roadY + 2.5*lw (curb lane - queue of stopped cars)
				busY := roadY + 1.5*lw
				busStartPos := sim.Vec2{X: stop4.Pos.X + 150.0, Y: busY}
				bus := sim.NewBus(busStartPos.X, busStartPos.Y, 0.0)
				bus.Speed = 40.0
				bus.MaxSpeedFwd = 80.0

				driver := autopilot.NewAutopilotDriver()
				driver.State = autopilot.AutopilotStateCruising
				driver.BerserkMode = berserk
				driver.LaneOffset = -lw
				driver.TargetLaneOffset = -lw
				driver.OvertakeState = autopilot.OvertakePassing
				if berserk {
					driver.BerserkMaxSpeed = 120.0
				}

				aiBus := &traffic.AIBus{
					ID:                  3,
					Bus:                 bus,
					Route:               route,
					Driver:              driver,
					LastExchangedStopID: 4,
				}
				abm := &traffic.AIBusManager{Buses: []*traffic.AIBus{aiBus}}

				// 4 traffic cars queued up at speed 0 in Lane 2 (curb lane)
				curbY := roadY + 2.5*lw
				var cars []*sim.TrafficCar
				for i := 0; i < 4; i++ {
					carPos := sim.Vec2{X: busStartPos.X + 80.0 + float64(i)*60.0, Y: curbY}
					car := &sim.TrafficCar{
						ID:          i + 1,
						Pos:         carPos,
						Heading:     0.0,
						Speed:       0.0,
						TargetSpeed: 0.0,
						Length:      48.0,
						Width:       22.0,
						LaneIndex:   2,
						IsEW:        true,
						RoadCenter:  roadY,
					}
					cars = append(cars, car)
				}

				// Cars in Lane 0 (inner lane)
				innerY := roadY + 0.5*lw
				carInner1 := &sim.TrafficCar{
					ID:          10,
					Pos:         sim.Vec2{X: busStartPos.X - 100.0, Y: innerY},
					Heading:     0.0,
					Speed:       0.0,
					TargetSpeed: 0.0,
					Length:      48.0,
					Width:       22.0,
					LaneIndex:   0,
					IsEW:        true,
					RoadCenter:  roadY,
				}
				carInner2 := &sim.TrafficCar{
					ID:          11,
					Pos:         sim.Vec2{X: busStartPos.X - 40.0, Y: innerY},
					Heading:     0.0,
					Speed:       0.0,
					TargetSpeed: 0.0,
					Length:      48.0,
					Width:       22.0,
					LaneIndex:   0,
					IsEW:        true,
					RoadCenter:  roadY,
				}
				cars = append(cars, carInner1, carInner2)

				tm := &traffic.TrafficManager{Cars: cars, AIBusMgr: abm}
				tls := &sim.TrafficLightSystem{}
				tls.BuildForWorld(w)

				dt := 1.0 / 60.0
				minSpeedSeen := 100.0
				passedQueue := false

				for step := 0; step < 720; step++ {
					oldPos := bus.Pos
					tm.Update(dt, nil, nil, 1, lw, w)
					abm.Update(dt, nil, w, tls, tm, lw, 1)

					// Verify physical continuity: no reset or teleportation
					dx := bus.Pos.Distance(oldPos)
					if dx > (bus.MaxSpeedFwd/60.0 + 10.0) {
						t.Fatalf("AI Bus was wrongfully teleported/reset at step %d: moved %.2f px from %v to %v",
							step, dx, oldPos, bus.Pos)
					}

					queueStart := busStartPos.X + 50.0
					queueEnd := busStartPos.X + 80.0 + 3.0*60.0 + 50.0

					// Track min speed while passing alongside the stopped queue
					if bus.Pos.X >= queueStart && bus.Pos.X <= queueEnd {
						if bus.Speed < minSpeedSeen {
							minSpeedSeen = bus.Speed
						}
					}

					// Verify no collision with any car
					busOBB := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
					for _, c := range cars {
						cOBB := sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}
						if hit, _, _ := sim.CheckOBBCollision(busOBB, cOBB); hit {
							t.Fatalf("Step %d: Collision between AI Bus and car %d! Bus: %v, Car: %v", step, c.ID, bus.Pos, c.Pos)
						}
					}

					if bus.Pos.X > queueEnd+50.0 {
						passedQueue = true
						break
					}
				}

				if !passedQueue {
					t.Fatalf("AI Bus failed to pass the stopped queue! Final pos: %v, speed: %.1f km/h, minSpeed: %.1f",
						bus.Pos, bus.Speed, minSpeedSeen)
				}

				// Min speed while cruising past the queue must remain healthy (> 20 km/h)
				if minSpeedSeen < 20.0 {
					t.Fatalf("AI Bus inappropriately slowed down for adjacent lane queue: min speed was %.1f km/h", minSpeedSeen)
				}
			})
		}
	}
}
